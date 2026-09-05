#include "DriveSegment.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cassert>

using namespace pyvrp;
using namespace pyvrp::search;

bool const pyvrp::search::clockTrigger = []
{
    auto const *env = std::getenv("PYVRP_CLOCK_TRIGGER");
    return env && *env && !(env[0] == '0' && env[1] == '\0');
}();

bool const pyvrp::search::composeEnabled = []
{
    if (!pyvrp::search::clockTrigger)
        return false;
    auto const *env = std::getenv("PYVRP_COMPOSE");
    return !(env && env[0] == '0' && env[1] == '\0');
}();

bool const pyvrp::search::composeNoLB = []
{
    if (!pyvrp::search::composeEnabled)
        return false;
    auto const *env = std::getenv("PYVRP_COMPOSE_NOLB");
    return env && *env && !(env[0] == '0' && env[1] == '\0');
}();

bool const pyvrp::search::composeCheck = []
{
    auto const *env = std::getenv("PYVRP_COMPOSE_CHECK");
    return env && *env && !(env[0] == '0' && env[1] == '\0');
}();

int const pyvrp::search::composeBug = []
{
    auto const *env = std::getenv("PYVRP_COMPOSE_BUG");
    return env && *env ? std::atoi(env) : 0;
}();

DriveSegment DriveSegment::fromClient(Duration service)
{
    auto const s = static_cast<int64_t>(service.get());
    return {0, s, s, 0, 0};
}

DriveSegment DriveSegment::fromDepot() { return {}; }

DriveSegment DriveSegment::fromVehicleType() { return {}; }

DriveSegment::DriveSegment(int64_t driveTime,
                           int64_t workTime,
                           int64_t dutyTime,
                           uint16_t breaksTakenMask,
                           int64_t lastResetAt)
    : driveTime_(driveTime),
      workTime_(workTime),
      dutyTime_(dutyTime),
      lastResetAt_(lastResetAt),
      breaksTakenMask_(breaksTakenMask)
{
}

DriveSegment DriveSegment::merge(Duration const edgeDur,
                                  DriveSegment const &first,
                                  DriveSegment const &second,
                                  std::vector<pyvrp::CustomBreak> const &breaks,
                                  Duration const atSecond,
                                  uint16_t const upcomingMask,
                                  Duration const extraWork,
                                  uint16_t *const breakDueMask,
                                  int64_t *const firstDueClock,
                                  Duration const twEarlyAnchor)
{
    using pyvrp::CustomBreakReset;
    using pyvrp::CustomBreakTrigger;

    auto const edge = static_cast<int64_t>(edgeDur.get());
    auto const atSecondVal = static_cast<int64_t>(atSecond.get());
    auto const extra = static_cast<int64_t>(extraWork.get());

    int64_t drive = first.driveTime_ + edge + second.driveTime_;
    int64_t work = first.workTime_ + edge + extra + second.workTime_;
    int64_t wait = std::max<int64_t>(
        0, atSecondVal - (first.dutyTime_ + first.lastResetAt_ + edge));
    int64_t duty = first.dutyTime_ + edge + wait + extra + second.dutyTime_;
    int64_t lastResetAt = first.lastResetAt_;
    uint16_t takenMask = first.breaksTakenMask_ | second.breaksTakenMask_;

    // Evaluate each configured break in priority order (pre-sorted by caller).
    for (auto const &brk : breaks)
    {
        auto const bit = static_cast<uint16_t>(1u) << (brk.id & 0xF);

        // Condition: skip if route duration is below this break's minimum.
        auto const minRoute = static_cast<int64_t>(brk.conditionMinRouteS.get());
        if (minRoute > 0 && duty < minRoute)
            continue;

        // Pure trigger condition (D2): computed BEFORE the taken/upcoming
        // gates so the FIRST-DUE moment is captured even when the break's
        // node is still scheduled ahead (upcoming) or the break was already
        // taken elsewhere. The gates below still decide the violation
        // (breakDueMask) and reset bookkeeping, but never erase the due
        // clock — that is what makes a break stacked at the end of the route
        // (or skipped entirely) price its lateness in seconds.
        auto const triggerVal = static_cast<int64_t>(brk.triggerValue.get());
        bool triggered = false;
        switch (brk.trigger)
        {
        case CustomBreakTrigger::DRIVE_TIME:
            triggered = drive > triggerVal;
            break;
        case CustomBreakTrigger::WORK_TIME:
            triggered = work > triggerVal;
            break;
        case CustomBreakTrigger::DUTY_TIME:
            triggered = duty > triggerVal;
            break;
        case CustomBreakTrigger::CLOCK_TIME:
            // CLOCK_TIME triggers when atSecond is past the last time window
            // end. Pure condition (no taken clause — the taken gate below
            // suppresses the violation/reset bookkeeping, not the due clock).
            // Relative windows are offsets from route start, so compare
            // against twEarlyAnchor + close in the absolute (midnight-
            // anchored) clock; absolute windows compare against close only.
            triggered = !brk.tws.empty()
                        && atSecondVal
                               > (brk.twsRelative
                                      ? static_cast<int64_t>(
                                            twEarlyAnchor.get())
                                      : 0)
                                     + static_cast<int64_t>(
                                         brk.tws.back().second.get());
            break;
        }

        if (triggered && firstDueClock && firstDueClock[brk.id] < 0)
            firstDueClock[brk.id]
                = (clockTrigger
                   && brk.trigger == CustomBreakTrigger::DUTY_TIME)
                      ? lastResetAt + std::max<int64_t>(triggerVal, minRoute)
                      : atSecondVal;

        // Skip if this specific break ID was already taken in either segment.
        if (takenMask & bit)
            continue;

        // Upcoming: this break's CUSTOM_BREAK node lies at a route position
        // strictly after the current boundary. It is still scheduled ahead,
        // so its trigger must NOT fire here: the violation (breakDueMask) is
        // only incurred at boundaries subsequent to the break's own position.
        // The gate at the break node decides service/eligibility; if the
        // break is positioned too early, the bit is removed there and the
        // trigger fires at the following boundaries instead.
        if (upcomingMask & bit)
            continue;

        if (triggered)
        {
            if (brk.mandatory && breakDueMask)
                *breakDueMask |= bit;

            // Mark this break as taken.
            takenMask |= bit;
            for (auto sid : brk.supersedes)
                takenMask |= static_cast<uint16_t>(1u) << (sid & 0xF);

            // Apply the reset to accumulators.
            switch (brk.reset)
            {
            case CustomBreakReset::ALL_TIMERS:
                drive = second.driveTime_;
                work = second.workTime_;
                duty = second.dutyTime_;
                if (clockTrigger)
                    lastResetAt = atSecondVal;  // lane 12: coherent clock
                takenMask = static_cast<uint16_t>(1u) << (brk.id & 0xF);
                for (auto sid : brk.supersedes)
                    takenMask |= static_cast<uint16_t>(1u) << (sid & 0xF);
                break;
            case CustomBreakReset::DRIVE_AND_WORK:
                drive = second.driveTime_;
                work = second.workTime_;
                break;
            case CustomBreakReset::DRIVE_TIMER:
                drive = second.driveTime_;
                break;
            case CustomBreakReset::WORK_TIMER:
                work = second.workTime_;
                break;
            case CustomBreakReset::NONE:
                break;  // No accumulators reset.
            }
        }
    }

    return {drive, work, duty, takenMask, lastResetAt};
}

#ifdef PYVRP_STREAM_STATS
namespace
{
// Where the per-rule loop actually exits. It runs at every node of every
// candidate, so its exit profile decides whether the loop is worth
// optimising and which exit to aim at. Compiled out in the shipped binary.
struct RuleStats
{
    unsigned long long rlIter = 0, rlSettled = 0, rlCond = 0;
    unsigned long long rlTested = 0, rlNoTrig = 0, rlUpcoming = 0;
    unsigned long long rlFire = 0, rlAllTimers = 0, rlDueRec = 0;
    ~RuleStats()
    {
        if (!rlIter)
            return;
        std::fprintf(stderr,
                     "[rule-stats] iters=%llu  settled=%.3f cond=%.3f "
                     "reached-trigger=%.3f of-which-no-fire=%.3f upcoming=%.3f\n"
                     "[rule-stats] fire=%llu all-timers-replace=%llu due-recorded=%llu\n",
                     rlIter,
                     double(rlSettled) / double(rlIter),
                     double(rlCond) / double(rlIter),
                     double(rlTested) / double(rlIter),
                     double(rlNoTrig) / double(rlIter),
                     double(rlUpcoming) / double(rlIter),
                     rlFire, rlAllTimers, rlDueRec);
    }
};
RuleStats ruleStats{};
}  // namespace
#define PYVRP_RULE_STAT(f) (ruleStats.f += 1)
#else
#define PYVRP_RULE_STAT(f) ((void)0)
#endif

DriveSegment DriveSegment::merge(Duration const edgeDur,
                                  DriveSegment const &first,
                                  DriveSegment const &second,
                                  std::vector<pyvrp::BreakRule> const &rules,
                                  Duration const atSecond,
                                  uint16_t const upcomingMask,
                                  Duration const extraWork,
                                  uint16_t *const breakDueMask,
                                  int64_t *const firstDueClock)
{
    using pyvrp::CustomBreakReset;
    using pyvrp::CustomBreakTrigger;

    auto const edge = static_cast<int64_t>(edgeDur.get());
    auto const atSecondVal = static_cast<int64_t>(atSecond.get());
    auto const extra = static_cast<int64_t>(extraWork.get());

    int64_t drive = first.driveTime_ + edge + second.driveTime_;
    int64_t work = first.workTime_ + edge + extra + second.workTime_;
    int64_t wait = std::max<int64_t>(
        0, atSecondVal - (first.dutyTime_ + first.lastResetAt_ + edge));
    int64_t duty = first.dutyTime_ + edge + wait + extra + second.dutyTime_;
    int64_t lastResetAt = first.lastResetAt_;
    uint16_t takenMask = first.breaksTakenMask_ | second.breaksTakenMask_;

    // Evaluate each configured break in priority order (pre-sorted by caller).
    for (auto const &rule : rules)
    {
        PYVRP_RULE_STAT(rlIter);
        auto const bit = rule.bit;

        // Settled rule: already taken AND already holding a first-due clock.
        // Everything below is then observationally a no-op -- the trigger test
        // exists only to record that clock (written once) and to reach the
        // taken gate, which skips the rest. Bailing out here keeps the fold
        // from re-testing, at every node, rules that can no longer do
        // anything. ``takenMask`` is re-read per rule because an ALL_TIMERS
        // reset rewrites it mid-loop.
        if ((takenMask & bit)
            && (!firstDueClock || firstDueClock[rule.id] >= 0))
        {
            PYVRP_RULE_STAT(rlSettled);
            continue;
        }

        // Same argument, applied to the OTHER way a rule stops being able to
        // do anything at this boundary. A break whose node lies ahead has its
        // violation and reset deferred to that node (see the upcoming gate
        // below), so the only thing the body can still accomplish for it is to
        // record the first-due clock -- and once that is recorded the whole
        // iteration is observationally a no-op. Without this the trigger test
        // is re-run at every subsequent node until the break's own position is
        // reached, which the counters put at ~6.4 of the 42.3 rule iterations
        // per duration() call, all of them on the expensive path.
        if ((upcomingMask & bit) && firstDueClock
            && firstDueClock[rule.id] >= 0)
        {
            PYVRP_RULE_STAT(rlUpcoming);
            continue;
        }

        // Condition: skip if route duration is below this break's minimum.
        if (rule.conditionMinRouteS > 0 && duty < rule.conditionMinRouteS)
        {
            PYVRP_RULE_STAT(rlCond);
            continue;
        }

        // Pure trigger condition (D2): computed BEFORE the taken/upcoming
        // gates so the FIRST-DUE moment is captured even when the break's
        // node is still scheduled ahead (upcoming) or the break was already
        // taken elsewhere. The gates below still decide the violation
        // (breakDueMask) and reset bookkeeping, but never erase the due
        // clock — that is what makes a break stacked at the end of the route
        // (or skipped entirely) price its lateness in seconds.
        auto const triggerVal = rule.triggerValue;
        bool triggered = false;
        switch (rule.trigger)
        {
        case CustomBreakTrigger::DRIVE_TIME:
            triggered = drive > triggerVal;
            break;
        case CustomBreakTrigger::WORK_TIME:
            triggered = work > triggerVal;
            break;
        case CustomBreakTrigger::DUTY_TIME:
            triggered = duty > triggerVal;
            break;
        case CustomBreakTrigger::CLOCK_TIME:
            // CLOCK_TIME triggers when atSecond is past the last time window
            // end. Pure condition (no taken clause — the taken gate below
            // suppresses the violation/reset bookkeeping, not the due
            // clock). closeAbs already folds in the (possibly relative)
            // anchor and is INT64_MAX when there is no window, so this is
            // unconditionally false for window-less breaks — same result as
            // the CustomBreak overload's ``!tws.empty() && ...`` check.
            triggered = atSecondVal > rule.closeAbs;
            break;
        }

        PYVRP_RULE_STAT(rlTested);
        if (!triggered)
            PYVRP_RULE_STAT(rlNoTrig);
        if (triggered && firstDueClock && firstDueClock[rule.id] < 0)
            PYVRP_RULE_STAT(rlDueRec);
        if (triggered && firstDueClock && firstDueClock[rule.id] < 0)
            firstDueClock[rule.id]
                = (clockTrigger
                   && rule.trigger == CustomBreakTrigger::DUTY_TIME)
                      ? lastResetAt
                            + std::max<int64_t>(triggerVal,
                                                rule.conditionMinRouteS)
                      : atSecondVal;

        // Skip if this specific break ID was already taken in either segment.
        if (takenMask & bit)
            continue;

        // Upcoming: this break's CUSTOM_BREAK node lies at a route position
        // strictly after the current boundary. It is still scheduled ahead,
        // so its trigger must NOT fire here: the violation (breakDueMask) is
        // only incurred at boundaries subsequent to the break's own position.
        // The gate at the break node decides service/eligibility; if the
        // break is positioned too early, the bit is removed there and the
        // trigger fires at the following boundaries instead.
        if (upcomingMask & bit)
            continue;

        if (triggered)
        {
            PYVRP_RULE_STAT(rlFire);
            if (rule.mandatory && breakDueMask)
                *breakDueMask |= bit;

            // Mark this break as taken.
            takenMask |= bit;
            takenMask |= rule.supersedesMask;

            // Apply the reset to accumulators.
            switch (rule.reset)
            {
            case CustomBreakReset::ALL_TIMERS:
                PYVRP_RULE_STAT(rlAllTimers);
                drive = second.driveTime_;
                work = second.workTime_;
                duty = second.dutyTime_;
                if (clockTrigger)
                    lastResetAt = atSecondVal;  // lane 12: coherent clock
                takenMask = bit | rule.supersedesMask;
                break;
            case CustomBreakReset::DRIVE_AND_WORK:
                drive = second.driveTime_;
                work = second.workTime_;
                break;
            case CustomBreakReset::DRIVE_TIMER:
                drive = second.driveTime_;
                break;
            case CustomBreakReset::WORK_TIMER:
                work = second.workTime_;
                break;
            case CustomBreakReset::NONE:
                break;  // No accumulators reset.
            }
        }
    }

    return {drive, work, duty, takenMask, lastResetAt};
}

ForwardEvalResult
pyvrp::search::evaluateForwardPass(std::vector<Activity> const &activities,
                                   std::vector<size_t> const &locations,
                                   std::vector<Duration> &atSecond,
                                   std::vector<DurationSegment> *durPrefixOut,
                                   std::vector<Duration> *extendedBreakServices,
                                   ProblemData const &data,
                                   VehicleType const &vehicleType,
                                   std::vector<DurationSegment> *durAtOut,
                                   std::vector<DriveSegment> *drivePrefixOut,
                                   int64_t *firstDueOut,
                                   size_t *firstDuePosOut,
                                   uint16_t *servedMaskOut,
                                   bool prefixReady)
{
    auto const n = activities.size();
    assert(n >= 2);
    assert(!prefixReady || (durAtOut && durPrefixOut));

    // Per-call working vectors. Kept per thread across calls so a call costs
    // no allocations; every vector is (re)assigned to its initial contents
    // below before use, exactly as the former locals were constructed.
    struct Scratch
    {
        std::vector<int64_t> firstDueClock;
        std::vector<size_t> breakNodeAt;
        std::vector<size_t> firstDuePos;
        std::vector<bool> breakEligibleAt;
        std::vector<bool> breakPastCloseAt;
        std::vector<DurationSegment> durAt;
        std::vector<DurationSegment> durLocal;
        std::vector<DriveSegment> driveAt;
        std::vector<DriveSegment> driveBefore;
        std::vector<uint16_t> upcomingBreakMaskAt;
    };
    thread_local Scratch scratch;

    auto const &breaks = vehicleType.custom_breaks;
    auto const resetAtReload = vehicleType.reset_breaks_at_reload;
    auto const profile = vehicleType.profile;
    auto const &durMatrix = data.durationMatrix(profile);

    // D2/D3: per-break-id first-due tracking, break node positions and the
    // served mask — used to price mandatory-break lateness in SECONDS.
    size_t maxBreakId = 0;
    for (auto const &brk : breaks)
        maxBreakId = std::max(maxBreakId, static_cast<size_t>(brk.id));
    auto &firstDueClock = scratch.firstDueClock;
    firstDueClock.assign(maxBreakId + 1, -1);
    auto &breakNodeAt = scratch.breakNodeAt;
    breakNodeAt.assign(maxBreakId + 1, std::numeric_limits<size_t>::max());
    // Boundary (flat index) where each break's pure trigger first fired on the
    // FINAL drive pass. Mirrors firstDueClock, reset per pass (D2/D3).
    auto &firstDuePos = scratch.firstDuePos;
    firstDuePos.assign(maxBreakId + 1, std::numeric_limits<size_t>::max());
    uint16_t servedMask = 0;
    // Per-node eligibility frozen on the FIRST drive pass (D3/N1): the second
    // pass (final clock) must not flip a served/not-served decision.
    auto &breakEligibleAt = scratch.breakEligibleAt;
    breakEligibleAt.assign(n, false);
    auto &breakPastCloseAt = scratch.breakPastCloseAt;
    breakPastCloseAt.assign(n, false);
    bool firstDrivePass = true;

    // ---- Step 1: Build durAt singletons (per-node DurationSegment) ----

    // With ``prefixReady`` the caller's singletons are used -- and mutated --
    // in place (they are what ``*durAtOut`` would have received anyway).
    auto &durAt = prefixReady ? *durAtOut : scratch.durAt;

    if (!prefixReady)
    {
        durAt.assign(n, DurationSegment());

        // Start depot (node 0)
        auto const &startDepot = data.depot(vehicleType.startDepot);
        DurationSegment const vehStart(vehicleType, vehicleType.startLate);
        DurationSegment const depotStart(startDepot, startDepot.serviceDuration);
        durAt[0] = DurationSegment::merge(vehStart, depotStart);

        // End depot (node n-1)
        auto const &endDepot = data.depot(vehicleType.endDepot);
        DurationSegment const depotEnd(endDepot, 0);
        DurationSegment const vehEnd(vehicleType, vehicleType.twLate);
        durAt[n - 1] = DurationSegment::merge(depotEnd, vehEnd);

        // Interior nodes
        for (size_t idx = 1; idx != n - 1; ++idx)
        {
            auto const &act = activities[idx];

            if (act.isCustomBreak())
            {
                auto const breakId = act.idx();
                Duration svc(0);
                Duration early = 0;
                Duration late = std::numeric_limits<Duration>::max();
                for (auto const &brk : vehicleType.custom_breaks)
                    if (brk.id == static_cast<size_t>(breakId))
                    {
                        svc = brk.service;
                        if (!brk.tws.empty())
                        {
                            if (brk.twsRelative)
                            {
                                // Anchor the relative offsets to the
                                // search-clock baseline (vehicle.twEarly).
                                // The search clock tracks progress from
                                // vehicle.twEarly, and startTime_ -
                                // vehicle.twEarly is absorbed by the forward
                                // pass, so offsets relative to route start
                                // map to [twEarly+early, twEarly+late] in
                                // search-clock units.
                                early = brk.tws.front().first
                                        + vehicleType.twEarly;
                                late = brk.tws.back().second
                                       + vehicleType.twEarly;
                            }
                            else
                            {
                                // Absolute (clock-time) windows: enforced
                                // directly. Arrival after the window close
                                // makes the break infeasible (time warp)
                                // when it is due.
                                early = brk.tws.front().first;
                                late = brk.tws.back().second;
                            }
                        }
                        break;
                    }
                breakNodeAt[static_cast<size_t>(breakId)] = idx;
                durAt[idx] = DurationSegment(svc, Duration(0), early, late);
            }
            else if (act.isDepot())
                durAt[idx] = {data.depot(act.idx()), 0};
            else
                durAt[idx] = {data.client(act.idx())};
        }
    }
    else
    {
        // The singletons are given; only the break-node positions (used by
        // the lateness terms below) still need collecting.
        for (size_t idx = 1; idx != n - 1; ++idx)
            if (activities[idx].isCustomBreak())
                breakNodeAt[static_cast<size_t>(activities[idx].idx())] = idx;
    }

    // ---- Step 2: Forward pass for DurationSegment + atSecond ----
    // Encapsulated in a lambda so it can be re-run after the D5 rest
    // extension (which lengthens a break's service) without duplicating the
    // location-aware setup/clamping logic.

    // Use caller-provided buffer when available, else allocate locally.
    auto &durLocal = scratch.durLocal;
    std::vector<DurationSegment> &durBefore
        = durPrefixOut ? *durPrefixOut : durLocal;

    if (!durPrefixOut)
        durLocal.assign(n, DurationSegment());

    auto runDurationPass = [&]()
    {
        durBefore[0] = durAt[0];
        atSecond[0] = durBefore[0].duration() - durBefore[0].timeWarp();

        for (size_t idx = 1; idx != n; ++idx)
        {
            auto const prev = idx - 1;
            bool const prevIsReloadDepot
                = activities[prev].isDepot() && prev > 0 && prev < n - 1;

            auto before = prevIsReloadDepot ? durBefore[prev].finaliseBack()
                                            : durBefore[prev];

            if (prevIsReloadDepot)
            {
                auto const &depot = data.depot(activities[prev].idx());
                before = DurationSegment::merge(before,
                                                {depot.serviceDuration});
            }

            auto const edgeDur = durMatrix(locations[prev], locations[idx]);

            // Setup: charged when entering a client whose location differs
            // from the previous node's location (VROOM canonical rule).
            Duration setup = 0;
            if (activities[idx].isClient()
                && locations[idx] != locations[prev])
                setup = data.setupDuration(locations[idx]);

            auto second
                = setup == 0 ? durAt[idx] : durAt[idx].withService(setup);
            durBefore[idx] = DurationSegment::merge(edgeDur, before, second);

            // atSecond: conservative arrival time at this node.
            // The duration fold carries travel+service in duration() and the
            // absorbed waiting as a startEarly() offset (diffWait preserves
            // duration_+startEarly_ invariant); timeWarp is a late-side
            // penalty that this fold does NOT include in the clock, so it
            // must NOT be subtracted here (D5: non-monotonic clock otherwise
            // collapses endClock below firstDue, zeroing the lateness).
            // The clock is ABSOLUTE (midnight-anchored), coherent with the
            // absolute nodeEarly clamps below: startEarly() is already the
            // complete absolute offset, so it is added directly (no anchor
            // subtraction). Normalisation to an anchor clock, where required,
            // is the caller's responsibility — not done here.
            Duration const earlyArrival
                = durBefore[prev].duration() + durBefore[prev].startEarly()
                  + edgeDur + setup;

            Duration nodeEarly = 0;
            if (activities[idx].isClient())
                nodeEarly = data.client(activities[idx].idx()).twEarly;
            else if (activities[idx].isDepot())
                nodeEarly = data.depot(activities[idx].idx()).twEarly;
            else if (activities[idx].isCustomBreak())
                // Clamp to the break's window startEarly, mirroring the
                // Proposal gate behaviour.
                nodeEarly = durAt[idx].startEarly();

            atSecond[idx] = std::max(earlyArrival, nodeEarly);
        }
    };

    // The caller's fold is this pass's first round when ``prefixReady``.
    if (!prefixReady)
        runDurationPass();

    // ---- Step 3: Build driveAt singletons ----

    auto &driveAt = scratch.driveAt;
    driveAt.assign(n, DriveSegment{});
    driveAt[0] = DriveSegment::fromDepot();
    // Initialize lastResetAt_ to the effective route start time so
    // DUTY_TIME does not count midnight-to-departure waiting.
    {
        auto const effectiveStart = std::max(
            atSecond[0],
            atSecond[1] - durMatrix(locations[0], locations[1]));
        driveAt[0].lastResetAt_ = effectiveStart.get();
    }
    driveAt[n - 1] = DriveSegment::fromDepot();

    for (size_t idx = 1; idx != n - 1; ++idx)
    {
        auto const &act = activities[idx];

        if (act.isCustomBreak())
        {
            auto const breakId = act.idx();
            driveAt[idx] = DriveSegment(
                0, 0, 0,
                static_cast<uint16_t>(1u) << (breakId & 0xF), 0);
        }
        else if (act.isDepot())
            driveAt[idx] = DriveSegment::fromDepot();
        else
            driveAt[idx]
                = DriveSegment::fromClient(data.client(act.idx()).serviceDuration);
    }

    // ---- Step 4: Forward pass for DriveSegment ----

    auto &driveBefore = scratch.driveBefore;
    driveBefore.assign(n, DriveSegment{});
    driveBefore[0] = driveAt[0];
    if (drivePrefixOut)
        drivePrefixOut->at(0) = driveAt[0];

    // Accumulators for the per-id due mask and the D5 rest extension.
    uint16_t dueMask = 0;
    Duration absorbedWaiting = 0;
    bool clearedWindows = false;
    if (extendedBreakServices)
        std::fill(extendedBreakServices->begin(), extendedBreakServices->end(),
                  Duration(0));

    // upcomingBreakMaskAt[idx]: bitmask of breaks whose CUSTOM_BREAK node
    // lies at a position strictly AFTER idx in this flat sequence. Their
    // triggers must not fire at this boundary — the break is still scheduled
    // ahead (the gate at its own node decides service; violations are only
    // incurred at boundaries subsequent to the break's position).
    auto &upcomingBreakMaskAt = scratch.upcomingBreakMaskAt;
    upcomingBreakMaskAt.assign(n, 0);
    {
        uint16_t run = 0;
        for (size_t idx = n; idx-- > 0;)
        {
            upcomingBreakMaskAt[idx] = run;
            if (activities[idx].isCustomBreak())
                run |= static_cast<uint16_t>(1u)
                       << (activities[idx].idx() & 0xF);
        }
    }

    auto runDrivePass = [&]()
    {
        std::fill(firstDueClock.begin(), firstDueClock.end(), -1);
        std::fill(firstDuePos.begin(), firstDuePos.end(),
                  std::numeric_limits<size_t>::max());
        // The due mask must NOT accumulate between the two passes of the D5
        // re-run: a break that is due on pass 1 but no longer due on pass 2
        // would otherwise keep its bit, making isFeasible() false even though
        // breakDue() == 0.
        dueMask = 0;
        for (size_t idx = 1; idx != n; ++idx)
        {
            auto const prev = idx - 1;
            auto const edgeDur = durMatrix(locations[prev], locations[idx]);

            // Setup is work, never drive: pass it as extraWork so it enters
            // the work/duty accumulators but not driveTime_.
            Duration setup = 0;
            if (activities[idx].isClient()
                && locations[idx] != locations[prev])
                setup = data.setupDuration(locations[idx]);

            auto drs = DriveSegment::merge(edgeDur,
                                           driveBefore[prev],
                                           driveAt[idx],
                                           vehicleType.breakRules,
                                           atSecond[idx],
                                           upcomingBreakMaskAt[idx],
                                           setup,
                                           &dueMask,
                                           firstDueClock.data());

        // reset_breaks_at_reload: arrival at a reload depot resets
        // accumulators (but preserves mask and breakDue).
        bool const curIsReloadDepot
            = activities[idx].isDepot() && idx > 0 && idx < n - 1;
        if (resetAtReload && curIsReloadDepot)
            drs = {0, 0, 0, drs.breaksTakenMask_, drs.lastResetAt_};

        // CUSTOM_BREAK: the break activity is visited at idx. The break is
        // only served (reset applied, bit kept) when the cumulative metric
        // has reached its trigger value (isBreakEligible). Otherwise the
        // reset is skipped and the optimistic taken-bit is removed from the
        // mask so the trigger can fire (and breakDue be accounted) at
        // subsequent route boundaries.
        if (activities[idx].isCustomBreak())
        {
            auto const breakId = activities[idx].idx();
            for (auto const &brk : breaks)
            {
                if (brk.id == static_cast<size_t>(breakId))
                {
                    // Eligibility frozen on the first pass (D3/N1): the
                    // second pass (final clock) must not flip a
                    // served/not-served decision — the reset and extension
                    // were already decided on the first pass.
                    bool const eligible
                        = firstDrivePass ? isBreakEligible(drs, brk)
                                         : breakEligibleAt[idx];
                    breakEligibleAt[idx] = eligible;

                    // Window-close gate (honest contract): a DUE break whose
                    // arrival falls after its window close is not servable —
                    // drop it (no reset, no served bit). The close is still
                    // enforced (time warp), so the route is infeasible rather
                    // than silently served late.
                    bool const pastClose
                        = firstDrivePass
                              ? isBreakPastWindowClose(brk,
                                                      atSecond[idx],
                                                      vehicleType.twEarly)
                              : breakPastCloseAt[idx];
                    breakPastCloseAt[idx] = pastClose;

                    bool const servable = eligible && !pastClose;
                    if (servable)
                    {
                        // Served: mark for the lateness formula (D3).
                        servedMask |= static_cast<uint16_t>(1u)
                                      << (breakId & 0xF);

                        // D5: extend the served rest to absorb the waiting
                        // before the next client's window opens. This applies
                        // to ANY served DUTY_TIME break followed by a client
                        // whose window is still closed (not only overnight
                        // rests). Only when the next activity is a client is
                        // the window-open anchor well defined; a following
                        // break or (reload) depot does not extend.
                        bool const extend
                            = brk.trigger == CustomBreakTrigger::DUTY_TIME
                              && idx + 1 < n
                              && activities[idx + 1].isClient();
                        Duration travel = 0;
                        Duration nextOpen = 0;
                        if (extend)
                        {
                            travel = durMatrix(locations[idx],
                                               locations[idx + 1]);
                            // nextOpen is the next client's absolute
                            // (midnight-anchored) twEarly, matching the
                            // absolute atSecond clock — no anchor subtraction.
                            nextOpen
                                = data.client(activities[idx + 1].idx()).twEarly;
                        }

                        // Effective service: idempotent across passes — the
                        // first pass decides and stores the extension; the
                        // second pass (final clock) reuses it so the durAt
                        // mutation and absorbedWaiting are never applied twice.
                        Duration effSvc;
                        if (extendedBreakServices
                            && (*extendedBreakServices)[idx] != 0)
                            effSvc = (*extendedBreakServices)[idx];
                        else
                        {
                            effSvc = breakEffectiveService(brk.service,
                                                           atSecond[idx],
                                                           extend,
                                                           travel,
                                                           nextOpen);
                            absorbedWaiting += effSvc - brk.service;
                            if (extendedBreakServices)
                                (*extendedBreakServices)[idx] = effSvc;
                            if (effSvc != brk.service)
                                // Lengthen the break's service in the
                                // duration fold too, so the absorbed waiting
                                // is not counted as idle (D5/D6). Duration
                                // unchanged.
                                durAt[idx] = durAt[idx].withService(
                                    effSvc - brk.service);
                        }

                        switch (brk.reset)
                        {
                        case CustomBreakReset::ALL_TIMERS:
                            drs.driveTime_ = 0;
                            drs.workTime_ = 0;
                            drs.dutyTime_ = 0;
                            drs.lastResetAt_ = atSecond[idx].get()
                                               + effSvc.get();
                            break;
                        case CustomBreakReset::DRIVE_AND_WORK:
                            drs.driveTime_ = 0;
                            drs.workTime_ = 0;
                            break;
                        case CustomBreakReset::DRIVE_TIMER:
                            drs.driveTime_ = 0;
                            break;
                        case CustomBreakReset::WORK_TIMER:
                            drs.workTime_ = 0;
                            break;
                        case CustomBreakReset::NONE:
                            break;
                        }
                        drs.breaksTakenMask_
                            |= static_cast<uint16_t>(1u)
                               << (breakId & 0xF);
                    }
                    else
                    {
                        // Not servable (either not yet due, or due-but-past-
                        // close): no reset, drop the optimistic bit so the
                        // trigger can fire at later boundaries. The mask is
                        // 16-bit (max 16 distinct break ids per vehicle,
                        // pre-existing limitation).
                        drs.breaksTakenMask_
                            &= ~(static_cast<uint16_t>(1u)
                                 << (breakId & 0xF));

                        // Due-ness gate for absolute break windows: the
                        // window close is only enforced when the break is due
                        // at this node. A non-due break's CUSTOM_BREAK node
                        // must not warp the route just because its (closed)
                        // absolute window has already passed. Clear the window
                        // close (startLate -> unbounded) so the duration fold
                        // treats the break as window-less. The window open
                        // (startEarly) is intentionally kept: it only clamps
                        // the arrival and is unaffected by due-ness.
                        //
                        // A due-but-past-close break KEEPS its close: it must
                        // warp (infeasible) rather than be silently served
                        // late. The gate below is therefore gated on the break
                        // not being due (``!eligible``).
                        if (!eligible && !brk.twsRelative && !brk.tws.empty())
                        {
                            durAt[idx] = DurationSegment(
                                durAt[idx].duration(),
                                Duration(0),
                                durAt[idx].startEarly(),
                                std::numeric_limits<Duration>::max());
                            clearedWindows = true;
                        }
                    }
                    break;
                }
            }
        }

            driveBefore[idx] = drs;

            // Record the per-position final drive state (seed cache) and the
            // boundary where each break's pure trigger first fired on this
            // (final) pass.
            if (drivePrefixOut)
                drivePrefixOut->at(idx) = drs;
            if (firstDuePosOut)
                for (auto const &brk : breaks)
                {
                    auto const id = static_cast<size_t>(brk.id);
                    auto const npos = std::numeric_limits<size_t>::max();
                    if (firstDueClock[id] >= 0 && firstDuePos[id] == npos)
                        firstDuePos[id] = idx;
                }
        }
        firstDrivePass = false;
    };

    runDrivePass();

    // Both the D5 rest extension and the due-ness gate mutate durAt *after*
    // the first duration pass (Step 2) has already produced durBefore and
    // atSecond. Re-run BOTH passes so the waiting/timeWarp measures AND the
    // firstDueClock/lastResetAt_ bookkeeping reflect the mutated break
    // windows/services on the FINAL clock (D3/N1 — never mix pre/post re-run
    // clocks in the lateness terms):
    //   - absorbedWaiting > 0: a served overnight rest was extended (the fold
    //     must use the lengthened break service instead of the minimum).
    //   - clearedWindows > 0: a non-due break's absolute window close was
    //     cleared, so the fold must no longer warp on that (closed) window.
    if (absorbedWaiting > 0 || clearedWindows)
    {
        runDurationPass();
        runDrivePass();
    }

    // D3: per-mandatory-break lateness in SECONDS, all terms on the final
    // clock:
    //   served late:       max(0, actualStart − firstDue)
    //   not served (due):  max(service, endClock − firstDue) — skipping is
    //                      never cheaper than serving.
    auto const endClock = atSecond[n - 1].get();
#ifdef PYVRP_STREAM_STATS
    {
        static bool const trace = []
        {
            auto const *env = std::getenv("PYVRP_FWD_TRACE");
            return env && *env && !(env[0] == '0' && env[1] == '\0');
        }();
        if (trace)
        {
            std::fprintf(stderr, "[fwd-trace] n=%zu endClock=%lld clockTrigger=%d\n",
                         n, (long long)endClock, int(clockTrigger));
            for (size_t idx = 0; idx != n; ++idx)
            {
                auto const &a = activities[idx];
                auto const &d = driveBefore[idx];
                std::fprintf(stderr,
                             "[fwd-trace]  idx=%2zu %c%-5zu atSecond=%8lld drive=%7lld work=%7lld duty=%7lld L=%8lld mask=%u elig=%d\n",
                             idx, a.isCustomBreak() ? 'B' : a.isDepot() ? 'D' : 'C', a.idx(),
                             (long long)atSecond[idx].get(), (long long)d.driveTime_,
                             (long long)d.workTime_, (long long)d.dutyTime_,
                             (long long)d.lastResetAt_, unsigned(d.breaksTakenMask_),
                             int(breakEligibleAt[idx]));
            }
            for (auto const &brk : breaks)
            {
                auto const id = static_cast<size_t>(brk.id);
                std::fprintf(stderr, "[fwd-trace]  rule id=%zu T=%lld minRoute=%lld mandatory=%d firstDue=%lld firstDuePos=%lld served=%d\n",
                             id, (long long)brk.triggerValue.get(), (long long)brk.conditionMinRouteS.get(),
                             int(brk.mandatory), (long long)firstDueClock[id],
                             firstDuePos[id] == std::numeric_limits<size_t>::max() ? -1LL : (long long)firstDuePos[id],
                             int((servedMask >> (id & 0xF)) & 1u));
            }
        }
    }
#endif
    int64_t breakDueSeconds = 0;
    for (auto const &brk : breaks)
    {
        if (!brk.mandatory)
            continue;
        auto const id = static_cast<size_t>(brk.id);
        if (id >= firstDueClock.size() || firstDueClock[id] < 0)
            continue;  // never due
        auto const bit = static_cast<uint16_t>(1u) << (brk.id & 0xF);
        if (servedMask & bit)
        {
            auto const nodeIdx = breakNodeAt[id];
            if (nodeIdx != std::numeric_limits<size_t>::max())
                breakDueSeconds += std::max<int64_t>(
                    0, atSecond[nodeIdx].get() - firstDueClock[id]);
        }
        else
        {
            breakDueSeconds += std::max<int64_t>(
                brk.service.get(), endClock - firstDueClock[id]);
        }
    }

    // Waiting is the portion of the duration that is neither travel, service,
    // setup, nor breaks. Waiting absorbed into an overnight rest extension is
    // excluded by construction (the duration fold uses the extended break
    // service).
    auto const waiting = durBefore.back().waiting();

    // Write-back of the FINAL per-node DurationSegment singletons (D5 parity):
    // the D5 rest extension and the due-ness gate mutate ``durAt`` during the
    // drive pass (extended break service, cleared window closes). Callers that
    // cache these singletons for their own segment folds (e.g. the search
    // route's ``durAt``, read by SegmentBetween) MUST receive the mutated
    // values — otherwise their atSecond/duty chains diverge from this shared
    // evaluator and accepted moves carry stale deltas (oscillation, D5).
    if (durAtOut && !prefixReady)  // with prefixReady, durAt IS *durAtOut
        *durAtOut = durAt;

    // Final-round per-id first-due clock, the boundary where each first fired,
    // and the served mask (Alternativa D seed cache).
    if (firstDueOut)
        std::copy(firstDueClock.begin(), firstDueClock.end(), firstDueOut);
    if (firstDuePosOut)
        std::copy(firstDuePos.begin(), firstDuePos.end(), firstDuePosOut);
    if (servedMaskOut)
        *servedMaskOut = servedMask;

#ifndef NDEBUG
    // Parity: the fold's waiting must be a non-negative part of the total
    // duration (waiting = duration - travel - service - setup - breaks).
    assert(waiting.get() >= 0);
    assert(waiting.get() <= durBefore.back().duration().get());
#endif

    return {durBefore.back().duration(),
            durBefore.back().timeWarp(vehicleType.maxDuration),
            breakDueSeconds,
            dueMask,
            waiting};
}
