#include "DriveSegment.h"

#include <algorithm>
#include <cassert>

using namespace pyvrp;
using namespace pyvrp::search;

DriveSegment DriveSegment::fromClient(Duration service)
{
    auto const s = static_cast<int64_t>(service.get());
    return {0, s, s, 0, 0, 0};
}

DriveSegment DriveSegment::fromDepot() { return {}; }

DriveSegment DriveSegment::fromVehicleType() { return {}; }

DriveSegment::DriveSegment(int64_t driveTime,
                           int64_t workTime,
                           int64_t dutyTime,
                           uint16_t breaksTakenMask,
                           uint16_t breakDue,
                           int64_t lastResetAt)
    : driveTime_(driveTime),
      workTime_(workTime),
      dutyTime_(dutyTime),
      lastResetAt_(lastResetAt),
      breaksTakenMask_(breaksTakenMask),
      breakDue_(breakDue)
{
}

DriveSegment DriveSegment::merge(Duration const edgeDur,
                                  DriveSegment const &first,
                                  DriveSegment const &second,
                                  std::vector<pyvrp::CustomBreak> const &breaks,
                                  Duration const atSecond,
                                  uint16_t const upcomingMask,
                                  Duration const extraWork,
                                  uint16_t *const breakDueMask)
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
    uint16_t breakDue = first.breakDue_ + second.breakDue_;

    // Evaluate each configured break in priority order (pre-sorted by caller).
    for (auto const &brk : breaks)
    {
        // Condition: skip if route duration is below this break's minimum.
        auto const minRoute = static_cast<int64_t>(brk.conditionMinRouteS.get());
        if (minRoute > 0 && duty < minRoute)
            continue;

        // Skip if this specific break ID was already taken in either segment.
        if (takenMask & (static_cast<uint16_t>(1u) << (brk.id & 0xF)))
            continue;

        // Upcoming: this break's CUSTOM_BREAK node lies at a route position
        // strictly after the current boundary. It is still scheduled ahead,
        // so its trigger must NOT fire here: the violation (breakDue) is only
        // incurred at boundaries subsequent to the break's own position. The
        // gate at the break node decides service/eligibility; if the break is
        // positioned too early, the bit is removed there and the trigger
        // fires at the following boundaries instead.
        if (upcomingMask & (static_cast<uint16_t>(1u) << (brk.id & 0xF)))
            continue;

        bool triggered = false;
        auto const triggerVal = static_cast<int64_t>(brk.triggerValue.get());

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
            // end AND this specific break has not been taken yet.
            triggered = !brk.tws.empty()
                        && atSecondVal
                               > static_cast<int64_t>(
                                   brk.tws.back().second.get())
                        && !(takenMask
                             & (static_cast<uint16_t>(1u)
                                << (brk.id & 0xF)));
            break;
        }

        if (triggered)
        {
            if (brk.mandatory)
            {
                breakDue += 1;
                if (breakDueMask)
                    *breakDueMask |= static_cast<uint16_t>(1u)
                                     << (brk.id & 0xF);
            }

            // Mark this break as taken.
            takenMask |= static_cast<uint16_t>(1u) << (brk.id & 0xF);
            for (auto sid : brk.supersedes)
                takenMask |= static_cast<uint16_t>(1u) << (sid & 0xF);

            // Apply the reset to accumulators.
            switch (brk.reset)
            {
            case CustomBreakReset::ALL_TIMERS:
                drive = second.driveTime_;
                work = second.workTime_;
                duty = second.dutyTime_;
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

    return {drive, work, duty, takenMask, breakDue, lastResetAt};
}

ForwardEvalResult
pyvrp::search::evaluateForwardPass(std::vector<Activity> const &activities,
                                   std::vector<size_t> const &locations,
                                   std::vector<Duration> &atSecond,
                                   std::vector<DurationSegment> *durPrefixOut,
                                   std::vector<Duration> *extendedBreakServices,
                                   ProblemData const &data,
                                   VehicleType const &vehicleType)
{
    auto const n = activities.size();
    assert(n >= 2);

    auto const &breaks = vehicleType.custom_breaks;
    auto const resetAtReload = vehicleType.reset_breaks_at_reload;
    auto const profile = vehicleType.profile;
    auto const &durMatrix = data.durationMatrix(profile);

    // ---- Step 1: Build durAt singletons (per-node DurationSegment) ----

    std::vector<DurationSegment> durAt(n);

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
                            // Anchor the relative offsets to the search-clock
                            // baseline (vehicle.twEarly). The search clock
                            // tracks progress from vehicle.twEarly, and
                            // startTime_ - vehicle.twEarly is absorbed by the
                            // forward pass, so offsets relative to route start
                            // map to [twEarly+early, twEarly+late] in
                            // search-clock units.
                            early = brk.tws.front().first
                                    + vehicleType.twEarly;
                            late = brk.tws.back().second
                                   + vehicleType.twEarly;
                        }
                        else
                        {
                            // Absolute (clock-time) windows: enforced directly.
                            // Arrival after the window close makes the break
                            // infeasible (time warp) when it is due.
                            early = brk.tws.front().first;
                            late = brk.tws.back().second;
                        }
                    }
                    break;
                }
            durAt[idx] = DurationSegment(svc, Duration(0), early, late);
        }
        else if (act.isDepot())
            durAt[idx] = {data.depot(act.idx()), 0};
        else
            durAt[idx] = {data.client(act.idx())};
    }

    // ---- Step 2: Forward pass for DurationSegment + atSecond ----
    // Encapsulated in a lambda so it can be re-run after the D5 rest
    // extension (which lengthens a break's service) without duplicating the
    // location-aware setup/clamping logic.

    // Use caller-provided buffer when available, else allocate locally.
    std::vector<DurationSegment> durLocal;
    std::vector<DurationSegment> &durBefore
        = durPrefixOut ? *durPrefixOut : durLocal;

    if (!durPrefixOut)
        durLocal.resize(n);

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
            Duration const earlyArrival
                = durBefore[prev].duration() - durBefore[prev].timeWarp()
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

    runDurationPass();

    // ---- Step 3: Build driveAt singletons ----

    std::vector<DriveSegment> driveAt(n);
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
                static_cast<uint16_t>(1u) << (breakId & 0xF), 0, 0);
        }
        else if (act.isDepot())
            driveAt[idx] = DriveSegment::fromDepot();
        else
            driveAt[idx]
                = DriveSegment::fromClient(data.client(act.idx()).serviceDuration);
    }

    // ---- Step 4: Forward pass for DriveSegment ----

    std::vector<DriveSegment> driveBefore(n);
    driveBefore[0] = driveAt[0];

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
    std::vector<uint16_t> upcomingBreakMaskAt(n, 0);
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

    for (size_t idx = 1; idx != n; ++idx)
    {
        auto const prev = idx - 1;
        auto const edgeDur = durMatrix(locations[prev], locations[idx]);

        // Setup is work, never drive: pass it as extraWork so it enters the
        // work/duty accumulators but not driveTime_.
        Duration setup = 0;
        if (activities[idx].isClient() && locations[idx] != locations[prev])
            setup = data.setupDuration(locations[idx]);

        auto drs = DriveSegment::merge(edgeDur,
                                       driveBefore[prev],
                                       driveAt[idx],
                                       breaks,
                                       atSecond[idx],
                                       upcomingBreakMaskAt[idx],
                                       setup,
                                       &dueMask);

        // reset_breaks_at_reload: arrival at a reload depot resets
        // accumulators (but preserves mask and breakDue).
        bool const curIsReloadDepot
            = activities[idx].isDepot() && idx > 0 && idx < n - 1;
        if (resetAtReload && curIsReloadDepot)
            drs = {0, 0, 0, drs.breaksTakenMask_, drs.breakDue_, drs.lastResetAt_};

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
                    if (isBreakEligible(drs, brk))
                    {
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
                            nextOpen
                                = data.client(activities[idx + 1].idx()).twEarly;
                        }

                        auto const effSvc
                            = breakEffectiveService(brk.service,
                                                    atSecond[idx],
                                                    extend,
                                                    travel,
                                                    nextOpen);
                        absorbedWaiting += effSvc - brk.service;
                        if (extendedBreakServices)
                            (*extendedBreakServices)[idx] = effSvc;
                        if (effSvc != brk.service)
                            // Lengthen the break's service in the duration
                            // fold too, so the absorbed waiting is not counted
                            // as idle (D5/D6). Duration is unchanged.
                            durAt[idx] = durAt[idx].withService(effSvc
                                                                - brk.service);

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
                        // Not eligible: no reset, drop the optimistic bit so
                        // the trigger can fire at later boundaries. The mask
                        // is 16-bit (max 16 distinct break ids per vehicle,
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
                        // Clearing the close is a no-op when the window is
                        // still open (no warp would be incurred anyway), so no
                        // explicit "already passed" check is needed: the only
                        // observable effect is the removal of the warp that a
                        // closed window would otherwise impose.
                        if (!brk.twsRelative && !brk.tws.empty())
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
    }

    // Both the D5 rest extension and the due-ness gate mutate durAt *after*
    // the first duration pass (Step 2) has already produced durBefore and
    // atSecond. Re-run the duration pass so the waiting and timeWarp measures
    // reflect the mutated break windows/services:
    //   - absorbedWaiting > 0: a served overnight rest was extended (the fold
    //     must use the lengthened break service instead of the minimum).
    //   - clearedWindows > 0: a non-due break's absolute window close was
    //     cleared, so the fold must no longer warp on that (closed) window.
    if (absorbedWaiting > 0 || clearedWindows)
        runDurationPass();

    // Waiting is the portion of the duration that is neither travel, service,
    // setup, nor breaks. Waiting absorbed into an overnight rest extension is
    // excluded by construction (the duration fold uses the extended break
    // service).
    auto const waiting = durBefore.back().waiting();

#ifndef NDEBUG
    // Parity: the fold's waiting must be a non-negative part of the total
    // duration (waiting = duration - travel - service - setup - breaks).
    assert(waiting.get() >= 0);
    assert(waiting.get() <= durBefore.back().duration().get());
#endif

    return {durBefore.back().duration(),
            durBefore.back().timeWarp(vehicleType.maxDuration),
            driveBefore.back().breakDue_,
            dueMask,
            waiting};
}
