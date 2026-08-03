#include "DriveSegment.h"

#include <cassert>

using namespace pyvrp;
using namespace pyvrp::search;

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
                           uint16_t breakDue)
    : driveTime_(driveTime),
      workTime_(workTime),
      dutyTime_(dutyTime),
      breaksTakenMask_(breaksTakenMask),
      breakDue_(breakDue)
{
}

DriveSegment DriveSegment::merge(Duration const edgeDur,
                                  DriveSegment const &first,
                                  DriveSegment const &second,
                                  std::vector<pyvrp::CustomBreak> const &breaks,
                                  Duration const atSecond)
{
    using pyvrp::CustomBreakReset;
    using pyvrp::CustomBreakTrigger;

    auto const edge = static_cast<int64_t>(edgeDur.get());
    auto const atSecondVal = static_cast<int64_t>(atSecond.get());

    int64_t drive = first.driveTime_ + edge + second.driveTime_;
    int64_t work = first.workTime_ + edge + second.workTime_;
    int64_t duty = first.dutyTime_ + edge + second.dutyTime_;
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

        // Supersedes: skip if any superseded break was already taken.
        bool superseded = false;
        for (auto sid : brk.supersedes)
        {
            if (takenMask
                & (static_cast<uint16_t>(1u) << (sid & 0xF)))
            {
                superseded = true;
                break;
            }
        }
        if (superseded)
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
                breakDue += 1;

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

    return {drive, work, duty, takenMask, breakDue};
}

ForwardEvalResult
pyvrp::search::evaluateForwardPass(std::vector<Activity> const &activities,
                                   std::vector<size_t> const &locations,
                                   std::vector<Duration> &atSecond,
                                   std::vector<DurationSegment> *durPrefixOut,
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
            auto const breakIdx = act.idx();
            Duration svc(0);
            if (breakIdx < vehicleType.custom_breaks.size())
                svc = vehicleType.custom_breaks[breakIdx].service;
            durAt[idx] = DurationSegment(svc, Duration(0));
        }
        else if (act.isDepot())
            durAt[idx] = {data.depot(act.idx()), 0};
        else
            durAt[idx] = {data.client(act.idx())};
    }

    // ---- Step 2: Forward pass for DurationSegment + atSecond ----

    // Use caller-provided buffer when available, else allocate locally.
    std::vector<DurationSegment> durLocal;
    std::vector<DurationSegment> &durBefore
        = durPrefixOut ? *durPrefixOut : durLocal;

    if (!durPrefixOut)
        durLocal.resize(n);

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
            before = DurationSegment::merge(before, {depot.serviceDuration});
        }

        auto const edgeDur = durMatrix(locations[prev], locations[idx]);
        durBefore[idx] = DurationSegment::merge(edgeDur, before, durAt[idx]);

        // atSecond: conservative arrival time at this node.
        Duration const earlyArrival
            = durBefore[prev].duration() - durBefore[prev].timeWarp()
              + edgeDur;

        Duration nodeEarly = 0;
        if (activities[idx].isClient())
            nodeEarly = data.client(activities[idx].idx()).twEarly;
        else if (activities[idx].isDepot())
            nodeEarly = data.depot(activities[idx].idx()).twEarly;
        // CUSTOM_BREAK: nodeEarly stays 0 (break's tw_early is already
        // reflected in durAt startEarly).

        atSecond[idx] = std::max(earlyArrival, nodeEarly);
    }

    // ---- Step 3: Build driveAt singletons ----

    std::vector<DriveSegment> driveAt(n);
    driveAt[0] = DriveSegment::fromDepot();
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

    std::vector<DriveSegment> driveBefore(n);
    driveBefore[0] = driveAt[0];

    for (size_t idx = 1; idx != n; ++idx)
    {
        auto const prev = idx - 1;
        auto const edgeDur = durMatrix(locations[prev], locations[idx]);

        auto drs = DriveSegment::merge(edgeDur,
                                       driveBefore[prev],
                                       driveAt[idx],
                                       breaks,
                                       atSecond[idx]);

        // reset_breaks_at_reload: arrival at a reload depot resets
        // accumulators (but preserves mask and breakDue).
        bool const curIsReloadDepot
            = activities[idx].isDepot() && idx > 0 && idx < n - 1;
        if (resetAtReload && curIsReloadDepot)
            drs = {0, 0, 0, drs.breaksTakenMask_, drs.breakDue_};

        // CUSTOM_BREAK: the break activity is visited at idx. Apply the
        // reset defined by this break's config.
        if (activities[idx].isCustomBreak())
        {
            auto const breakId = activities[idx].idx();
            for (auto const &brk : breaks)
            {
                if (brk.id == static_cast<size_t>(breakId))
                {
                    switch (brk.reset)
                    {
                    case CustomBreakReset::ALL_TIMERS:
                        drs.driveTime_ = 0;
                        drs.workTime_ = 0;
                        drs.dutyTime_ = 0;
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
                        |= static_cast<uint16_t>(1u) << (breakId & 0xF);
                    break;
                }
            }
        }

        driveBefore[idx] = drs;
    }

    return {durBefore.back().duration(),
            durBefore.back().timeWarp(vehicleType.maxDuration),
            driveBefore.back().breakDue_};
}
