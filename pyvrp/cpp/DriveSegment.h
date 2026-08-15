#ifndef PYVRP_DRIVESEGMENT_H
#define PYVRP_DRIVESEGMENT_H

#include "Activity.h"
#include "CustomBreak.h"
#include "DurationSegment.h"
#include "Measure.h"
#include "ProblemData.h"

#include <cstdint>
#include <vector>

namespace pyvrp::search
{
/**
 * DriveSegment
 *
 * A compact 40-byte struct that tracks driving, working, and duty time
 * accumulators across a route segment, along with a bitmask of breaks taken
 * and a count of mandatory break violations (breakDue).
 *
 * DriveSegment is stored in separate parallel arrays within search::Route and
 * is only merged when VehicleType::hasBreaks() returns true.
 *
 * Fields
 * ------
 * driveTime_
 *     Accumulated driving time in the segment.
 * workTime_
 *     Accumulated working time (driving + service) in the segment.
 * dutyTime_
 *     Clock time since last ALL_TIMERS reset, including waiting (CLT art. 4º).
 * lastResetAt_
 *     Absolute route-clock time (atSecond) at the last ALL_TIMERS reset,
 *     used to compute waiting between that reset point and the current
 *     boundary so that dutyTime_ accumulates waiting time as well.
 * breaksTakenMask_
 *     Bitmask of break IDs already taken (max 16 breaks, bit per id).
 *     The 16-bit width limits distinct break IDs per vehicle to 16;
 *     IDs >= 16 are truncated via ``id & 0xF`` during merge to avoid
 *     out-of-bounds bit shifts.
 * breakDue_
 *     Number of mandatory breaks violated (not taken) in this segment.
 */
struct DriveSegment
{
    int64_t driveTime_ = 0;
    int64_t workTime_ = 0;
    int64_t dutyTime_ = 0;
    int64_t lastResetAt_ = 0;
    uint16_t breaksTakenMask_ = 0;
    uint16_t breakDue_ = 0;

    /**
     * Creates a DriveSegment from a client visit. The client contributes its
     * service duration to workTime and dutyTime, but not to driveTime.
     */
    [[nodiscard]] static DriveSegment fromClient(Duration service);

    /**
     * Creates a zero-initialized DriveSegment for a depot visit.
     */
    [[nodiscard]] static DriveSegment fromDepot();

    /**
     * Creates a zero-initialized DriveSegment for a vehicle type.
     */
    [[nodiscard]] static DriveSegment fromVehicleType();

    DriveSegment() = default;

    DriveSegment(int64_t driveTime,
                 int64_t workTime,
                 int64_t dutyTime,
                 uint16_t breaksTakenMask,
                 uint16_t breakDue,
                 int64_t lastResetAt = 0);

    /**
     * Merges two DriveSegments across an edge, evaluating all configured
     * breaks for trigger violations and applying resets accordingly.
     *
     * Parameters
     * ----------
     * edgeDur
     *     Duration of the edge connecting first and second.
     * first
     *     DriveSegment of the preceding part of the route.
     * second
     *     DriveSegment of the succeeding part of the route.
     * breaks
     *     Configured CustomBreak rules for this vehicle (pre-sorted by
     *     descending priority).
     * atSecond
     *     Exact scheduled arrival at the merge boundary (max of earliest
     *     arrival and destination tw_early), used for CLOCK_TIME
     *     interval-crossing detection.
     * upcomingMask
     *     Bitmask of break ids whose CUSTOM_BREAK node lies at a route
     *     position strictly AFTER this boundary. Those breaks are still
     *     scheduled ahead in the route: their trigger must not fire here
     *     (the violation is only incurred at boundaries subsequent to the
     *     break's own position, per the eligibility-gate semantics).
     * extraWork
     *     Extra work duration charged at this boundary (e.g. a setup duration
     *     when entering a client at a new location). Added to work and duty
     *     time, but never to drive time or ``lastResetAt_``.
     *
     * Returns
     * -------
     * DriveSegment
     *     The merged segment with accumulated times, updated breaksTakenMask
     *     (OR of both), and accumulated breakDue count.
     *
 * Notes
 * -----
 * condition_min_route_s is compared against accumulated duty time,
 * which is clock-based (includes waiting time) for ALL_TIMERS reset
 * breaks, ensuring CLT-correct behaviour.
     */
    [[nodiscard]] static DriveSegment
    merge(Duration edgeDur,
          DriveSegment const &first,
          DriveSegment const &second,
          std::vector<pyvrp::CustomBreak> const &breaks,
          Duration atSecond,
          uint16_t upcomingMask = 0,
          Duration extraWork = 0);

};

/**
 * Eligibility gate for serving a break at a route position.
 *
 * A cumulative-trigger break (DUTY_TIME, WORK_TIME, DRIVE_TIME) may only be
 * served when the corresponding accumulated metric at the break position is
 * at least the break's trigger_value. Until then the break must not be
 * served: no reset is applied, the break is not marked taken, and the
 * trigger may fire (accounting breakDue) at subsequent route boundaries.
 *
 * CLOCK_TIME breaks are always eligible: their windows already restrict
 * when they may occur, and the cumulative gate does not apply (the
 * pre-existing clock-window semantics are preserved).
 *
 * Note: the gate uses >= while DriveSegment::merge() triggers on strict
 * '>' — a break positioned exactly at the trigger value is served; the
 * violation only fires once the metric strictly exceeds it.
 */
inline bool isBreakEligible(DriveSegment const &drs,
                            pyvrp::CustomBreak const &brk)
{
    using pyvrp::CustomBreakTrigger;
    auto const triggerVal = static_cast<int64_t>(brk.triggerValue.get());

    // Mirror DriveSegment::merge(): the break rule is not "active" until the
    // route's accumulated duty reaches conditionMinRouteS. Below that the
    // merge skips the break entirely (no trigger, no violation), so the gate
    // must not serve it either — otherwise a break with triggerValue <
    // conditionMinRouteS could be served inside the inactive window.
    auto const minRoute = static_cast<int64_t>(brk.conditionMinRouteS.get());
    if (minRoute > 0 && drs.dutyTime_ < minRoute)
        return false;

    switch (brk.trigger)
    {
    case CustomBreakTrigger::DUTY_TIME:
        return drs.dutyTime_ >= triggerVal;
    case CustomBreakTrigger::WORK_TIME:
        return drs.workTime_ >= triggerVal;
    case CustomBreakTrigger::DRIVE_TIME:
        return drs.driveTime_ >= triggerVal;
    case CustomBreakTrigger::CLOCK_TIME:
        return true;
    }
    return true;
}

// Verify compact layout: 4×8B int64 + 2×2B uint16 + 4B padding = 40B.
// The two-per-cache-line packing is lost (acceptable trade-off for the
// additional lastResetAt_ field required for CLT-correct duty tracking).
static_assert(sizeof(DriveSegment) == 40,
              "DriveSegment must be exactly 40 bytes");

/**
 * Result of a shared forward-pass evaluation over a flat activity/location
 * sequence. Used by Route::update() and Proposal to compute breakDue with
 * literally the same forward-pass semantics, guaranteeing parity by
 * construction.
 */
struct ForwardEvalResult
{
    Duration duration;   // total route duration
    Duration timeWarp;   // total time warp (vs maxDuration)
    uint16_t breakDue;   // mandatory break violations
};

/**
 * Evaluates a flat route activity sequence using the exact same forward-pass
 * logic as Route::update(). Computes duration, timeWarp, and breakDue for the
 * given vehicle type and problem data.
 *
 * Parameters
 * ----------
 * activities
 *     Flat vector of activities in forward order (start depot first, end
 *     depot last). Must have at least two entries.
 * locations
 *     Flat vector of location indices, one per activity.
 * atSecond
 *     Output vector (pre-sized to n) filled with the conservative arrival
 *     time at each node, used for CLOCK_TIME interval-crossing detection.
 * durPrefixOut
 *     Optional output vector (pre-sized to n) for the DurationSegment prefix
 *     after each node. When nullptr, only computed internally.
 * data
 *     Problem data instance.
 * vehicleType
 *     Vehicle type for the route being evaluated.
 */
ForwardEvalResult evaluateForwardPass(
    std::vector<Activity> const &activities,
    std::vector<size_t> const &locations,
    std::vector<Duration> &atSecond,
    std::vector<DurationSegment> *durPrefixOut,
    ProblemData const &data,
    VehicleType const &vehicleType);

}  // namespace pyvrp::search

#endif  // PYVRP_DRIVESEGMENT_H
