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
 * A compact struct that tracks driving, working, and duty time accumulators
 * across a route segment, along with a bitmask of breaks taken. The mandatory
 * break-violation ACCOUNTING (breakDue, in SECONDS of lateness) lives in the
 * shared forward pass (``evaluateForwardPass``), not in this fold — the fold
 * only tracks the due MASK (which breaks are violated) for feasibility.
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
 */
struct DriveSegment
{
    int64_t driveTime_ = 0;
    int64_t workTime_ = 0;
    int64_t dutyTime_ = 0;
    int64_t lastResetAt_ = 0;
    uint16_t breaksTakenMask_ = 0;

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
     * breakDueMask
     *     Optional in/out bitmask of break ids that have been violated
     *     (due). When non-null, the caller accumulates the full due mask
     *     across the forward pass; ``merge()`` ORs in the bit of each
     *     mandatory break that triggers at this boundary.
     * firstDueClock
     *     Optional per-break-id array (sized max_id+1, pre-initialised to -1)
     *     recording the search-clock at the FIRST boundary where the break's
     *     pure trigger fired (D2: computed before the taken/upcoming gates, so
     *     a break whose node is still ahead — or already taken — still has its
     *     due moment captured). Used by the forward pass to price lateness in
     *     seconds.
     * twEarlyAnchor
     *     The vehicle's ``twEarly`` (route start offset). Used only by the
     *     CLOCK_TIME trigger for RELATIVE windows (``twsRelative``): the
     *     window close is an offset from route start, so the absolute
     *     (midnight-anchored) ``atSecond`` is compared against
     *     ``twEarlyAnchor + close``. Defaults to 0, which keeps the legacy
     *     behaviour for call sites that do not pass an anchor.
     *
     * Returns
     * -------
     * DriveSegment
     *     The merged segment with accumulated times and updated
     *     breaksTakenMask (OR of both).
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
          Duration extraWork = 0,
          uint16_t *breakDueMask = nullptr,
          int64_t *firstDueClock = nullptr,
          Duration twEarlyAnchor = 0);

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

/**
 * Window-close gate: returns whether a break's (conservative) arrival at its
 * own node falls strictly after its time-window close.
 *
 * A break with no window (``tws`` empty — e.g. the regulatory DUTY_TIME
 * breaks) is never past its close: its trigger alone governs eligibility.
 * RELATIVE windows are offsets from the route start, so their close is
 * compared against ``twEarlyAnchor + close``; ABSOLUTE windows compare
 * against ``close`` directly. Both are expressed in the absolute
 * (midnight-anchored) solver clock, matching ``atSecond``.
 *
 * A DUE break that is past its close is not servable: the solver must drop it
 * (no reset, no served bit) rather than silently serve it late. The close is
 * still enforced (time warp), so the route remains infeasible — matching the
 * honest-window contract when ``relax_breaks`` is off.
 */
inline bool isBreakPastWindowClose(pyvrp::CustomBreak const &brk,
                                   Duration arrival,
                                   Duration twEarlyAnchor = Duration(0))
{
    if (brk.tws.empty())
        return false;

    auto const close = brk.twsRelative
                           ? static_cast<int64_t>(brk.tws.back().second.get())
                                 + static_cast<int64_t>(twEarlyAnchor.get())
                           : static_cast<int64_t>(brk.tws.back().second.get());
    return static_cast<int64_t>(arrival.get()) > close;
}

// Verify compact layout: 4×8B int64 + 1×2B uint16 + 6B padding = 40B.
// The two-per-cache-line packing is lost (acceptable trade-off for the
// additional lastResetAt_ field required for CLT-correct duty tracking).
static_assert(sizeof(DriveSegment) == 40,
              "DriveSegment must be exactly 40 bytes");

/**
 * D5: effective (extended) service duration for a served rest break.
 *
 * The extension applies to ANY served DUTY_TIME break whose next activity is
 * a client with a time window that opens after the minimum rest + travel: the
 * rest is extended to land exactly on the window open, absorbing the would-be
 * waiting. It is not limited to overnight rests — a break followed by a client
 * whose window is still closed (regardless of the time of day) extends the
 * same way. Otherwise the minimum service is returned unchanged.
 *
 * Clock contract: ``arrivalAtBreak`` and ``nextWindowOpen`` MUST be expressed
 * in the same clock. Both the forward passes and client ``twEarly`` values use
 * the ABSOLUTE (midnight-anchored) clock, so callers pass ``nextWindowOpen``
 * as the client's ``twEarly`` directly — no anchor subtraction.
 */
inline Duration breakEffectiveService(Duration serviceMin,
                                      Duration arrivalAtBreak,
                                      bool extend,
                                      Duration travelToNext,
                                      Duration nextWindowOpen)
{
    if (!extend)
        return serviceMin;

    auto const restEnd = arrivalAtBreak + serviceMin;
    if (restEnd + travelToNext >= nextWindowOpen)
        return serviceMin;

    return nextWindowOpen - travelToNext - arrivalAtBreak;
}

/**
 * Result of a shared forward-pass evaluation over a flat activity/location
 * sequence. Used by Route::update() and Proposal to compute breakDue with
 * literally the same forward-pass semantics, guaranteeing parity by
 * construction.
 */
struct ForwardEvalResult
{
    Duration duration;        // total route duration
    Duration timeWarp;        // total time warp (vs maxDuration)
    int64_t breakDue;         // mandatory break lateness, in SECONDS
    uint16_t breakDueMask;    // bitmask of violated (due) break ids
    Duration waiting;         // total idle waiting (excl. absorbed rest)
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
 * extendedBreakServices
 *     Optional output vector (pre-sized to n) filled with the effective
 *     (possibly extended) service duration of each CUSTOM_BREAK node, and 0
 *     for non-break nodes. When nullptr, the extensions are still applied
 *     internally (to the duty reset and waiting measure). The D5 extension
 *     applies to any served DUTY_TIME break whose next activity is a client
 *     with a still-closed time window (not only overnight rests).
 * data
 *     Problem data instance.
 * vehicleType
 *     Vehicle type for the route being evaluated.
 *
 * .. note::
 *
 *    The forward pass also applies the due-ness gate for absolute break
 *    windows: a CUSTOM_BREAK node whose break is not yet due does not enforce
 *    the (possibly closed) absolute window close, so it cannot introduce time
 *    warp. The close is only enforced when the break is due at its node.
 */
ForwardEvalResult evaluateForwardPass(
    std::vector<Activity> const &activities,
    std::vector<size_t> const &locations,
    std::vector<Duration> &atSecond,
    std::vector<DurationSegment> *durPrefixOut,
    std::vector<Duration> *extendedBreakServices,
    ProblemData const &data,
    VehicleType const &vehicleType,
    std::vector<DurationSegment> *durAtOut = nullptr);

}  // namespace pyvrp::search

#endif  // PYVRP_DRIVESEGMENT_H
