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
 * A compact 32-byte struct that tracks driving, working, and duty time
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
 *     Accumulated duty time since last full reset (only ALL_TIMERS resets this).
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
                 uint16_t breakDue);

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
     *
     * Returns
     * -------
     * DriveSegment
     *     The merged segment with accumulated times, updated breaksTakenMask
     *     (OR of both), and accumulated breakDue count.
     *
     * Notes
     * -----
     * condition_min_route_s is compared against accumulated duty time
     * (driving + service), excluding waiting time.
     */
    [[nodiscard]] static DriveSegment
    merge(Duration edgeDur,
          DriveSegment const &first,
          DriveSegment const &second,
          std::vector<pyvrp::CustomBreak> const &breaks,
          Duration atSecond);

};

// Verify compact layout: 3×8B int64 + 2×2B uint16 + 4B padding = 32B.
// This ensures two DriveSegments fit in one 64-byte cache line.
static_assert(sizeof(DriveSegment) == 32,
              "DriveSegment must be exactly 32 bytes");

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
