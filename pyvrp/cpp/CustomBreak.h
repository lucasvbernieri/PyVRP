#ifndef PYVRP_CUSTOMBREAK_H
#define PYVRP_CUSTOMBREAK_H

#include "Measure.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace pyvrp
{
enum class CustomBreakTrigger
{
    CLOCK_TIME,
    DRIVE_TIME,
    WORK_TIME,
    DUTY_TIME,
};

enum class CustomBreakReset
{
    NONE,
    DRIVE_TIMER,
    WORK_TIMER,
    DRIVE_AND_WORK,
    ALL_TIMERS,
};

/**
 * CustomBreak(
 *     id: int,
 *     tws: list[tuple[int, int]] = [],
 *     service: int = 0,
 *     trigger: CustomBreakTrigger = CustomBreakTrigger.CLOCK_TIME,
 *     trigger_value: int = 0,
 *     reset: CustomBreakReset = CustomBreakReset.NONE,
 *     mandatory: bool = False,
 *     condition_min_route_s: int = 0,
 *     priority: int = 0,
 *     supersedes: list[int] = [],
 *     tws_relative: bool = False,
 *     relaxable: bool = False,
 *     *,
 *     eligible_from: int = -1,
 * )
 *
 * Custom break data object. Defines a break rule for vehicle routes, including
 * the trigger condition, reset behaviour, and valid start time windows.
 *
 * When ``tws_relative`` is True, the ``tws`` are interpreted as offsets
 * relative to the route start time instead of absolute clock times.
 *
 * ``eligible_from`` selects the rest model. ``-1`` (default) keeps the
 * trigger-based behaviour bit for bit. A value in ``0..trigger_value`` makes
 * the rule a free-start rest of the vehicle's duty model: it may be served at
 * any boundary whose duty since the last reset is at least ``eligible_from``,
 * it never fires virtually (no due mask, no first-due clock, no breakDue),
 * and its cost comes from the vehicle's duty-model fields (see
 * ``VehicleType``). Such a rule is only accepted on a vehicle type with
 * ``duty_limit >= 0`` and ``duty_trigger == trigger_value``.
 */
struct CustomBreak
{
    size_t const id;
    std::vector<std::pair<Duration, Duration>> const tws;
    Duration const service;
    CustomBreakTrigger const trigger;
    Duration const triggerValue;
    CustomBreakReset const reset;
    bool const mandatory;
    Duration const conditionMinRouteS;
    int const priority;
    std::vector<size_t> const supersedes;
    bool const twsRelative;
    bool const relaxable;
    Duration const eligibleFrom;  // -1: legacy rule; >= 0: duty-model rest

    CustomBreak(size_t id,
                std::vector<std::pair<Duration, Duration>> tws = {},
                Duration service = 0,
                CustomBreakTrigger trigger = CustomBreakTrigger::CLOCK_TIME,
                Duration triggerValue = 0,
                CustomBreakReset reset = CustomBreakReset::NONE,
                bool mandatory = false,
                Duration conditionMinRouteS = 0,
                int priority = 0,
                std::vector<size_t> supersedes = {},
                bool twsRelative = false,
                bool relaxable = false,
                Duration eligibleFrom = -1);

    /**
     * Returns true if the given arrival time falls within any of the valid
     * start time windows (tws).
     */
    bool isValidStart(Duration arrival) const;

    /**
     * Returns whether this rule belongs to the duty model (``eligible_from >=
     * 0``) rather than the trigger-based one.
     */
    bool isDutyRest() const { return eligibleFrom >= 0; }
};

/**
 * BreakRule
 *
 * A flat, trivially-copyable POD view of a ``CustomBreak``, pre-computed once
 * per ``VehicleType`` so the hot-path ``DriveSegment::merge()`` fold never
 * has to chase heap-allocated ``std::vector`` members (``tws``,
 * ``supersedes``) on every route node.
 *
 * ``openAbs``/``closeAbs`` fold the (possibly relative) time-window bounds
 * into ABSOLUTE (midnight-anchored) clock values once, at construction time
 * (see ``makeBreakRule``). A break with no configured window gets the
 * sentinel ``openAbs = 0`` / ``closeAbs = INT64_MAX``, so the CLOCK_TIME
 * trigger test ``atSecondVal > closeAbs`` is unconditionally false — the same
 * result as the old ``!brk.tws.empty() && ...`` check, without a branch.
 *
 * ``bit`` and ``supersedesMask`` are likewise pre-computed from ``id`` and
 * ``supersedes`` so the merge fold only needs bitwise ORs instead of a
 * nested loop over ``supersedes``.
 */
struct BreakRule
{
    int64_t triggerValue;        // brk.triggerValue
    int64_t openAbs;             // window open, ABSOLUTE clock; 0 if no window
    int64_t closeAbs;            // window close, ABSOLUTE clock; INT64_MAX if
                                  // no window
    int64_t conditionMinRouteS;  // brk.conditionMinRouteS
    int64_t service;             // brk.service
    int64_t eligibleFrom;        // brk.eligibleFrom (-1: legacy rule)
    size_t id;                   // brk.id (raw id, indexes firstDueClock)
    uint16_t bit;                // 1u << (id & 0xF)
    uint16_t supersedesMask;     // OR of 1u << (sid & 0xF) for sid in
                                  // brk.supersedes
    CustomBreakTrigger trigger;
    CustomBreakReset reset;
    bool mandatory;
    bool hasWindow;  // !brk.tws.empty()
    bool twsRelative;
    bool relaxable;
};

/**
 * Builds a ``BreakRule`` from a ``CustomBreak`` and the vehicle's
 * ``twEarly`` anchor (used only to fold RELATIVE windows into the absolute
 * clock). See ``BreakRule`` for the exact field semantics.
 */
BreakRule makeBreakRule(CustomBreak const &brk, Duration twEarlyAnchor);
}  // namespace pyvrp

#endif  // PYVRP_CUSTOMBREAK_H
