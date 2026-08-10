#ifndef PYVRP_CUSTOMBREAK_H
#define PYVRP_CUSTOMBREAK_H

#include "Measure.h"

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
 * )
 *
 * Custom break data object. Defines a break rule for vehicle routes, including
 * the trigger condition, reset behaviour, and valid start time windows.
 *
 * When ``tws_relative`` is True, the ``tws`` are interpreted as offsets
 * relative to the route start time instead of absolute clock times.
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
                bool twsRelative = false);

    /**
     * Returns true if the given arrival time falls within any of the valid
     * start time windows (tws).
     */
    bool isValidStart(Duration arrival) const;
};
}  // namespace pyvrp

#endif  // PYVRP_CUSTOMBREAK_H
