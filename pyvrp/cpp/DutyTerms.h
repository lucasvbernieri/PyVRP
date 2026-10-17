#ifndef PYVRP_DUTYTERMS_H
#define PYVRP_DUTYTERMS_H

#include <cstdint>

namespace pyvrp
{
/**
 * Per-route terms of the duty model (vehicle types with ``duty_limit >= 0``),
 * computed by the shared forward pass (``search::evaluateForwardPass``) and
 * priced by ``CostEvaluator::dutyPenalty``. All zero for every other vehicle
 * type.
 *
 * A stretch runs from the effective route start, or from the end of a served
 * rest, to the next served rest or to the end depot; ``J`` is its duty (clock
 * time, waiting and lateness included). With ``T = duty_trigger`` and ``L =
 * duty_limit``:
 *
 * excessCapped / excessRemainder
 *     Per stretch ``e = max(0, J - L)``; ``min(e, L)`` is summed into
 *     ``excessCapped`` (charged at the excess weight) and the rest into
 *     ``excessRemainder`` (charged at the time-warp penalty only).
 * bandDepot / bandRest
 *     Per stretch ``max(0, min(J, L) - T)``: the overtime band, of the stretch
 *     that ends at the end depot and of those that end at a served rest.
 * anticipation
 *     Per served rest ``max(0, T - J)`` of the stretch it closes.
 * restsNeeded / restsUnneeded
 *     Served rests classified by the counterfactual duty of the stretch they
 *     separate (see the forward pass).
 * depotWarp
 *     Time warp incurred at the end depot (late return), already part of the
 *     route's total time warp.
 */
struct DutyTerms
{
    int64_t excessCapped = 0;
    int64_t excessRemainder = 0;
    int64_t bandDepot = 0;
    int64_t bandRest = 0;
    int64_t anticipation = 0;
    int64_t restsNeeded = 0;
    int64_t restsUnneeded = 0;
    int64_t depotWarp = 0;

    bool operator==(DutyTerms const &other) const = default;

    /**
     * Total duty above ``duty_limit``, over all stretches. Positive makes the
     * route infeasible.
     */
    [[nodiscard]] int64_t dutyExcess() const
    {
        return excessCapped + excessRemainder;
    }

    /**
     * Total overtime band, over all stretches.
     */
    [[nodiscard]] int64_t overtimeBand() const { return bandDepot + bandRest; }
};

/**
 * The duty-model fields of a vehicle type that price ``DutyTerms`` (see
 * ``VehicleType``). ``dutyLimit < 0`` means the vehicle type is not on the
 * duty model. Kept on solution routes, which do not reference their vehicle
 * type, so ``CostEvaluator`` can price them.
 */
struct DutyRates
{
    int64_t dutyLimit = -1;
    int64_t dutyExcessRate = 0;
    int64_t overtimeRate = 0;
    int64_t preRestOvertimeRate = 0;
    int64_t earlyRate = 1;
    int64_t restUseCost = 0;
    int64_t restUnneededCost = 0;

    bool operator==(DutyRates const &other) const = default;

    [[nodiscard]] bool active() const { return dutyLimit >= 0; }
};
}  // namespace pyvrp

#endif  // PYVRP_DUTYTERMS_H
