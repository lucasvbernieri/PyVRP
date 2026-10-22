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

/**
 * True when the duty clock starts at the (fixed) departure instead of at the
 * latest-possible start: the vehicle type has a duty model and
 * ``startLate == twEarly`` (the router sends this when the user fixes the
 * departure time). Then waiting for the first client to open counts as duty.
 *
 * Edge case: a type with no explicit departure but a ceiling
 * ``twLate <= twEarly`` also satisfies ``startLate == twEarly``; it is
 * treated as a fixed departure too (coherent with the Python side).
 */
template <typename VT> inline bool dutyClockFromDeparture(VT const &vt)
{
    return vt.hasDutyModel() && vt.startLate == vt.twEarly;
}

/**
 * Effective route start (the value of ``driveAt[0].lastResetAt_``), in raw
 * clock units. ``at0`` is the arrival at the start depot, ``at1`` the arrival
 * at node 1 already clamped to its window opening, ``raw1`` the arrival at
 * node 1 BEFORE that clamp (i.e. without waiting for the window), and ``e01``
 * the depot-to-node-1 travel time.
 *
 * Default: ``max(at0, at1 - e01)``; the wait before the first client opens is
 * left out of the duty clock. With a fixed departure (see
 * ``dutyClockFromDeparture``) the clock starts at the departure instead,
 * ``max(at0, raw1 - e01)``, so that wait counts. With node 1 open on arrival
 * (``at1 == raw1``) both are identical.
 */
inline int64_t effectiveRouteStart(bool fromDeparture,
                                   int64_t at0,
                                   int64_t at1,
                                   int64_t raw1,
                                   int64_t e01)
{
    return (at0 > (fromDeparture ? raw1 : at1) - e01)
               ? at0
               : (fromDeparture ? raw1 : at1) - e01;
}
}  // namespace pyvrp

#endif  // PYVRP_DUTYTERMS_H
