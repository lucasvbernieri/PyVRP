#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "DutyTerms.h"
#include "Measure.h"
#include "VehicleType.h"

#include <algorithm>
#include <cassert>
#ifdef PYVRP_STREAM_STATS
#include <cstdio>
#include <cstdlib>
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <x86intrin.h>
#endif
#endif
#include <concepts>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace pyvrp::search
{
// Whether duration() runs the O(1) composed evaluator (PYVRP_CLOCK_TRIGGER=1
// and PYVRP_COMPOSE != 0); defined in DriveSegment.cpp.
extern bool const composeEnabled;
// PYVRP_BREAK_FREE_DURATION: a served break's service leaves the duration
// cost; defined in DriveSegment.cpp. Read here only by a stats-build probe.
extern int const breakFreeDuration;
}  // namespace pyvrp::search

namespace pyvrp
{
// The following methods must be available before a type's delta cost can be
// evaluated by the CostEvaluator.
template <typename T>
concept DeltaCostEvaluatable = requires(T arg, size_t dimension) {
    { arg.route() };
    { arg.fixedVehicleCost() } -> std::same_as<Cost>;
    { arg.distance() } -> std::convertible_to<std::pair<Cost, Distance>>;
    { arg.duration() } -> std::convertible_to<std::pair<Cost, Duration>>;
    { arg.excessLoad(dimension) } -> std::same_as<Load>;
};

/**
 * CostEvaluator(
 *     load_penalties: list[float],
 *     tw_penalty: float,
 *     dist_penalty: float,
 * )
 *
 * Creates a CostEvaluator instance.
 *
 * This class stores various penalty terms, and can be used to determine the
 * costs of certain constraint violations.
 *
 * Parameters
 * ----------
 * load_penalties
 *    The penalty terms (one for each load dimension) for each unit of load in
 *    excess of the vehicle capacity.
 * tw_penalty
 *    The penalty for each unit of time warp.
 * dist_penalty
 *    The penalty for each unit of distance in excess of the vehicle's maximum
 *    distance constraint.
 *
 * Raises
 * ------
 * ValueError
 *     When any of the given penalty terms are negative.
 */
class CostEvaluator
{
    std::vector<double> loadPenalties_;  // per load dimension
    double twPenalty_;
    double distPenalty_;
    double breakDuePenalty_;
    double waitCostRate_;

    /**
     * Computes the cost penalty incurred from the given excess loads. This is
     * a convenient shorthand for calling ``loadPenalty`` for each dimension.
     */
    [[nodiscard]] inline Cost
    excessLoadPenalties(std::vector<Load> const &excessLoads) const;

public:
    CostEvaluator(std::vector<double> loadPenalties,
                  double twPenalty,
                  double distPenalty,
                  double breakDuePenalty = 0,
                  double waitCostRate = 0);

    /**
     * Computes the total excess load penalty for the given load and vehicle
     * capacity, and dimension.
     */
    [[nodiscard]] inline Cost
    loadPenalty(Load load, Load capacity, size_t dimension) const;

    /**
     * Computes the time warp penalty for the given time warp.
     */
    [[nodiscard]] inline Cost twPenalty(Duration timeWarp) const;

    /**
     * Computes the break due penalty for the given number of mandatory break
     * violations.
     */
    [[nodiscard]] inline Cost breakDuePenalty(int64_t breakDue) const;

    /**
     * Computes the idle waiting penalty for the given waiting duration, at
     * the configured per-second wait rate. This is the TOTAL charge for idle
     * time: waiting is NOT part of the duration cost (duration cost covers
     * travel, service and setup only — see the class-level objective math
     * below, where :math:`t_R` excludes waiting).
     */
    [[nodiscard]] inline Cost waitPenalty(Duration waiting) const;

    /**
     * Computes the total excess distance penalty for the given distance.
     */
    [[nodiscard]] inline Cost distPenalty(Distance distance,
                                          Distance maxDistance) const;

    /**
     * Duty model: saturation bound of every duty-model term and of their sum
     * (``2^62``). Only these terms saturate there; every other term keeps its
     * own arithmetic, bit for bit.
     */
    static constexpr int64_t DUTY_SAT = int64_t(1) << 62;

    /**
     * Duty model: the weight per second of duty excess and of late return,
     * ``max(10 * tw_penalty, duty_excess_rate)``. Derived from the time-warp
     * penalty, which ``hasSameParameters`` already compares.
     */
    [[nodiscard]] inline int64_t dutyExcessWeight(int64_t dutyExcessRate) const;

    /**
     * Duty model: the penalty of a route's time warp and duty terms, which
     * replaces ``twPenalty(timeWarp)`` for vehicle types with ``duty_limit >=
     * 0``. The late return (``duty.depotWarp``, part of ``timeWarp``) and each
     * stretch's excess are charged at ``dutyExcessWeight`` on at most
     * ``duty_limit`` seconds, the rest of them at the time-warp penalty; the
     * client time warp at the time-warp penalty as always; then the overtime
     * bands, the anticipation and the rests at the vehicle's rates. Every
     * product and the sum saturate at ``DUTY_SAT``.
     */
    [[nodiscard]] inline Cost dutyPenalty(Duration timeWarp,
                                          DutyTerms const &duty,
                                          DutyRates const &rates) const;

    /**
     * Convenience overload of ``dutyPenalty`` taking the rates from the given
     * vehicle type.
     */
    [[nodiscard]] inline Cost dutyPenalty(Duration timeWarp,
                                          DutyTerms const &duty,
                                          VehicleType const &vehicleType) const;

    /**
     * Duty model: the ``DutyRates`` of the given vehicle type.
     */
    [[nodiscard]] static inline DutyRates
    dutyRatesOf(VehicleType const &vehicleType);

    /**
     * Duty model: the most gaps ``dutyPenaltyLowerBound`` takes (see
     * ``search::Route::Proposal::dutyGapBounds``); a proposal with more is
     * not bounded.
     */
    static constexpr size_t DUTY_MAX_GAPS = 33;

    /**
     * Duty model: an admissible lower bound on ``dutyPenalty`` of a route
     * whose ``numGaps`` gaps (the runs of nodes between consecutive break
     * nodes) have duty at least ``gaps[i]`` each. Whichever rests the route
     * serves, each stretch is a union of consecutive gaps and its duty
     * ``J`` at least their sum, so each of its terms is at least what that
     * sum implies: the excess ``max(0, J - L)`` (at the excess weight up to
     * ``L``, at the time-warp penalty beyond), the overtime band
     * ``max(0, min(J, L) - T)`` (at the pre-rest or the depot rate) and, for
     * a stretch closed by a rest, that rest at ``min(rest_use_cost,
     * rest_unneeded_cost)``; every other term is non-negative. The bound is
     * the least such sum over every choice of served rests (a dynamic
     * programme over the break nodes, O(numGaps^2)), with the same
     * saturating arithmetic as ``dutyPenalty``. ``trigger`` is the
     * vehicle type's ``duty_trigger`` (``T``).
     */
    [[nodiscard]] inline Cost dutyPenaltyLowerBound(int64_t const *gaps,
                                                    size_t numGaps,
                                                    DutyRates const &rates,
                                                    int64_t trigger) const;

    /**
     * Computes the excess distance penalty for the given excess distance.
     */
    [[nodiscard]] inline Cost excessDistPenalty(Distance excessDistance) const;

    /**
     * Computes a smoothed objective (penalised cost) for a given solution.
     */
    template <typename T> [[nodiscard]] Cost penalisedCost(T const &arg) const;
    // We only expose penalisedCost() for the Solution class to Python.

    /**
     * Hand-waving some details, each solution consists of a set of non-empty
     * routes :math:`\mathcal{R}`. Each route :math:`R \in \mathcal{R}` can be
     * represented as a sequence of edges, starting and ending at a depot. A
     * route :math:`R` has an assigned vehicle type that equips the route with
     * fixed vehicle cost :math:`f_R`, and unit distance, duration and overtime
     * costs :math:`c^\text{distance}_R`, :math:`c^\text{duration}_R`,
     * :math:`c^\text{overtime}_R`, respectively. Let
     * :math:`V_R = \{i : (i, j) \in R \}` be the set of clients and shipments
     * visited by route :math:`R`, and :math:`d_R`, :math:`t_R`, :math:`w_R`,
     * and :math:`o_R` the total route distance, duration (excluding waiting),
     * waiting, and overtime, respectively. The objective value is then given
     * by
     *
     * .. math::
     *
     *    \sum_{R \in \mathcal{R}}
     *      \left[
     *          f_R + c^\text{distance}_R d_R
     *              + c^\text{duration}_R t_R
     *              + c^\text{wait} w_R
     *              + c^\text{overtime}_R o_R
     *      \right]
     *    + \sum_{i \in V} p_i - \sum_{R \in \mathcal{R}} \sum_{i \in V_R} p_i,
     *
     * where the first part lists each route's fixed, distance, duration,
     * waiting and overtime costs, respectively, and the second part the
     * uncollected prizes of unvisited clients and shipments. Overtime is
     * charged on the FULL duration (including waiting — a driver held beyond
     * the shift pays both the wait rate and the overtime rate for the excess).
     *
     * .. note::
     *
     *    The above cost computation only holds for feasible solutions. If the
     *    solution argument is *infeasible*, we return a very large number.
     *    If that is not what you want, consider calling :meth:`penalised_cost`
     *    instead.
     */
    template <typename T> [[nodiscard]] Cost cost(T const &arg) const;
    // We only expose cost() for the Solution class to Python.

    /**
     * Evaluates the cost delta of the given route proposal, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposal does not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    template <bool exact = false,
              typename... Args,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<Args...>>)
    bool deltaCost(Cost &out, T<Args...> const &proposal) const;

    /**
     * Evaluates the cost delta of the given route proposals, and writes the
     * resulting cost delta to the ``out`` parameter. The evaluation can be
     * exact, if the relevant template argument is set. Else it may shortcut
     * once it determines that the proposals do not constitute an improving
     * move. Optionally, several aspects of the evaluation may be skipped.
     *
     * The return value indicates whether the evaluation was exact or not.
     */
    template <bool exact = false,
              typename... uArgs,
              typename... vArgs,
              template <typename...>
              class T>
        requires(DeltaCostEvaluatable<T<uArgs...>>
                 && DeltaCostEvaluatable<T<vArgs...>>)
    bool deltaCost(Cost &out,
                   T<uArgs...> const &uProposal,
                   T<vArgs...> const &vProposal) const;

    /**
     * Returns whether this evaluator has exactly the same parameters as the
     * given other evaluator (bitwise). Used by the local search to decide
     * whether evaluation results cached across invocations remain valid.
     */
    bool hasSameParameters(CostEvaluator const &other) const;
};

Cost CostEvaluator::excessLoadPenalties(
    std::vector<Load> const &excessLoads) const
{
    assert(excessLoads.size() == loadPenalties_.size());

    Cost cost = 0;
    for (size_t dim = 0; dim != loadPenalties_.size(); ++dim)
        cost += loadPenalties_[dim] * excessLoads[dim].get();

    return cost;
}

Cost CostEvaluator::loadPenalty(Load load,
                                Load capacity,
                                size_t dimension) const
{
    assert(dimension < loadPenalties_.size());
    auto const excessLoad = std::max<Load>(load - capacity, 0);
    return static_cast<Cost>(excessLoad.get() * loadPenalties_[dimension]);
}

Cost CostEvaluator::twPenalty([[maybe_unused]] Duration timeWarp) const
{
    return static_cast<Cost>(timeWarp.get() * twPenalty_);
}

Cost CostEvaluator::breakDuePenalty(int64_t breakDue) const
{
    // Saturating cast (D5): the per-second lateness can be large (multi-day
    // overdue × rate). Clamp the double product into the int64 Cost range
    // instead of overflowing (the PenaltyParams docstring warns about large
    // maximum penalties).
    auto const value = static_cast<double>(breakDue) * breakDuePenalty_;
    auto const clamped = std::clamp(value, -9e15, 9e15);
    return static_cast<Cost>(clamped);
}

Cost CostEvaluator::waitPenalty(Duration waiting) const
{
    // Saturating cast (D5): mirror breakDuePenalty — clamp the product into
    // the int64 Cost range instead of overflowing on large waiting × rate.
    auto const value = static_cast<double>(waiting.get()) * waitCostRate_;
    auto const clamped = std::clamp(value, -9e15, 9e15);
    return static_cast<Cost>(clamped);
}

Cost CostEvaluator::distPenalty(Distance distance, Distance maxDistance) const
{
    auto const excessDistance = std::max<Distance>(distance - maxDistance, 0);
    return excessDistPenalty(excessDistance);
}

Cost CostEvaluator::excessDistPenalty(Distance excessDistance) const
{
    return static_cast<Cost>(excessDistance.get() * distPenalty_);
}

namespace detail
{
// Saturating arithmetic of the duty-model terms (non-negative operands).
inline int64_t dutySatAdd(int64_t lhs, int64_t rhs)
{
    auto const sat = CostEvaluator::DUTY_SAT;
    return lhs >= sat - rhs ? sat : lhs + rhs;
}

inline int64_t dutySatMul(int64_t lhs, int64_t rhs)
{
    auto const sat = CostEvaluator::DUTY_SAT;
    if (lhs <= 0 || rhs <= 0)
        return 0;

    return lhs > sat / rhs ? sat : std::min(lhs * rhs, sat);
}

inline int64_t dutySatMul(int64_t seconds, double rate)
{
    auto const value = static_cast<double>(seconds) * rate;
    auto const sat = static_cast<double>(CostEvaluator::DUTY_SAT);
    return value <= 0 ? 0
                      : (value >= sat ? CostEvaluator::DUTY_SAT
                                      : static_cast<int64_t>(value));
}
}  // namespace detail

int64_t CostEvaluator::dutyExcessWeight(int64_t dutyExcessRate) const
{
    auto const tw10 = detail::dutySatMul(10, twPenalty_);
    return std::max<int64_t>(tw10, dutyExcessRate);
}

DutyRates CostEvaluator::dutyRatesOf(VehicleType const &vehicleType)
{
    return {vehicleType.dutyLimit.get(),
            vehicleType.dutyExcessRate.get(),
            vehicleType.overtimeRate.get(),
            vehicleType.preRestOvertimeRate.get(),
            vehicleType.earlyRate.get(),
            vehicleType.restUseCost.get(),
            vehicleType.restUnneededCost.get()};
}

Cost CostEvaluator::dutyPenalty(Duration timeWarp,
                                DutyTerms const &duty,
                                VehicleType const &vehicleType) const
{
    return dutyPenalty(timeWarp, duty, dutyRatesOf(vehicleType));
}

Cost CostEvaluator::dutyPenalty(Duration timeWarp,
                                DutyTerms const &duty,
                                DutyRates const &rates) const
{
    using detail::dutySatAdd;
    using detail::dutySatMul;

    auto const limit = rates.dutyLimit;
    auto const weight = dutyExcessWeight(rates.dutyExcessRate);

    // Late return: the end depot's share of the time warp. Up to duty_limit
    // seconds of it at the excess weight; the rest, like the client warp, at
    // the time-warp penalty.
    auto const depotWarp = std::clamp<int64_t>(duty.depotWarp, 0, timeWarp.get());
    auto const lateCapped = std::min(depotWarp, limit);
    auto const atTwRate = timeWarp.get() - lateCapped + duty.excessRemainder;

    int64_t total = dutySatMul(atTwRate, twPenalty_);
    total = dutySatAdd(total, dutySatMul(lateCapped, weight));
    total = dutySatAdd(total, dutySatMul(duty.excessCapped, weight));
    total = dutySatAdd(total, dutySatMul(duty.bandDepot, rates.overtimeRate));
    total = dutySatAdd(total,
                       dutySatMul(duty.bandRest, rates.preRestOvertimeRate));
    total = dutySatAdd(total, dutySatMul(duty.anticipation, rates.earlyRate));
    total = dutySatAdd(total, dutySatMul(duty.restsNeeded, rates.restUseCost));
    total = dutySatAdd(total,
                       dutySatMul(duty.restsUnneeded, rates.restUnneededCost));
    return total;
}

Cost CostEvaluator::dutyPenaltyLowerBound(int64_t const *gaps,
                                          size_t numGaps,
                                          DutyRates const &rates,
                                          int64_t trigger) const
{
    using detail::dutySatAdd;
    using detail::dutySatMul;

    assert(numGaps >= 1 && numGaps <= DUTY_MAX_GAPS);

    auto const limit = rates.dutyLimit;
    auto const weight = dutyExcessWeight(rates.dutyExcessRate);
    auto const restCost = std::min(rates.restUseCost, rates.restUnneededCost);

    // Least terms of a stretch of duty at least J, closed by a rest or by
    // the end depot (excess and overtime band, as in dutyPenalty).
    auto const restLB = dutySatMul(1, restCost);     // >= 0
    auto const freeUpTo = std::min(limit, trigger);  // no excess, no band
    auto const stretch = [&](int64_t J, bool atRest) -> int64_t
    {
        if (J <= freeUpTo)
            return 0;

        auto const excess = std::max<int64_t>(0, J - limit);
        auto const capped = std::min(excess, limit);
        auto const band = std::max<int64_t>(0, std::min(J, limit) - trigger);
        int64_t total = dutySatMul(capped, weight);
        total = dutySatAdd(total, dutySatMul(excess - capped, twPenalty_));
        total = dutySatAdd(total,
                           dutySatMul(band,
                                      atRest ? rates.preRestOvertimeRate
                                             : rates.overtimeRate));
        return total;
    };

    // prefix[k]: summed bound of gaps 0, ..., k - 1. Break node j (1 <= j <
    // numGaps) sits between gaps j - 1 and j.
    int64_t prefix[DUTY_MAX_GAPS + 1];
    prefix[0] = 0;
    for (size_t k = 0; k != numGaps; ++k)
        prefix[k + 1] = prefix[k] + gaps[k];

    // A break node at the same prefix as the last node kept before it (or
    // the start) adds nothing the programme needs: a choice of rests that
    // serves it gives every stretch the same J when it serves that earlier
    // node instead, and one serving both pays an extra empty stretch and
    // rest, at least zero. Dropping such nodes (the rests parked together
    // after the last stop, above all) leaves the least sum -- the bound --
    // unchanged, bit for bit, and the programme below quadratic in the
    // nodes that remain only.
    if (dutySatAdd(stretch(0, true), restLB) >= 0)
    {
        size_t kept = 1;  // prefix[0], the start, is always kept
        for (size_t j = 1; j != numGaps; ++j)
            if (prefix[j] != prefix[kept - 1])
                prefix[kept++] = prefix[j];
        prefix[kept] = prefix[numGaps];  // the end depot
        numGaps = kept;
    }

    // best[j]: least terms of the stretches up to a served rest at break
    // node j (best[0]: the route start).
    int64_t best[DUTY_MAX_GAPS];
    best[0] = 0;
    for (size_t j = 1; j != numGaps; ++j)
    {
        best[j] = DUTY_SAT;
        for (size_t i = 0; i != j; ++i)
        {
            auto const J = prefix[j] - prefix[i];
            auto total = dutySatAdd(best[i], stretch(J, true));
            total = dutySatAdd(total, restLB);
            best[j] = std::min(best[j], total);
        }
    }

    int64_t bound = DUTY_SAT;
    for (size_t i = 0; i != numGaps; ++i)
    {
        auto const J = prefix[numGaps] - prefix[i];
        bound = std::min(bound, dutySatAdd(best[i], stretch(J, false)));
    }

    return bound;
}

template <typename T> Cost CostEvaluator::cost(T const &arg) const
{
    // Penalties are zero when the solution is feasible, so we can fall back to
    // penalised cost in that case.
    return arg.isFeasible() ? penalisedCost(arg)
                            : std::numeric_limits<Cost>::max();
}

#ifdef PYVRP_STREAM_STATS
namespace detail
{
// Sizes the headroom left in the duration lower bound: of the proposals that
// survive it and pay for duration(), how many turn out non-improving anyway?
// Those are what a tighter bound could still catch.
struct BoundStats
{
    unsigned long long reached = 0;   // reached the duration term
    unsigned long long pruned = 0;    // rejected by the bound
    unsigned long long paid = 0;      // paid for duration()
    unsigned long long wasted = 0;    // ... and were non-improving anyway
    unsigned long long dutyPruned = 0;  // rejected by the duty-model gate

    // Duty gate cost split (stats only): cycles in dutyGapBounds() and in
    // dutyPenaltyLowerBound(), gaps seen, zero gaps among them, and gates
    // the travel-and-service part alone (gap sum) would already prune.
    unsigned long long dutyGates = 0;
    unsigned long long dutyGapCycles = 0;
    unsigned long long dutyDpCycles = 0;
    unsigned long long dutyGapsSeen = 0;
    unsigned long long dutyZeroGaps = 0;
    unsigned long long dutySumPrunes = 0;

    // PYVRP_DUTY_LB_CHECK=1: the duty-model gate never prunes; its bound is
    // compared against the exact increment instead (admissibility check).
    unsigned long long dutyChecked = 0;
    unsigned long long dutyWouldPrune = 0;
    unsigned long long dutyViolations = 0;

    // Fase 2: admissibility check for the candidate fold-based bound
    // (Proposal::durationLowerBoundFold()). Computed but NEVER used to
    // prune -- only compared against the real increment duration() (and the
    // terms that follow it in the same hasDurationCost() block) add to
    // ``out``, whenever the CURRENT bound did not already prune. A
    // violation is lbFold > actual, i.e. the candidate would have pruned a
    // proposal the exact computation did not reject -- which would make it
    // inadmissible.
    unsigned long long lbFoldChecked = 0;
    unsigned long long lbFoldViolations = 0;
    double lbFoldViolationSum = 0.0;   // sum of (lbFold - actual) over violations
    Cost lbFoldViolationMax = 0;       // largest single violation magnitude

    ~BoundStats()
    {
        if (!reached)
            return;
        std::fprintf(stderr,
                     "[bound-stats] reached=%llu pruned=%.3f "
                     "paid_but_rejected=%.3f (of paid)\n"
,
                     reached, double(pruned) / double(reached),
                     paid ? double(wasted) / double(paid) : 0.0);
        std::fprintf(stderr,
                     "[bound-stats][duty] pruned=%llu checked=%llu "
                     "would_prune=%llu violations=%llu\n",
                     dutyPruned, dutyChecked, dutyWouldPrune, dutyViolations);
        if (dutyGates)
            std::fprintf(stderr,
                         "[bound-stats][duty-gate] gates=%llu "
                         "gap_cyc=%.1f dp_cyc=%.1f gaps=%.2f zero=%.2f "
                         "sum_prunes=%.3f\n",
                         dutyGates,
                         double(dutyGapCycles) / double(dutyGates),
                         double(dutyDpCycles) / double(dutyGates),
                         double(dutyGapsSeen) / double(dutyGates),
                         double(dutyZeroGaps) / double(dutyGates),
                         double(dutySumPrunes) / double(dutyGates));
        std::fprintf(stderr,
                     "[bound-stats][lbfold] checked=%llu violations=%llu "
                     "(rate=%.6f) avg_violation=%.3f max_violation=%lld\n",
                     lbFoldChecked, lbFoldViolations,
                     lbFoldChecked
                         ? double(lbFoldViolations) / double(lbFoldChecked)
                         : 0.0,
                     lbFoldViolations
                         ? lbFoldViolationSum / double(lbFoldViolations)
                         : 0.0,
                     static_cast<long long>(lbFoldViolationMax));
    }
};

inline BoundStats boundStats{};

inline bool const dutyLbCheck = []
{
    auto const *env = std::getenv("PYVRP_DUTY_LB_CHECK");
    return env && *env && !(env[0] == '0' && env[1] == '\0');
}();
}  // namespace detail
#endif

template <bool exact, typename... Args, template <typename...> class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();
    out -= penalisedCost(*route);
    out += proposal.fixedVehicleCost();
#ifdef PYVRP_STREAM_STATS
    Cost dutyCheckLb = 0;
    Cost dutyCheckOut = 0;
    bool dutyCheckOn = false;
#endif

    if (route->hasDistanceCost())
    {
        auto const [cost, excess] = proposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    auto const &capacity = route->capacity();
    for (size_t dim = 0; dim != capacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(proposal.excessLoad(dim), 0, dim);
    }

    if (route->hasDurationCost())
    {
        if constexpr (!exact)
        {
            // duration() is the most expensive term on the break path. Everything it
            // and the terms after it add is bounded below by the route's unavoidable
            // travel and service, so a proposal already non-improving at that bound
            // cannot become improving.
            //
            // IMPORTANT: callers of deltaCost() (Relocate, Swap, ...) ignore the
            // returned bool entirely and decide whether to apply a move purely from
            // the SIGN of the ``out`` accumulator (e.g. ``deltaCost < 0``, see
            // Relocate::evaluate()). Every early return in this function must
            // therefore leave ``out`` itself >= 0 -- exactly like the load-penalty
            // gate above, which only exits once ``out`` (not some bound on it) has
            // already reached zero. Exiting here with ``out + lb >= 0`` while ``out``
            // itself is still negative would make the caller wrongly apply a move
            // this bound just proved is NOT improving. Folding ``lb`` into ``out``
            // before returning keeps the same safety invariant: ``out`` after the
            // fold is still an admissible (partial, non-exact) underestimate of the
            // true delta, and is provably >= 0 whenever we take this branch.
#ifdef PYVRP_STREAM_STATS
            ::pyvrp::detail::boundStats.reached++;
#endif
            // Duty model: the duty bound below replaces the duration bound
            // (it adds the duty terms to the same travel and service, from
            // the same walk), so a proposal it covers is gated once.
            Cost lbUsed = 0;  // the duration bound, when it was computed
            bool dutyGated = false;
            if (route->dutyBoundOk())
            {
                // The incumbent's duty penalty (its rests, above all) is what
                // a proposal has to recover, and the proposal pays at least
                // the excess, bands and rests the unavoidable duty between
                // its break nodes implies (see dutyPenaltyLowerBound), on top
                // of its travel and service at the duration rate (the gaps'
                // sum, below durationLowerBound()). Same safety invariant as
                // the duration bound: duration() and the terms after it add
                // at least this, so ``out`` is >= 0 when we exit. Exact: it
                // only rejects proposals whose full evaluation is
                // non-improving.
                int64_t gaps[DUTY_MAX_GAPS];
#ifdef PYVRP_STREAM_STATS
                auto const tGap0 = __rdtsc();
#endif
                auto const numGaps
                    = proposal.dutyGapBounds(gaps, DUTY_MAX_GAPS);
#ifdef PYVRP_STREAM_STATS
                auto const tGap1 = __rdtsc();
#endif
                if (numGaps > 0)
                {
                    dutyGated = true;
                    int64_t gapSum = 0;
                    for (size_t g = 0; g != numGaps; ++g)
                        gapSum += gaps[g];

                    auto const &vt = route->vehicleTypeData();
#ifdef PYVRP_STREAM_STATS
                    auto const tDp0 = __rdtsc();
#endif
                    auto const lb
                        = route->unitDurationCost() * static_cast<Cost>(gapSum)
                          + dutyPenaltyLowerBound(gaps,
                                                  numGaps,
                                                  dutyRatesOf(vt),
                                                  vt.dutyTrigger.get());
#ifdef PYVRP_STREAM_STATS
                    {
                        auto &bs = ::pyvrp::detail::boundStats;
                        bs.dutyDpCycles += __rdtsc() - tDp0;
                        bs.dutyGapCycles += tGap1 - tGap0;
                        bs.dutyGates++;
                        bs.dutyGapsSeen += numGaps;
                        for (size_t g = 0; g != numGaps; ++g)
                            bs.dutyZeroGaps += gaps[g] == 0;
                        if (out + route->unitDurationCost()
                                      * static_cast<Cost>(gapSum)
                            >= 0)
                            bs.dutySumPrunes++;
                    }
#endif
#ifdef PYVRP_STREAM_STATS
                    if (::pyvrp::detail::dutyLbCheck)
                    {
                        // Check mode: never prune; compare with the exact
                        // increment below.
                        dutyCheckLb = lb;
                        dutyCheckOut = out;
                        dutyCheckOn = true;
                    }
                    else
#endif
                    if (out + lb >= 0)
                    {
#ifdef PYVRP_STREAM_STATS
                        ::pyvrp::detail::boundStats.pruned++;
                        ::pyvrp::detail::boundStats.dutyPruned++;
#endif
                        out += lb;
                        return false;
                    }
                }
            }

            if (!dutyGated)
            {
                // The bound is at most this ceiling (an O(#segments) sum of
                // per-route totals). When even that cannot lift ``out`` to
                // zero the bound cannot prune, so computing it would only
                // cost the per-range prefix work -- skip it. Exact: the
                // proposal goes on to duration() exactly as it would have
                // after a non-pruning bound, and ``out`` is untouched either
                // way.
                auto const ceiling
                    = route->unitDurationCost()
                      * static_cast<Cost>(
                          proposal.durationLowerBoundCeiling().get());
                if (out + ceiling >= 0)
                {
                    auto const lb
                        = route->unitDurationCost()
                          * static_cast<Cost>(
                              proposal.durationLowerBound().get());
                    if (out + lb >= 0)
                    {
#ifdef PYVRP_STREAM_STATS
                        ::pyvrp::detail::boundStats.pruned++;
#endif
                        out += lb;
                        return false;
                    }
                    lbUsed = lb;
                }
            }

            // Second gate, on the break-lateness penalty: with production's
            // penalty rates the incumbent's breakDue term is most of what a
            // proposal has to recover, and a proposal whose only break node
            // cannot be served where it sits is charged at least the rule's
            // service for it. Same safety invariant as above: everything
            // duration() and the terms after it add is >= lbUsed + bd, so
            // folding both into ``out`` leaves it >= 0 whenever we exit.
            //
            // Lane 13: skipped under the composed evaluator. The bound was
            // priced against the streaming walk (~1 450 cycles); against the
            // O(1) composition (~400-500) its pass 2 -- the same prefix fold
            // runComposed() performs again on every candidate it does not
            // prune -- no longer pays. Measured per binaryOps sweep on the
            // production instances (break arm, flag on, stats build): the
            // bound costs 190 / 70 / 45 cycles per call and prunes 0.96 /
            // 0.26 / 0.29 duration() calls per sweep; net -3 / +135 / +73
            // cycles per sweep from skipping it (45 / 72 / 54 activities).
            // Exact: the bound only ever rejected proposals whose full
            // evaluation is non-improving anyway, so the search trajectory
            // is unchanged. Untouched off the flag.
            // Duty model: breakDueLowerBound() makes no claim (its rests
            // never become due), so the gate is skipped there outright.
            if (breakDuePenalty_ != 0 && route->hasBreaks()
                && !route->hasDutyModel() && !pyvrp::search::composeEnabled)
            {
                auto const bdSec = proposal.breakDueLowerBound();
                if (bdSec > 0)
                {
                    auto const bd = breakDuePenalty(bdSec.get());
                    if (out + lbUsed + bd >= 0)
                    {
#ifdef PYVRP_STREAM_STATS
                        ::pyvrp::detail::boundStats.pruned++;
#endif
                        out += lbUsed + bd;
                        return false;
                    }
                }
            }
#ifdef PYVRP_STREAM_STATS
            ::pyvrp::detail::boundStats.paid++;
#endif
        }

#ifdef PYVRP_STREAM_STATS
        // Fase 2: candidate-bound admissibility check. Computed here (bound
        // NOT pruned above, so we always fall through to duration() below
        // regardless of what lbFold says) and compared against the actual
        // increment once it is known. NEVER used to prune -- see
        // Proposal::durationLowerBoundFold()'s docstring for why this must
        // be verified before it can be folded into the gate.
        // break-free-duration: the monoid fold prices a served break's
        // service (it knows no break decisions), so under the switch it is
        // not a lower bound on the increment; the probe is skipped rather
        // than reported as a violation. Stats builds only; never prunes.
        Cost lbFold = 0;
        if constexpr (!exact)
            if (!pyvrp::search::breakFreeDuration && !route->hasDutyModel())
                lbFold = route->unitDurationCost()
                         * static_cast<Cost>(
                             proposal.durationLowerBoundFold().get());
        auto const outBeforeDuration = out;
#endif

        auto const [cost, timeWarp] = proposal.duration();
        out += cost;
        if (route->hasDutyModel())
            out += dutyPenalty(timeWarp,
                               proposal.dutyTerms(),
                               route->vehicleTypeData());
        else
            out += twPenalty(timeWarp);

        if (breakDuePenalty_ != 0)  // breakDue() is a no-op when the rate is 0
            out += breakDuePenalty(proposal.breakDue());

#ifdef PYVRP_STREAM_STATS
        if constexpr (!exact)
        {
            auto const actualIncrement = out - outBeforeDuration;
            ::pyvrp::detail::boundStats.lbFoldChecked++;
            if (lbFold > actualIncrement)
            {
                auto const violation = lbFold - actualIncrement;
                ::pyvrp::detail::boundStats.lbFoldViolations++;
                ::pyvrp::detail::boundStats.lbFoldViolationSum
                    += double(violation);
                if (violation > ::pyvrp::detail::boundStats.lbFoldViolationMax)
                    ::pyvrp::detail::boundStats.lbFoldViolationMax = violation;
            }
        }
#endif
    }

    if (waitCostRate_ != 0)
    {
        // Waiting delta can be negative (a move may reduce idle time), so no
        // exact shortcut here: always account for it.
        out += waitPenalty(proposal.waiting());
    }

#ifdef PYVRP_STREAM_STATS
    if constexpr (!exact)
        if (out >= 0)
            ::pyvrp::detail::boundStats.wasted++;
    if (dutyCheckOn)
    {
        auto &bs = ::pyvrp::detail::boundStats;
        bs.dutyChecked++;
        if (dutyCheckOut + dutyCheckLb >= 0)
            bs.dutyWouldPrune++;
        if (dutyCheckLb > out - dutyCheckOut)
        {
            bs.dutyViolations++;
            if (bs.dutyViolations <= 5)
                std::fprintf(stderr,
                             "[duty-lb] VIOLATION lb=%lld actual=%lld\n",
                             (long long)dutyCheckLb,
                             (long long)(out - dutyCheckOut));
        }
    }
#endif

    return true;
}

template <bool exact,
          typename... uArgs,
          typename... vArgs,
          template <typename...>
          class T>
    requires(DeltaCostEvaluatable<T<uArgs...>>
             && DeltaCostEvaluatable<T<vArgs...>>)
bool CostEvaluator::deltaCost(Cost &out,
                              T<uArgs...> const &uProposal,
                              T<vArgs...> const &vProposal) const
{
    auto const *uRoute = uProposal.route();
    auto const *vRoute = vProposal.route();

    out -= penalisedCost(*uRoute);
    out -= penalisedCost(*vRoute);

    out += uProposal.fixedVehicleCost();
    out += vProposal.fixedVehicleCost();

    if (uRoute->hasDistanceCost())
    {
        auto const [cost, excess] = uProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    if (vRoute->hasDistanceCost())
    {
        auto const [cost, excess] = vProposal.distance();
        out += cost;
        out += excessDistPenalty(excess);
    }

    auto const &uCapacity = uRoute->capacity();
    for (size_t dim = 0; dim != uCapacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(uProposal.excessLoad(dim), 0, dim);
    }

    auto const &vCapacity = vRoute->capacity();
    for (size_t dim = 0; dim != vCapacity.size(); ++dim)
    {
        if constexpr (!exact)
            if (out >= 0)
                return false;

        out += loadPenalty(vProposal.excessLoad(dim), 0, dim);
    }

    if constexpr (!exact)
        if (out >= 0)
            return false;

    if (uRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = uProposal.duration();
        out += cost;
        if (uRoute->hasDutyModel())
            out += dutyPenalty(timeWarp,
                               uProposal.dutyTerms(),
                               uRoute->vehicleTypeData());
        else
            out += twPenalty(timeWarp);

        if (breakDuePenalty_ != 0)
            out += breakDuePenalty(uProposal.breakDue());
    }

    if (vRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = vProposal.duration();
        out += cost;
        if (vRoute->hasDutyModel())
            out += dutyPenalty(timeWarp,
                               vProposal.dutyTerms(),
                               vRoute->vehicleTypeData());
        else
            out += twPenalty(timeWarp);

        if (breakDuePenalty_ != 0)
            out += breakDuePenalty(vProposal.breakDue());
    }

    if (waitCostRate_ != 0)
    {
        // Waiting delta can be negative (a move may reduce idle time), so no
        // exact shortcut here: always account for it.
        out += waitPenalty(uProposal.waiting());
        out += waitPenalty(vProposal.waiting());
    }

    return true;
}
}  // namespace pyvrp

inline bool pyvrp::CostEvaluator::hasSameParameters(
    CostEvaluator const &other) const
{
    // twPenalty_ also fixes the duty model's excess weight (dutyExcessWeight).
    return loadPenalties_ == other.loadPenalties_
        && twPenalty_ == other.twPenalty_
        && distPenalty_ == other.distPenalty_
        && breakDuePenalty_ == other.breakDuePenalty_
        && waitCostRate_ == other.waitCostRate_;
}

#endif  // PYVRP_COSTEVALUATOR_H
