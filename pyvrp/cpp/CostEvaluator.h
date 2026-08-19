#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "Measure.h"

#include <cassert>
#include <concepts>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace pyvrp
{
// The following methods must be available before a type's delta cost can be
// evaluated by the CostEvaluator.
template <typename T>
concept DeltaCostEvaluatable = requires(T arg, size_t dimension) {
    { arg.route() };
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
    double unitWaitCost_;

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
                  double unitWaitCost = 0);

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
     * Computes the idle waiting penalty for the given waiting duration. This
     * is additive over the duration cost (waiting is already part of the
     * route duration).
     */
    [[nodiscard]] inline Cost waitPenalty(Duration waiting) const;

    /**
     * Computes the total excess distance penalty for the given distance.
     */
    [[nodiscard]] inline Cost distPenalty(Distance distance,
                                          Distance maxDistance) const;

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
     * :math:`V_R = \{i : (i, j) \in R \}` be the set of locations visited by
     * route :math:`R`, and :math:`d_R`, :math:`t_R`, and :math:`o_R` the total
     * route distance, duration, and overtime, respectively. The objective value
     * is then given by
     *
     * .. math::
     *
     *    \sum_{R \in \mathcal{R}}
     *      \left[
     *          f_R + c^\text{distance}_R d_R
     *              + c^\text{duration}_R t_R
     *              + c^\text{overtime}_R o_R
     *      \right]
     *    + \sum_{i \in V} p_i - \sum_{R \in \mathcal{R}} \sum_{i \in V_R} p_i,
     *
     * where the first part lists each route's fixed, distance, duration and
     * overtime costs, respectively, and the second part the uncollected prizes
     * of unvisited clients.
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
    auto const value = static_cast<double>(waiting.get()) * unitWaitCost_;
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

template <typename T> Cost CostEvaluator::cost(T const &arg) const
{
    // Penalties are zero when the solution is feasible, so we can fall back to
    // penalised cost in that case.
    return arg.isFeasible() ? penalisedCost(arg)
                            : std::numeric_limits<Cost>::max();
}

template <bool exact, typename... Args, template <typename...> class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();
    if (!route->empty())
    {
        out -= route->distanceCost();
        out -= excessDistPenalty(route->excessDistance());

        out -= excessLoadPenalties(route->excessLoad());

        out -= route->durationCost();
        out -= twPenalty(route->timeWarp());
        out -= breakDuePenalty(route->breakDue());
        if (unitWaitCost_ != 0)
            out -= waitPenalty(route->waiting());
    }

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
        auto const [cost, timeWarp] = proposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
        out += breakDuePenalty(proposal.breakDue());
    }

    if (unitWaitCost_ != 0)
    {
        // Waiting delta can be negative (a move may reduce idle time), so no
        // exact shortcut here: always account for it.
        out += waitPenalty(proposal.waiting());
    }

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
    if (!uRoute->empty())
    {
        out -= uRoute->distanceCost();
        out -= excessDistPenalty(uRoute->excessDistance());

        out -= excessLoadPenalties(uRoute->excessLoad());

        out -= uRoute->durationCost();
        out -= twPenalty(uRoute->timeWarp());
        out -= breakDuePenalty(uRoute->breakDue());
        if (unitWaitCost_ != 0)
            out -= waitPenalty(uRoute->waiting());
    }

    auto const *vRoute = vProposal.route();
    if (!vRoute->empty())
    {
        out -= vRoute->distanceCost();
        out -= excessDistPenalty(vRoute->excessDistance());

        out -= excessLoadPenalties(vRoute->excessLoad());

        out -= vRoute->durationCost();
        out -= twPenalty(vRoute->timeWarp());
        out -= breakDuePenalty(vRoute->breakDue());
        if (unitWaitCost_ != 0)
            out -= waitPenalty(vRoute->waiting());
    }

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
        out += twPenalty(timeWarp);
        out += breakDuePenalty(uProposal.breakDue());
    }

    if (vRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = vProposal.duration();
        out += cost;
        out += twPenalty(timeWarp);
        out += breakDuePenalty(vProposal.breakDue());
    }

    if (unitWaitCost_ != 0)
    {
        // Waiting delta can be negative (a move may reduce idle time), so no
        // exact shortcut here: always account for it.
        out += waitPenalty(uProposal.waiting());
        out += waitPenalty(vProposal.waiting());
    }

    return true;
}
}  // namespace pyvrp

#endif  // PYVRP_COSTEVALUATOR_H
