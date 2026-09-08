#ifndef PYVRP_COSTEVALUATOR_H
#define PYVRP_COSTEVALUATOR_H

#include "Measure.h"

#include <cassert>
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
}  // namespace detail
#endif

template <bool exact, typename... Args, template <typename...> class T>
    requires(DeltaCostEvaluatable<T<Args...>>)
bool CostEvaluator::deltaCost(Cost &out, T<Args...> const &proposal) const
{
    auto const *route = proposal.route();
    out -= penalisedCost(*route);
    out += proposal.fixedVehicleCost();

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
            // The bound is at most this ceiling (an O(#segments) sum of
            // per-route totals). When even that cannot lift ``out`` to zero
            // the bound cannot prune, so computing it would only cost the
            // per-range prefix work -- skip it. Exact: the proposal goes on
            // to duration() exactly as it would have after a non-pruning
            // bound, and ``out`` is untouched either way.
            auto const ceiling
                = route->unitDurationCost()
                  * static_cast<Cost>(
                      proposal.durationLowerBoundCeiling().get());
            Cost lbUsed = 0;  // the duration bound, when it was computed
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
            if (breakDuePenalty_ != 0 && route->hasBreaks()
                && !pyvrp::search::composeEnabled)
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
        Cost lbFold = 0;
        if constexpr (!exact)
            lbFold = route->unitDurationCost()
                     * static_cast<Cost>(proposal.durationLowerBoundFold().get());
        auto const outBeforeDuration = out;
#endif

        auto const [cost, timeWarp] = proposal.duration();
        out += cost;
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
        out += twPenalty(timeWarp);

        if (breakDuePenalty_ != 0)
            out += breakDuePenalty(uProposal.breakDue());
    }

    if (vRoute->hasDurationCost())
    {
        auto const [cost, timeWarp] = vProposal.duration();
        out += cost;
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
    return loadPenalties_ == other.loadPenalties_
        && twPenalty_ == other.twPenalty_
        && distPenalty_ == other.distPenalty_
        && breakDuePenalty_ == other.breakDuePenalty_
        && waitCostRate_ == other.waitCostRate_;
}

#endif  // PYVRP_COSTEVALUATOR_H
