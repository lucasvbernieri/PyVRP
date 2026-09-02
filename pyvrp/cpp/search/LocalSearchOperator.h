#ifndef PYVRP_SEARCH_LOCALSEARCHOPERATOR_H
#define PYVRP_SEARCH_LOCALSEARCHOPERATOR_H

#include "CostEvaluator.h"
#include "Measure.h"
#include "ProblemData.h"
#include "Route.h"
#include "Solution.h"  // pyvrp::search::Solution

#include <string>
#include <utility>

namespace pyvrp::search
{
/**
 * Simple data structure that tracks statistics about the number of times
 * an operator was evaluated and applied.
 *
 * Attributes
 * ----------
 * num_evaluations
 *     Number of evaluated moves.
 * num_applications
 *     Number of applied, improving moves.
 */
struct OperatorStatistics
{
    size_t numEvaluations = 0;
    size_t numApplications = 0;
};

template <std::same_as<Route::Node *>... Args> class LocalSearchOperator
{
protected:
    ProblemData const &data;
    mutable OperatorStatistics stats_;

public:
    /**
     * Determines the cost delta of applying this move to the arguments. If the
     * cost delta is negative, this is an improving move. The second, boolean
     * return value indicates whether the operator believes the move should be
     * applied (i.e., improves the solution).
     *
     * Moves that the operator believes should be applied must be fully
     * evaluated. The operator, however, is free to return early if it knows
     * the move will never be good: that is, when it determines the cost delta
     * cannot become negative at all. In that case, the returned (non-negative)
     * cost delta may not be a complete evaluation.
     */
    virtual std::pair<Cost, bool> evaluate(Args... args,
                                           CostEvaluator const &costEvaluator)
        = 0;

    /**
     * Applies this move to the given arguments. Should only be called
     * when ``evaluate()`` suggests applying the move.
     */
    virtual void apply(Args... args) const = 0;

    /**
     * Called once after loading the solution to improve. This can be used to
     * e.g. update local operator state.
     */
    virtual void init([[maybe_unused]] Solution &solution)
    {
        stats_ = {};  // reset call statistics
    };

    /**
     * Operator name.
     */
    virtual std::string name() const = 0;

    /**
     * Returns whether this operator can evaluate moves on CUSTOM_BREAK
     * nodes. Defaults to false: most operators only work on client nodes
     * and would corrupt the route if handed a break node.
     */
    virtual bool supportsBreakNodes() const { return false; }

    /**
     * Returns whether the segment of the given ``node``'s route, starting at
     * ``node`` and spanning ``segLength`` consecutive nodes, contains a
     * CUSTOM_BREAK node. CUSTOM_BREAK nodes are immutable in local search
     * (except via ShiftBreak), so operators that move client segments must
     * reject segments that contain one.
     */
    bool hasCustomBreak(Route::Node *node, size_t segLength) const
    {
        auto const &route = *node->route();

        // No break-configured rules on this route means no node in it can be
        // a CUSTOM_BREAK activity, so the scan below is always false. Short-
        // circuit it to keep the break-less local-search hot path cheap.
        if (!route.hasBreaks())
            return false;

        auto const first = node->pos();
        auto const last = first + segLength - 1;
        for (size_t pos = first; pos <= last; ++pos)
            if (route[pos]->isCustomBreak())
                return true;
        return false;
    }

    /**
     * Returns evaluation and application statistics collected since the last
     * solution initialisation.
     */
    OperatorStatistics const &statistics() const { return stats_; }

    /**
     * Determines whether this operator can find improving moves for the given
     * data instance.
     */
    static bool supports([[maybe_unused]] ProblemData const &data);

    /**
     * Called when a route has been changed.
     */
    virtual void update([[maybe_unused]] Route const *route) {};

    LocalSearchOperator(ProblemData const &data) : data(data){};
    virtual ~LocalSearchOperator() = default;
};

using UnaryOperator = LocalSearchOperator<Route::Node *>;
using BinaryOperator = LocalSearchOperator<Route::Node *, Route::Node *>;
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_LOCALSEARCHOPERATOR_H
