#ifndef PYVRP_SEARCH_SHIFTBREAK_H
#define PYVRP_SEARCH_SHIFTBREAK_H

#include "LocalSearchOperator.h"

namespace pyvrp::search
{
/**
 * ShiftBreak(data: ProblemData)
 *
 * Repositions a single CUSTOM_BREAK activity within its route. Only moves
 * that strictly improve the solution (deltaCost < 0) are accepted.
 */
class ShiftBreak : public UnaryOperator
{
    using UnaryOperator::UnaryOperator;

    // Best position found during the last evaluate() call.
    size_t bestPos_ = 0;

public:
    std::pair<Cost, bool> evaluate(Route::Node *U,
                                   CostEvaluator const &costEvaluator) override;

    void apply(Route::Node *U) const override;

    std::string name() const override;
};

template <> bool supports<ShiftBreak>(ProblemData const &data);
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_SHIFTBREAK_H
