#include "CostEvaluator.h"

#include <stdexcept>

using pyvrp::CostEvaluator;

CostEvaluator::CostEvaluator(std::vector<double> loadPenalties,
                             double twPenalty,
                             double distPenalty,
                             double breakDuePenalty,
                             double waitCostRate)
    : loadPenalties_(std::move(loadPenalties)),
      twPenalty_(twPenalty),
      distPenalty_(distPenalty),
      breakDuePenalty_(breakDuePenalty),
      waitCostRate_(waitCostRate)
{
    for (auto const penalty : loadPenalties_)
        if (penalty < 0)
            throw std::invalid_argument("load_penalties must be >= 0.");

    if (twPenalty_ < 0)
        throw std::invalid_argument("tw_penalty must be >= 0.");

    if (distPenalty_ < 0)
        throw std::invalid_argument("dist_penalty must be >= 0.");

    if (breakDuePenalty_ < 0)
        throw std::invalid_argument("break_due_penalty must be >= 0.");

    if (waitCostRate_ < 0)
        throw std::invalid_argument("wait_cost_rate must be >= 0.");
}
