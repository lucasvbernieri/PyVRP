#include "PerturbationManager.h"

#include "DynamicBitset.h"

#include <cassert>
#include <stdexcept>
#include <utility>
#include <vector>

using pyvrp::search::PerturbationManager;
using pyvrp::search::PerturbationParams;
using pyvrp::search::Route;

PerturbationParams::PerturbationParams(size_t minPerturbations,
                                       size_t maxPerturbations)
    : minPerturbations(minPerturbations), maxPerturbations(maxPerturbations)
{
    if (minPerturbations > maxPerturbations)
        throw std::invalid_argument(
            "min_perturbations must be <= max_perturbations.");
}

PerturbationManager::PerturbationManager(ProblemData const &data,
                                         PerturbationParams params)
    : data_(data), params_(params), numPerturbations_(params_.minPerturbations)
{
}

size_t PerturbationManager::numPerturbations() const
{
    return numPerturbations_;
}

void PerturbationManager::shuffle(RandomNumberGenerator &rng)
{
    auto const range = params_.maxPerturbations - params_.minPerturbations;
    numPerturbations_ = params_.minPerturbations + rng.randint(range + 1);
}

void PerturbationManager::perturb(Solution &solution,
                                  SearchSpace &searchSpace,
                                  CostEvaluator const &costEvaluator) const
{
    size_t movesLeft = numPerturbations_;

    if (!movesLeft)  // nothing to do
        return;

    // Clear the set of promising nodes. Perturbation determines the initial
    // set of promising nodes for further (local search) improvement.
    searchSpace.unmarkAllPromising();

    DynamicBitset perturbed
        = {solution.clients.size() + solution.shipments.size()};
    auto const perturb = [&](Route::Node *node, bool const remove)
    {
        assert(node->isClient() || node->isPickup());
        auto const idx = node->isClient()
                             ? node->idx()
                             : solution.clients.size() + node->idx();

        // This node has already been touched by a previous perturbation, so
        // we skip it here.
        if (perturbed[idx])
            return;

        // Remove if node is in a route and we are currently removing.
        auto *route = node->route();
        if (route && remove)
        {
            searchSpace.markPromising(node);
            route->remove(node->pos());

            if (node->isPickup())  // then we also remove the associated
            {                      // delivery node
                auto const *delivery = node + 1;
                assert(delivery->route() == route);

                searchSpace.markPromising(delivery);
                route->remove(delivery->pos());
            }

            route->update();
        }
        // Insert if node is not in a route and we are currently inserting.
        else if (!route && !remove)
        {
            if (node->isClient())
            {
                auto const &client = data_.client(node->idx());
                if (client.group)
                    // If another group member is already planned, we first
                    // remove it before inserting the client.
                    for (auto const member : data_.group(*client.group))
                    {
                        auto *other = &solution.clients[member];
                        if (other->route())
                        {
                            // Remove the planned group member without counting
                            // it as an additional perturbation.
                            auto *otherRoute = other->route();
                            searchSpace.markPromising(other);
                            otherRoute->remove(other->pos());
                            otherRoute->update();
                            break;
                        }
                    }

                solution.insert(node, searchSpace, costEvaluator, true);
                node->route()->update();
                searchSpace.markPromising(node);
            }
            else if (node->isPickup())
            {
                auto *pickup = node;
                auto *delivery = node + 1;
                solution.insert(
                    pickup, delivery, searchSpace, costEvaluator, true);

                auto *route = pickup->route();
                assert(delivery->route() == route);

                route->update();
                searchSpace.markPromising(pickup);
                searchSpace.markPromising(delivery);
            }
        }
        else  // no-op
            return;

        perturbed[idx] = true;
        movesLeft--;
    };

    // We do numPerturbations if we can. We perturb the local neighbourhood of
    // randomly selected clients or pickups U: if U is in the solution, we
    // remove it and its neighbours, while if it is not, we try to insert
    // instead. Each removal or insertion counts as one perturbation.
    for (auto const &uActivity : searchSpace.activityOrder())
    {
        Route::Node *U = solution[uActivity];
        assert(U);

        auto const remove = U->route() != nullptr;
        perturb(U, remove);

        if (!movesLeft)
            return;

        for (auto const &vActivity : searchSpace.neighboursOf(uActivity))
        {
            Route::Node *V = nullptr;
            switch (vActivity.type())
            {
            case Activity::ActivityType::CLIENT:
                V = &solution.clients[vActivity.idx()];
                break;

            case Activity::ActivityType::PICKUP:  // with shipments we perturb
                [[fallthrough]];                  // the pickup, not delivery
            case Activity::ActivityType::DELIVERY:
                V = &solution.shipments[vActivity.idx()].first;
                break;

            default:
                continue;
            }

            assert(V);
            perturb(V, remove);

            if (!movesLeft)
                return;
        }
    }
}
