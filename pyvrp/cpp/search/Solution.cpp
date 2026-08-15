#include "Solution.h"

#include "ClientSegment.h"

#include <algorithm>
#include <cassert>
#include <iterator>

using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;

using pyvrp::search::Solution;

namespace
{
Cost insertCost(pyvrp::search::Route::Node *U,
                pyvrp::search::Route::Node *V,
                pyvrp::ProblemData const &data,
                pyvrp::CostEvaluator const &costEvaluator)
{
    assert(V->route() && U->isClient());

    auto *route = V->route();
    auto const &client = data.client(U->idx());

    Cost deltaCost
        = Cost(route->empty()) * route->fixedVehicleCost() - client.prize;

    costEvaluator.deltaCost<true>(
        deltaCost,
        pyvrp::search::Route::Proposal(
            route->before(V->pos()),
            pyvrp::search::ClientSegment(data, U->idx()),
            route->after(V->pos() + 1)));

    return deltaCost;
}

// Comparison operator to determine if pyvrp::Route and search::Route are
// equivalent - if so, the pyvrp::Route does not need to be loaded.
bool operator==(pyvrp::Route const &pyvrp, pyvrp::search::Route const &search)
{
    // clang-format off
    bool const simpleChecks = pyvrp.distance() == search.distance()
                              && pyvrp.duration() == search.duration()
                              && pyvrp.timeWarp() == search.timeWarp()
                              && pyvrp.vehicleType() == search.vehicleType()
                              && pyvrp.size() == search.size();
    // clang-format on

    if (!simpleChecks)
        return false;

    size_t idx = 0;
    for (auto const &activity : pyvrp)
        if (search[idx++]->activity() != activity)
            return false;

    return true;
}
}  // namespace

// D8 contiguity tie-break: prefer insertion positions that place a client
// adjacent (immediately before or after) to another client at the same
// location. See the declaration in Solution.h for the rationale.
bool pyvrp::search::prefersContiguity(Route::Node *client,
                                      Route::Node *insertAfter,
                                      ProblemData const &data)
{
    auto const loc = data.client(client->idx()).location;

    if (insertAfter->isClient()
        && data.client(insertAfter->idx()).location == loc)
        return true;

    auto const *successor = n(insertAfter);
    return successor->isClient()
           && data.client(successor->idx()).location == loc;
}

Solution::Solution(ProblemData const &data) : data_(data)
{
    nodes.reserve(data.numClients());
    for (size_t loc = 0; loc != data.numClients(); ++loc)
        nodes.emplace_back(Activity::ActivityType::CLIENT, loc);

    routes.reserve(data.numVehicles());
    for (size_t vehType = 0; vehType != data.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data.vehicleType(vehType).numAvailable;
        for (size_t vehicle = 0; vehicle != numAvailable; ++vehicle)
            routes.emplace_back(data, vehType);
    }
}

void Solution::load(pyvrp::Solution const &solution)
{
    // Determine offsets for vehicle types.
    std::vector<size_t> vehicleOffset(data_.numVehicleTypes(), 0);
    for (size_t vehType = 1; vehType < data_.numVehicleTypes(); vehType++)
    {
        auto const prevAvail = data_.vehicleType(vehType - 1).numAvailable;
        vehicleOffset[vehType] = vehicleOffset[vehType - 1] + prevAvail;
    }

    for (auto const &solRoute : solution.routes())
    {
        // Determine index of next route of this type to load, where we rely
        // on solution to be valid to not exceed the number of vehicles per
        // vehicle type.
        auto const idx = vehicleOffset[solRoute.vehicleType()]++;
        auto &route = routes[idx];

        if (route == solRoute)  // then the current route is still OK and we
            continue;           // can skip inserting and updating

        // Else we need to clear the route and insert the updated route from
        // the solution.
        route.clear();

        route.reserve(solRoute.size());
        for (size_t idx = 1; idx != solRoute.size() - 1; ++idx)
        {
            auto const &activity = solRoute[idx];
            if (activity.isDepot())
            {
                Route::Node depot = activity;
                route.push_back(&depot);
            }
            else if (activity.isCustomBreak())
            {
                // CUSTOM_BREAK: insert as an owned break node into the route.
                Route::Node breakNode = activity;
                route.push_back(&breakNode);
        }
        else
        {
            assert(activity.isClient());
            route.push_back(&nodes[activity.idx()]);
        }
    }

    route.update();

        // Warm-start auto-insertion: if this route has break rules
        // configured but no CUSTOM_BREAK activities are present yet,
        // insert break nodes at positions that satisfy each break's
        // time window (relative to the vehicle departure time).
        // The solver's ShiftBreak operator will reposition them
        // optimally during local search.
        if (route.hasBreaks())
        {
            bool hasBreakNodes = false;
            for (size_t i = 1; i < route.size() - 1; ++i)
                if (route[i]->isCustomBreak())
                {
                    hasBreakNodes = true;
                    break;
                }

            if (!hasBreakNodes)
            {
                auto const &breaks
                    = data_.vehicleType(route.vehicleType()).custom_breaks;

                size_t const innerLen = route.size() - 2;

                // Insert at strictly increasing, unique interior positions:
                // pos = max(raw, lastPos + 1), clamped to the interior range.
                // Edge case: with innerLen == 0 (route with no clients) the
                // clamp yields pos == 0 (depot) — an unreal scenario; the
                // debug assertion below detects it.
                size_t lastPos = 0;
                for (size_t b = 0; b != breaks.size(); ++b)
                {
                    auto const brk = breaks[b];
                    Route::Node breakNode(
                        Activity::ActivityType::CUSTOM_BREAK, brk.id);

                    auto const raw
                        = 1
                          + (innerLen * (b + 1)) / (breaks.size() + 1);
                    auto const pos = std::min(std::max(raw, lastPos + 1),
                                              route.size() - 2);
                    assert(pos > lastPos
                           && "warm-start break insertion position must be "
                              "strictly increasing (no duplicates)");
                    route.insert(pos, &breakNode);
                    lastPos = pos;
                }

                route.update();
            }
        }
    }

    // Finally, we clear any routes that we have not re-used or inserted from
    // the solution.
    size_t firstOfType = 0;
    for (size_t vehType = 0; vehType != data_.numVehicleTypes(); ++vehType)
    {
        auto const numAvailable = data_.vehicleType(vehType).numAvailable;
        auto const firstOfNextType = firstOfType + numAvailable;
        for (size_t idx = vehicleOffset[vehType]; idx != firstOfNextType; ++idx)
            routes[idx].clear();

        firstOfType = firstOfNextType;
    }
}

pyvrp::Solution Solution::unload() const
{
    std::vector<pyvrp::Route> solRoutes;
    solRoutes.reserve(data_.numVehicles());

    for (auto const &route : routes)
    {
        if (route.empty())
            continue;

        std::vector<Activity> activities;
        activities.reserve(route.size());

        for (size_t idx = 1; idx != route.size() - 1; ++idx)
            activities.emplace_back(route[idx]->activity());

        auto &solRoute = solRoutes.emplace_back(
            data_, std::move(activities), route.vehicleType());

        // Propagate breakDue from the search route to the output route.
        solRoute.setBreakDue(route.breakDue());

        // Propagate served-break ids from the search route to the output
        // route (gate decision: only breaks actually served are exposed).
        solRoute.setBreaksServed(route.breaksServed());
    }

    return {data_, std::move(solRoutes)};
}

bool Solution::insert(Route::Node *U,
                      SearchSpace const &searchSpace,
                      CostEvaluator const &costEvaluator,
                      bool required)
{
    assert(U->isClient());
    assert(size_t(std::distance(nodes.data(), U)) < nodes.size());

    Route::Node *UAfter = routes[0][0];  // fallback option
    auto bestCost = insertCost(U, UAfter, data_, costEvaluator);

    // First attempt a neighbourhood search to place U into routes that are
    // already in use.
    for (auto const vClient : searchSpace.neighboursOf(U->idx()))
    {
        auto *V = &nodes[vClient];

        if (!V->route())
            continue;

        auto const cost = insertCost(U, V, data_, costEvaluator);
        // D8 tie-break: on equal cost, prefer a position contiguous with a
        // same-location client (deterministic secondary criterion).
        if (cost < bestCost
            || (cost == bestCost && prefersContiguity(U, V, data_)
                && !prefersContiguity(U, UAfter, data_)))
        {
            bestCost = cost;
            UAfter = V;
        }
    }

    // Next consider empty routes, of each vehicle type. We insert into the
    // first improving route.
    for (auto const &[vehType, offset] : searchSpace.vehTypeOrder())
    {
        auto const begin = routes.begin() + offset;
        auto const end = begin + data_.vehicleType(vehType).numAvailable;
        auto const pred = [](auto const &route) { return route.empty(); };
        auto empty = std::find_if(begin, end, pred);

        if (empty == end)
            continue;

        auto const cost = insertCost(U, (*empty)[0], data_, costEvaluator);
        if (cost < bestCost
            || (cost == bestCost && prefersContiguity(U, (*empty)[0], data_)
                && !prefersContiguity(U, UAfter, data_)))
        {
            bestCost = cost;
            UAfter = (*empty)[0];
            break;
        }
    }

    if (required || bestCost < 0)
    {
        auto *route = UAfter->route();
        route->insert(UAfter->pos() + 1, U);
        return true;
    }

    return false;
}

template <>
pyvrp::Cost pyvrp::CostEvaluator::penalisedCost(
    pyvrp::search::Solution const &solution) const
{
    auto const &data = solution.data_;

    Cost cost = 0;  // cost is route cost + uncollected prizes
    for (size_t idx = 0; idx != data.numClients(); ++idx)
        if (!solution.nodes[idx].route())
            cost += data.client(idx).prize;

    for (auto const &route : solution.routes)
        cost += penalisedCost(route);

    return cost;
}
