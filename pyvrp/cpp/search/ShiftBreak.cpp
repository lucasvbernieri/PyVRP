#include "ShiftBreak.h"

#include <cassert>
#include <limits>
#include <utility>

using pyvrp::search::ShiftBreak;

namespace
{
/**
 * A lightweight segment that represents a CUSTOM_BREAK activity at a proposed
 * position in the route. The break's location is inherited from the node
 * immediately preceding it in the proposal chain, ensuring zero travel cost
 * from the predecessor.
 */
struct BreakSegment
{
    pyvrp::search::Route const &route_;
    size_t breakPos_;  // position of the break node in the route (for dur/drive)
    size_t predPos_;   // position of the predecessor in the proposal chain

    BreakSegment(pyvrp::search::Route const &route,
                 size_t breakPos,
                 size_t predPos)
        : route_(route), breakPos_(breakPos), predPos_(predPos)
    {
    }

    pyvrp::search::Route const *route() const { return &route_; }

    pyvrp::search::SegmentProxy front() const
    {
        // Break inherits predecessor's location (zero travel).
        auto const predProxy = route_.at(predPos_).front();
        return {route_[breakPos_]->activity(), predProxy.location()};
    }

    pyvrp::search::SegmentProxy back() const { return front(); }

    size_t size() const { return 1; }
    size_t numClients() const { return 0; }

    bool startsAtReloadDepot() const { return false; }
    bool endsAtReloadDepot() const { return false; }

    pyvrp::Distance distance([[maybe_unused]] size_t profile) const
    {
        return 0;
    }

    pyvrp::DurationSegment duration([[maybe_unused]] size_t profile) const
    {
        return route_.at(breakPos_).duration(profile);
    }

    pyvrp::LoadSegment load(size_t dimension) const
    {
        (void)dimension;
        return {};
    }

    pyvrp::search::DriveSegment driveState(
        [[maybe_unused]] size_t profile) const
    {
        return route_.between(breakPos_, breakPos_).driveState(profile);
    }
};
}  // namespace

std::pair<pyvrp::Cost, bool>
ShiftBreak::evaluate(Route::Node *U, CostEvaluator const &costEvaluator)
{
    stats_.numEvaluations++;

    // U must be a CUSTOM_BREAK in a route.
    if (!U->route() || !U->isCustomBreak())
        return std::make_pair(0, false);

    auto *route = U->route();
    auto const curPos = U->pos();
    auto const n = route->size();

    // Minimum route length: start depot + end depot + at least one other node.
    if (n < 3)
        return std::make_pair(0, false);

    Cost bestCost = 0;  // only strictly improving moves (delta < 0)
    bestPos_ = curPos;

    // Scan candidate positions. Break must have at least one client before it
    // (pos > 1) and not be at/adjacent to depots.
    for (size_t pos = 2; pos <= n - 2; ++pos)
    {
        if (pos == curPos)
            continue;

        // Guards: break not adjacent to another break or depot.
        auto const *prevNode = route->operator[](pos - 1);
        auto const *atNode = route->operator[](pos);

        if (prevNode->isCustomBreak() || atNode->isCustomBreak())
            continue;
        if (prevNode->isDepot() || atNode->isDepot())
            continue;

        Cost deltaCost = 0;

        if (pos < curPos)
        {
            // Proposed route: [0..pos-1] + break + [pos..curPos-1] + [curPos+1..n-1]
            auto brk = BreakSegment(*route, curPos, pos - 1);
            costEvaluator.deltaCost<true>(
                deltaCost,
                Route::Proposal(route->before(pos - 1),
                                std::move(brk),
                                route->between(pos, curPos - 1),
                                route->after(curPos + 1)));
        }
        else
        {
            // pos > curPos
            if (pos == curPos + 1)
            {
                // Proposed: [0..curPos-1] + break + [curPos+1..n-1]
                auto brk = BreakSegment(*route, curPos, curPos - 1);
                costEvaluator.deltaCost<true>(
                    deltaCost,
                    Route::Proposal(route->before(curPos - 1),
                                    std::move(brk),
                                    route->after(curPos + 1)));
            }
            else
            {
                // pos > curPos + 1
                // Proposed: [0..curPos-1] + [curPos+1..pos-1] + break + [pos..n-1]
                auto brk = BreakSegment(*route, curPos, pos - 1);
                costEvaluator.deltaCost<true>(
                    deltaCost,
                    Route::Proposal(route->before(curPos - 1),
                                    route->between(curPos + 1, pos - 1),
                                    std::move(brk),
                                    route->after(pos)));
            }
        }

        if (deltaCost < bestCost)
        {
            bestCost = deltaCost;
            bestPos_ = pos;
        }
    }

    return std::make_pair(bestCost, bestCost < 0);
}

void ShiftBreak::apply(Route::Node *U) const
{
    stats_.numApplications++;

    auto *route = U->route();
    route->remove(U->pos());
    route->insert(bestPos_, U);
}

std::string ShiftBreak::name() const { return "ShiftBreak"; }

template <>
bool pyvrp::search::supports<ShiftBreak>(ProblemData const &data)
{
    // ShiftBreak is only useful when at least one vehicle type has break rules.
    for (auto const &vehType : data.vehicleTypes())
        if (vehType.hasBreaks())
            return true;

    return false;
}
