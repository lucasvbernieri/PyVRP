// break-free-duration (PYVRP_BREAK_FREE_DURATION): a served break's service
// leaves the duration cost when the switch is on, exactly as waiting does;
// off, the cost is the old ``unitDurationCost * (duration - waiting)``.
//
// The switch is read once at load time, so this executable is registered
// twice in meson.build (default environment, on since default flipped, and
// PYVRP_BREAK_FREE_DURATION=0) and derives its expectations from
// ``pyvrp::search::breakFreeDuration``, the value actually in force.
//
// Checks, on a D5-absorbing overnight rest (the production shape):
//   1. Route::update(): durationCost == unit * (duration - waiting - S) with
//      S = the served break's D5-EXTENDED service (S == 0 off the switch),
//      and breakService() reports S.
//   2. Proposal parity: the proposal that INSERTS the break into the
//      break-less route prices exactly like the updated route with the break,
//      and the proposal that REMOVES it prices exactly like the break-less
//      route -- so CostEvaluator::deltaCost equals the update() difference.
//   3. Solution-level parity: penalisedCost(search::Solution) equals
//      cost(unloaded Solution), i.e. the solution-level Route applies the
//      same rule.
#include "CostEvaluator.h"
#include "CustomBreak.h"
#include "DriveSegment.h"
#include "ProblemData.h"
#include "search/Route.h"
#include "search/Solution.h"

#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#define CHECK(expr)                                            \
    do                                                         \
    {                                                          \
        if (!(expr))                                           \
        {                                                      \
            std::fprintf(stderr,                               \
                         "FAIL: %s at %s:%d\n",                \
                         #expr,                                \
                         __FILE__,                             \
                         __LINE__);                            \
            std::exit(1);                                      \
        }                                                      \
    } while (0)

using pyvrp::Activity;
using pyvrp::Client;
using pyvrp::Coordinate;
using pyvrp::Cost;
using pyvrp::CostEvaluator;
using pyvrp::CustomBreak;
using pyvrp::CustomBreakReset;
using pyvrp::CustomBreakTrigger;
using pyvrp::Depot;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::DurationSegment;
using pyvrp::Load;
using pyvrp::LoadSegment;
using pyvrp::Location;
using pyvrp::Matrix;
using pyvrp::ProblemData;
using pyvrp::VehicleType;
using pyvrp::search::breakFreeDuration;
using pyvrp::search::DriveSegment;
using pyvrp::search::SegmentProxy;
using Route = pyvrp::search::Route;        // the search route, not pyvrp::Route
using Solution = pyvrp::search::Solution;  // idem

namespace
{
Duration const TRAVEL = Duration(1000);
Duration const SVC = Duration(600);
Duration const REST = Duration(3600);
Duration const MAX = std::numeric_limits<Duration>::max();

// depot -> C0..C3 (open at 0) -> [break] -> C4..C7 (open at 43200) -> depot.
// Arrival at the break is 4 x (1000 + 600) = 6400; C4 opens at 43200 and
// is 1000 away, so the served rest extends (D5) to 43200 - 1000 - 6400 =
// 35800 and the route carries no idle waiting. Fixed departure (startLate 0).
// unitDurationCost = 1, unitDistanceCost = 0 (isolates the duration term).
ProblemData const &data()
{
    static ProblemData const instance = []()
    {
        size_t const nClients = 8;
        size_t const numLoc = nClients + 1;

        std::vector<Location> locations;
        for (size_t i = 0; i != numLoc; ++i)
            locations.emplace_back(static_cast<Coordinate>(i), 0.0);

        std::vector<Client> clients;
        for (size_t i = 0; i != nClients; ++i)
            clients.emplace_back(Client(i + 1, {0}, {}, SVC,
                                        i < 4 ? Duration(0) : Duration(43200),
                                        MAX));

        std::vector<Depot> depots;
        depots.emplace_back(Depot(0, Duration(0), MAX));

        Matrix<Distance> distMat(numLoc, numLoc, Distance(0));
        Matrix<Duration> durMat(numLoc, numLoc, Duration(0));
        for (size_t i = 0; i != numLoc; ++i)
            for (size_t j = 0; j != numLoc; ++j)
                if (i != j)
                {
                    distMat(i, j) = Distance(TRAVEL.get());
                    durMat(i, j) = TRAVEL;
                }

        std::vector<CustomBreak> breaks
            = {CustomBreak(1, {}, REST, CustomBreakTrigger::DUTY_TIME,
                           Duration(1), CustomBreakReset::ALL_TIMERS, true)};

        std::vector<VehicleType> vts;
        vts.emplace_back(1, std::vector<Load>{9999}, 0, 0, 0, 0, MAX, MAX,
                         std::numeric_limits<Distance>::max(), 0, 1, 0,
                         Duration(0), std::vector<Load>{},
                         std::vector<size_t>{},
                         std::numeric_limits<size_t>::max(), 0, 0, breaks,
                         false, "vt");

        return ProblemData(std::move(locations), std::move(clients),
                           std::move(depots), std::move(vts),
                           std::vector<Matrix<Distance>>{distMat},
                           std::vector<Matrix<Duration>>{durMat});
    }();
    return instance;
}

Activity cl(size_t idx) { return {Activity::ActivityType::CLIENT, idx}; }
Activity brk(size_t id) { return {Activity::ActivityType::CUSTOM_BREAK, id}; }

// Generic single-node segment (the Proposal branch a ShiftBreak-style
// operator's BreakSegment takes), lifted from test_stream_parity.cpp.
struct NodeSeg
{
    Route const &route_;
    size_t pos_;

    NodeSeg(Route const &route, size_t pos) : route_(route), pos_(pos) {}

    Route const *route() const { return &route_; }
    SegmentProxy front() const { return route_.at(pos_).front(); }
    SegmentProxy back() const { return front(); }
    size_t size() const { return 1; }
    size_t numClients() const { return route_[pos_]->isClient() ? 1u : 0u; }
    size_t numPickups() const { return 0; }
    bool startsAtReloadDepot() const { return false; }
    bool endsAtReloadDepot() const { return false; }
    Distance distance(size_t profile) const
    {
        return route_.at(pos_).distance(profile);
    }
    DurationSegment duration(size_t profile) const
    {
        return route_.at(pos_).duration(profile);
    }
    LoadSegment load(size_t dimension) const
    {
        return route_.at(pos_).load(dimension);
    }
    DriveSegment driveState(size_t profile) const
    {
        return route_.at(pos_).driveState(profile);
    }
};

// Fills the solution's single route with the given activities (depots are
// implicit) and updates it. The node store must outlive the solution.
void fill(Solution &sol, std::vector<Activity> const &acts,
          std::vector<Route::Node> &store)
{
    store.reserve(acts.size());
    for (auto const &act : acts)
        store.emplace_back(act);
    auto &route = sol.routes[0];
    for (auto &node : store)
        route.push_back(&node);
    route.update();
}
}  // namespace

int main()
{
    bool const on = breakFreeDuration != 0;
    std::printf("test_break_free_duration -- PYVRP_BREAK_FREE_DURATION=%d\n",
                breakFreeDuration);

    auto const &dat = data();

    // Route A: with the break after C3. Route B: the same without it.
    Solution solA(dat);
    std::vector<Route::Node> storeA;
    fill(solA, {cl(0), cl(1), cl(2), cl(3), brk(1), cl(4), cl(5), cl(6), cl(7)},
         storeA);
    auto const &A = solA.routes[0];

    Solution solB(dat);
    std::vector<Route::Node> storeB;
    fill(solB, {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5), cl(6), cl(7)},
         storeB);
    auto const &B = solB.routes[0];

    // ---- 1. Route::update() ----
    Duration const extended = Duration(43200) - TRAVEL - Duration(6400);
    CHECK(extended.get() == 35800);
    CHECK(A.breakService() == extended);  // D5-extended, not the minimum
    CHECK(A.waiting().get() == 0);        // absorbed into the rest
    CHECK(A.timeWarp().get() == 0);
    CHECK(B.breakService().get() == 0);

    // A: 1000 + 4 x (600 + 1000) ... spelled out: travel 9 edges, service
    // 8 x 600, rest 35800, no waiting.
    Duration const durA = Duration(9 * 1000 + 8 * 600) + extended;
    CHECK(A.duration() == durA);
    Cost const S = on ? static_cast<Cost>(extended) : Cost(0);
    CHECK(A.durationCost() == static_cast<Cost>(durA - A.waiting()) - S);
    CHECK(A.durationCost() == static_cast<Cost>(A.duration() - A.waiting())
                                  - (on ? static_cast<Cost>(A.breakService())
                                        : Cost(0)));
    // The switch moves the cost by exactly unitDurationCost x S.
    if (on)
        CHECK(A.durationCost()
              == static_cast<Cost>(A.duration() - A.waiting()) - Cost(35800));
    else
        CHECK(A.durationCost()
              == static_cast<Cost>(A.duration() - A.waiting()));

    // B without the rest waits for C4's window instead: the same clock, all
    // of it idle. Its cost never depends on the switch.
    CHECK(B.waiting().get() == 35800);
    CHECK(B.durationCost() == static_cast<Cost>(B.duration() - B.waiting()));
    std::printf("  update(): durationCost A=%lld B=%lld breakService A=%lld\n",
                (long long)A.durationCost().get(),
                (long long)B.durationCost().get(),
                (long long)A.breakService().get());

    // ---- 2. Proposal parity (deltaCost == update() difference) ----
    CostEvaluator ev({0}, 0, 0, 0, 0);  // no wait rate: isolate the term

    // Insert the break into B after C3 (positions: depot 0, C0..C3 1..4).
    {
        Route::Proposal insert(B.before(4), NodeSeg(A, 5), B.after(5));
        auto const [cost, warp] = insert.duration();
        CHECK(warp.get() == 0);
        CHECK(cost == A.durationCost());
        CHECK(insert.waiting() == A.waiting());

        Cost delta = 0;
        CHECK(ev.deltaCost<true>(delta, insert));
        CHECK(delta == ev.penalisedCost(A) - ev.penalisedCost(B));
        std::printf("  insert proposal: cost %lld (A %lld), delta %lld\n",
                    (long long)cost.get(), (long long)A.durationCost().get(),
                    (long long)delta.get());
    }

    // Remove the break from A.
    {
        Route::Proposal remove(A.before(4), A.after(6));
        auto const [cost, warp] = remove.duration();
        CHECK(warp.get() == 0);
        CHECK(cost == B.durationCost());
        CHECK(remove.waiting() == B.waiting());

        Cost delta = 0;
        CHECK(ev.deltaCost<true>(delta, remove));
        CHECK(delta == ev.penalisedCost(B) - ev.penalisedCost(A));
        std::printf("  remove proposal: cost %lld (B %lld), delta %lld\n",
                    (long long)cost.get(), (long long)B.durationCost().get(),
                    (long long)delta.get());
    }

    // ---- 3. Solution-level parity ----
    {
        CostEvaluator evWait({0}, 0, 0, 0, 25);  // wait rate 25/s
        auto const unloadedA = solA.unload();
        auto const unloadedB = solB.unload();
        CHECK(evWait.penalisedCost(solA) == evWait.cost(unloadedA));
        // B never serves the mandatory rest, so its unloaded solution is
        // infeasible and cost() is not defined for it: compare penalised.
        CHECK(evWait.penalisedCost(solB) == evWait.penalisedCost(unloadedB));
        CHECK(unloadedA.routes()[0].durationCost() == A.durationCost());
        std::printf("  unload parity: A %lld, B %lld\n",
                    (long long)evWait.cost(unloadedA).get(),
                    (long long)evWait.penalisedCost(unloadedB).get());
    }

    std::printf("test_break_free_duration: PASS (switch %s)\n",
                on ? "on" : "off");
    return 0;
}
