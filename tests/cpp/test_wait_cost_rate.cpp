#include "CostEvaluator.h"
#include "DurationSegment.h"
#include "ProblemData.h"
#include "search/Route.h"
#include "search/Solution.h"

// wait-cost-root-fix fork regression test (task 2.7).
//
// Run with: meson test -C <builddir> --suite cpp
// (registered in meson.build; `meson test ... test_wait_cost_rate` runs just this one)

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
using pyvrp::CostEvaluator;
using pyvrp::Depot;
using pyvrp::Client;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::DurationSegment;
using pyvrp::Location;
using pyvrp::Matrix;
using pyvrp::ProblemData;
using pyvrp::VehicleType;
using pyvrp::search::Route;
using pyvrp::search::Solution;

// ---------------------------------------------------------------------------
// Instance: depot -> C0 -> C1 -> depot. TRAVEL = 1000, SVC = 600.
// C1's window opens at 5000 and the vehicle departs at time 0 (startLate=0),
// so arrival at C1 is 2600 and the route carries 2400 units of waiting.
// unitDistanceCost = 0 isolates the duration/wait terms; unitDurationCost = 1.
//
// The ProblemData must OUTLIVE every Solution built from it (search::Solution
// keeps a reference into the data), so it lives in a static.
// ---------------------------------------------------------------------------
static ProblemData const &data()
{
    static ProblemData const instance = []()
    {
        std::vector<Location> locs = {
            Location(0, 0), Location(1, 0), Location(2, 0)};

        Depot depot(0, Duration(0), Duration(200'000));
        Client c0(1, {0}, {0}, Duration(600), Duration(0), Duration(200'000));
        Client c1(2, {0}, {0}, Duration(600), Duration(5'000),
                  Duration(200'000));

        VehicleType vt(1,                                  // numAvailable
                       {9999},                             // capacity
                       0,                                  // startDepot
                       0,                                  // endDepot
                       0,                                  // fixedCost
                       Duration(0),                        // twEarly
                       Duration(200'000),                  // twLate
                       std::numeric_limits<Duration>::max(),  // shiftDuration
                       std::numeric_limits<Distance>::max(),  // maxDistance
                       0,                                  // unitDistanceCost
                       1,                                  // unitDurationCost
                       0,                                  // profile
                       Duration(0));                       // startLate (fixed)

        Matrix<Distance> distMats(3, 3, 0);
        Matrix<Duration> durMats(3, 3, 0);
        for (size_t i = 0; i != 3; ++i)
            for (size_t j = 0; j != 3; ++j)
                if (i != j)
                    durMats(i, j) = 1000;

        return ProblemData(
            locs, {c0, c1}, {depot}, {vt}, {distMats}, {durMats});
    }();

    return instance;
}

static Solution makeSearchSolution()
{
    auto const &dataRef = data();
    Solution sol(dataRef);
    auto &route = sol.routes[0];

    auto *n0 = new Route::Node(Activity::ActivityType::CLIENT, 0);
    auto *n1 = new Route::Node(Activity::ActivityType::CLIENT, 1);
    route.push_back(n0);
    route.push_back(n1);
    route.update();
    return sol;
}

static void test_closed_form_duration_cost_excludes_waiting()
{
    auto sol = makeSearchSolution();
    auto const &route = sol.routes[0];

    auto const duration = route.duration();
    auto const waiting = route.waiting();
    CHECK(waiting.get() == 2'400);
    // No overtime (shift unconstrained) and unitDurationCost = 1, so the
    // duration cost is exactly the active duration (total - waiting).
    CHECK(route.durationCost() == (duration - waiting).get());

    std::printf("  test_closed_form_duration_cost_excludes_waiting: PASS\n");
}

static void test_evaluator_closed_form_and_parity()
{
    auto sol = makeSearchSolution();
    auto const &route = sol.routes[0];

    auto const duration = route.duration();
    auto const waiting = route.waiting();

    CostEvaluator ev({0}, 0, 0, 0, 25);  // wait cost rate = 25/s

    // Closed form: active x 1 + waiting x 25 (distance 0, fixed 0, prizes 0).
    auto const expected = (duration - waiting).get() + 25 * waiting.get();
    CHECK(ev.penalisedCost(sol) == expected);

    // Parity: penalisedCost(search::Solution) == cost(unloaded Solution).
    auto const unloaded = sol.unload();
    CHECK(ev.cost(unloaded) == expected);

    std::printf("  test_evaluator_closed_form_and_parity: PASS\n");
}

static void test_proposal_waiting_negative_cache_path()
{
    auto sol = makeSearchSolution();
    auto const &route = sol.routes[0];

    // Proposal::waiting() with cache == -1 (never called duration()) must
    // fall back to the duration() fold and produce the same value as the
    // cached path — and must not recurse infinitely. Segment types are
    // private, so obtain the proposal via route accessors and decltype.
    // between(0, 1) + between(2, 3) reconstructs the FULL route
    // [start, C0, C1, end] (valid segment indices, same shape operators
    // use), whose waiting is 2400.
    auto seg1 = route.between(0, 1);
    auto seg2 = route.between(2, 3);
    Route::Proposal<decltype(seg1), decltype(seg2)> prop(
        std::move(seg1), std::move(seg2));

    auto const w1 = prop.waiting();  // negative-cache path (fold fallback)
    auto const w2 = prop.waiting();  // cached path
    CHECK(w1.get() == 2'400);
    CHECK(w2.get() == 2'400);
    CHECK(w1 == w2);

    std::printf("  test_proposal_waiting_negative_cache_path: PASS\n");
}

static void test_multitrip_boundary_waiting()
{
    // Trip 1 ends at 100 (prevEndLate = 100); trip 2 may only start at 300
    // (startEarly = releaseTime = 300). The boundary wait (200) must be part
    // of both duration() and waiting(); the active part is the 50s trip.
    DurationSegment seg(50,                                    // duration
                        0,                                     // timeWarp
                        300,                                   // startEarly
                        std::numeric_limits<Duration>::max(),   // startLate
                        300,                                   // releaseTime
                        0,                                     // cumDuration
                        0,                                     // cumTimeWarp
                        100,                                   // prevEndLate
                        0,                                     // waiting
                        0);                                    // cumWaiting

    CHECK(seg.duration().get() == 250);   // 50 + max(300 - 100, 0)
    CHECK(seg.waiting().get() == 200);    // boundary wait only
    CHECK(seg.duration().get() - seg.waiting().get() == 50);

    std::printf("  test_multitrip_boundary_waiting: PASS\n");
}

int main()
{
    test_closed_form_duration_cost_excludes_waiting();
    test_evaluator_closed_form_and_parity();
    test_proposal_waiting_negative_cache_path();
    test_multitrip_boundary_waiting();
    std::printf("All wait-cost-rate C++ tests passed.\n");
    return 0;
}
