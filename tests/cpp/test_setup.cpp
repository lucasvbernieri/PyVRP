#include "ProblemData.h"
#include "DriveSegment.h"
#include "search/Route.h"
#include "search/Solution.h"

#include <cassert>
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

using namespace pyvrp;
using namespace pyvrp::search;

namespace
{
Client makeClient(size_t loc,
                  Duration service = 0,
                  Duration twEarly = 0,
                  Duration twLate = std::numeric_limits<Duration>::max())
{
    return Client(loc, {0}, {}, service, twEarly, twLate);
}

CustomBreak makeDriveBreak(size_t id, Duration triggerValue)
{
    return CustomBreak(id,
                       {},                    // tws
                       Duration(0),           // service
                       CustomBreakTrigger::DRIVE_TIME,
                       triggerValue,
                       CustomBreakReset::DRIVE_TIMER,
                       true);                 // mandatory
}

ProblemData makeData(std::vector<Client> clients,
                     std::vector<Duration> setup,
                     size_t numLocations,
                     size_t depotLoc = 0,
                     std::vector<CustomBreak> breaks = {})
{
    std::vector<Location> locations;
    for (size_t i = 0; i != numLocations; ++i)
        locations.emplace_back(static_cast<Coordinate>(i), 0.0);

    std::vector<Depot> depots;
    depots.emplace_back(depotLoc);

    Matrix<Distance> distMat(numLocations, numLocations, 0);
    Matrix<Duration> durMat(numLocations, numLocations, 0);
    for (size_t i = 0; i != numLocations; ++i)
        for (size_t j = 0; j != numLocations; ++j)
            if (i != j)
            {
                distMat(i, j) = 10;
                durMat(i, j) = 10;
            }

    std::vector<VehicleType> vts;
    vts.emplace_back(1,                          // numAvailable
                     std::vector<Load>{1},       // capacity
                     0,                          // startDepot
                     0,                          // endDepot
                     0,                          // fixedCost
                     0,                          // twEarly
                     std::numeric_limits<Duration>::max(),  // twLate
                     std::numeric_limits<Duration>::max(),  // shiftDuration
                     std::numeric_limits<Distance>::max(),  // maxDistance
                     0,                          // unitDistanceCost
                     0,                          // unitDurationCost
                     0,                          // profile
                     std::nullopt,               // startLate
                     std::vector<Load>{},        // initialLoad
                     std::vector<size_t>{},      // reloadDepots
                     std::numeric_limits<size_t>::max(),  // maxReloads
                     0,                          // maxOvertime
                     0,                          // unitOvertimeCost
                     std::move(breaks),          // custom_breaks
                     false,                      // reset_breaks_at_reload
                     "vt");

    return ProblemData(std::move(locations),
                       std::move(clients),
                       std::move(depots),
                       std::move(vts),
                       std::vector<Matrix<Distance>>{distMat},
                       std::vector<Matrix<Duration>>{durMat},
                       {},
                       std::move(setup));
}

ForwardEvalResult eval(ProblemData const &data,
                       VehicleType const &vt,
                       std::vector<Activity> const &acts,
                       std::vector<size_t> const &locs)
{
    std::vector<Duration> atSecond(acts.size());
    return evaluateForwardPass(acts, locs, atSecond, nullptr, data, vt);
}

// depot(0) and end depot(0) wrappers for conciseness.
Activity DEPOT = {Activity::ActivityType::DEPOT, 0};

void test_contiguous_block_pays_once()
{
    // locs: depot=0, A=1, B=2. setup[A]=100, setup[B]=100.
    auto data = makeData({makeClient(1), makeClient(1), makeClient(2)},
                         {0, 100, 100},
                         3);
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0},
        {Activity::ActivityType::CLIENT, 1},
        {Activity::ActivityType::CLIENT, 2}, DEPOT};
    std::vector<size_t> locs = {0, 1, 1, 2, 0};

    auto res = eval(data, vt, acts, locs);
    // travel 10+0+10+10 = 30; setup: c0=100, c1=0, c2=100 -> 200
    CHECK(res.duration.get() == 230);
    CHECK(res.timeWarp.get() == 0);
    std::printf("  test_contiguous_block_pays_once: PASS\n");
}

void test_non_contiguous_revisit_pays_again()
{
    auto data = makeData({makeClient(1), makeClient(2), makeClient(1)},
                         {0, 100, 100},
                         3);
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0},
        {Activity::ActivityType::CLIENT, 1},
        {Activity::ActivityType::CLIENT, 2}, DEPOT};
    std::vector<size_t> locs = {0, 1, 2, 1, 0};

    auto res = eval(data, vt, acts, locs);
    // travel 40; setup c0=100, c1=100, c2=100 -> 300
    CHECK(res.duration.get() == 340);
    std::printf("  test_non_contiguous_revisit_pays_again: PASS\n");
}

void test_first_client_start_same_location_skips()
{
    // Depot and client both at location 1 -> first client pays no setup.
    auto data = makeData({makeClient(1)}, {0, 100}, 2, /*depotLoc=*/1);
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0}, DEPOT};
    std::vector<size_t> locs = {1, 1, 1};

    auto res = eval(data, vt, acts, locs);
    CHECK(res.duration.get() == 0);  // no travel, no setup
    std::printf("  test_first_client_start_same_location_skips: PASS\n");
}

void test_first_client_depot_differs_charges()
{
    // Depot at 0, client at 1 -> first client pays setup.
    auto data = makeData({makeClient(1)}, {0, 100}, 2);
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0}, DEPOT};
    std::vector<size_t> locs = {0, 1, 0};

    auto res = eval(data, vt, acts, locs);
    CHECK(res.duration.get() == 120);  // travel 20 + setup 100
    std::printf("  test_first_client_depot_differs_charges: PASS\n");
}

void test_break_between_same_location_no_reapply()
{
    // c0@1, BREAK (inherits loc 1), c1@1: break does not reset the block, so
    // c1 does not re-pay setup.
    auto data = makeData({makeClient(1), makeClient(1)},
                         {0, 100},
                         2,
                         0,
                         {makeDriveBreak(0, 10000)});
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0},
        {Activity::ActivityType::CUSTOM_BREAK, 0},
        {Activity::ActivityType::CLIENT, 1}, DEPOT};
    std::vector<size_t> locs = {0, 1, 1, 1, 0};  // break inherits loc 1

    auto res = eval(data, vt, acts, locs);
    // travel 10+0+0+10 = 20; setup c0=100, c1=0 -> 100
    CHECK(res.duration.get() == 120);
    std::printf("  test_break_between_same_location_no_reapply: PASS\n");
}

void test_break_at_route_start()
{
    // BREAK at route start (inherits depot loc 0), then c0@1: first client
    // still pays setup relative to the depot.
    auto data = makeData({makeClient(1)}, {0, 100}, 2, 0,
                         {makeDriveBreak(0, 10000)});
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CUSTOM_BREAK, 0},
        {Activity::ActivityType::CLIENT, 0}, DEPOT};
    std::vector<size_t> locs = {0, 0, 1, 0};  // break inherits depot loc 0

    auto res = eval(data, vt, acts, locs);
    // travel 0+10+10 = 20; setup c0=100
    CHECK(res.duration.get() == 120);
    std::printf("  test_break_at_route_start: PASS\n");
}

void test_tw_invariant_setup_starts_in_window()
{
    // c0@1 with tw [50, 100], setup 100: feasible (arrival 10 <= 100); setup
    // starts at max(10, 50) = 50, ending at 150 (after the window). Setup must
    // NOT shift the feasibility check.
    auto data = makeData({makeClient(1, 0, 50, 100)}, {0, 100}, 2);
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0}, DEPOT};
    std::vector<size_t> locs = {0, 1, 0};

    auto res = eval(data, vt, acts, locs);
    // Minimal-duration semantics: departure is delayed so the vehicle arrives
    // at tw_early (50); no forced wait. duration = travel 20 + setup 100.
    // The key invariant is timeWarp == 0: setup starts at max(arrival,
    // tw_start) and ends after the window, but feasibility is arrival <=
    // tw_end.
    CHECK(res.duration.get() == 120);
    CHECK(res.timeWarp.get() == 0);

    // Tight tw_late (5): arrival 10 > 5 -> infeasible (timeWarp > 0).
    auto tight = makeData({makeClient(1, 0, 0, 5)}, {0, 100}, 2);
    auto resTight = eval(tight, tight.vehicleType(0), acts, locs);
    CHECK(resTight.timeWarp.get() > 0);

    std::printf("  test_tw_invariant_setup_starts_in_window: PASS\n");
}

void test_setup_never_in_drive()
{
    // Direct DriveSegment::merge check: extraWork adds to work/duty, never
    // drive.
    {
        DriveSegment first(10, 10, 10, 0, 0, 0);
        DriveSegment second(10, 10, 10, 0, 0, 0);
        auto m = DriveSegment::merge(Duration(10), first, second, {},
                                     Duration(0), 0, Duration(50));
        CHECK(m.driveTime_ == 30);  // 10+10+10; extraWork NOT in drive
        CHECK(m.workTime_ == 80);   // 10+10+50+10
        CHECK(m.dutyTime_ == 80);
    }

    // Route-level: DRIVE_TIME break trigger 25, route drive = 20 (2 edges),
    // setup = 100. breakDue must stay 0: setup never counts as drive.
    auto data = makeData({makeClient(1)}, {0, 100}, 2, 0,
                         {makeDriveBreak(0, 25)});
    auto const &vt = data.vehicleType(0);

    std::vector<Activity> acts = {
        DEPOT, {Activity::ActivityType::CLIENT, 0}, DEPOT};
    std::vector<size_t> locs = {0, 1, 0};

    auto res = eval(data, vt, acts, locs);
    CHECK(res.breakDue == 0);

    std::printf("  test_setup_never_in_drive: PASS\n");
}

void test_neighbor_recharges_after_block_removal()
{
    // VROOM #754 class: removing the middle client re-computes the neighbor's
    // setup (its predecessor changed). Asymmetric setup makes this observable.
    //   locs: depot=0, A=1, B=2. setup[A]=100, setup[B]=50.
    auto data = makeData({makeClient(1), makeClient(2), makeClient(1)},
                         {0, 100, 50},
                         3);
    auto const &vt = data.vehicleType(0);

    // Before removal: c0@A, c1@B, c2@A -> c0=100, c1=50, c2=100 = 250.
    {
        std::vector<Activity> acts = {
            DEPOT, {Activity::ActivityType::CLIENT, 0},
            {Activity::ActivityType::CLIENT, 1},
            {Activity::ActivityType::CLIENT, 2}, DEPOT};
        std::vector<size_t> locs = {0, 1, 2, 1, 0};
        auto res = eval(data, vt, acts, locs);
        // travel 40 + setup 250
        CHECK(res.duration.get() == 290);
    }

    // After removal (c1 removed): c0@A, c2@A -> c0=100, c2=0 = 100.
    {
        std::vector<Activity> acts = {
            DEPOT, {Activity::ActivityType::CLIENT, 0},
            {Activity::ActivityType::CLIENT, 2}, DEPOT};
        std::vector<size_t> locs = {0, 1, 1, 0};
        auto res = eval(data, vt, acts, locs);
        // travel 10+0+10 = 20 + setup 100
        CHECK(res.duration.get() == 120);
    }

    std::printf("  test_neighbor_recharges_after_block_removal: PASS\n");
}

void test_contiguity_tiebreak()
{
    // D8: prefersContiguity must prefer an insertion position adjacent to a
    // same-location client. Clients 0 and 1 at loc 1, client 2 at loc 2.
    auto data = makeData({makeClient(1), makeClient(1), makeClient(2)},
                         {0, 100, 100},
                         3);

    pyvrp::search::Route::Node n0(Activity::ActivityType::CLIENT, 0);
    pyvrp::search::Route::Node n2(Activity::ActivityType::CLIENT, 2);
    pyvrp::search::Route::Node inserted(
        Activity::ActivityType::CLIENT, 1);  // client 1 @ loc 1

    pyvrp::search::Route route(data, 0);
    route.push_back(&n0);  // route: depot -> c0(1) -> c2(2) -> depot
    route.push_back(&n2);
    route.update();

    // Inserting after c0 (loc 1) places client 1 next to a same-location
    // client -> contiguous.
    CHECK(prefersContiguity(&inserted, &n0, data));

    // Inserting after c2 (loc 2) is not contiguous (successor is end depot).
    CHECK(!prefersContiguity(&inserted, &n2, data));

    std::printf("  test_contiguity_tiebreak: PASS\n");
}
}  // namespace

int main()
{
    std::printf("Running setup primitive tests...\n");

    test_contiguous_block_pays_once();
    test_non_contiguous_revisit_pays_again();
    test_first_client_start_same_location_skips();
    test_first_client_depot_differs_charges();
    test_break_between_same_location_no_reapply();
    test_break_at_route_start();
    test_tw_invariant_setup_starts_in_window();
    test_setup_never_in_drive();
    test_neighbor_recharges_after_block_removal();
    test_contiguity_tiebreak();

    std::printf("All setup primitive tests PASSED.\n");
    return 0;
}
