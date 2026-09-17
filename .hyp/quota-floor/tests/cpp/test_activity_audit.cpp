#include "Activity.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <stdexcept>

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

// ---------------------------------------------------------------------------
// 1. ActivityType enumeration values
// ---------------------------------------------------------------------------

static void test_activity_type_values()
{
    // Verify the sentinel values for DEPOT, CLIENT, and CUSTOM_BREAK.
    CHECK(static_cast<int>(Activity::ActivityType::DEPOT) == 0);
    CHECK(static_cast<int>(Activity::ActivityType::CLIENT) == 1);
    CHECK(static_cast<int>(Activity::ActivityType::CUSTOM_BREAK) == 100);

    std::printf("  test_activity_type_values: PASS\n");
}

// ---------------------------------------------------------------------------
// 2. Activity classification (isClient / isDepot / isCustomBreak)
// ---------------------------------------------------------------------------

static void test_activity_classification()
{
    Activity depot(Activity::ActivityType::DEPOT, 0);
    Activity client(Activity::ActivityType::CLIENT, 5);
    Activity brk(Activity::ActivityType::CUSTOM_BREAK, 0);

    // DEPOT
    CHECK(depot.isDepot());
    CHECK(!depot.isClient());
    CHECK(!depot.isCustomBreak());
    CHECK(depot.type() == Activity::ActivityType::DEPOT);
    CHECK(depot.idx() == 0);

    // CLIENT
    CHECK(client.isClient());
    CHECK(!client.isDepot());
    CHECK(!client.isCustomBreak());
    CHECK(client.type() == Activity::ActivityType::CLIENT);
    CHECK(client.idx() == 5);

    // CUSTOM_BREAK
    CHECK(brk.isCustomBreak());
    CHECK(!brk.isClient());
    CHECK(!brk.isDepot());
    CHECK(brk.type() == Activity::ActivityType::CUSTOM_BREAK);
    CHECK(brk.idx() == 0);

    // Mutually exclusive: no Activity is both isClient and isCustomBreak
    // (or isDepot and isCustomBreak).
    CHECK(!(depot.isClient() && depot.isCustomBreak()));
    CHECK(!(client.isDepot() && client.isCustomBreak()));
    CHECK(!(brk.isDepot() && brk.isClient()));

    std::printf("  test_activity_classification: PASS\n");
}

// ---------------------------------------------------------------------------
// 3. Activity equality
// ---------------------------------------------------------------------------

static void test_activity_equality()
{
    Activity a1(Activity::ActivityType::CUSTOM_BREAK, 0);
    Activity a2(Activity::ActivityType::CUSTOM_BREAK, 0);
    Activity a3(Activity::ActivityType::CUSTOM_BREAK, 1);
    Activity a4(Activity::ActivityType::CLIENT, 0);

    CHECK(a1 == a2);
    CHECK(!(a1 == a3));  // different idx
    CHECK(!(a1 == a4));  // different type

    std::printf("  test_activity_equality: PASS\n");
}

// ---------------------------------------------------------------------------
// 4. Activity string constructor supports 'B' for CUSTOM_BREAK
// ---------------------------------------------------------------------------

static void test_activity_string_constructor()
{
    // The Activity(std::string) constructor parses "B0" as CUSTOM_BREAK idx 0.
    Activity brk("B3");
    CHECK(brk.isCustomBreak());
    CHECK(brk.idx() == 3);

    Activity depot("D1");
    CHECK(depot.isDepot());
    CHECK(depot.idx() == 1);

    Activity client("C42");
    CHECK(client.isClient());
    CHECK(client.idx() == 42);

    std::printf("  test_activity_string_constructor: PASS\n");
}

// ---------------------------------------------------------------------------
// 5. Activity stream output
// ---------------------------------------------------------------------------

static void test_activity_stream_output()
{
    Activity brk(Activity::ActivityType::CUSTOM_BREAK, 7);

    std::ostringstream out;
    out << brk;
    CHECK(out.str() == "B7");  // CUSTOM_BREAK renders as 'B'

    std::printf("  test_activity_stream_output: PASS\n");
}

// ---------------------------------------------------------------------------
// 6. Activity type discrimination round-trip via stream
// ---------------------------------------------------------------------------

static void test_activity_roundtrip()
{
    // Verify that construction via type+idx matches stream output and
    // classification.
    for (auto type :
         {Activity::ActivityType::DEPOT,
          Activity::ActivityType::CLIENT,
          Activity::ActivityType::CUSTOM_BREAK})
    {
        Activity act(type, 42);
        CHECK(act.type() == type);
        CHECK(act.idx() == 42);

        // Stream round-trip: serialize → deserialize
        std::ostringstream out;
        out << act;
        std::string encoded = out.str();

        Activity decoded(encoded);
        CHECK(decoded == act);
        CHECK(decoded.type() == type);
        CHECK(decoded.idx() == 42);
    }

    std::printf("  test_activity_roundtrip: PASS\n");
}

// ---------------------------------------------------------------------------
// 7. Invalid activity string throws
// ---------------------------------------------------------------------------

static void test_activity_invalid_string()
{
    bool threw = false;
    try
    {
        Activity("X0");  // 'X' is not a valid type char
    }
    catch (std::invalid_argument const &)
    {
        threw = true;
    }
    CHECK(threw);

    std::printf("  test_activity_invalid_string: PASS\n");
}

// ---------------------------------------------------------------------------
// 8. Node-level classification in search::Route
//
// The search::Route::Node class wraps Activity and exposes isClient(),
// isDepot(), and isCustomBreak(). Since Node requires full ProblemData
// for a constructed Route, we verify the classification via Activity
// directly here. The search::Route::Node::isCustomBreak() method
// is a one-liner that delegates to activity_.isCustomBreak() — tested
// implicitly by Activity tests above.
//
// Full operator integration tests (ExchangeNM with a CUSTOM_BREAK in the
// segment, SwapTails with a CUSTOM_BREAK in the tail) require a fully
// constructed search::Route with ProblemData, clients, depots, and a
// route schedule. That infrastructure will be provided by the Route lane
// (task 3.7). Once CUSTOM_BREAK nodes can be inserted into search routes
// via Route::setSchedule / Solution::load, the following scenarios should
// be tested:
//
//   a. Exchange10/20/30/32/33 rejects a move when the source segment
//      contains a CUSTOM_BREAK node (tests containsCustomBreak()).
//   b. SwapTails rejects a move when n(U) or n(V) is CUSTOM_BREAK.
//   c. Route::remove() asserts (in debug) if called on CUSTOM_BREAK.
//   d. Route::swap() asserts (in debug) if called with CUSTOM_BREAK.
//   e. Route::update() completes without crashing when the route
//      contains CUSTOM_BREAK nodes.
// ---------------------------------------------------------------------------

static void test_operator_invariants_placeholders()
{
    // Verify Activity-level invariants that operators rely on:
    // CUSTOM_BREAK is not a depot and not a client.
    Activity brk(Activity::ActivityType::CUSTOM_BREAK, 0);
    CHECK(!brk.isDepot());
    CHECK(!brk.isClient());
    CHECK(brk.isCustomBreak());

    std::printf("  test_operator_invariants_placeholders: PASS\n");
    std::printf("  (Full operator integration tests deferred to Route lane 3.7)\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    std::printf("Running CUSTOM_BREAK immutability audit tests...\n");

    test_activity_type_values();
    test_activity_classification();
    test_activity_equality();
    test_activity_string_constructor();
    test_activity_stream_output();
    test_activity_roundtrip();
    test_activity_invalid_string();
    test_operator_invariants_placeholders();

    std::printf("All CUSTOM_BREAK immutability audit tests PASSED.\n");
    return 0;
}
