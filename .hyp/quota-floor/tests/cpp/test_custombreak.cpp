#include "CustomBreak.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>

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

static void test_is_valid_start()
{
    // Break with TWs [[100, 200), [300, 400)].
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(200)},
           {Duration(300), Duration(400)}};
    CustomBreak brk(0, tws, Duration(1800), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    // Before first TW
    CHECK(!brk.isValidStart(Duration(50)));
    // Inside first TW
    CHECK(brk.isValidStart(Duration(150)));
    // Between TWs
    CHECK(!brk.isValidStart(Duration(250)));
    // Inside second TW
    CHECK(brk.isValidStart(Duration(350)));
    // After all TWs
    CHECK(!brk.isValidStart(Duration(450)));
    // Exactly at open edge
    CHECK(brk.isValidStart(Duration(100)));
    CHECK(brk.isValidStart(Duration(300)));
    // Exactly at close edge — NOT valid (interval is [open, close))
    CHECK(!brk.isValidStart(Duration(200)));
    CHECK(!brk.isValidStart(Duration(400)));

    std::printf("  test_is_valid_start: PASS\n");
}

static void test_is_valid_start_empty_tws()
{
    // Break with empty TWs → isValidStart always false.
    CustomBreak brk(0, {}, Duration(1800), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    CHECK(!brk.isValidStart(Duration(0)));
    CHECK(!brk.isValidStart(Duration(100)));
    CHECK(!brk.isValidStart(Duration(100000)));

    std::printf("  test_is_valid_start_empty_tws: PASS\n");
}

static void test_is_valid_start_single_tw()
{
    // Single TW [500, 800).
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(500), Duration(800)}};
    CustomBreak brk(0, tws, Duration(1800), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    CHECK(!brk.isValidStart(Duration(499)));
    CHECK(brk.isValidStart(Duration(500)));
    CHECK(brk.isValidStart(Duration(700)));
    CHECK(!brk.isValidStart(Duration(800)));

    std::printf("  test_is_valid_start_single_tw: PASS\n");
}

static void test_constructor_defaults()
{
    // Verify default values when constructor is called with only required args.
    CustomBreak brk(42);

    CHECK(brk.id == 42);
    CHECK(brk.tws.empty());
    CHECK(brk.service.get() == 0);
    CHECK(brk.trigger == CustomBreakTrigger::CLOCK_TIME);
    CHECK(brk.triggerValue.get() == 0);
    CHECK(brk.reset == CustomBreakReset::NONE);
    CHECK(brk.mandatory == false);
    CHECK(brk.conditionMinRouteS.get() == 0);
    CHECK(brk.priority == 0);
    CHECK(brk.supersedes.empty());

    std::printf("  test_constructor_defaults: PASS\n");
}

static void test_constructor_full()
{
    // Verify all fields are set correctly.
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(200)}};
    std::vector<size_t> supersedes = {1, 2, 3};
    CustomBreak brk(7,
                    tws,
                    Duration(3600),
                    CustomBreakTrigger::DRIVE_TIME,
                    Duration(16200),
                    CustomBreakReset::DRIVE_AND_WORK,
                    true,
                    Duration(21600),
                    10,
                    supersedes);

    CHECK(brk.id == 7);
    CHECK(brk.tws.size() == 1);
    CHECK(brk.tws[0].first.get() == 100);
    CHECK(brk.tws[0].second.get() == 200);
    CHECK(brk.service.get() == 3600);
    CHECK(brk.trigger == CustomBreakTrigger::DRIVE_TIME);
    CHECK(brk.triggerValue.get() == 16200);
    CHECK(brk.reset == CustomBreakReset::DRIVE_AND_WORK);
    CHECK(brk.mandatory == true);
    CHECK(brk.conditionMinRouteS.get() == 21600);
    CHECK(brk.priority == 10);
    CHECK(brk.supersedes.size() == 3);
    CHECK(brk.supersedes[0] == 1);
    CHECK(brk.supersedes[1] == 2);
    CHECK(brk.supersedes[2] == 3);

    std::printf("  test_constructor_full: PASS\n");
}

static void test_trigger_enum_values()
{
    // Verify enum values for the trigger types.
    CHECK(static_cast<uint8_t>(CustomBreakTrigger::CLOCK_TIME) == 0);
    CHECK(static_cast<uint8_t>(CustomBreakTrigger::DRIVE_TIME) == 1);
    CHECK(static_cast<uint8_t>(CustomBreakTrigger::WORK_TIME) == 2);
    CHECK(static_cast<uint8_t>(CustomBreakTrigger::DUTY_TIME) == 3);

    std::printf("  test_trigger_enum_values: PASS\n");
}

static void test_reset_enum_values()
{
    // Verify enum values for the reset modes.
    CHECK(static_cast<uint8_t>(CustomBreakReset::NONE) == 0);
    CHECK(static_cast<uint8_t>(CustomBreakReset::DRIVE_TIMER) == 1);
    CHECK(static_cast<uint8_t>(CustomBreakReset::WORK_TIMER) == 2);
    CHECK(static_cast<uint8_t>(CustomBreakReset::DRIVE_AND_WORK) == 3);
    CHECK(static_cast<uint8_t>(CustomBreakReset::ALL_TIMERS) == 4);

    std::printf("  test_reset_enum_values: PASS\n");
}

static void test_supersedes_empty()
{
    // Break with empty supersedes → no superseding logic needed.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::DRIVE_TIMER, true);

    CHECK(brk.supersedes.empty());

    std::printf("  test_supersedes_empty: PASS\n");
}

static void test_supersedes_ordering()
{
    // Verify supersedes vector preserves insertion order.
    std::vector<size_t> supersedes = {10, 5, 7};
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, false,
                    Duration(0), 20, supersedes);

    CHECK(brk.supersedes.size() == 3);
    CHECK(brk.supersedes[0] == 10);
    CHECK(brk.supersedes[1] == 5);
    CHECK(brk.supersedes[2] == 7);

    std::printf("  test_supersedes_ordering: PASS\n");
}

static void test_priority_value()
{
    // Priority is stored as-is.
    CustomBreak high(0, {}, Duration(0), CustomBreakTrigger::DUTY_TIME,
                     Duration(57600), CustomBreakReset::ALL_TIMERS, true,
                     Duration(0), 100);
    CustomBreak low(1, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(16200), CustomBreakReset::DRIVE_AND_WORK, true,
                    Duration(0), 5);

    CHECK(high.priority == 100);
    CHECK(low.priority == 5);
    CHECK(high.priority > low.priority);

    std::printf("  test_priority_value: PASS\n");
}

static void test_service_and_trigger_value()
{
    // Verify service and trigger_value are stored correctly.
    CustomBreak brk(0, {}, Duration(2700), CustomBreakTrigger::DRIVE_TIME,
                    Duration(16200), CustomBreakReset::DRIVE_AND_WORK, true);

    CHECK(brk.service.get() == 2700);
    CHECK(brk.triggerValue.get() == 16200);

    std::printf("  test_service_and_trigger_value: PASS\n");
}

static void test_condition_min_route()
{
    // Break with condition: only required if route > condition_min_route_s.
    CustomBreak conditional(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                            Duration(100), CustomBreakReset::DRIVE_TIMER, true,
                            Duration(3600),  // condition_min_route_s = 3600
                            0);
    CHECK(conditional.conditionMinRouteS.get() == 3600);

    CustomBreak unconditional(0, {}, Duration(0),
                              CustomBreakTrigger::DRIVE_TIME, Duration(100),
                              CustomBreakReset::DRIVE_TIMER, true);
    // Default is 0 = always
    CHECK(unconditional.conditionMinRouteS.get() == 0);

    std::printf("  test_condition_min_route: PASS\n");
}

int main()
{
    std::printf("Running CustomBreak tests...\n");

    test_is_valid_start();
    test_is_valid_start_empty_tws();
    test_is_valid_start_single_tw();
    test_constructor_defaults();
    test_constructor_full();
    test_trigger_enum_values();
    test_reset_enum_values();
    test_supersedes_empty();
    test_supersedes_ordering();
    test_priority_value();
    test_service_and_trigger_value();
    test_condition_min_route();

    std::printf("All CustomBreak tests PASSED.\n");
    return 0;
}
