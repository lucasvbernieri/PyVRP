#include "DriveSegment.h"
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
using namespace pyvrp::search;

static void test_sizeof()
{
    // Verify the 40-byte layout invariant.
    CHECK(sizeof(DriveSegment) == 40);
    std::printf("  test_sizeof: PASS\n");
}

static void test_merge_no_violations()
{
    // Two segments with drive/work/duty below all break thresholds.
    // No breaks configured → no violations.
    CustomBreak brk(0,                      // id
                    {},                      // tws
                    Duration(0),             // service
                    CustomBreakTrigger::DRIVE_TIME,
                    Duration(1000),          // triggerValue
                    CustomBreakReset::DRIVE_AND_WORK,
                    true);                   // mandatory

    std::vector<CustomBreak> breaks = {brk};

    // first: 200 drive, 300 work, 300 duty, no breaks taken
    DriveSegment first(200, 300, 300, 0, 0, 0);
    // second: 100 drive, 200 work, 200 duty, no breaks taken
    DriveSegment second(100, 200, 200, 0, 0, 0);
    // edge: 50 time units
    Duration edgeDur(50);
    // atSecond: arbitrary (not past any TW)
    Duration atSecond(0);

    auto merged = DriveSegment::merge(edgeDur, first, second, breaks, atSecond);

    // drive = 200 + 50 + 100 = 350 (< 1000) → no violation
    // work  = 300 + 50 + 200 = 550 (< 1000) → no violation
    CHECK(merged.driveTime_ == 350);
    CHECK(merged.workTime_ == 550);
    CHECK(merged.dutyTime_ == 550);
    CHECK(merged.breaksTakenMask_ == 0);
    CHECK(merged.breakDue_ == 0);

    std::printf("  test_merge_no_violations: PASS\n");
}

static void test_merge_no_violations_no_breaks()
{
    // Empty breaks vector → no violations at all.
    std::vector<CustomBreak> breaks;

    DriveSegment first(5000, 5000, 5000, 0, 0, 0);
    DriveSegment second(5000, 5000, 5000, 0, 0, 0);
    Duration edgeDur(1000);

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    // drive = 5000 + 1000 + 5000 = 11000 (but no break to check against)
    CHECK(merged.driveTime_ == 11000);
    CHECK(merged.workTime_ == 11000);
    CHECK(merged.dutyTime_ == 11000);
    CHECK(merged.breaksTakenMask_ == 0);
    CHECK(merged.breakDue_ == 0);

    std::printf("  test_merge_no_violations_no_breaks: PASS\n");
}

static void test_merge_drive_violation()
{
    // DRIVE_TIME trigger: drive exceeds trigger_value → violation.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(500), CustomBreakReset::DRIVE_TIMER, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(300, 200, 200, 0, 0, 0);
    DriveSegment second(200, 100, 100, 0, 0, 0);
    Duration edgeDur(100);

    // drive = 300 + 100 + 200 = 600 > 500 → triggered
    // After DRIVE_TIMER reset: drive = second.driveTime_ = 200
    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 200);  // reset to second's driveTime_
    CHECK(merged.workTime_ == 400);   // 200 + 100 + 100 (not reset)
    CHECK(merged.dutyTime_ == 400);
    CHECK(merged.breaksTakenMask_ == 1u);  // break 0 taken
    CHECK(merged.breakDue_ == 1);          // mandatory violation

    std::printf("  test_merge_drive_violation: PASS\n");
}

static void test_merge_work_violation()
{
    // WORK_TIME trigger: work exceeds trigger_value → violation.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::WORK_TIME,
                    Duration(600), CustomBreakReset::WORK_TIMER, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(200, 400, 400, 0, 0, 0);
    DriveSegment second(200, 200, 200, 0, 0, 0);
    Duration edgeDur(100);

    // work = 400 + 100 + 200 = 700 > 600 → triggered
    // After WORK_TIMER reset: work = second.workTime_ = 200
    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 500);  // 200 + 100 + 200 (not reset)
    CHECK(merged.workTime_ == 200);   // reset to second's workTime_
    CHECK(merged.dutyTime_ == 700);
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_work_violation: PASS\n");
}

static void test_merge_duty_violation()
{
    // DUTY_TIME trigger: duty exceeds trigger_value → violation.
    // Duty only resets on ALL_TIMERS, not on DRIVE_AND_WORK.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DUTY_TIME,
                    Duration(800), CustomBreakReset::DRIVE_AND_WORK, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(300, 300, 500, 0, 0, 0);
    DriveSegment second(200, 200, 300, 0, 0, 0);
    Duration edgeDur(100);

    // duty = 500 + 100 + 300 = 900 > 800 → triggered
    // DRIVE_AND_WORK reset: drive = second.driveTime_, work = second.workTime_
    // BUT dutyTime_ is NOT reset (only ALL_TIMERS resets it)
    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 200);  // reset by DRIVE_AND_WORK
    CHECK(merged.workTime_ == 200);   // reset by DRIVE_AND_WORK
    CHECK(merged.dutyTime_ == 900);   // NOT reset (only ALL_TIMERS resets duty)
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_duty_violation: PASS\n");
}

static void test_merge_clock_violation()
{
    // CLOCK_TIME trigger: atSecond past the end of the break's TW end
    // and the specific break not yet taken → violation.
    // Use a single TW for simplicity.
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(200)}};  // TW [100, 200)
    CustomBreak brk(0, tws, Duration(0), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0),  // trigger_value ignored for CLOCK_TIME
                    CustomBreakReset::NONE, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(50, 50, 50, 0, 0, 0);
    DriveSegment second(50, 50, 50, 0, 0, 0);
    Duration edgeDur(10);
    // atSecond = 250 → past TW end (200) → triggered
    Duration atSecond(250);

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, atSecond);

    // NONE reset: no accumulators change
    CHECK(merged.driveTime_ == 110);
    CHECK(merged.workTime_ == 110);
    CHECK(merged.dutyTime_ == 110);
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_clock_violation: PASS\n");
}

static void test_merge_clock_not_triggered_within_tw()
{
    // CLOCK_TIME: atSecond is still within TW → no trigger.
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(500)}};
    CustomBreak brk(0, tws, Duration(0), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(50, 50, 50, 0, 0, 0);
    DriveSegment second(50, 50, 50, 0, 0, 0);

    // atSecond = 300 → within TW [100, 500) → NOT triggered
    auto merged = DriveSegment::merge(Duration(10), first, second, breaks,
                                      Duration(300));

    CHECK(merged.driveTime_ == 110);
    CHECK(merged.breakDue_ == 0);   // not violated
    CHECK(merged.breaksTakenMask_ == 0);

    std::printf("  test_merge_clock_not_triggered_within_tw: PASS\n");
}

static void test_merge_multi_break()
{
    // Two DRIVE_TIME breaks: drive exceeds 2× threshold → 2 violations.
    CustomBreak brk0(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                     Duration(100), CustomBreakReset::DRIVE_TIMER, true);
    CustomBreak brk1(1, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                     Duration(200), CustomBreakReset::DRIVE_TIMER, true,
                     Duration(0), 0);

    // Note: breaks are evaluated in vector order.
    // After brk0 triggers (drive > 100) and resets drive to second's value,
    // we re-check brk1 with the RESET drive value.
    // But since both have the same trigger type and the reset happens in-loop,
    // brk1 may or may not trigger depending on reset values.

    // Let's craft a scenario where both trigger:
    // We need drive before ANY reset to exceed trigger_values.
    // But since DRIVE_TIMER resets drive after each trigger,
    // subsequent breaks check the RESET drive value.

    // Simpler test: two different triggers, both fire independently.
    // brk0: DRIVE_TIME threshold 400, brk1: WORK_TIME threshold 500
    CustomBreak dBrk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                     Duration(400), CustomBreakReset::DRIVE_TIMER, true);
    CustomBreak wBrk(1, {}, Duration(0), CustomBreakTrigger::WORK_TIME,
                     Duration(500), CustomBreakReset::WORK_TIMER, true);

    std::vector<CustomBreak> breaks = {dBrk, wBrk};

    DriveSegment first(300, 400, 400, 0, 0, 0);
    DriveSegment second(150, 100, 100, 0, 0, 0);
    Duration edgeDur(50);

    // drive = 300 + 50 + 150 = 500 > 400 → brk0 triggered
    //   brk0 resets drive to second.driveTime_ = 150
    // work = 400 + 50 + 100 = 550 > 500 → brk1 triggered
    //   brk1 resets work to second.workTime_ = 100
    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 150);  // reset by DRIVE_TIMER
    CHECK(merged.workTime_ == 100);   // reset by WORK_TIMER
    CHECK(merged.dutyTime_ == 550);   // not reset
    CHECK(merged.breaksTakenMask_ == 3u);  // bits 0 and 1 set
    CHECK(merged.breakDue_ == 2);          // 2 mandatory violations

    std::printf("  test_merge_multi_break: PASS\n");
}

static void test_merge_reset_none()
{
    // NONE reset: no accumulators change, only breakDue incremented.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::NONE, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(80, 50, 50, 0, 0, 0);
    DriveSegment second(30, 20, 20, 0, 0, 0);
    Duration edgeDur(10);
    // drive = 80 + 10 + 30 = 120 > 100 → triggered

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    // NONE: no reset
    CHECK(merged.driveTime_ == 120);  // unchanged
    CHECK(merged.workTime_ == 80);    // unchanged
    CHECK(merged.dutyTime_ == 80);    // unchanged
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_reset_none: PASS\n");
}

static void test_merge_reset_drive_timer()
{
    // DRIVE_TIMER reset: only driveTime resets.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::DRIVE_TIMER, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(80, 50, 50, 0, 0, 0);
    DriveSegment second(30, 20, 20, 0, 0, 0);
    Duration edgeDur(10);
    // drive = 80 + 10 + 30 = 120 > 100

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 30);   // reset to second.driveTime_
    CHECK(merged.workTime_ == 80);    // unchanged
    CHECK(merged.dutyTime_ == 80);    // unchanged
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_reset_drive_timer: PASS\n");
}

static void test_merge_reset_work_timer()
{
    // WORK_TIMER reset: only workTime resets.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::WORK_TIME,
                    Duration(100), CustomBreakReset::WORK_TIMER, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(30, 80, 80, 0, 0, 0);
    DriveSegment second(10, 30, 30, 0, 0, 0);
    Duration edgeDur(10);
    // work = 80 + 10 + 30 = 120 > 100

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 50);   // unchanged
    CHECK(merged.workTime_ == 30);    // reset to second.workTime_
    CHECK(merged.dutyTime_ == 120);   // unchanged
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_reset_work_timer: PASS\n");
}

static void test_merge_reset_drive_and_work()
{
    // DRIVE_AND_WORK reset: driveTime and workTime reset, dutyTime unchanged.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::DRIVE_AND_WORK, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(80, 80, 100, 0, 0, 0);
    DriveSegment second(30, 30, 50, 0, 0, 0);
    Duration edgeDur(10);
    // drive = 80 + 10 + 30 = 120 > 100

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 30);   // reset
    CHECK(merged.workTime_ == 30);    // reset
    CHECK(merged.dutyTime_ == 160);   // NOT reset
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_reset_drive_and_work: PASS\n");
}

static void test_merge_reset_all_timers()
{
    // ALL_TIMERS reset: all three accumulators reset.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DUTY_TIME,
                    Duration(100), CustomBreakReset::ALL_TIMERS, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(80, 80, 80, 0, 0, 0);
    DriveSegment second(30, 30, 30, 0, 0, 0);
    Duration edgeDur(10);
    // duty = 80 + 10 + 30 = 120 > 100

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 30);  // reset
    CHECK(merged.workTime_ == 30);   // reset
    CHECK(merged.dutyTime_ == 30);   // reset
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 1);

    std::printf("  test_merge_reset_all_timers: PASS\n");
}

static void test_merge_non_mandatory_no_breakdue()
{
    // Non-mandatory break: accumulators reset, but breakDue NOT incremented.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::DRIVE_TIMER, false);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(80, 50, 50, 0, 0, 0);
    DriveSegment second(30, 20, 20, 0, 0, 0);
    Duration edgeDur(10);
    // drive = 120 > 100

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 30);   // reset
    CHECK(merged.workTime_ == 80);    // unchanged
    CHECK(merged.breaksTakenMask_ == 1u);
    CHECK(merged.breakDue_ == 0);     // non-mandatory → no violation

    std::printf("  test_merge_non_mandatory_no_breakdue: PASS\n");
}

static void test_merge_condition_min_route()
{
    // Break with condition_min_route_s > 0: skipped when duty < threshold.
    CustomBreak brk(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(100), CustomBreakReset::DRIVE_TIMER, true,
                    Duration(500),  // condition_min_route_s = 500
                    0);

    std::vector<CustomBreak> breaks = {brk};

    // duty = 200 + 10 + 100 = 310 < 500 → break skipped
    DriveSegment first(100, 100, 200, 0, 0, 0);
    DriveSegment second(50, 50, 100, 0, 0, 0);
    Duration edgeDur(10);

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    // drive = 100 + 10 + 50 = 160 > 100, but break skipped due to condition
    CHECK(merged.driveTime_ == 160);  // unchanged (break skipped)
    CHECK(merged.breakDue_ == 0);
    CHECK(merged.breaksTakenMask_ == 0);

    // Now test with duty >= condition: should trigger
    DriveSegment first2(300, 300, 400, 0, 0, 0);  // duty starts high
    DriveSegment second2(200, 200, 200, 0, 0, 0);
    // duty = 400 + 10 + 200 = 610 >= 500 → break NOT skipped
    // drive = 300 + 10 + 200 = 510 > 100 → triggered
    auto merged2
        = DriveSegment::merge(edgeDur, first2, second2, breaks, Duration(0));

    CHECK(merged2.driveTime_ == 200);  // reset
    CHECK(merged2.breakDue_ == 1);
    CHECK(merged2.breaksTakenMask_ == 1u);

    std::printf("  test_merge_condition_min_route: PASS\n");
}

static void test_merge_supersedes()
{
    // Break 1 (id=1, supersedes=[0]):
    // If break 0 was already taken, break 1 is skipped.
    CustomBreak brk0(0, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                     Duration(100), CustomBreakReset::DRIVE_TIMER, true);
    CustomBreak brk1(1, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                     Duration(100), CustomBreakReset::DRIVE_TIMER, true,
                     Duration(0), 0, {0});  // supersedes break 0

    std::vector<CustomBreak> breaks = {brk1};  // only brk1 configured

    // First segment already has break 0 taken (bit 0 set)
    DriveSegment first(80, 50, 50, 1u, 0, 0);  // breaksTakenMask = 1 (break 0)
    DriveSegment second(30, 20, 20, 0, 0, 0);
    Duration edgeDur(10);
    // drive = 80 + 10 + 30 = 120 > 100 → trigger check
    // But brk1 supersedes [0], and bit 0 is set → skipped

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.driveTime_ == 120);  // unchanged (break skipped)
    CHECK(merged.breakDue_ == 0);
    CHECK(merged.breaksTakenMask_ == 1u);  // still just break 0

    // Now WITHOUT break 0 taken → brk1 should fire
    DriveSegment first2(80, 50, 50, 0, 0, 0);
    auto merged2
        = DriveSegment::merge(edgeDur, first2, second, breaks, Duration(0));

    CHECK(merged2.driveTime_ == 30);   // reset
    CHECK(merged2.breakDue_ == 1);
    CHECK(merged2.breaksTakenMask_ == 3u);  // bits 0 and 1: brk1 + superseded brk0

    std::printf("  test_merge_supersedes: PASS\n");
}

// ============================================================================
// C1: Supersedes must suppress superseded breaks within the SAME merge call
// ============================================================================
static void test_merge_supersedes_within_same_call()
{
    // Setup: three DRIVE_TIME breaks.
    //   brkHigh (id=3, prio=20, supersedes={1,2}) triggers first.
    //   brkLow1 (id=1, prio=5, supersedes={3}) and
    //   brkLow2 (id=2, prio=5, supersedes={3}) would also trigger,
    //   but the pre-trigger supersedes check sees bit 3 already set
    //   (because brkHigh also marked bits 1 & 2 on trigger)
    //   and skips them. → Only 1 mandatory violation.
    //   takenMask must have bits for all three ids (0b1110 = 14).
    CustomBreak brkHigh(3,                       // id
                        {},                       // tws
                        Duration(0),              // service
                        CustomBreakTrigger::DRIVE_TIME,
                        Duration(100),            // triggerValue
                        CustomBreakReset::NONE,   // reset
                        true,                     // mandatory
                        Duration(0),              // conditionMinRouteS
                        20,                       // priority
                        {1, 2});                  // supersedes
    CustomBreak brkLow1(1, {}, Duration(0),
                        CustomBreakTrigger::DRIVE_TIME,
                        Duration(200),
                        CustomBreakReset::NONE,
                        true,
                        Duration(0),
                        5,                        // priority
                        {3});                     // superseded by brkHigh
    CustomBreak brkLow2(2, {}, Duration(0),
                        CustomBreakTrigger::DRIVE_TIME,
                        Duration(200),
                        CustomBreakReset::NONE,
                        true,
                        Duration(0),
                        5,                        // priority
                        {3});                     // superseded by brkHigh

    // Pass in priority order: brkHigh first, then the lower-priority breaks.
    std::vector<CustomBreak> breaks = {brkHigh, brkLow1, brkLow2};

    // drive = 300 + 50 + 200 = 550 > 100 and > 200 → all would trigger
    DriveSegment first(300, 50, 50, 0, 0, 0);
    DriveSegment second(200, 20, 20, 0, 0, 0);
    Duration edgeDur(50);

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    // NONE reset → drive unchanged
    CHECK(merged.driveTime_ == 550);
    CHECK(merged.breakDue_ == 1);  // Only brkHigh triggered
    // Bits: id=1 → bit 1 = 2, id=2 → bit 2 = 4, id=3 → bit 3 = 8
    CHECK(merged.breaksTakenMask_ == 14u);  // 0b1110

    std::printf("  test_merge_supersedes_within_same_call: PASS\n");
}

// ============================================================================
// MN1: CLOCK_TIME boundary — atSecond between two time windows
// ============================================================================
static void test_merge_clock_between_windows()
{
    // Two time windows: [100, 200) and [300, 400)
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(200)},
           {Duration(300), Duration(400)}};
    CustomBreak brk(0, tws, Duration(0), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(50, 50, 50, 0, 0, 0);
    DriveSegment second(50, 50, 50, 0, 0, 0);
    Duration edgeDur(10);

    // atSecond=250 → between windows: past first TW end (200) but not past
    // last TW end (400) → NOT triggered
    {
        auto merged
            = DriveSegment::merge(edgeDur, first, second, breaks, Duration(250));
        CHECK(merged.breakDue_ == 0);
        CHECK(merged.breaksTakenMask_ == 0);
    }

    // atSecond=450 → past both TW ends → triggered
    {
        auto merged
            = DriveSegment::merge(edgeDur, first, second, breaks, Duration(450));
        CHECK(merged.breakDue_ == 1);
        CHECK(merged.breaksTakenMask_ == 1u);
    }

    std::printf("  test_merge_clock_between_windows: PASS\n");
}

// ============================================================================
// MN2: CLOCK_TIME strict > semantics — exactly at TW end is NOT triggered
// ============================================================================
static void test_merge_clock_exact_end_not_triggered()
{
    // Single TW [100, 200); atSecond=200 exactly → NOT triggered
    std::vector<std::pair<Duration, Duration>> tws
        = {{Duration(100), Duration(200)}};
    CustomBreak brk(0, tws, Duration(0), CustomBreakTrigger::CLOCK_TIME,
                    Duration(0), CustomBreakReset::NONE, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(50, 50, 50, 0, 0, 0);
    DriveSegment second(50, 50, 50, 0, 0, 0);
    Duration edgeDur(10);
    Duration atSecond(200);  // exactly at TW end

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, atSecond);

    CHECK(merged.breakDue_ == 0);       // NOT triggered (strict >)
    CHECK(merged.breaksTakenMask_ == 0);

    std::printf("  test_merge_clock_exact_end_not_triggered: PASS\n");
}

static void test_merge_waiting_increments_duty()
{
    // Verify that waiting time between two segments is included in the
    // merged dutyTime_ via the CLT-correct formula:
    //   wait = max(0, atSecond - (first.dutyTime_ + first.lastResetAt_ + edge))
    //   duty = first.dutyTime_ + edge + wait + second.dutyTime_
    //
    // Without waiting (atSecond=0): duty = first.duty + edge + second.duty
    // With waiting (atSecond = first.duty + edge + 100): duty includes 100.

    std::vector<CustomBreak> breaks;  // no breaks — purely accumulator test

    DriveSegment first(200, 300, 300, 0, 0, 0);
    DriveSegment second(100, 200, 200, 0, 0, 0);
    Duration edgeDur(50);

    // Without waiting: atSecond = 0, lastResetAt = 0
    //   wait = max(0, 0 - (300 + 0 + 50)) = 0
    //   duty = 300 + 50 + 0 + 200 = 550
    {
        auto merged
            = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));
        CHECK(merged.driveTime_ == 350);
        CHECK(merged.workTime_ == 550);
        CHECK(merged.dutyTime_ == 550);  // no waiting
    }

    // With waiting: atSecond = first.dutyTime_ + edge + 100 = 300 + 50 + 100
    //   wait = max(0, 450 - (300 + 0 + 50)) = 100
    //   duty = 300 + 50 + 100 + 200 = 650
    {
        Duration atSecond(450);
        auto merged
            = DriveSegment::merge(edgeDur, first, second, breaks, atSecond);
        CHECK(merged.driveTime_ == 350);
        CHECK(merged.workTime_ == 550);
        CHECK(merged.dutyTime_ == 650);  // includes 100 waiting
    }

    std::printf("  test_merge_waiting_increments_duty: PASS\n");
}

static void test_from_client()
{
    auto ds = DriveSegment::fromClient(Duration(42));
    CHECK(ds.driveTime_ == 0);
    CHECK(ds.workTime_ == 42);
    CHECK(ds.dutyTime_ == 42);
    CHECK(ds.breaksTakenMask_ == 0);
    CHECK(ds.breakDue_ == 0);

    std::printf("  test_from_client: PASS\n");
}

static void test_from_depot()
{
    auto ds = DriveSegment::fromDepot();
    CHECK(ds.driveTime_ == 0);
    CHECK(ds.workTime_ == 0);
    CHECK(ds.dutyTime_ == 0);
    CHECK(ds.breaksTakenMask_ == 0);
    CHECK(ds.breakDue_ == 0);

    std::printf("  test_from_depot: PASS\n");
}

static void test_from_vehicle_type()
{
    auto ds = DriveSegment::fromVehicleType();
    CHECK(ds.driveTime_ == 0);
    CHECK(ds.workTime_ == 0);
    CHECK(ds.dutyTime_ == 0);
    CHECK(ds.breaksTakenMask_ == 0);
    CHECK(ds.breakDue_ == 0);

    std::printf("  test_from_vehicle_type: PASS\n");
}

static void test_taken_mask_accumulates()
{
    // Verify that breaksTakenMask is OR-ed correctly across merges.
    CustomBreak brk(1, {}, Duration(0), CustomBreakTrigger::DRIVE_TIME,
                    Duration(1000), CustomBreakReset::DRIVE_TIMER, true);

    std::vector<CustomBreak> breaks = {brk};

    DriveSegment first(50, 50, 50, 0b0011, 0, 0);  // breaks 0 and 1
    DriveSegment second(50, 50, 50, 0b0100, 2, 0);  // break 2, 2 violations
    Duration edgeDur(10);

    auto merged
        = DriveSegment::merge(edgeDur, first, second, breaks, Duration(0));

    CHECK(merged.breaksTakenMask_ == 0b0111);  // OR'd
    CHECK(merged.breakDue_ == 2);              // accumulated

    std::printf("  test_taken_mask_accumulates: PASS\n");
}

int main()
{
    std::printf("Running DriveSegment tests...\n");

    test_sizeof();
    test_merge_no_violations();
    test_merge_no_violations_no_breaks();
    test_merge_drive_violation();
    test_merge_work_violation();
    test_merge_duty_violation();
    test_merge_clock_violation();
    test_merge_clock_not_triggered_within_tw();
    test_merge_multi_break();
    test_merge_reset_none();
    test_merge_reset_drive_timer();
    test_merge_reset_work_timer();
    test_merge_reset_drive_and_work();
    test_merge_reset_all_timers();
    test_merge_non_mandatory_no_breakdue();
    test_merge_condition_min_route();
    test_merge_supersedes();
    test_merge_supersedes_within_same_call();
    test_merge_clock_between_windows();
    test_merge_clock_exact_end_not_triggered();
    test_merge_waiting_increments_duty();
    test_from_client();
    test_from_depot();
    test_from_vehicle_type();
    test_taken_mask_accumulates();

    std::printf("All DriveSegment tests PASSED.\n");
    return 0;
}
