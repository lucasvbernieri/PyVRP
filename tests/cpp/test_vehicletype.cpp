#include <cassert>
#include <cstdio>

#include "VehicleType.h"
#include "CustomBreak.h"

using namespace pyvrp;

// Helper: create a CustomBreak with a given id and priority
static CustomBreak makeBreak(size_t id, int priority = 0)
{
    return CustomBreak(id,          // id
                       {},          // tws (empty)
                       0,           // service
                       CustomBreakTrigger::DRIVE_TIME,
                       14400,       // triggerValue (4h)
                       CustomBreakReset::DRIVE_AND_WORK,
                       true,        // mandatory
                       0,           // conditionMinRouteS
                       priority,    // priority
                       {});         // supersedes
}

int main()
{
    int passed = 0;
    int failed = 0;

#define CHECK(cond, msg)                                   \
    do                                                     \
    {                                                      \
        if (cond)                                          \
        {                                                  \
            passed++;                                      \
            std::printf("  PASS: %s\n", msg);              \
        }                                                  \
        else                                               \
        {                                                  \
            failed++;                                      \
            std::printf("  FAIL: %s\n", msg);              \
        }                                                  \
    } while (0)

    // ================================================================
    // 1. hasBreaks() returns false with empty breaks (default)
    // ================================================================
    {
        VehicleType vt(1);  // default: no custom_breaks
        CHECK(!vt.hasBreaks(), "1.1 hasBreaks() false with default-constructed VT");
        CHECK(vt.custom_breaks.empty(), "1.2 custom_breaks is empty by default");
    }

    // ================================================================
    // 2. hasBreaks() returns true with configured breaks
    // ================================================================
    {
        std::vector<CustomBreak> breaks;
        breaks.push_back(makeBreak(1, 10));
        breaks.push_back(makeBreak(2, 5));

        VehicleType vt(1, {}, 0, 0, 0, 0,
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Distance>::max(),
                       1, 0, 0, std::nullopt,
                       {}, {}, std::numeric_limits<size_t>::max(),
                       0, 0, std::move(breaks), false, "test");

        CHECK(vt.hasBreaks(), "2.1 hasBreaks() true with configured breaks");
        CHECK(vt.custom_breaks.size() == 2, "2.2 custom_breaks has 2 elements");
    }

    // ================================================================
    // 3. custom_breaks sorted by descending priority after construction
    // ================================================================
    {
        std::vector<CustomBreak> breaks;
        breaks.push_back(makeBreak(1, 5));   // priority 5
        breaks.push_back(makeBreak(2, 20));  // priority 20
        breaks.push_back(makeBreak(3, 10));  // priority 10
        breaks.push_back(makeBreak(4, 0));   // priority 0

        VehicleType vt(1, {}, 0, 0, 0, 0,
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Distance>::max(),
                       1, 0, 0, std::nullopt,
                       {}, {}, std::numeric_limits<size_t>::max(),
                       0, 0, std::move(breaks), false, "sorted");

        CHECK(vt.custom_breaks.size() == 4, "3.1 size == 4");
        // After sorting by descending priority: 20, 10, 5, 0
        CHECK(vt.custom_breaks[0].priority == 20, "3.2 first break has priority 20 (highest)");
        CHECK(vt.custom_breaks[1].priority == 10, "3.3 second break has priority 10");
        CHECK(vt.custom_breaks[2].priority == 5,  "3.4 third break has priority 5");
        CHECK(vt.custom_breaks[3].priority == 0,  "3.5 fourth break has priority 0 (lowest)");
        // Verify IDs correspond correctly
        CHECK(vt.custom_breaks[0].id == 2, "3.6 highest priority break has id 2");
        CHECK(vt.custom_breaks[1].id == 3, "3.7 break with priority 10 has id 3");
        CHECK(vt.custom_breaks[2].id == 1, "3.8 break with priority 5 has id 1");
        CHECK(vt.custom_breaks[3].id == 4, "3.9 break with priority 0 has id 4");
    }

    // ================================================================
    // 4. replace() correctly composes breaks with existing constraints
    // ================================================================
    {
        // Create a base VT with capacity [100] and no breaks
        VehicleType base(2, {Load(100)}, 1, 2, Cost(50), Duration(36000),
                         Duration(86400), Duration(57600), Distance(500000),
                         Cost(2), Cost(3), size_t(0), std::nullopt,
                         {}, {size_t(3)}, size_t(5),
                         Duration(7200), Cost(10), {}, false, "base");

        CHECK(base.numAvailable == 2, "4.1 base numAvailable == 2");
        CHECK(base.capacity.size() == 1 && base.capacity[0] == Load(100), "4.2 base capacity[0] == 100");
        CHECK(!base.hasBreaks(), "4.3 base has no breaks");

        // Use replace() to change capacity and add breaks
        std::vector<CustomBreak> newBreaks;
        newBreaks.push_back(makeBreak(10, 1));  // id=10, priority=1
        newBreaks.push_back(makeBreak(11, 2));  // id=11, priority=2

        VehicleType replaced = base.replace(
            std::nullopt,                                 // numAvailable (keep)
            std::vector<Load>{Load(200)},                 // capacity (change)
            std::nullopt, std::nullopt, std::nullopt,     // depots, fixedCost
            std::nullopt, std::nullopt, std::nullopt,     // twEarly, twLate, shiftDuration
            std::nullopt, std::nullopt, std::nullopt,     // maxDistance, unitDistCost, unitDurCost
            std::nullopt, std::nullopt,                   // profile, startLate
            std::nullopt, std::nullopt,                   // initialLoad, reloadDepots
            std::nullopt, std::nullopt,                   // maxReloads, maxOvertime
            std::nullopt,                                 // unitOvertimeCost
            std::move(newBreaks),                         // custom_breaks (add)
            std::nullopt,                                 // reset_breaks_at_reload (keep)
            std::nullopt                                  // name (keep)
        );

        CHECK(replaced.numAvailable == 2, "4.4 replace kept numAvailable == 2");
        CHECK(replaced.capacity.size() == 1 && replaced.capacity[0] == Load(200), "4.5 replace changed capacity to 200");
        CHECK(replaced.hasBreaks(), "4.6 replace added breaks → hasBreaks() true");
        CHECK(replaced.custom_breaks.size() == 2, "4.7 replace breaks count == 2");
        // Breaks should be sorted: priority 2 first, then 1
        CHECK(replaced.custom_breaks[0].id == 11, "4.8 highest priority break has id 11");
        CHECK(replaced.custom_breaks[1].id == 10, "4.9 lower priority break has id 10");

        // Other fields should be preserved
        CHECK(replaced.startDepot == 1, "4.10 startDepot preserved");
        CHECK(replaced.endDepot == 2, "4.11 endDepot preserved");
        CHECK(replaced.fixedCost == Cost(50), "4.12 fixedCost preserved");
    }

    // ================================================================
    // 5. reset_breaks_at_reload default false and settable true
    // ================================================================
    {
        VehicleType vtDefault(1);
        CHECK(vtDefault.reset_breaks_at_reload == false, "5.1 reset_breaks_at_reload defaults to false");

        std::vector<CustomBreak> breaks;
        breaks.push_back(makeBreak(1, 10));

        VehicleType vtTrue(1, {}, 0, 0, 0, 0,
                           std::numeric_limits<Duration>::max(),
                           std::numeric_limits<Duration>::max(),
                           std::numeric_limits<Distance>::max(),
                           1, 0, 0, std::nullopt,
                           {}, {}, std::numeric_limits<size_t>::max(),
                           0, 0, std::move(breaks), true, "reset");

        CHECK(vtTrue.reset_breaks_at_reload == true, "5.2 reset_breaks_at_reload settable to true");
        CHECK(vtTrue.hasBreaks(), "5.3 hasBreaks() still works with reset_breaks_at_reload=true");
    }

    // ================================================================
    // 6. replace() with reset_breaks_at_reload override
    // ================================================================
    {
        VehicleType base(1, {}, 0, 0, 0, 0,
                         std::numeric_limits<Duration>::max(),
                         std::numeric_limits<Duration>::max(),
                         std::numeric_limits<Distance>::max(),
                         1, 0, 0, std::nullopt,
                         {}, {}, std::numeric_limits<size_t>::max(),
                         0, 0, {}, false, "base");

        CHECK(base.reset_breaks_at_reload == false, "6.1 base reset_breaks_at_reload == false");

        VehicleType replaced = base.replace(
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt,
            std::nullopt,          // custom_breaks (keep)
            true,                  // reset_breaks_at_reload (change)
            std::nullopt           // name (keep)
        );

        CHECK(replaced.reset_breaks_at_reload == true, "6.2 replace changed reset_breaks_at_reload to true");
    }

    // ================================================================
    // C2: break.id >= 16 must throw std::invalid_argument
    // ================================================================
    {
        std::vector<CustomBreak> breaks;
        breaks.push_back(CustomBreak(16,  // id = 16 (invalid)
                                     {},
                                     Duration(0),
                                     CustomBreakTrigger::DRIVE_TIME,
                                     Duration(14400),
                                     CustomBreakReset::DRIVE_AND_WORK,
                                     true));

        bool threw = false;
        try
        {
            VehicleType vt(1, {}, 0, 0, 0, 0,
                           std::numeric_limits<Duration>::max(),
                           std::numeric_limits<Duration>::max(),
                           std::numeric_limits<Distance>::max(),
                           1, 0, 0, std::nullopt,
                           {}, {}, std::numeric_limits<size_t>::max(),
                           0, 0, std::move(breaks), false, "bad");
        }
        catch (std::invalid_argument const &)
        {
            threw = true;
        }
        CHECK(threw, "C2: break.id >= 16 throws std::invalid_argument");
    }

    // ================================================================
    // M2: Deterministic sort — equal-priority breaks keep input order
    // ================================================================
    {
        std::vector<CustomBreak> breaks;
        breaks.push_back(CustomBreak(10, {}, Duration(0),
                                     CustomBreakTrigger::DRIVE_TIME,
                                     Duration(14400),
                                     CustomBreakReset::DRIVE_AND_WORK,
                                     true, Duration(0),
                                     5));  // priority 5, id=10
        breaks.push_back(CustomBreak(11, {}, Duration(0),
                                     CustomBreakTrigger::DRIVE_TIME,
                                     Duration(14400),
                                     CustomBreakReset::DRIVE_AND_WORK,
                                     true, Duration(0),
                                     5));  // priority 5, id=11
        // Both have equal priority. With stable_sort, input order (10, 11)
        // must be preserved.
        breaks.push_back(CustomBreak(12, {}, Duration(0),
                                     CustomBreakTrigger::DRIVE_TIME,
                                     Duration(14400),
                                     CustomBreakReset::DRIVE_AND_WORK,
                                     true, Duration(0),
                                     10));  // priority 10, id=12

        VehicleType vt(1, {}, 0, 0, 0, 0,
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Duration>::max(),
                       std::numeric_limits<Distance>::max(),
                       1, 0, 0, std::nullopt,
                       {}, {}, std::numeric_limits<size_t>::max(),
                       0, 0, std::move(breaks), false, "stable");

        CHECK(vt.custom_breaks.size() == 3, "M2.1 size == 3");
        // Sorted by priority: 12 (prio 10) first, then 10 and 11 (both prio 5)
        // in their original input order.
        CHECK(vt.custom_breaks[0].id == 12, "M2.2 highest priority (10) → id 12");
        CHECK(vt.custom_breaks[1].id == 10, "M2.3 equal priority → first input (id 10)");
        CHECK(vt.custom_breaks[2].id == 11, "M2.4 equal priority → second input (id 11)");
    }

    // ================================================================
    // Summary
    // ================================================================
    std::printf("\n=== Results: %d passed, %d failed ===\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
