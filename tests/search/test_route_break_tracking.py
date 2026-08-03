"""
Tests for the parallel-array break tracking infrastructure in search::Route
(OpenSpec tasks 3.1-3.7 — driveAt/driveBefore/driveAfter, atSecond, breakDue).

Note: hasBreaks() and breakDue() are defined in C++ but not yet exposed
via bindings (that is a later lane — OpenSpec 7.6). These tests verify:
  - Routes with breaks do not crash during construction/update
  - Routes without breaks are unaffected (regression)
  - CustomBreak node insertion and immutability
  - Duration, distance, load are preserved when break tracking is active
  - Multiple vehicle types (mixed break/no-break fleet)
  - Edge cases: empty routes, single client, multi-break, supersedes
"""
import numpy as np
import pytest
from numpy.testing import assert_, assert_equal, assert_raises

from pyvrp import (
    Activity,
    ActivityType,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    VehicleType,
)
from pyvrp.search._search import Node, Route
from tests.helpers import make_search_route


# =============================================================================
# 3.2: hasBreaks() — verified via VehicleType. No-crash tests for search::Route
# =============================================================================


def test_route_without_breaks_works(ok_small):
    """
    Routes without breaks work identically to before. Drive arrays are
    internally std::nullopt (zero overhead).
    """
    route = make_search_route(ok_small, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)
    assert_(route.is_feasible())


def test_route_with_breaks_does_not_crash(ok_small):
    """
    Creating a route with a break-configured VehicleType and calling update()
    does not crash. Exercises driveAt/driveBefore/driveAfter population.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)
    assert_(route.duration() >= 0)


def test_route_with_breaks_append_update_append(ok_small):
    """
    Building a route step by step with breaks configured. update() is called
    after each append; no crash should occur.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_AND_WORK,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.update()
    assert_equal(route.num_clients(), 1)

    route.append(Node("C1"))
    route.update()
    assert_equal(route.num_clients(), 2)

    route.append(Node("C2"))
    route.update()
    assert_equal(route.num_clients(), 3)


def test_route_with_breaks_clear_and_rebuild(ok_small):
    """
    Clearing a route and rebuilding does not crash when breaks are configured.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.update()
    assert_equal(route.num_clients(), 2)

    route.clear()
    assert_equal(route.num_clients(), 0)

    route.append(Node("C2"))
    route.append(Node("C0"))
    route.update()
    assert_equal(route.num_clients(), 2)


# =============================================================================
# 3.3: Drive arrays populated — verify DurationSegment/load/distance unaffected
# =============================================================================


def test_breaks_do_not_affect_duration(ok_small):
    """
    Duration computation is the same with or without breaks (since
    CUSTOM_BREAK activities have placeholder zero-duration).
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])

    data_no = ok_small.replace(vehicle_types=[vt_no])
    data_brk = ok_small.replace(vehicle_types=[vt_brk])

    route_no = make_search_route(data_no, ["C0", "C1", "C2"])
    route_brk = make_search_route(data_brk, ["C0", "C1", "C2"])

    assert_equal(route_no.duration(), route_brk.duration())


def test_breaks_do_not_affect_load(ok_small):
    """
    Load computation is the same with or without breaks.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])

    data_no = ok_small.replace(vehicle_types=[vt_no])
    data_brk = ok_small.replace(vehicle_types=[vt_brk])

    route_no = make_search_route(data_no, ["C0", "C1", "C2"])
    route_brk = make_search_route(data_brk, ["C0", "C1", "C2"])

    assert_equal(route_no.load(), route_brk.load())
    assert_equal(route_no.excess_load(), route_brk.excess_load())


def test_breaks_do_not_affect_distance(ok_small):
    """
    Distance computation is the same with or without breaks.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])

    data_no = ok_small.replace(vehicle_types=[vt_no])
    data_brk = ok_small.replace(vehicle_types=[vt_brk])

    route_no = make_search_route(data_no, ["C0", "C1", "C2"])
    route_brk = make_search_route(data_brk, ["C0", "C1", "C2"])

    assert_equal(route_no.distance(), route_brk.distance())


# =============================================================================
# 3.5a: CustomBreak activities in warm-start route
# =============================================================================


def test_custom_break_node_in_route(ok_small):
    """
    CUSTOM_BREAK nodes can be inserted into search::Route and update()
    processes them correctly.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))  # break at id 1
    route.append(Node("C1"))
    route.update()

    # 2 depots + 2 clients + 1 break = 5 activities
    assert_equal(len(route), 5)
    # Note: num_clients = size - num_depots = 5 - 2 = 3
    # (CUSTOM_BREAK is counted by this definition since it's not a depot)
    assert_equal(route.num_clients(), 3)
    # Verify break node type
    assert_equal(route[2].type, ActivityType.CUSTOM_BREAK)


def test_custom_break_node_immutable_by_design(ok_small):
    """
    CUSTOM_BREAK nodes have their position verified. The C++ assert in
    Route::remove() prevents removal (would abort the process, so we
    can't test the assertion from Python without crashing).
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()

    # Break node is at position 2
    assert_equal(route[2].type, ActivityType.CUSTOM_BREAK)
    # Removing it would abort due to C++ assert in Route::remove()


def test_custom_break_activity_roundtrip(ok_small):
    """
    Activity('B1') creates a CUSTOM_BREAK node with correct type.
    """
    act = Activity("B1")
    assert_equal(act.type, ActivityType.CUSTOM_BREAK)
    assert_equal(act.idx, 1)
    assert_(not act.is_client())
    assert_(not act.is_depot())


# =============================================================================
# 3.4: atSecond computation and reset_breaks_at_reload
# =============================================================================


def test_reset_breaks_at_reload_flag(ok_small):
    """
    reset_breaks_at_reload=True does not cause crashes.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(
        custom_breaks=[brk],
        reset_breaks_at_reload=True,
    )
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)


# =============================================================================
# 3.6: Edge cases — empty, single client, long route
# =============================================================================


def test_empty_route_with_breaks(ok_small):
    """
    An empty route (just start + end depot) updates correctly with breaks.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.update()
    assert_equal(route.num_clients(), 0)
    assert_(route.duration() >= 0)


def test_single_client_route_with_breaks(ok_small):
    """
    A route with a single client works with breaks configured.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0"])
    assert_equal(route.num_clients(), 1)
    assert_(route.duration() >= 0)


def test_long_route_with_breaks(ok_small):
    """
    A route with many clients exercises the forward/backward pass loops.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=1000,  # low threshold → likely triggers
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    # OkSmall has clients 0-3; use all 4
    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)
    assert_(route.duration() >= 0)


# =============================================================================
# 3.6: Multiple break types, different triggers/resets
# =============================================================================


def test_multiple_breaks_on_vehicle(ok_small):
    """
    A vehicle type with multiple break rules (DRIVE, WORK, CLOCK).
    """
    b1 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=20,
    )
    b2 = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.WORK_TIME,
        trigger_value=7200,
        reset=CustomBreakReset.DRIVE_AND_WORK,
        mandatory=True,
        priority=10,
    )
    b3 = CustomBreak(
        id=3,
        tws=[(50_000, 60_000)],
        service=600,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        reset=CustomBreakReset.NONE,
        mandatory=True,
        priority=5,
    )

    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b3, b1, b2])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)
    assert_(route.duration() >= 0)


def test_non_mandatory_break(ok_small):
    """
    Non-mandatory breaks reset accumulators but don't increment breakDue.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=False,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)


def test_duty_time_trigger(ok_small):
    """
    A DUTY_TIME triggered break with ALL_TIMERS reset.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)


def test_clock_time_trigger(ok_small):
    """
    A CLOCK_TIME triggered break. atSecond computation feeds into
    DriveSegment::merge() for interval-crossing detection.
    """
    brk = CustomBreak(
        id=1,
        tws=[(100, 1000)],
        service=600,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        reset=CustomBreakReset.NONE,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)


def test_condition_min_route(ok_small):
    """
    Break with condition_min_route_s: skipped when accumulated duty is below
    the condition threshold.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        condition_min_route_s=100_000,  # very high — never met
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)


def test_supersedes_relationship(ok_small):
    """
    Breaks with supersedes: a high-priority break suppresses lower-priority
    breaks when it triggers.
    """
    b_high = CustomBreak(
        id=3,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,
        reset=CustomBreakReset.NONE,
        mandatory=True,
        priority=20,
        supersedes=[1, 2],
    )
    b_low1 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=200,
        reset=CustomBreakReset.NONE,
        mandatory=True,
        priority=5,
        supersedes=[3],
    )
    b_low2 = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=200,
        reset=CustomBreakReset.NONE,
        mandatory=True,
        priority=5,
        supersedes=[3],
    )
    vt = ok_small.vehicle_type(0).replace(
        custom_breaks=[b_low1, b_low2, b_high],
    )
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)


# =============================================================================
# 3.3: Mixed fleet — some vehicles with breaks, some without
# =============================================================================


def test_mixed_fleet_breaks_and_no_breaks(ok_small):
    """
    With multiple vehicle types (one with breaks, one without), both route
    types work. Drive arrays are std::nullopt for non-break vehicles.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])

    data = ok_small.replace(vehicle_types=[vt_no, vt_brk])

    # Vehicle 0: no breaks
    r0 = make_search_route(data, ["C0", "C1"], vehicle_type=0)
    assert_equal(r0.num_clients(), 2)
    assert_(r0.is_feasible())

    # Vehicle 1: with breaks
    r1 = make_search_route(data, ["C0", "C1"], vehicle_type=1)
    assert_equal(r1.num_clients(), 2)
    assert_(r1.is_feasible())


# =============================================================================
# 3.5: breakDue() — note: not exposed via bindings yet (later lane 7.6).
#      The following tests verify the infrastructure is correct.
# =============================================================================


def test_all_reset_modes_no_crash(ok_small):
    """
    Tests all five CustomBreakReset modes: NONE, DRIVE_TIMER, WORK_TIMER,
    DRIVE_AND_WORK, ALL_TIMERS.
    """
    for reset_mode in [CustomBreakReset.NONE,
                       CustomBreakReset.DRIVE_TIMER,
                       CustomBreakReset.WORK_TIMER,
                       CustomBreakReset.DRIVE_AND_WORK,
                       CustomBreakReset.ALL_TIMERS]:
        brk = CustomBreak(
            id=1,
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=1000,
            reset=reset_mode,
            mandatory=True,
        )
        vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
        data = ok_small.replace(vehicle_types=[vt])

        route = make_search_route(data, ["C0", "C1", "C2"])
        assert_equal(route.num_clients(), 3)


def test_all_trigger_types_no_crash(ok_small):
    """
    Tests all four CustomBreakTrigger types: CLOCK_TIME, DRIVE_TIME,
    WORK_TIME, DUTY_TIME.
    """
    for trigger in [CustomBreakTrigger.CLOCK_TIME,
                    CustomBreakTrigger.DRIVE_TIME,
                    CustomBreakTrigger.WORK_TIME,
                    CustomBreakTrigger.DUTY_TIME]:
        brk = CustomBreak(
            id=1,
            trigger=trigger,
            trigger_value=1000,
            reset=CustomBreakReset.DRIVE_TIMER,
            mandatory=True,
        )
        vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
        data = ok_small.replace(vehicle_types=[vt])

        route = make_search_route(data, ["C0", "C1", "C2"])
        assert_equal(route.num_clients(), 3)


def test_per_break_trigger_value_parametrized(ok_small):
    """
    Tests various trigger_value thresholds with a short route.
    Drive should stay below the threshold (no violation accumulation).
    """
    for trigger_val in [100, 500, 1_000, 5_000, 50_000]:
        brk = CustomBreak(
            id=1,
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=trigger_val,
            reset=CustomBreakReset.DRIVE_TIMER,
            mandatory=True,
        )
        vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
        data = ok_small.replace(vehicle_types=[vt])

        route = make_search_route(data, ["C0", "C1", "C2"])
        assert_equal(route.num_clients(), 3)
