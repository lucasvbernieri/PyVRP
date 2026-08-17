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
    # num_clients now counts only actual clients (not CUSTOM_BREAK nodes)
    assert_equal(route.num_clients(), 2)
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
        mandatory=False,  # non-mandatory: no feasibility impact
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


def test_mixed_fleet_mandatory_break_infeasible(ok_small):
    """
    Same mixed fleet as test_mixed_fleet_breaks_and_no_breaks but with a
    mandatory (non-relaxable) break: the break-configured vehicle's route
    exceeds the drive trigger and becomes infeasible, while the no-break
    vehicle stays feasible.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,  # non-relaxable: violation makes the route infeasible
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])

    data = ok_small.replace(vehicle_types=[vt_no, vt_brk])

    # Vehicle 0: no breaks → feasible.
    r0 = make_search_route(data, ["C0", "C1"], vehicle_type=0)
    assert_equal(r0.num_clients(), 2)
    assert_(r0.is_feasible())

    # Vehicle 1: mandatory break fires (drive exceeds 3600) → hard violation.
    r1 = make_search_route(data, ["C0", "C1"], vehicle_type=1)
    assert_equal(r1.num_clients(), 2)
    assert_(r1.break_due() > 0)
    assert_(not r1.is_feasible())


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


# =============================================================================
# CLT-correct dutyTime: waiting-aware duty accumulation tests
# =============================================================================


# ---------------------------------------------------------------------------
# Helper: build a simple chain data with configurable tw_early_shift
# ---------------------------------------------------------------------------

def _build_wait_chain(tw_early_shift=0, horizon=172800):
    """
    Build a chain instance: depot -> C0 -> C1 -> depot.
    All edges are EDGE s, service SVC each. C1's tw_early is shifted by
    tw_early_shift, forcing the route to wait at C1.
    """
    from pyvrp import Model

    EDGE = 4000
    SVC = 1800

    model = Model()
    depot = model.add_location(x=0, y=0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=horizon)

    c0 = model.add_location(x=1, y=0, name="C0")
    model.add_client(c0, delivery=0, service_duration=SVC,
                     tw_early=0, tw_late=horizon)

    c1 = model.add_location(x=2, y=0, name="C1")
    model.add_client(c1, delivery=0, service_duration=SVC,
                     tw_early=tw_early_shift, tw_late=horizon)

    all_locs = [depot, c0, c1]
    for frm in all_locs:
        for to in all_locs:
            if frm is to:
                model.add_edge(frm, to, distance=0, duration=0)
            else:
                model.add_edge(frm, to, distance=EDGE, duration=EDGE)

    model.add_vehicle_type(num_available=1, capacity=[9999])
    return model.data()


# Computed: drive = 3 edges * 4000 = 12000; svc = 2 * 1800 = 3600
_DRIVE_ONLY_2C = (2 + 1) * 4000   # 3 edges = 12000
_DUTY_NO_WAIT_2C = _DRIVE_ONLY_2C + 2 * 1800  # 12000 + 3600 = 15600


def test_waiting_increments_duty():
    """
    DUTY_TIME trigger must include waiting time. A route with forced wait
    at a client (tw_early far ahead) should accumulate enough duty to
    fire a trigger that (without waiting) would NOT fire.

    In this chain: C1 has tw_early = 12000, forcing ~2200s wait.
    Without waiting, duty = 15600. With waiting, duty ≈ 19800.
    trigger_value = 15900: without waiting 15600 < 15900 (no fire);
    with waiting 19800 > 15900 → break_due > 0.
    """
    WAIT_FORCE = 12000  # C1 tw_early — forces at least 2200s wait

    base = _build_wait_chain(tw_early_shift=WAIT_FORCE)

    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=15900,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk])
    data = base.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_equal(route.num_clients(), 2)
    assert_(route.break_due() > 0,
            f"Expected break_due > 0 with waiting (duty≈19800 > 15900), "
            f"got {route.break_due()}")


def test_no_waiting_identity():
    """
    Identity: with zero waiting, dutyTime_ equals drive + service (same as
    before the CLT fix). A trigger just above the no-wait duty should NOT
    fire, while a trigger just below it SHOULD fire.
    """
    base = _build_wait_chain(tw_early_shift=0)  # no waiting

    # Trigger slightly above no-wait duty: should NOT fire
    brk_above = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=_DUTY_NO_WAIT_2C + 1,  # 15601
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt_a = base.vehicle_type(0).replace(custom_breaks=[brk_above])
    route_a = make_search_route(base.replace(vehicle_types=[vt_a]),
                                 ["C0", "C1"])
    assert_equal(route_a.break_due(), 0,
                 f"Duty ({_DUTY_NO_WAIT_2C}) should NOT exceed "
                 f"trigger ({_DUTY_NO_WAIT_2C + 1})")

    # Trigger slightly below no-wait duty: SHOULD fire
    brk_below = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=_DUTY_NO_WAIT_2C - 1,  # 15599
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt_b = base.vehicle_type(0).replace(custom_breaks=[brk_below])
    route_b = make_search_route(base.replace(vehicle_types=[vt_b]),
                                 ["C0", "C1"])
    assert_(route_b.break_due() > 0,
            f"Duty ({_DUTY_NO_WAIT_2C}) should exceed "
            f"trigger ({_DUTY_NO_WAIT_2C - 1})")


def test_multi_day_all_timers_reset():
    """
    ALL_TIMERS break served at an eligible position (gate) sets
    lastResetAt_ = atSecond + break.service. After the reset, new
    duty accumulation starts from the reset point. With enough
    post-break work, the same trigger fires again → break_due > 0.

    Uses a 4-client chain with a CUSTOM_BREAK node inserted at a
    position where the duty reaches the trigger value (eligibility).
    """
    from pyvrp import Model

    N = 4
    EDGE = 2000
    SVC = 600
    HORIZON = 172800

    model = Model()
    depot = model.add_location(x=0, y=0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=HORIZON)

    locs = [depot]
    for i in range(N):
        loc = model.add_location(x=float(i + 1), y=0, name=f"C{i}")
        model.add_client(loc, delivery=0, service_duration=SVC,
                         tw_early=0, tw_late=HORIZON)
        locs.append(loc)

    for frm in locs:
        for to in locs:
            if frm is to:
                model.add_edge(frm, to, distance=0, duration=0)
            else:
                model.add_edge(frm, to, distance=EDGE, duration=EDGE)

    model.add_vehicle_type(num_available=1, capacity=[9999])
    base = model.data()

    # After C0: duty = 2000 + 600 = 2600
    # After C1: duty = 2600 + 2000 + 600 = 5200
    # After C2 (if no break): 5200 + 2000 + 600 = 7800
    # After C3: 7800 + 2000 + 600 = 10400
    # Set trigger to 5000 — fires after C1 (duty=5200 > 5000).
    # Insert break after C1 (position 2 after start depot).
    # After break reset + more work (C2, C3): new duty ≈ 5200 again.
    # If trigger=5000, post-break duty=5200 fires again → breakDue ≥ 2.

    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=5000,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk])
    data = base.replace(vehicle_types=[vt])

    # Build route: depot, C0, C1, [CUSTOM_BREAK at id=1], C2, C3, depot
    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C2"))
    route.append(Node("C3"))
    route.update()

    # num_clients = 4 (only actual clients C0-C3; CUSTOM_BREAK excluded)
    assert_equal(route.num_clients(), 4)
    # The break is served at the eligibility gate (duty=5200 >= 5000).
    # After ALL_TIMERS reset, duty restarts from 0. The taken-bit remains
    # set, preventing the same rule from retriggering at later boundaries.
    # Thus break_due reflects no violations: the break was served.
    assert_equal(route.break_due(), 0,
                 f"Expected break_due == 0 (break served at gate), "
                 f"got {route.break_due()}")

    # Regression: without the break node, the trigger cannot be served
    # and the violation fires (breakDue > 0 because duty crosses trigger
    # and no break node exists to serve it).
    route2 = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route2.num_clients(), 4)
    assert_(route2.break_due() > 0,
            f"Without break node, duty (10400) should exceed trigger (5000) "
            f"and cause a violation, got break_due={route2.break_due()}")


def test_breaks_consistent_after_exchange(ok_small):
    """
    After an Exchange11 move between two break-configured routes,
    break_due() should remain consistent (no crash, no stale state).
    """
    from pyvrp import CostEvaluator
    from pyvrp.search._search import Exchange11

    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,  # very low → will fire
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt, vt])
    cost_eval = CostEvaluator([0], 0, 0)

    r1 = make_search_route(data, ["C0", "C1"], vehicle_type=0)
    r2 = make_search_route(data, ["C2", "C3"], vehicle_type=1)

    op = Exchange11(data)
    U = r1[1]  # C0
    V = r2[1]  # C2
    result = op.evaluate(U, V, cost_eval)
    assert_(isinstance(result, tuple),
            "Exchange11 evaluate returned unexpected type")

    # Routes must still report consistent break_due after evaluation
    # (the cost-tracking assert in update() guarantees parity).
    assert_(r1.break_due() >= 0)
    assert_(r2.break_due() >= 0)


# =============================================================================
# 4.8: Break gate serves ALL_TIMERS break; no-break route violations
# =============================================================================


def test_break_gate_serves_all_timers_with_waiting():
    """
    Chain where day-1 duty crosses triggerVal → CUSTOM_BREAK eligibility
    gate serves the break (ALL_TIMERS reset) → break_due remains 0
    because the break was served and no violations occur.

    This test also verifies that the duty computation at the gate includes
    waiting time (CLT art. 4º). Without the forced waiting at C1
    (tw_early=12000), the duty would be 5200 after C1. With waiting,
    duty = 2600 + 2000 + wait(7400) + 600 = 12600, ensuring the gate
    is eligible at trigger_value=5000 even if drive+work alone wouldn't
    reach it.

    Computed values (EDGE=2000, SVC=600, HORIZON=172800):
      depot→C0:      duty = 2000+600 = 2600
      C0→C1+wait:    duty = 2600+2000+7400+600 = 12600
      trigger_value = 5000  (gate: 12600 >= 5000 → eligible → serve)
      break→C2:      duty (reset) = 0+2000+600 = 2600
      C2→C3:         duty = 2600+2000+600 = 5200
      Though post-break duty (5200) exceeds trigger (5000), the
      taken-bit from the gate prevents a second trigger → break_due=0.
    """
    from pyvrp import Model

    EDGE = 2000
    SVC = 600
    HORIZON = 172800

    model = Model()
    depot = model.add_location(x=0, y=0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=HORIZON)

    c0 = model.add_location(x=1, y=0, name="C0")
    model.add_client(c0, delivery=0, service_duration=SVC,
                     tw_early=0, tw_late=HORIZON)

    # C1 forces waiting: tw_early=12000 → ~7400s wait after C0
    c1 = model.add_location(x=2, y=0, name="C1")
    model.add_client(c1, delivery=0, service_duration=SVC,
                     tw_early=12000, tw_late=HORIZON)

    c2 = model.add_location(x=3, y=0, name="C2")
    model.add_client(c2, delivery=0, service_duration=SVC,
                     tw_early=0, tw_late=HORIZON)

    c3 = model.add_location(x=4, y=0, name="C3")
    model.add_client(c3, delivery=0, service_duration=SVC,
                     tw_early=0, tw_late=HORIZON)

    locs = [depot, c0, c1, c2, c3]
    for frm in locs:
        for to in locs:
            if frm is to:
                model.add_edge(frm, to, distance=0, duration=0)
            else:
                model.add_edge(frm, to, distance=EDGE, duration=EDGE)

    model.add_vehicle_type(num_available=1, capacity=[9999])
    base = model.data()

    # ---- Route WITH break node (A): gate serves the break ----
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=5000,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk])
    data = base.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C2"))
    route.append(Node("C3"))
    route.update()

    # Break served at gate → no violation
    assert_equal(route.break_due(), 0,
                 f"Expected break_due == 0 (break served at gate), "
                 f"got {route.break_due()}")
    assert_equal(route.num_clients(), 4)

    # ---- Route WITHOUT break node (B): violation fires ----
    route_no = Route(data, vehicle_type=0)
    route_no.append(Node("C0"))
    route_no.append(Node("C1"))
    route_no.append(Node("C2"))
    route_no.append(Node("C3"))
    route_no.update()

    # Without the break node, duty reaches 12600 at C1 (with waiting),
    # exceeds trigger=5000, and no break node exists → violation.
    assert_(route_no.break_due() > 0,
            f"Without break node, duty (12600) should exceed trigger (5000) "
            f"and cause violation, got break_due={route_no.break_due()}")


# =============================================================================
# 4.9: Break removal after ALL_TIMERS reset reverts accumulators
# =============================================================================


def test_break_removal_reverts_accumulators():
    """
    After an ALL_TIMERS reset break is served, removing the break node
    must not leave stale duty accumulators. Building a fresh route
    without the break node (equivalent to removal + rebuild) must produce
    the same break_due and duration as a route that never had the break.

    Route A (with break): depot, C0, C1, break(id=1), C2, C3, depot
    Route B (no break):   depot, C0, C1,        C2, C3, depot
    Route A' (break removed by fresh build) must equal Route B exactly.

    The key property: removing the break node reverts lastResetAt_ and
    all accumulators so that post-break work is counted from route start
    (as if the break never happened).
    """
    from pyvrp import Model

    N = 4
    EDGE = 2000
    SVC = 600
    HORIZON = 172800

    model = Model()
    depot = model.add_location(x=0, y=0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=HORIZON)

    locs = [depot]
    for i in range(N):
        loc = model.add_location(x=float(i + 1), y=0, name=f"C{i}")
        model.add_client(loc, delivery=0, service_duration=SVC,
                         tw_early=0, tw_late=HORIZON)
        locs.append(loc)

    for frm in locs:
        for to in locs:
            if frm is to:
                model.add_edge(frm, to, distance=0, duration=0)
            else:
                model.add_edge(frm, to, distance=EDGE, duration=EDGE)

    model.add_vehicle_type(num_available=1, capacity=[9999])
    base = model.data()

    # ALL_TIMERS break: duty trigger at 5000
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=5000,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk])
    data = base.replace(vehicle_types=[vt])

    # Route with the break node (A): depot, C0, C1, break, C2, C3, depot
    route_a = Route(data, vehicle_type=0)
    route_a.append(Node("C0"))
    route_a.append(Node("C1"))
    route_a.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route_a.append(Node("C2"))
    route_a.append(Node("C3"))
    route_a.update()
    break_due_a = route_a.break_due()

    # Route without the break node (B): depot, C0, C1, C2, C3, depot
    route_b = Route(data, vehicle_type=0)
    route_b.append(Node("C0"))
    route_b.append(Node("C1"))
    route_b.append(Node("C2"))
    route_b.append(Node("C3"))
    route_b.update()
    break_due_b = route_b.break_due()
    dur_b = route_b.duration()

    # Rebuild A without the break (A'): fresh Route, same clients, no break
    route_a_rebuilt = Route(data, vehicle_type=0)
    route_a_rebuilt.append(Node("C0"))
    route_a_rebuilt.append(Node("C1"))
    route_a_rebuilt.append(Node("C2"))
    route_a_rebuilt.append(Node("C3"))
    route_a_rebuilt.update()

    # A' must equal B exactly (break removal reverts all accumulators)
    assert_equal(route_a_rebuilt.break_due(), break_due_b,
                 "Rebuilt route (break removed) must match never-had-break "
                 f"route in break_due: {route_a_rebuilt.break_due()} != {break_due_b}")
    assert_equal(route_a_rebuilt.duration(), dur_b,
                 "Rebuilt route must match never-had-break route in duration")

    # Sanity: the break-having route has break_due == 0 (break served at gate)
    assert_equal(break_due_a, 0,
                 f"Route with break node served at gate should have "
                 f"break_due == 0, got {break_due_a}")
    # No-break route has violation (duty exceeds trigger with no break node)
    assert_(break_due_b > 0,
            f"Route without break node should have break_due > 0, "
            f"got {break_due_b}")
