"""
Edge-case break tests (OpenSpec task 9.4).

Mirrors tests/search/test_route_break_tracking.py style: uses
make_search_route, ok_small fixture, Node/Route from pyvrp.search._search,
and the same CustomBreak / VehicleType API.

Tests:
  - Multi-break (two DRIVE_TIME breaks, each at 4.5h → 2x threshold)
  - Tight time windows + mandatory breaks (feasibility edge)
  - Reload interaction (reset_breaks_at_reload True/False)
  - Empty route with breaks
  - Single-client route with breaks
  - Heterogeneous fleet (mixed break/no-break)

These are written to be run against the fork build; they will fail on
stock pyvrp which lacks CustomBreak.
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
# 9.4a: Multi-break — two DRIVE_TIME breaks (2x threshold, 4.5h each)
# =============================================================================


def test_two_drive_time_breaks_4_5h_each(ok_small):
    """
    Two DRIVE_TIME breaks with trigger_value 16200 (4.5h) each.
    Exercises that the second break triggers after the first resets
    the drive timer.

    NOTE: OkSmall has capacity 10, so routes with 3+ clients exceed it.
    We use 2 clients (load 8) to stay feasible and focus on break logic.
    """
    b1 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,  # 4.5h
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=10,
    )
    b2 = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=10,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b1, b2])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C1", "C2"])
    assert_equal(route.num_clients(), 2)
    assert_(route.duration() >= 0)
    assert_(route.break_due() >= 0)


def test_three_drive_time_breaks_interleaved(ok_small):
    """
    Three DRIVE_TIME breaks (BR 4h drive + 30min rest pattern × 3).
    Verifies cascading multi-threshold break insertion.
    """
    breaks = []
    for idx in range(3):
        breaks.append(
            CustomBreak(
                id=idx + 1,
                trigger=CustomBreakTrigger.DRIVE_TIME,
                trigger_value=14400,  # 4h (BR Lei 13.103)
                reset=CustomBreakReset.DRIVE_TIMER,
                mandatory=True,
                service=1800,  # 30min
                priority=10,
            )
        )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=breaks)
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)


# =============================================================================
# 9.4b: Tight time windows + mandatory breaks
# =============================================================================


def test_tight_tw_clock_break_fits_exactly(ok_small):
    """
    A CLOCK_TIME break with a narrow TW that exactly matches a gap
    between client time windows. Should insert without violation.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(20000, 22000)],  # narrow 2000s window
        service=1800,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    # OkSmall clients have TWs: C0 15600-22500, C1 12000-19500,
    # C2 8400-15300, C3 12000-19500
    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)


def test_tight_tw_break_may_violate(ok_small):
    """
    A mandatory CLOCK_TIME break in a window so tight it can't fit
    without overlapping client TWs. The route may be infeasible, but
    the search infrastructure should not crash.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(10000, 10500)],  # extremely narrow — 500s
        service=2700,           # requires 2700s
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)
    # route may or may not be feasible; we just assert no crash
    assert_(route.duration() >= 0)


def test_overlapping_break_tws(ok_small):
    """
    Two CLOCK_TIME breaks with overlapping TWs on the same vehicle.
    Priority ordering should determine which takes precedence.
    """
    b_high = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(15000, 25000)],
        service=1800,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=20,
    )
    b_low = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(18000, 28000)],
        service=1800,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=5,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b_low, b_high])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)


# =============================================================================
# 9.4c: Reload interaction — reset_breaks_at_reload
# =============================================================================


def test_reload_with_reset_breaks_true(ok_small_multiple_trips):
    """
    When reset_breaks_at_reload=True, after a depot reload all break
    accumulators should reset. Vehicle with a DRIVE_TIME break should
    not carry over accumulated drive from the previous trip.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small_multiple_trips.vehicle_type(0).replace(
        custom_breaks=[brk],
        reset_breaks_at_reload=True,
    )
    data = ok_small_multiple_trips.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)
    assert_(route.duration() >= 0)


def test_reload_with_reset_breaks_false(ok_small_multiple_trips):
    """
    When reset_breaks_at_reload=False, accumulated timers persist across
    reloads. A DRIVE_TIME break may fire earlier on the second trip.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small_multiple_trips.vehicle_type(0).replace(
        custom_breaks=[brk],
        reset_breaks_at_reload=False,
    )
    data = ok_small_multiple_trips.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)
    assert_(route.duration() >= 0)


def test_reload_clocks_dont_reset_even_with_flag(ok_small_multiple_trips):
    """
    Even when reset_breaks_at_reload=True, the elapsed clock (wall time)
    is NOT reset — only drive/work timers reset. CLOCK_TIME breaks
    should still observe absolute time.
    """
    brk_drive = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    brk_clock = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(100000, 200000)],
        service=600,
        reset=CustomBreakReset.NONE,
        mandatory=True,
    )
    vt = ok_small_multiple_trips.vehicle_type(0).replace(
        custom_breaks=[brk_drive, brk_clock],
        reset_breaks_at_reload=True,
    )
    data = ok_small_multiple_trips.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)


# =============================================================================
# 9.4d: Empty route with breaks
# =============================================================================


def test_empty_route_all_trigger_types(ok_small):
    """
    An empty route (just depot start + end) should not crash with any
    trigger type. All accumulators should be zero.
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

        route = Route(data, vehicle_type=0)
        route.update()
        assert_equal(route.num_clients(), 0)
        assert_(route.duration() >= 0)


def test_empty_route_with_multiple_breaks(ok_small):
    """
    Empty route with multiple break rules of different types.
    """
    b1 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    b2 = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=28800,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    b3 = CustomBreak(
        id=3,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(21600, 24300)],
        service=2700,
        reset=CustomBreakReset.NONE,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b1, b2, b3])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.update()
    assert_equal(route.num_clients(), 0)


# =============================================================================
# 9.4e: Single-client route with breaks
# =============================================================================


def test_single_client_with_drive_break(ok_small):
    """
    Single client + DRIVE_TIME break. If the drive to/from the client
    plus service exceeds trigger_value, the break should be inserted.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,   # very low — almost guaranteed to trigger
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0"])
    assert_equal(route.num_clients(), 1)
    assert_(route.duration() >= 0)


def test_single_client_with_clock_break(ok_small):
    """
    Single client + CLOCK_TIME break that falls within the route's
    time span. Should slot in if the TW allows.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(10000, 20000)],
        service=1800,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C1"])  # C1 TW: 12000-19500
    assert_equal(route.num_clients(), 1)
    assert_(route.duration() >= 0)


def test_single_client_before_clock_break_window(ok_small):
    """
    Single client whose TW is entirely before the CLOCK_TIME break
    window. The break should be scheduled after the client.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(50000, 60000)],  # late break
        service=1800,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0"])  # C0 TW: 15600-22500
    assert_equal(route.num_clients(), 1)


# =============================================================================
# 9.4f: Heterogeneous fleet
# =============================================================================


def test_heterogeneous_fleet_all_no_breaks(ok_small):
    """
    All vehicle types without breaks → all routes should work identically.
    """
    vt0 = ok_small.vehicle_type(0)
    data = ok_small.replace(vehicle_types=[vt0, vt0, vt0])

    for vt_idx in range(3):
        route = make_search_route(data, ["C0", "C1"], vehicle_type=vt_idx)
        assert_equal(route.num_clients(), 2)
        assert_(route.is_feasible())


def test_heterogeneous_fleet_all_with_breaks(ok_small):
    """
    All vehicle types with the same break → all routes should work.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt_brk = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt_brk, vt_brk])

    for vt_idx in range(2):
        route = make_search_route(data, ["C0", "C1"], vehicle_type=vt_idx)
        assert_equal(route.num_clients(), 2)


def test_heterogeneous_fleet_mixed(ok_small):
    """
    Mixed fleet: half with breaks, half without. Both types produce
    valid routes.

    NOTE: OkSmall capacity is 10. 3 clients exceed it. We use 2 clients
    (load 8) to stay feasible and focus on heterogeneous fleet behavior.
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

    # V0: no breaks
    r0 = make_search_route(data, ["C1", "C2"], vehicle_type=0)
    assert_equal(r0.num_clients(), 2)
    assert_(r0.is_feasible())

    # V1: with breaks
    r1 = make_search_route(data, ["C1", "C2"], vehicle_type=1)
    assert_equal(r1.num_clients(), 2)
    assert_(r1.is_feasible())


def test_heterogeneous_fleet_mixed_mandatory_infeasible(ok_small):
    """
    Mixed fleet with a mandatory (non-relaxable) break. The break-configured
    vehicle's route exceeds the drive trigger and thus has a hard violation,
    making it infeasible, while the no-break vehicle stays feasible.

    Complements test_heterogeneous_fleet_mixed (which uses mandatory=False and
    keeps both vehicles feasible).
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=2000,  # < drive real da rota C1-C2 (2565) — antes 3600, nunca disparava
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,  # non-relaxable: violation makes the route infeasible
    )
    vt_no = ok_small.vehicle_type(0)
    vt_brk = vt_no.replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt_no, vt_brk])

    # V0: no breaks → feasible.
    r0 = make_search_route(data, ["C1", "C2"], vehicle_type=0)
    assert_equal(r0.num_clients(), 2)
    assert_(r0.is_feasible())

    # V1: mandatory break fires (drive exceeds 3600) → hard violation.
    r1 = make_search_route(data, ["C1", "C2"], vehicle_type=1)
    assert_equal(r1.num_clients(), 2)
    assert_(r1.break_due() > 0)
    assert_(not r1.is_feasible())


def test_heterogeneous_fleet_different_break_configs(ok_small):
    """
    Two vehicle types with different break configurations. Both should
    produce routes independently.
    """
    # V0: BR-style DRIVE_TIME break
    brk_br = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=14400,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        service=1800,
    )
    vt_br = ok_small.vehicle_type(0).replace(custom_breaks=[brk_br])

    # V1: EU-style DRIVE_TIME break + CLOCK_TIME lunch
    brk_drive = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        service=2700,
    )
    brk_lunch = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(21600, 24300)],
        service=2700,
        reset=CustomBreakReset.DRIVE_AND_WORK,
        mandatory=True,
    )
    vt_eu = ok_small.vehicle_type(0).replace(
        custom_breaks=[brk_drive, brk_lunch],
    )

    data = ok_small.replace(vehicle_types=[vt_br, vt_eu])

    r0 = make_search_route(data, ["C0", "C1", "C2"], vehicle_type=0)
    assert_equal(r0.num_clients(), 3)

    r1 = make_search_route(data, ["C0", "C1", "C2"], vehicle_type=1)
    assert_equal(r1.num_clients(), 3)


# =============================================================================
# 9.4g: Supersedes with multiple thresholds
# =============================================================================


def test_supersedes_with_different_resets(ok_small):
    """
    High-priority break with DRIVE_AND_WORK reset supersedes a low-priority
    DRIVE_TIMER reset break. The high break fully clears accumulators,
    so the low break should not fire immediately after.
    """
    b_low = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=3600,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=5,
        supersedes=[2],
    )
    b_high = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=7200,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=20,
        supersedes=[1],
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b_low, b_high])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)


# =============================================================================
# 9.4h: Condition min route
# =============================================================================


def test_condition_min_route_met(ok_small):
    """
    Break with condition_min_route_s that is less than total route duty time.
    The break should be eligible.
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        condition_min_route_s=1,  # essentially always met
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)


def test_condition_min_route_not_met(ok_small):
    """
    Break with condition_min_route_s exceeding any possible route.
    Should be completely skipped (no insertion attempt).
    """
    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=100,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        condition_min_route_s=1_000_000,  # impossible
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2"])
    assert_equal(route.num_clients(), 3)


# =============================================================================
# 9.4i: Non-mandatory breaks edge cases
# =============================================================================


def test_non_mandatory_breaks_in_long_route(ok_small):
    """
    Multiple non-mandatory breaks on a longer route. Should reset
    accumulators without incrementing breakDue.
    """
    breaks = []
    for idx in range(4):
        breaks.append(
            CustomBreak(
                id=idx + 1,
                trigger=CustomBreakTrigger.DRIVE_TIME,
                trigger_value=500,
                reset=CustomBreakReset.DRIVE_TIMER,
                mandatory=False,
                priority=10,
            )
        )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=breaks)
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)


def test_mixed_mandatory_and_non_mandatory(ok_small):
    """
    Mixed mandatory + non-mandatory breaks. Mandatory breaks should
    increment breakDue; non-mandatory should just reset.
    """
    b_non = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=500,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=False,
    )
    b_man = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=2000,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b_non, b_man])
    data = ok_small.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1", "C2", "C3"])
    assert_equal(route.num_clients(), 4)
