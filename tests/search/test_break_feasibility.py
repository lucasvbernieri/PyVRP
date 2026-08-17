"""
Break feasibility tests (OpenSpec tasks 1.2, 1.5, 1.6, 1.7).

Verifies:
  - CustomBreak.relaxable flag (default False, set, pickle)            (1.2)
  - VehicleType rejects > 16 breaks per vehicle                         (1.5)
  - breakDue > 0 makes a solution infeasible (plan), except relaxable
    violations (reopt) which are penalised instead                     (1.6)
  - break window close is hard: arrival after close is infeasible      (1.7)
"""
import pickle

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal, assert_raises

from pyvrp import (
    Activity,
    ActivityType,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Model,
    Solution,
    VehicleType,
)
from pyvrp.search._search import (
    Node,
    Route as SearchRoute,
    Solution as SearchSolution,
)
from tests.helpers import make_search_route


# =============================================================================
# 1.2: relaxable flag
# =============================================================================


def test_relaxable_default_false():
    assert_equal(CustomBreak(id=1).relaxable, False)


def test_relaxable_settable():
    brk = CustomBreak(id=1, relaxable=True)
    assert_equal(brk.relaxable, True)


def test_relaxable_pickle():
    before = CustomBreak(id=7, relaxable=True, mandatory=True)
    after = pickle.loads(pickle.dumps(before))
    assert_equal(after.id, 7)
    assert_equal(after.relaxable, True)


# =============================================================================
# 1.5: > 16 breaks per vehicle rejected
# =============================================================================


def test_vehicle_type_rejects_more_than_16_breaks():
    with assert_raises(ValueError):
        VehicleType(custom_breaks=[CustomBreak(id=i) for i in range(17)])


# =============================================================================
# 1.6: breakDue feasibility (plan vs reopt)
# =============================================================================


def _duty_break(relaxable=False):
    return CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,  # triggers almost immediately
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        relaxable=relaxable,
    )


def _two_client_chain(relaxable):
    """
    depot -> C0 -> C1 -> depot, all required, with a duty-time break that
    triggers almost immediately (no CUSTOM_BREAK node → violation).
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()
    vt = base.vehicle_type(0).replace(custom_breaks=[_duty_break(relaxable)])
    return base.replace(vehicle_types=[vt])


def _search_solution_with_clients(data, clients):
    sol = SearchSolution(data)
    route = sol.routes[0]
    for client in clients:
        route.append(Node(client))
    route.update()
    return sol


def test_plan_break_due_is_infeasible():
    """
    A mandatory (non-relaxable) break violation makes the search route and
    the solution infeasible.
    """
    data = _two_client_chain(relaxable=False)

    route = make_search_route(data, ["C0", "C1"])
    assert_(route.break_due() > 0)
    assert_equal(route.break_due_mask(), 0b10)  # id 1 violated
    assert_(not route.is_feasible())

    out = _search_solution_with_clients(data, ["C0", "C1"]).unload()
    assert_(not out.is_feasible())


def test_reopt_relaxable_break_due_is_feasible():
    """
    A relaxable break violation keeps the solution feasible and is penalised
    instead (break_due_penalty applies to the objective).
    """
    data = _two_client_chain(relaxable=True)

    route = make_search_route(data, ["C0", "C1"])
    assert_(route.break_due() > 0)
    assert_equal(route.break_due_mask(), 0b10)
    assert_(route.is_feasible())

    out = _search_solution_with_clients(data, ["C0", "C1"]).unload()
    assert_(out.is_feasible())
    assert_equal(out.break_due(), 1)

    # The relaxable violation pays break_due_penalty on top of the base cost.
    base = CostEvaluator([0], 0, 0, break_due_penalty=0)
    pen = CostEvaluator([0], 0, 0, break_due_penalty=100)
    assert_equal(pen.penalised_cost(out), base.penalised_cost(out) + 100)


def test_reopt_non_relaxable_break_due_is_infeasible():
    """
    A non-relaxable break violation remains infeasible even when another break
    on the vehicle is relaxable.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk1 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    brk2 = CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        relaxable=True,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk1, brk2])
    data = base.replace(vehicle_types=[vt])

    route = make_search_route(data, ["C0", "C1"])
    assert_(route.break_due() > 0)
    # id 1 (non-relaxable) is violated, so the route is infeasible.
    assert_equal(route.break_due_mask() & 0b10, 0b10)
    assert_(not route.is_feasible())


# =============================================================================
# 1.7: break window close is hard
# =============================================================================


def _hard_window_chain():
    """
    depot -> C0 -> (break) -> depot, with the break's absolute window closing
    well before the vehicle can arrive.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000)
    for f in [depot, c0]:
        for t in [depot, c0]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    return m.data()


def test_break_window_close_is_hard():
    """
    A break (DUTY_TIME, due) with an absolute window close that the vehicle
    cannot make produces time warp and is infeasible.
    """
    base = _hard_window_chain()

    # Absolute window [100, 200]: the vehicle arrives at the break at 1600
    # (1000 travel + 600 service at C0), well after the close (200).
    brk = CustomBreak(
        id=1,
        tws=[(100, 200)],
        service=100,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    data = base.replace(
        vehicle_types=[base.vehicle_type(0).replace(custom_breaks=[brk])]
    )

    route = SearchRoute(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.update()

    # Arrival after the window close → time warp → infeasible.
    assert_(route.time_warp() > 0)
    assert_(not route.is_feasible())


def test_break_window_close_wide_is_feasible():
    """
    The same break with a wide window is feasible (no spurious time warp).
    """
    base = _hard_window_chain()

    brk = CustomBreak(
        id=1,
        tws=[(100, 200_000)],
        service=100,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    data = base.replace(
        vehicle_types=[base.vehicle_type(0).replace(custom_breaks=[brk])]
    )

    route = SearchRoute(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.update()

    assert_(route.is_feasible())


def test_non_due_break_closed_absolute_window_no_warp():
    """
    A CUSTOM_BREAK node whose absolute window has already closed does NOT warp
    the route when the break is not yet due at that node. The window close is
    only enforced when the break is due (see test_break_window_close_is_hard).
    """
    base = _hard_window_chain()

    brk = CustomBreak(
        id=1,
        tws=[(100, 200)],
        service=100,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1_000_000,  # never reached: not due at its node
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    data = base.replace(
        vehicle_types=[base.vehicle_type(0).replace(custom_breaks=[brk])]
    )

    route = SearchRoute(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.update()

    # The vehicle arrives at the break at 1600 (1000 travel + 600 service),
    # well past the close (200). Because the break is not due there, the close
    # is not enforced: no warp, no violation, route is feasible.
    assert_equal(route.time_warp(), 0)
    assert_equal(route.break_due(), 0)
    assert_(route.is_feasible())


def test_due_break_closed_absolute_window_still_warps():
    """
    Regression guard for the due-ness gate: when the break IS due at its node,
    the closed absolute window must still produce time warp (existing
    behaviour, see test_break_window_close_is_hard). This pins the boundary
    between the two cases in the same route shape as the non-due test.
    """
    base = _hard_window_chain()

    brk = CustomBreak(
        id=1,
        tws=[(100, 200)],
        service=100,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,  # due immediately
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
    )
    data = base.replace(
        vehicle_types=[base.vehicle_type(0).replace(custom_breaks=[brk])]
    )

    route = SearchRoute(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.update()

    assert_(route.time_warp() > 0)
    assert_(not route.is_feasible())
