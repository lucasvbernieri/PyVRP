"""
Idle waiting cost tests (OpenSpec task 3.5 / design D6).

Verifies:
  - CostEvaluator gains unit_wait_cost (default 0) and wait_penalty.
  - The wait cost is additive over unit_duration_cost.
  - The default preserves current behaviour (no wait cost).
  - Waiting absorbed into an overnight rest extension is not costed as idle.
  - Solution.waiting() aggregates per-route waiting.
"""
import numpy as np
import pytest
from numpy.testing import assert_, assert_equal

from pyvrp import (
    Activity,
    ActivityType,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Model,
    Solution,
)
from pyvrp.search import NeighbourhoodParams, compute_neighbours
from pyvrp.search._search import (
    Exchange10,
    Exchange11,
    LocalSearch,
    Node,
    Solution as SearchSolution,
)

TRAVEL = 1_000
SVC = 600


def _wait_chain(tw_early_c1=5_000):
    """
    depot -> C0 -> C1 -> depot, fixed departure (start_late=0). C1's window
    opens at tw_early_c1, forcing real (unavoidable) waiting at C1.

    With TRAVEL=1000, SVC=600: arrival at C1 = 2600; waiting = tw_early_c1 - 2600.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=200_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=tw_early_c1,
                 tw_late=200_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()
    # unit_distance_cost=0 isolates the duration/wait terms.
    vt = base.vehicle_type(0).replace(unit_distance_cost=0,
                                      unit_duration_cost=1,
                                      start_late=0)
    return base.replace(vehicle_types=[vt])


def _solution_with_waiting(tw_early_c1=5_000):
    data = _wait_chain(tw_early_c1)
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.update()
    return sol.unload()


def test_wait_penalty():
    cost_eval = CostEvaluator([0], 0, 0, 0, 25)
    assert_equal(cost_eval.wait_penalty(0), 0)
    assert_equal(cost_eval.wait_penalty(100), 2_500)


def test_unit_wait_cost_default_zero():
    cost_eval = CostEvaluator([0], 0, 0)
    assert_equal(cost_eval.wait_penalty(3_600), 0)


def test_wait_cost_is_additive():
    """
    The wait cost adds unit_wait_cost x waiting on top of the duration cost
    (which already includes the waiting at unit_duration_cost).
    """
    solution = _solution_with_waiting(tw_early_c1=5_000)
    waiting = solution.waiting()
    assert_equal(waiting, 2_400)  # 5000 - 2600 arrival

    base = CostEvaluator([0], 0, 0, 0, 0)
    with_wait = CostEvaluator([0], 0, 0, 0, 25)

    assert_equal(with_wait.penalised_cost(solution),
                 base.penalised_cost(solution) + 25 * waiting)


def test_solution_waiting_aggregates():
    solution = _solution_with_waiting(tw_early_c1=5_000)
    assert_equal(solution.waiting(), 2_400)
    assert_equal(solution.routes()[0].wait_duration(), 2_400)


def test_extension_does_not_charge_idle():
    """
    Waiting absorbed into an overnight rest extension is not costed as idle:
    the extension lengthens the rest service and the route's waiting is zero.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=45_800,
                 tw_late=300_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk = CustomBreak(id=1, trigger=CustomBreakTrigger.DUTY_TIME,
                      trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                      mandatory=True, service=39_600)
    vt = base.vehicle_type(0).replace(custom_breaks=[brk],
                                      unit_distance_cost=0,
                                      unit_duration_cost=1,
                                      start_late=0)
    data = base.replace(vehicle_types=[vt])

    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()
    out = sol.unload()

    # The 1h would-be waiting is absorbed into the 12h rest: zero idle.
    assert_equal(out.waiting(), 0)
    assert_equal(out.routes()[0].wait_duration(), 0)
    assert_equal(out.routes()[0].break_services()[1], 43_200)  # 12h

    base_cost = CostEvaluator([0], 0, 0, 0, 0)
    wait_cost = CostEvaluator([0], 0, 0, 0, 25)
    assert_equal(wait_cost.penalised_cost(out), base_cost.penalised_cost(out))


def test_delta_cost_accounts_for_waiting():
    """
    The wait cost flows through the search's delta-cost evaluation: running
    local search with unit_wait_cost produces a solution whose penalised cost
    includes unit_wait_cost x waiting.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=100_000)
    a = m.add_location(x=1, y=0, name="A")
    m.add_client(a, delivery=0, service_duration=0, tw_early=0,
                 tw_late=100_000, required=True)
    b = m.add_location(x=2, y=0, name="B")
    m.add_client(b, delivery=0, service_duration=0, tw_early=10_000,
                 tw_late=100_000, required=True)
    for f in [depot, a, b]:
        for t in [depot, a, b]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=0, duration=100)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()
    vt = base.vehicle_type(0).replace(unit_distance_cost=0,
                                      unit_duration_cost=1,
                                      start_late=0)
    data = base.replace(vehicle_types=[vt])

    neighbours = compute_neighbours(data, NeighbourhoodParams())

    def run(unit_wait):
        sol = SearchSolution(data)
        route = sol.routes[0]
        route.append(Node("C1"))
        route.append(Node("C0"))
        route.update()
        ls = LocalSearch(data, neighbours)
        ls.add_operator(Exchange10(data))
        ls.add_operator(Exchange11(data))
        cost_eval = CostEvaluator([0], 0, 0, 0, unit_wait)
        return ls(sol.unload(), cost_eval), cost_eval

    out_0, cost_0 = run(0)
    out_w, cost_w = run(25)

    # Both runs converge to the low-waiting order [A, B].
    assert_(out_0.is_feasible())
    assert_(out_w.is_feasible())
    assert_equal(out_w.routes()[0].wait_duration(),
                 out_0.routes()[0].wait_duration())

    waiting = out_w.routes()[0].wait_duration()
    assert_(waiting > 0)
    # The wait cost is additive over the base (duration) cost.
    assert_equal(cost_w.penalised_cost(out_w),
                 cost_0.penalised_cost(out_w) + 25 * waiting)
