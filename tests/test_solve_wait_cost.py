"""
End-to-end idle waiting cost tests for the production solve path.

Verifies that the ``wait_cost_rate`` calibration actually reaches the
``CostEvaluator`` used by :meth:`Model.solve` (via ``pyvrp.solve.solve``), and
that it can change the reported solution when the instance contains waiting.
Waiting is NOT part of the duration cost (single-charge semantics).
"""
from numpy.testing import assert_, assert_equal

from pyvrp import (
    CostEvaluator,
    IteratedLocalSearchCallbacks,
    IteratedLocalSearchParams,
    Model,
    Solution,
)
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations


def _wait_tradeoff():
    """
    Builds a tiny single-vehicle instance with a waiting trade-off.

    ``C1`` opens at 5_000; the vehicle departs at time 0 (``start_late=0``),
    so visiting ``C1`` first forces 4_900 units of waiting, while visiting
    ``C0`` first forces only 3_000. Both visit orders have the same distance
    (2_100) and ``unit_duration_cost=0``, so the idle penalty is the only
    term that distinguishes them.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(
        c0, delivery=0, service_duration=0, tw_early=0, tw_late=200_000,
        required=True,
    )
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(
        c1, delivery=0, service_duration=0, tw_early=5_000, tw_late=200_000,
        required=True,
    )

    for frm in [depot, c0, c1]:
        for to in [depot, c0, c1]:
            if frm is to:
                m.add_edge(frm, to, distance=0, duration=0)
            elif (frm is depot and to is c1) or (to is depot and frm is c1):
                m.add_edge(frm, to, distance=100, duration=100)
            else:
                m.add_edge(frm, to, distance=1_000, duration=1_000)

    # ``start_late=0`` forces a fixed departure at time 0, so the late window
    # of C1 produces waiting; ``unit_duration_cost=0`` keeps the idle penalty
    # as the only term that distinguishes the two visit orders.
    m.add_vehicle_type(
        num_available=1,
        capacity=[9999],
        unit_distance_cost=1,
        unit_duration_cost=0,
        start_late=0,
    )
    data = m.data()
    # Initial solution visits C1 (idx 1) before C0 (idx 0): the high-waiting
    # order (4_900 units of waiting).
    init = Solution(data, [[1, 0]])
    return m, init


class _CaptureWaitCost(IteratedLocalSearchCallbacks):
    def __init__(self):
        self.waits: list[int] = []

    def on_iteration(self, current, candidate, best, cost_evaluator):
        self.waits.append(cost_evaluator.wait_penalty(1))


def test_solve_plumbs_wait_cost_rate():
    """
    ``wait_cost_rate`` flows from ``Model.solve`` into the ``CostEvaluator``
    used by the iterated local search loop.
    """
    m, init = _wait_tradeoff()

    cb = _CaptureWaitCost()
    params = SolveParams(IteratedLocalSearchParams(callbacks=cb))
    m.solve(
        stop=MaxIterations(5),
        seed=0,
        params=params,
        initial_solution=init,
        wait_cost_rate=25,
    )
    assert_(cb.waits, "expected at least one iteration")
    assert_equal(cb.waits, [25] * len(cb.waits))


def test_solve_wait_cost_rate_changes_solution():
    """
    A positive ``wait_cost_rate`` changes the best solution when the instance
    contains avoidable waiting: the search reorders the route to reduce idle.
    """
    m, init = _wait_tradeoff()

    res_0 = m.solve(
        stop=MaxIterations(20),
        seed=0,
        initial_solution=init,
        wait_cost_rate=0,
    )
    res_w = m.solve(
        stop=MaxIterations(20),
        seed=0,
        initial_solution=init,
        wait_cost_rate=25,
    )

    # Without a wait cost the high-waiting order is not improved upon.
    assert_equal(res_0.best.routes()[0].wait_duration(), 4_900)
    # With the wait cost the search finds the low-waiting order.
    assert_equal(res_w.best.routes()[0].wait_duration(), 3_000)

    # The wait-inclusive cost is strictly lower after calibration.
    cost_w = CostEvaluator([0], 0, 0, 0, 25)
    assert_(
        cost_w.penalised_cost(res_w.best) < cost_w.penalised_cost(res_0.best)
    )


def test_solve_wait_cost_rate_default_zero_removes_waiting_from_objective():
    """
    Not passing ``wait_cost_rate`` is equivalent to passing 0: waiting is not
    charged anywhere in the objective (duration cost excludes it and the wait
    rate is zero), so the search keeps the high-waiting order. This is the
    NEW intentional behaviour — the old "waiting inside the duration cost"
    is no longer reachable.
    """
    m, init = _wait_tradeoff()

    res_default = m.solve(
        stop=MaxIterations(20), seed=0, initial_solution=init
    )
    res_zero = m.solve(
        stop=MaxIterations(20), seed=0, initial_solution=init,
        wait_cost_rate=0,
    )

    assert_equal(res_default.best, res_zero.best)
    assert_equal(res_default.best.routes()[0].wait_duration(), 4_900)


def test_result_cost_includes_wait_cost_term():
    """
    ``Result.cost()`` — and the ``objective`` line of ``Result.summary()``,
    which is derived from it — must reflect the wait-cost term of the solve
    that produced the result.

    Today ``Result.cost()`` rebuilds a ZERO wait-cost-rate evaluator, so it
    under-reports the waiting charged during ``solve(wait_cost_rate=25)``: the
    business objective is ``distance + wait×25`` (duration cost is zero here),
    but ``Result.cost()`` returns distance alone.
    """
    m, init = _wait_tradeoff()

    res = m.solve(
        stop=MaxIterations(20),
        seed=0,
        initial_solution=init,
        wait_cost_rate=25,
    )

    # Sanity: the calibrated solve actually left waiting to charge (the
    # low-waiting order still waits 3_000 units), so the wait term is nonzero
    # and the assertion below is meaningful rather than vacuously 0 == 0.
    assert_equal(res.best.routes()[0].wait_duration(), 3_000)

    num_load_dims = len(res.best.excess_load())
    wait_inclusive = CostEvaluator([0] * num_load_dims, 0, 0, 0, 25)
    expected = wait_inclusive.cost(res.best)

    assert_equal(res.cost(), expected)

    obj_line = next(
        line for line in res.summary().splitlines()
        if line.strip().startswith("objective:")
    )
    assert_equal(obj_line.strip(), f"objective: {expected}")
