"""
End-to-end idle waiting cost tests for the production solve path.

Verifies that the ``unit_wait_cost`` calibration actually reaches the
``CostEvaluator`` used by :meth:`Model.solve` (via ``pyvrp.solve.solve``), and
that it can change the reported solution when the instance contains waiting.
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


def test_solve_plumbs_unit_wait_cost():
    """
    ``unit_wait_cost`` flows from ``Model.solve`` into the ``CostEvaluator``
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
        unit_wait_cost=25,
    )
    assert_(cb.waits, "expected at least one iteration")
    assert_equal(cb.waits, [25] * len(cb.waits))


def test_solve_unit_wait_cost_changes_solution():
    """
    A positive ``unit_wait_cost`` changes the best solution when the instance
    contains avoidable waiting: the search reorders the route to reduce idle.
    """
    m, init = _wait_tradeoff()

    res_0 = m.solve(
        stop=MaxIterations(20),
        seed=0,
        initial_solution=init,
        unit_wait_cost=0,
    )
    res_w = m.solve(
        stop=MaxIterations(20),
        seed=0,
        initial_solution=init,
        unit_wait_cost=25,
    )

    # Without an idle penalty the high-waiting order is not improved upon.
    assert_equal(res_0.best.routes()[0].wait_duration(), 4_900)
    # With the idle penalty the search finds the low-waiting order.
    assert_equal(res_w.best.routes()[0].wait_duration(), 3_000)

    # The wait-inclusive cost is strictly lower after calibration.
    cost_w = CostEvaluator([0], 0, 0, 0, 25)
    assert_(
        cost_w.penalised_cost(res_w.best) < cost_w.penalised_cost(res_0.best)
    )


def test_solve_unit_wait_cost_default_zero_preserves_behaviour():
    """
    Not passing ``unit_wait_cost`` is equivalent to passing 0: the search does
    not apply an idle penalty and keeps the high-waiting order.
    """
    m, init = _wait_tradeoff()

    res_default = m.solve(
        stop=MaxIterations(20), seed=0, initial_solution=init
    )
    res_zero = m.solve(
        stop=MaxIterations(20), seed=0, initial_solution=init,
        unit_wait_cost=0,
    )

    assert_equal(res_default.best, res_zero.best)
    assert_equal(res_default.best.routes()[0].wait_duration(), 4_900)
