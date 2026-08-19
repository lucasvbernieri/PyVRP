"""
Unit tests for the LocalSearch termination valve (D5).

Verifies:
  - set_max_updates() bounds the number of solution updates in a single
    search() invocation and makes valve_triggered() True.
  - set_time_budget() with an already-expired deadline triggers the valve.
  - parity_violations() exposes a non-negative count and is 0 on a simple
    break-less instance (a healthy search has exact cost deltas).

Run:
    python -m pytest tests/test_local_search_valve.py -q -v
"""
import pyvrp
from numpy.testing import assert_, assert_equal

from pyvrp import CostEvaluator, RandomNumberGenerator
from pyvrp.search import LocalSearch, PerturbationManager, compute_neighbours
from pyvrp.solve import SolveParams


def _make_ls(data, seed=42):
    """
    Build a LocalSearch with the default SolveParams operators (mirroring the
    ILS setup), so the search actually applies improving moves on a
    sub-optimal initial solution.
    """
    params = SolveParams()
    rng = RandomNumberGenerator(seed=seed)
    ls = LocalSearch(
        data,
        rng,
        compute_neighbours(data, params.neighbourhood),
        PerturbationManager(params.perturbation),
    )
    for op in params.operators:
        if op.supports(data):
            ls.add_operator(op(data))
    return ls


def test_set_max_updates_triggers_valve(ok_small):
    """
    set_max_updates() bounds the number of solution updates in a single
    search() call: with a sub-optimal initial solution and a move cap of 1,
    the search applies an improving move and the valve fires on the next
    pass instead of oscillating forever.
    """
    ls = _make_ls(ok_small)
    solution = pyvrp.Solution.make_random(ok_small,
                                          RandomNumberGenerator(seed=42))
    ls.set_max_updates(1)
    ls(solution, CostEvaluator([20], 1, 1), exhaustive=True)
    assert_(ls.valve_triggered())


def test_set_time_budget_triggers_valve(ok_small):
    """
    set_time_budget() with an already-expired deadline makes the valve fire:
    the wall-clock deadline is checked before each search step, so a budget
    that is already past terminates the search gracefully.
    """
    ls = _make_ls(ok_small)
    solution = pyvrp.Solution.make_random(ok_small,
                                          RandomNumberGenerator(seed=42))
    ls.set_time_budget(1e-9)  # deadline já expira antes do search começar
    ls(solution, CostEvaluator([20], 1, 1), exhaustive=True)
    assert_(ls.valve_triggered())


def test_parity_violations_zero_simple_case(ok_small):
    """
    parity_violations() is non-negative and 0 on a simple break-less
    instance: a healthy search has exact cost deltas, so no parity
    divergence between the evaluated delta and the recomputed cost.
    """
    ls = _make_ls(ok_small)
    solution = pyvrp.Solution.make_random(ok_small,
                                          RandomNumberGenerator(seed=42))
    ls(solution, CostEvaluator([20], 1, 1), exhaustive=True)
    assert_(ls.parity_violations() >= 0)
    assert_equal(ls.parity_violations(), 0)
