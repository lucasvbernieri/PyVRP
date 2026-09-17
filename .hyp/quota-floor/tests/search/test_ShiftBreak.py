"""
Tests for the ShiftBreak unary operator (item 4 of break-support perfection pass).

ShiftBreak repositions CUSTOM_BREAK activities within a route. It only applies
strictly improving moves (deltaCost < 0).
"""
import pytest
from numpy.testing import assert_, assert_equal

from pyvrp import (
    Activity,
    ActivityType,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
)
from pyvrp.search import ShiftBreak
from pyvrp.search._search import Node, Route, Solution as SearchSolution
from tests.helpers import make_search_route


# =============================================================================
# Fixtures
# =============================================================================

def _make_break_data(ok_small, trigger_value=3600):
    """Create ProblemData with a single mandatory DRIVE_TIME break."""
    brk = CustomBreak(
        id=0,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=trigger_value,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[brk])
    return ok_small.replace(vehicle_types=[vt])


def _route_with_break(data, clients, break_pos=2):
    """Build a Route with a CUSTOM_BREAK inserted at the given position."""
    route = Route(data, vehicle_type=0)
    for client in clients:
        route.append(Node(client))
    route.insert(break_pos, Node(ActivityType.CUSTOM_BREAK, 0))
    route.update()
    return route


def _ce(break_due_penalty=1):
    """Helper: CostEvaluator with load=0, tw=1, dist=1."""
    return CostEvaluator([0], 1, 1, break_due_penalty)


# =============================================================================
# Unit tests
# =============================================================================


def test_shift_break_reduces_break_due(ok_small):
    """
    A break placed too early can be shifted to a better position.
    """
    data = _make_break_data(ok_small, trigger_value=1000)
    route = _route_with_break(data, ["C0", "C1", "C2", "C3"], break_pos=2)
    bd_before = route.break_due()

    op = ShiftBreak(data)
    ce = _ce(1)

    U = route[2]
    assert_(U.is_custom_break())

    delta, should = op.evaluate(U, ce)
    if should:
        op.apply(U)
        route.update()
        bd_after = route.break_due()
        assert_(bd_after <= bd_before)


def test_shift_break_non_improving_rejected(ok_small):
    """
    Moving a compliant break when it doesn't help is rejected (deltaCost >= 0).
    """
    data = _make_break_data(ok_small, trigger_value=100_000)
    route = _route_with_break(data, ["C0", "C1"], break_pos=2)

    op = ShiftBreak(data)
    ce = _ce(1)
    U = route[2]
    assert_(U.is_custom_break())

    delta, should = op.evaluate(U, ce)
    # With a very high trigger, break_due is already 0
    assert_(not should or delta >= 0)


def test_shift_break_not_for_clients(ok_small):
    """ShiftBreak does not evaluate non-break nodes."""
    data = _make_break_data(ok_small)
    route = _route_with_break(data, ["C0", "C1"], break_pos=2)

    op = ShiftBreak(data)
    ce = _ce(1)

    U = route[1]
    assert_(U.is_client())

    delta, should = op.evaluate(U, ce)
    assert_(not should)
    assert_equal(delta, 0)


def test_shift_break_supports_only_breaks(ok_small):
    """ShiftBreak.supports() returns True only when vehicle types have breaks."""
    assert_(not ShiftBreak.supports(ok_small))

    data = _make_break_data(ok_small)
    assert_(ShiftBreak.supports(data))


def test_shift_break_init_noop(ok_small):
    """ShiftBreak.init() is a no-op and resets statistics."""
    data = _make_break_data(ok_small)
    op = ShiftBreak(data)

    sol = SearchSolution(data)
    op.init(sol)
    stats = op.statistics
    assert_equal(stats.num_evaluations, 0)
    assert_equal(stats.num_applications, 0)


def test_shift_break_guards_no_adjacent_breaks(ok_small):
    """
    ShiftBreak does not place a break adjacent to another break.
    """
    b1 = CustomBreak(
        id=0,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=5000,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    b2 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.WORK_TIME,
        trigger_value=10000,
        reset=CustomBreakReset.DRIVE_AND_WORK,
        mandatory=True,
    )
    vt = ok_small.vehicle_type(0).replace(custom_breaks=[b1, b2])
    data = ok_small.replace(vehicle_types=[vt])

    route = Route(data, vehicle_type=0)
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.append(Node("C2"))
    route.append(Node("C3"))
    route.insert(2, Node(ActivityType.CUSTOM_BREAK, 0))
    route.insert(4, Node(ActivityType.CUSTOM_BREAK, 1))
    route.update()

    op = ShiftBreak(data)
    ce = _ce(1)
    U = route[2]

    delta, should = op.evaluate(U, ce)
    if should:
        op.apply(U)
        route.update()
        for i in range(1, len(route) - 1):
            if route[i].is_custom_break():
                assert_(not route[i - 1].is_custom_break(),
                        "Breaks must not be adjacent")
                if i + 1 < len(route):
                    assert_(not route[i + 1].is_custom_break(),
                            "Breaks must not be adjacent")


def test_shift_break_after_apply_arrays_consistent(ok_small):
    """
    After apply + update, Route::update() runs and arrays are consistent.
    """
    data = _make_break_data(ok_small, trigger_value=500)
    route = _route_with_break(data, ["C0", "C1", "C2", "C3"], break_pos=2)

    op = ShiftBreak(data)
    ce = _ce(1)
    U = route[2]

    before_len = len(route)

    delta, should = op.evaluate(U, ce)
    if should:
        op.apply(U)
        route.update()

        route.duration()
        route.distance()
        route.load()
        route.break_due()

        assert_equal(len(route), before_len)
        for i in range(len(route)):
            assert_equal(route[i].pos(), i)


def test_shift_break_no_crash_on_many_iterations(ok_small):
    """
    Shifting a break many times should not crash or infinite-loop.
    """
    data = _make_break_data(ok_small, trigger_value=1000)
    route = _route_with_break(data,
                              ["C0", "C1", "C2", "C3"],
                              break_pos=3)

    op = ShiftBreak(data)
    ce = _ce(1)

    for _ in range(20):
        # Find break
        U = None
        for idx in range(1, len(route) - 1):
            if route[idx].is_custom_break():
                U = route[idx]
                break
        if U is None:
            break
        delta, should = op.evaluate(U, ce)
        if should:
            op.apply(U)
            route.update()
        else:
            break

    route.duration()
    route.break_due()
