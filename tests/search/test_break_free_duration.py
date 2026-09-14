"""
break-free-duration (``PYVRP_BREAK_FREE_DURATION``): a served break's service
leaves the duration cost when the switch is on, exactly as waiting does; off,
``duration_cost == unit * (duration - waiting)`` as before.

The switch is read once when the extension loads, so the expectations follow
``_search.BREAK_FREE_DURATION`` (the value in force) rather than the
environment. Run the suite twice to cover both regimes.
"""
from numpy.testing import assert_equal

from pyvrp import (
    ActivityType,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Model,
)
from pyvrp.search._search import (
    BREAK_FREE_DURATION,
    Node,
    Solution as SearchSolution,
)

SVC = 600
TRAVEL = 1_000
REST = 39_600  # 11h
EXTENDED = 43_200  # 12h: the rest extends to C1's window (D5)


def _overnight_chain(tw_early_c1=1_600 + EXTENDED + TRAVEL):
    """
    depot -> C0 -> (break) -> C1 -> depot, fixed departure (start_late=0),
    unit_duration_cost=1 and unit_distance_cost=0. The break arrives at 1600;
    C1 opens at ``tw_early_c1``, so the served rest extends to EXTENDED.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=tw_early_c1,
                 tw_late=300_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        service=REST,
    )
    vt = base.vehicle_type(0).replace(
        custom_breaks=[brk],
        start_late=0,
        unit_duration_cost=1,
        unit_distance_cost=0,
    )
    return base.replace(vehicle_types=[vt])


def _route(data, with_break: bool):
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    if with_break:
        route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()
    return sol, route


def test_served_break_service_leaves_duration_cost_when_on():
    data = _overnight_chain()
    _, route = _route(data, with_break=True)

    assert_equal(route.break_service(), EXTENDED)  # D5-extended, served
    assert_equal(route.waiting(), 0)  # absorbed into the rest
    assert_equal(route.duration(), 1_600 + EXTENDED + 1_000 + SVC + 1_000)

    active = route.duration() - route.waiting()
    if BREAK_FREE_DURATION:
        # Exactly unit_duration_cost x S cheaper, S the extended service.
        assert_equal(route.duration_cost(), active - EXTENDED)
    else:
        assert_equal(route.duration_cost(), active)


def test_break_less_route_is_unaffected():
    data = _overnight_chain()
    _, route = _route(data, with_break=False)

    assert_equal(route.break_service(), 0)
    # Without the rest the same clock is idle waiting instead.
    assert_equal(route.waiting(), EXTENDED)
    assert_equal(route.duration_cost(), route.duration() - route.waiting())


def test_solution_level_route_applies_the_same_rule():
    """
    ``CostEvaluator.penalised_cost`` on the unloaded solution (what the
    production harness reports) must price the served rest exactly like the
    search route did, in either regime.
    """
    data = _overnight_chain()
    sol, route = _route(data, with_break=True)
    unloaded = sol.unload()
    out = unloaded.routes()[0]

    assert_equal(out.break_services()[1], EXTENDED)
    assert_equal(out.duration_cost(), route.duration_cost())

    # The calibrated objective on the unloaded solution is the search route's
    # own closed form: duration cost + waiting at the wait rate (distance 0).
    ev = CostEvaluator([0], 0, 0, 0, 25)  # wait rate 25/s
    assert_equal(ev.cost(unloaded), route.duration_cost() + 25 * route.waiting())
