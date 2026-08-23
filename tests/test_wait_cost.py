"""
Idle waiting cost tests (wait-cost-root-fix).

Verifies:
  - CostEvaluator gains wait_cost_rate (default 0) and wait_penalty.
  - Duration cost EXCLUDES waiting; the wait cost is the TOTAL charge for
    idle time (no double charge).
  - Default 0 means waiting costs nothing (new, intentional behaviour).
  - Waiting absorbed into an overnight rest extension is not costed as idle.
  - Solution.waiting() aggregates per-route waiting.
  - A wait-reducing move is accepted by the non-exact delta-cost path.
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
    Model,
    Solution,
)
from pyvrp.search import (
    NeighbourhoodParams,
    PerturbationManager,
    PerturbationParams,
    compute_neighbours,
)
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


def test_wait_cost_rate_default_zero():
    cost_eval = CostEvaluator([0], 0, 0)
    assert_equal(cost_eval.wait_penalty(3_600), 0)


def test_wait_cost_is_single_charge():
    """
    Duration cost excludes waiting, so the wait cost is the SINGLE charge
    for idle time: penalised cost with rate r equals the zero-rate cost plus
    exactly r x waiting (no double counting through the duration cost).
    """
    solution = _solution_with_waiting(tw_early_c1=5_000)
    waiting = solution.waiting()
    assert_equal(waiting, 2_400)  # 5000 - 2600 arrival

    base = CostEvaluator([0], 0, 0, 0, 0)
    with_wait = CostEvaluator([0], 0, 0, 0, 25)

    assert_equal(with_wait.penalised_cost(solution),
                 base.penalised_cost(solution) + 25 * waiting)


def test_duration_cost_excludes_waiting():
    """
    Closed form: with unit_distance_cost=0 the duration cost covers only the
    active time (travel + service + setup), never the waiting.
    """
    solution = _solution_with_waiting(tw_early_c1=5_000)
    waiting = solution.waiting()  # 2400
    total_duration = solution.duration()

    cost_eval = CostEvaluator([0], 0, 0, 0, 0)
    # duration cost = 1 x (total - waiting); no wait term at rate 0.
    assert_equal(cost_eval.penalised_cost(solution),
                 (total_duration - waiting) * 1)


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


def test_wait_rate_selects_low_waiting_order():
    """
    Single-charge semantics via direct evaluator comparison (no local
    search: the PerturbationManager re-inserts clients at cheapest
    positions on every LS call, which would obscure the delta accounting).
    Two orders of the same route: [B, A] (waiting 9900) and [A, B]
    (waiting 9800). Both have identical active duration (300) and no
    distance/fixed/prize terms, so:
      - at rate 0 waiting is free: the orders cost the same;
      - at rate 25 the low-waiting order is cheaper by exactly 25 x 100
        (the duration cost excludes waiting — no double charge).
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

    def make(order):
        sol = SearchSolution(data)
        route = sol.routes[0]
        for name in order:
            route.append(Node(name))
        route.update()
        return sol.unload()

    high = make(["C1", "C0"])  # [B, A]: waiting 9900
    low = make(["C0", "C1"])   # [A, B]: waiting 9800
    assert_equal(high.routes()[0].wait_duration(), 9_900)
    assert_equal(low.routes()[0].wait_duration(), 9_800)

    ce0 = CostEvaluator([0], 0, 0, 0, 0)
    ce25 = CostEvaluator([0], 0, 0, 0, 25)

    # Active duration identical (300): equal cost at rate 0.
    assert_equal(ce0.penalised_cost(high), ce0.penalised_cost(low))
    # Single charge at rate 25: low-wait cheaper by exactly 25 x 100.
    assert_equal(ce25.penalised_cost(high) - ce25.penalised_cost(low),
                 25 * 100)
    # Closed form on the low-waiting solution: active x 1 + waiting x 25.
    assert_equal(ce25.penalised_cost(low), 300 + 25 * 9_800)


def test_wait_reducing_move_accepted_non_exact():
    """
    D7: a wait-reducing exchange is improving at rate > 0 and is applied by
    the non-exact delta-cost path; at rate 0 it is non-improving and the
    wait-heavy order survives. The LS runs with ZERO perturbations
    (min=max=0), so the input order is preserved and the only source of
    change is the operator delta evaluation. Clients B and A are made
    NON-adjacent (C sits between them) because Exchange10/11 skip adjacent
    pairs by design.
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
    c = m.add_location(x=3, y=0, name="C")
    m.add_client(c, delivery=0, service_duration=0, tw_early=0,
                 tw_late=100_000, required=True)
    for f in [depot, a, b, c]:
        for t in [depot, a, b, c]:
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
    no_perturb = PerturbationManager(
        PerturbationParams(min_perturbations=0, max_perturbations=0))

    def run(rate):
        sol = SearchSolution(data)
        route = sol.routes[0]
        route.append(Node("C1"))  # [B, C, A]: waiting at B = 9900
        route.append(Node("C2"))
        route.append(Node("C0"))
        route.update()
        ls = LocalSearch(data, neighbours, perturbation_manager=no_perturb)
        ls.add_operator(Exchange10(data))
        ls.add_operator(Exchange11(data))
        # exhaustive=True marks every client promising WITHOUT perturbing
        # the input order (the zero-perturbation manager removes nothing);
        # the move deltas are still evaluated on the non-exact path.
        return ls(sol.unload(), CostEvaluator([0], 0, 0, 0, rate),
                  exhaustive=True)

    out_0 = run(0)
    out_25 = run(25)

    # At rate 0 the exchange is non-improving (delta 0): the wait-heavy
    # order survives.
    assert_equal(out_0.routes()[0].wait_duration(), 9_900)
    # At rate 25 a wait-reducing move (exchange B<->A or relocate, both
    # non-adjacent) is applied on the non-exact path.
    assert_(out_25.routes()[0].wait_duration() < 9_900)


def test_closed_form_with_overtime():
    """
    Overtime stays on the FULL duration (including waiting): with
    shift_duration set below the route duration, the closed form is
    active×c_dur + W×rate + overtime×c_ovt.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=600, tw_early=5_000,
                 tw_late=200_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()
    vt = base.vehicle_type(0).replace(unit_distance_cost=0,
                                      unit_duration_cost=1,
                                      unit_overtime_cost=1,
                                      shift_duration=2_000,
                                      start_late=0)
    data = base.replace(vehicle_types=[vt])

    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node("C1"))
    route.update()
    out = sol.unload()

    r = out.routes()[0]
    total = int(r.duration())        # 8200 (3000 travel + 1200 svc + 2400 wait... )
    wait = int(r.wait_duration())    # 2400
    overtime = int(r.overtime())     # 8200 - 2000 = 6200 (full duration)
    assert_equal(wait, 2_400)
    assert_(overtime > 0)

    ce = CostEvaluator([0], 0, 0, 0, 25)
    expected = (total - wait) * 1 + wait * 25 + overtime * 1
    assert_equal(ce.penalised_cost(out), expected)


def test_closed_form_with_uncollected_prizes():
    """
    Uncollected prizes of unvisited OPTIONAL clients are part of the
    penalised cost: dist×c_dist + (T−W)×c_dur + W×rate + prizes.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=200_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=600, tw_early=0,
                 tw_late=200_000, required=False, prize=500)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()
    vt = base.vehicle_type(0).replace(unit_distance_cost=1,
                                      unit_duration_cost=1,
                                      start_late=0)
    data = base.replace(vehicle_types=[vt])

    # Visit only C0: optional C1's prize (500) remains uncollected.
    out = Solution(data, [[0]])
    assert_(out.is_feasible())
    r = out.routes()[0]
    dist = int(r.distance())
    total = int(r.duration())
    wait = int(r.wait_duration())

    ce = CostEvaluator([0], 0, 0, 0, 25)
    expected = dist * 1 + (total - wait) * 1 + wait * 25 + 500
    assert_equal(ce.penalised_cost(out), expected)


def test_break_route_accessor_parity():
    """
    On a break-configured route the search-side and Solution-side waiting
    accessors agree (the D5 rest-extension divergence point), and the
    penalised cost equals the closed form.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=600, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=600, tw_early=45_800,
                 tw_late=300_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=1_000, duration=1_000)
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

    # Parity: search-side waiting == Solution-side waiting (break route).
    assert_equal(route.waiting(), out.routes()[0].wait_duration())
    # Closed form on the Solution (waiting absorbed into the rest extension:
    # here the extension absorbs the would-be idle, so waiting == 0).
    r = out.routes()[0]
    assert_equal(r.wait_duration(), 0)
    total = int(r.duration())
    ce = CostEvaluator([0], 0, 0, 0, 25)
    expected = (total - 0) * 1 + 0 * 25
    assert_equal(ce.penalised_cost(out), expected)
