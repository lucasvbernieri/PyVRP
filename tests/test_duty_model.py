"""
Duty model (vehicle types with ``duty_limit >= 0``) and free-start rests
(``CustomBreak.eligible_from >= 0``).

Every fixed-route test below asserts numbers derived by hand from the route,
so running this module under each ``PYVRP_*`` mode of the test matrix
(``PYVRP_INERT_BREAK=0``, ``PYVRP_COMPOSE=0``, ``PYVRP_CLOCK_TRIGGER=0``,
``PYVRP_BREAK_FREE_DURATION=0``, ``PYVRP_PROX_WAIT=0``) is itself the
invariance check of the duty model under those switches. The legacy (-1)
parity against the previous fork lives in ``test_duty_model_legacy_parity``.

The weights mirror what the hows-router sends (its design, section D2), so the
search preferences tested here are the ones production relies on.
"""

import math
import pickle
import warnings
from dataclasses import dataclass, field
from itertools import permutations

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal, assert_raises

import pyvrp
from pyvrp import (
    Activity,
    ActivityType,
    Client,
    ClientGroup,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Depot,
    IteratedLocalSearch,
    Location,
    ProblemData,
    Route,
    Solution,
    VehicleType,
)
from pyvrp.IteratedLocalSearch import IteratedLocalSearchParams
from pyvrp.PenaltyManager import (
    PenaltyBoundWarning,
    PenaltyManager,
    PenaltyParams,
)
from pyvrp.Result import Result
from pyvrp.search import Relocate1, ShiftBreak, Swap11
from pyvrp.search._search import Node
from pyvrp.search._search import Route as SearchRoute
from pyvrp.search._search import Solution as SearchSolution
from pyvrp.search._search import evaluate_forward_pass
from pyvrp.Statistics import Statistics
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations, NoImprovement

H = 3600
DAY = 86400
UNIT_DURATION_COST = 20
SAT = 2**62
BIG = 30 * DAY


# --------------------------------------------------------------- weights (D2)
@dataclass
class Weights:
    T: int
    OT: int
    limit: int
    overtime: int
    early: int
    fork_late: int
    pre_rest: int
    rest_use: int
    unneeded: int
    excess: int


def router_weights(wait_rate, T, OT, n_stops, h_abs, late_mult=10):
    """The weights the hows-router derives (design D2), unsaturated."""
    limit = T + OT
    overtime = max(21, math.ceil(wait_rate) + 1)
    early = max(1, math.ceil(wait_rate))
    late = late_mult * wait_rate
    fork_late = max(
        UNIT_DURATION_COST + 1,
        math.ceil(wait_rate) + 1,
        math.ceil(late) + UNIT_DURATION_COST,
    )
    pre_rest = early * limit + 1
    rest_use = pre_rest * OT + early * limit + 1
    unneeded = (overtime + early + fork_late * (n_stops + 1)) * h_abs
    unneeded += rest_use + 1
    excess = max((overtime + early) * limit, 15 * unneeded, 15 * rest_use) + 1
    return Weights(
        T, OT, limit, overtime, early, fork_late, pre_rest, rest_use,
        unneeded, excess,
    )  # fmt: skip


# ---------------------------------------------------------------- instances
@dataclass
class Stop:
    x: int  # position on the line = travel time from the depot (line mode)
    service: int
    windows: list = field(default_factory=lambda: [(0, BIG)])


@dataclass
class Inst:
    data: ProblemData
    w: Weights
    wait_rate: float
    clients: list  # per stop: client indices, one per window

    def ce(self, tw_penalty=None):
        tw = self.w.fork_late if tw_penalty is None else tw_penalty
        return CostEvaluator([], tw, 0, 0, self.wait_rate)


def rest_rule(idx, T, rest=11 * H, eligible_from=0):
    return CustomBreak(
        id=idx,
        service=rest,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=T,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=20,
        eligible_from=eligible_from,
    )


def build(
    stops,
    *,
    T=8 * H,
    OT=2 * H,
    rest=11 * H,
    n_rests=2,
    wait_rate=21.4,
    departure=8 * H,
    veh_tw_late=BIG,
    start_late=None,
    durations=None,
    distances=None,
    unit_distance_cost=0,
    late_mult=10,
    h_abs=None,
    weights=None,
    duty=True,
):
    """
    ``stops`` sit on a line (travel = |x_i - x_j|) unless ``durations`` gives
    the full (stops + 1) x (stops + 1) matrix, depot first. A stop with more
    than one window becomes one client per window, in a required mutually
    exclusive group (how the router models multi-window stops).
    """
    n = len(stops)
    if durations is None:
        xs = [0] + [s.x for s in stops]
        durations = [[abs(a - b) for b in xs] for a in xs]
    dur = np.array(durations, dtype=np.int64)
    dist = dur if distances is None else np.array(distances, dtype=np.int64)

    h_abs = h_abs if h_abs is not None else departure % DAY + 2 * DAY
    w = weights or router_weights(wait_rate, T, OT, n, h_abs, late_mult)

    locations = [Location(0, 0)] + [Location(i + 1, 0) for i in range(n)]
    clients, groups, per_stop = [], [], []
    for loc, stop in enumerate(stops, start=1):
        idxs = []
        multi = len(stop.windows) > 1
        for early, late in stop.windows:
            idxs.append(len(clients))
            clients.append(
                Client(
                    location=loc,
                    service_duration=stop.service,
                    tw_early=early,
                    tw_late=late,
                    required=not multi,
                    group=len(groups) if multi else None,
                )
            )
        if multi:
            groups.append(ClientGroup(idxs, required=True))
        per_stop.append(idxs)

    kwargs = {}
    if duty:
        kwargs = dict(
            duty_trigger=T,
            duty_limit=w.limit,
            duty_excess_rate=w.excess,
            overtime_rate=w.overtime,
            pre_rest_overtime_rate=w.pre_rest,
            early_rate=w.early,
            rest_use_cost=w.rest_use,
            rest_unneeded_cost=w.unneeded,
        )
    rules = [
        rest_rule(k, T, rest, 0 if duty else -1) for k in range(n_rests)
    ]
    vt = VehicleType(
        capacity=[],
        tw_early=departure,
        start_late=departure if start_late is None else start_late,
        tw_late=veh_tw_late,
        unit_distance_cost=unit_distance_cost,
        unit_duration_cost=UNIT_DURATION_COST,
        custom_breaks=rules,
        **kwargs,
    )
    depot = Depot(location=0)
    data = ProblemData(
        locations, clients, [depot], [vt], [dist], [dur], groups
    )
    return Inst(data, w, wait_rate, per_stop)


def acts(inst, seq):
    """
    Activities for ``seq``: an int is a stop (its first window), a tuple
    ``(stop, window)`` a given window, and ``"B"`` the next rest node.
    """
    out, rest = [], 0
    for tok in seq:
        if tok == "B":
            out.append(Activity(ActivityType.CUSTOM_BREAK, rest))
            rest += 1
            continue
        stop, win = tok if isinstance(tok, tuple) else (tok, 0)
        out.append(Activity(ActivityType.CLIENT, inst.clients[stop][win]))
    return out


def route_of(inst, seq):
    return Route(inst.data, acts(inst, seq), 0)


def pen(inst, route, tw_penalty=None):
    return inst.ce(tw_penalty).penalised_cost(Solution(inst.data, [route]))


def served_rests(route):
    return route.rests_needed() + route.rests_unneeded()


def search_route(inst, seq):
    """The search route the local search evaluates for ``seq``."""
    sol = SearchSolution(inst.data)
    sol.load(Solution(inst.data, [route_of(inst, seq)]))
    return sol, sol.routes[0]


def solve(inst, seed, iters=400, init=None, penalised_best=True):
    fl = inst.w.fork_late
    params = SolveParams(
        penalty=PenaltyParams(min_penalty=fl, max_penalty=fl),
        penalised_best=penalised_best,
    )
    return pyvrp.solve(
        inst.data,
        MaxIterations(iters),
        seed=seed,
        params=params,
        wait_cost_rate=inst.wait_rate,
        initial_solution=init,
        collect_stats=False,
    )


def duty_terms(route):
    return (
        route.duty_excess(),
        route.overtime_band(),
        route.anticipation(),
        route.rests_needed(),
        route.rests_unneeded(),
    )


def assert_all_evaluators(inst, seq, expected):
    """
    The forward pass, the solution route (activities constructor) and the
    search route (Route::update) give the same duty terms, equal to
    ``expected``.
    """
    fwd = evaluate_forward_pass(inst.data, acts(inst, seq), 0)
    assert_equal(duty_terms(fwd), expected)
    assert_equal(duty_terms(route_of(inst, seq)), expected)
    _, sroute = search_route(inst, seq)
    assert_equal(duty_terms(sroute), expected)


# ================================================================ validation
def test_custom_break_eligible_from_validation():
    CustomBreak(id=0, trigger_value=100, eligible_from=0)
    CustomBreak(id=0, trigger_value=100, eligible_from=100)
    CustomBreak(id=0, trigger_value=100)  # -1 default
    assert_equal(CustomBreak(id=0).eligible_from, -1)

    with assert_raises(ValueError):
        CustomBreak(id=0, trigger_value=100, eligible_from=101)
    with assert_raises(ValueError):
        CustomBreak(id=0, trigger_value=100, eligible_from=-2)


def test_vehicle_type_duty_fields_validation():
    vt = VehicleType()
    assert_equal(vt.duty_trigger, -1)
    assert_equal(vt.duty_limit, -1)
    assert_equal(vt.early_rate, 1)
    assert_(not vt.has_duty_model())

    ok = VehicleType(duty_trigger=10, duty_limit=12)
    assert_(ok.has_duty_model())

    # duty_trigger = -1 (or 0) with duty_limit >= 0 is rejected.
    with assert_raises(ValueError):
        VehicleType(duty_trigger=-1, duty_limit=12)
    with assert_raises(ValueError):
        VehicleType(duty_trigger=0, duty_limit=12)
    # duty_limit below duty_trigger.
    with assert_raises(ValueError):
        VehicleType(duty_trigger=10, duty_limit=9)
    with assert_raises(ValueError):
        VehicleType(duty_limit=-2)
    # Weights: >= 0, no upper bound; early_rate >= 1 only on the model.
    for name in (
        "duty_excess_rate",
        "overtime_rate",
        "pre_rest_overtime_rate",
        "rest_use_cost",
        "rest_unneeded_cost",
    ):
        with assert_raises(ValueError):
            VehicleType(duty_trigger=10, duty_limit=12, **{name: -1})
        VehicleType(duty_trigger=10, duty_limit=12, **{name: 2**62})
    with assert_raises(ValueError):
        VehicleType(duty_trigger=10, duty_limit=12, early_rate=0)
    VehicleType(early_rate=0)  # off the model: not validated


def test_rest_models_never_mix():
    new = rest_rule(0, 10)
    old = rest_rule(1, 10, eligible_from=-1)

    # A free-start rule needs a duty-model vehicle...
    with assert_raises(ValueError):
        VehicleType(custom_breaks=[new])
    # ... a duty-model vehicle takes only free-start rules...
    with assert_raises(ValueError):
        VehicleType(duty_trigger=10, duty_limit=12, custom_breaks=[old])
    with assert_raises(ValueError):
        VehicleType(duty_trigger=10, duty_limit=12, custom_breaks=[new, old])
    # ... and the rule's trigger is the vehicle's T.
    with assert_raises(ValueError):
        VehicleType(duty_trigger=11, duty_limit=12, custom_breaks=[new])

    # ... and it is a DUTY_TIME rule resetting ALL_TIMERS.
    for kwargs in (
        dict(trigger=CustomBreakTrigger.WORK_TIME),
        dict(reset=CustomBreakReset.DRIVE_TIMER),
    ):
        args = dict(
            id=0,
            service=5,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=10,
            reset=CustomBreakReset.ALL_TIMERS,
            eligible_from=0,
        )
        args.update(kwargs)
        with assert_raises(ValueError):
            VehicleType(
                duty_trigger=10,
                duty_limit=12,
                custom_breaks=[CustomBreak(**args)],
            )

    VehicleType(duty_trigger=10, duty_limit=12, custom_breaks=[new])
    VehicleType(custom_breaks=[old])


def test_reload_depots_rejected_on_duty_model():
    vt = VehicleType(
        capacity=[1], reload_depots=[0], duty_trigger=10, duty_limit=12
    )
    mat = np.zeros((2, 2), dtype=np.int64)
    with assert_raises(ValueError):
        ProblemData(
            [Location(0, 0), Location(1, 0)],
            [Client(location=1, delivery=[1])],
            [Depot(location=0)],
            [vt],
            [mat],
            [mat],
        )

    # Without reloads, or off the model, the instance is fine.
    ProblemData(
        [Location(0, 0), Location(1, 0)],
        [Client(location=1, delivery=[1])],
        [Depot(location=0)],
        [VehicleType(capacity=[1], reload_depots=[0])],
        [mat],
        [mat],
    )


# =================================================================== pickle
def test_pickle_custom_break_vehicle_type_route():
    brk = rest_rule(3, 100, eligible_from=40)
    after = pickle.loads(pickle.dumps(brk))
    assert_equal(after.eligible_from, 40)

    inst = build([Stop(H, 600), Stop(2 * H, 600)], OT=H)
    vt = inst.data.vehicle_type(0)
    vt_after = pickle.loads(pickle.dumps(vt))
    assert_(vt_after == vt)
    for name in (
        "duty_trigger",
        "duty_limit",
        "duty_excess_rate",
        "overtime_rate",
        "pre_rest_overtime_rate",
        "early_rate",
        "rest_use_cost",
        "rest_unneeded_cost",
    ):
        assert_equal(getattr(vt_after, name), getattr(vt, name))
    assert_(vt.replace(early_rate=7) != vt)
    assert_equal(vt.replace(early_rate=7).early_rate, 7)

    route = route_of(inst, [0, "B", 1])
    route_after = pickle.loads(pickle.dumps(route))
    assert_equal(duty_terms(route_after), duty_terms(route))
    assert_equal(pen(inst, route_after), pen(inst, route))
    assert_equal(route_after.is_feasible(), route.is_feasible())


# ======================================================= fixed-route terms
def test_anticipation():
    # Rest after the first stop: duty 1h + 30min there, T = 8h.
    inst = build([Stop(H, 30 * 60), Stop(2 * H, 30 * 60)])
    seq = [0, "B", 1]
    anticipation = 8 * H - (H + 30 * 60)
    assert_all_evaluators(inst, seq, (0, 0, anticipation, 0, 1))

    # The term is priced at early_rate per second.
    cheaper = build(
        [Stop(H, 30 * 60), Stop(2 * H, 30 * 60)],
        weights=_replace(inst.w, early=inst.w.early - 1),
    )
    diff = pen(inst, route_of(inst, seq)) - pen(cheaper, route_of(cheaper, seq))
    assert_equal(diff, anticipation)


def _replace(w, **kw):
    out = Weights(**w.__dict__)
    for k, v in kw.items():
        setattr(out, k, v)
    return out


def test_overtime_band_and_excess_per_stretch():
    # Stretch 1: 9h (band 1h, ends at a rest); stretch 2: 11h (band 2h, excess
    # 1h, ends at the depot). T = 8h, limit 10h.
    durations = [
        [0, 8 * H, 99 * H],
        [99 * H, 0, 5 * H],
        [5 * H, 99 * H, 0],
    ]
    inst = build([Stop(0, H), Stop(0, H)], durations=durations, n_rests=1)
    seq = [0, "B", 1]
    # J1 = 8h + 1h = 9h; J2 = 5h + 1h + 5h = 11h.
    assert_all_evaluators(inst, seq, (H, H + 2 * H, 0, 1, 0))

    route = route_of(inst, seq)
    w = inst.w
    rest_end_extra = 0
    expected = (
        UNIT_DURATION_COST * (8 * H + H + 5 * H + H + 5 * H)  # active time
        + w.pre_rest * H  # band of the stretch ending at the rest
        + w.overtime * 2 * H  # band of the final stretch
        + w.excess * H  # excess, at max(10 x tw, duty_excess_rate)
        + w.rest_use  # needed: 9h + 0 + 11h > 10h
        + rest_end_extra
    )
    assert_equal(pen(inst, route), expected)
    assert_(not route.is_feasible())


def test_excess_weight_floor_against_low_tw_penalty():
    # The excess weight is max(10 x tw_penalty, duty_excess_rate): with a low
    # tw penalty the duty_excess_rate floor holds, so the fork never trades a
    # second of excess for a day of anticipation.
    durations = [
        [0, 4 * H, 99 * H],
        [99 * H, 0, 3 * H],
        [3 * H + 1, 99 * H, 0],
    ]
    inst = build([Stop(0, 0), Stop(0, 0)], durations=durations, n_rests=1)
    excess_route = route_of(inst, [0, 1, "B"])  # rest parked; 10h + 1 s
    assert_equal(excess_route.duty_excess(), 0)  # rest before return serves
    no_rest = route_of(inst, [0, 1])
    assert_equal(no_rest.duty_excess(), 1)

    early_rest = route_of(inst, [0, "B", 1])  # rest at duty 4h
    assert_equal(early_rest.duty_excess(), 0)
    assert_equal(early_rest.anticipation(), 4 * H)

    for tw_penalty in (1, inst.w.fork_late):
        assert_(pen(inst, no_rest, tw_penalty) > pen(inst, early_rest, tw_penalty))

    # The weight is 10 x tw_penalty when that is larger than the floor.
    # (The excess route is also 1 s longer: one second of duration cost.)
    tw_high = inst.w.excess  # 10 x tw_high > duty_excess_rate
    base = pen(inst, no_rest, tw_high) - 10 * tw_high - UNIT_DURATION_COST
    no_excess = build(
        [Stop(0, 0), Stop(0, 0)],
        durations=[[0, 4 * H, 99 * H], [99 * H, 0, 3 * H], [3 * H, 99 * H, 0]],
        n_rests=1,
    )
    assert_equal(base, pen(no_excess, route_of(no_excess, [0, 1]), tw_high))


def test_zero_rules_with_duty_limit():
    # The fold runs without any rule: excess, band and infeasibility.
    inst = build([Stop(3 * H, H), Stop(6 * H, H)], n_rests=0)
    seq = [0, 1]
    # 3h + 1h + 3h + 1h + 6h = 14h; limit 10h.
    assert_all_evaluators(inst, seq, (4 * H, 2 * H, 0, 0, 0))
    route = route_of(inst, seq)
    assert_(not route.is_feasible())
    assert_(not Solution(inst.data, [route]).is_feasible())

    # deltaCost on the zero-rule vehicle: every improving move's delta is the
    # difference of the penalised costs.
    line = build([Stop(i * 5400, 30 * 60) for i in range(1, 6)], n_rests=0)
    checked = _check_move_parity(line, [4, 0, 3, 1, 2], Relocate1)
    checked += _check_move_parity(line, [4, 0, 3, 1, 2], Swap11)
    assert_(checked > 0)


def test_parked_rests_are_inert():
    inst = build([Stop(H, 600), Stop(2 * H, 600)])
    with_rests = route_of(inst, [0, 1, "B", "B"])
    without = route_of(inst, [0, 1])

    assert_equal(list(with_rests.break_services()), [0, 0, 0, 0])
    assert_equal(with_rests.duration(), without.duration())
    assert_equal(pen(inst, with_rests), pen(inst, without))
    assert_equal(duty_terms(with_rests), (0, 0, 0, 0, 0))

    # The search route agrees, and reports no rest served.
    _, sroute = search_route(inst, [0, 1, "B", "B"])
    assert_equal(sroute.breaks_served(), [])
    assert_equal(duty_terms(sroute), (0, 0, 0, 0, 0))


def test_rest_before_return():
    # Duty at the last stop 9h; the return takes 2h: 11h > 10h without a rest,
    # so the first rest after the last stop is served before the return (the
    # second stays parked).
    durations = [[0, 8 * H], [2 * H, 0]]
    inst = build([Stop(0, H)], durations=durations)
    seq = [0, "B", "B"]
    route = route_of(inst, seq)
    assert_equal(list(route.break_services()), [0, 11 * H, 0])
    # J1 = 9h (band 1h before the rest), J2 = 2h; needed: 9 + 0 + 2 > 10.
    assert_all_evaluators(inst, seq, (0, H, 0, 1, 0))
    _, sroute = search_route(inst, seq)
    assert_equal(sroute.breaks_served(), [0])


def test_rest_before_return_is_forced():
    # Divergence (n): the fork serves the rest before the return whenever the
    # duty to the depot would pass duty_limit, even when that returns hours
    # after the vehicle's end -- the Python criterion may prefer the excess.
    durations = [[0, 8 * H], [H + 30 * 60, 0]]
    inst = build(
        [Stop(0, H)], durations=durations, veh_tw_late=8 * H + 12 * H
    )  # end 20:00
    route = route_of(inst, [0, "B"])
    assert_equal(served_rests(route), 1)
    assert_equal(route.duty_excess(), 0)
    # 8:00 + 9h, rest 11h, 1.5h: back at 05:30 next day, 9.5h after 20:00.
    assert_equal(route.time_warp(), 9 * H + 30 * 60)


def test_overnight_for_a_window_not_open_yet_is_needed():
    # Stop 0 must be served on day 1; stop 1 only opens the next morning:
    # waiting there through the night passes duty_limit, so the rest that
    # absorbs the wait is NEEDED, with OT. (Waiting before the FIRST stop
    # is not duty -- the effective start moves -- so stop 0 closes on day 1.)
    inst = build(
        [
            Stop(H, 30 * 60, [(8 * H, 12 * H)]),
            Stop(H + 30 * 60, 30 * 60, [(DAY + 8 * H, BIG)]),
        ]
    )
    seq = [0, "B", 1]
    route = route_of(inst, seq)
    assert_(route.is_feasible())
    # J1 = 1h30 (anticipation 6h30); counterfactual: 1h30 + 22h of waiting
    # for the next morning + the next stretch > duty_limit: needed.
    assert_all_evaluators(inst, seq, (0, 0, 8 * H - (H + 30 * 60), 1, 0))


def test_client_lateness_counts_once_in_duty():
    # Stop 0 closes at 8:00 and is reached at 9:00: one hour late. The duty at
    # the rest after it is the real 1h + 30min (the clock already includes the
    # lateness); adding the warp again would make it 2h30.
    inst = build(
        [
            Stop(H, 30 * 60, [(0, 8 * H)]),
            Stop(H + 30 * 60, 30 * 60, [(DAY, BIG)]),
        ]
    )
    seq = [0, "B", 1]
    route = route_of(inst, seq)
    assert_equal(route.time_warp(), H)
    assert_all_evaluators(inst, seq, (0, 0, 8 * H - (H + 30 * 60), 1, 0))


# ========================================================== MARILIA (D1/D4)
# Production route of the incident (fork 690fee3 response): departure 04:00,
# stops 1..27 with the arrival/departure the response reported. A rest after
# stop 25 (SUPER GODOY, 3 s late against its 59400 close) is served at duty
# 59906 - 14400 = 45506 s, so the anticipation is 50400 - 45506 = 4894 s in
# every evaluator -- 4891 would mean the 3 s were counted twice.
MARILIA = [
    # (arrival, departure, service, window)
    (29112, 29601, 489, (28800, 64800)),
    (30017, 30520, 503, (28800, 64800)),
    (32400, 32891, 491, (32400, 72000)),
    (33106, 33603, 497, (32400, 68400)),
    (33763, 34244, 481, (28800, 75600)),
    (36063, 36611, 548, (25200, 43200)),
    (36645, 37126, 481, (28800, 64800)),
    (37220, 37709, 489, (28800, 64800)),
    (37734, 38334, 600, (28800, 43200)),
    (38422, 38907, 485, (21600, 43200)),
    (41233, 43040, 1807, (25200, 41400)),
    (43444, 43926, 482, (28800, 64800)),
    (44001, 44485, 484, (28800, 64800)),
    (44639, 45157, 518, (28800, 64800)),
    (45383, 45887, 504, (18000, 79200)),
    (46029, 46528, 499, (28800, 64800)),
    (46668, 47212, 544, (32400, 68400)),
    (47256, 47840, 584, (28800, 64800)),
    (47840, 48364, 524, (28800, 64800)),
    (48645, 49144, 499, (28800, 64800)),
    (50838, 51357, 519, (28800, 64800)),
    (53168, 53654, 486, (30600, 64800)),
    (53654, 55607, 1953, (50400, 63000)),
    (55852, 57742, 1890, (54000, 63000)),
    (59403, 59906, 503, (46800, 59400)),  # SUPER GODOY, 3 s late
    (60020, 60500, 480, (DAY + 28800, DAY + 64800)),
    (63649, 65492, 1843, (DAY + 27300, DAY + 64800)),
]
MARILIA_DEPARTURE = 14400
MARILIA_RETURN = 23995


def marilia(OT=0):
    n = len(MARILIA)
    legs, prev = [], MARILIA_DEPARTURE
    for arrival, departure, service, _ in MARILIA:
        leg = arrival - prev
        if arrival == 32400:  # stop 3 waited 1688 s for its window
            leg -= 1688
        legs.append(leg)
        prev = departure

    big = 99 * DAY
    durations = [[big] * (n + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        durations[i][i] = 0
    durations[0][1] = legs[0]
    for i in range(1, n):
        durations[i][i + 1] = legs[i]
    for i in range(1, n + 1):
        durations[i][0] = MARILIA_RETURN

    stops = [Stop(0, s, [w]) for _, _, s, w in MARILIA]
    return build(
        stops,
        T=50400,
        OT=OT,
        rest=39600,
        n_rests=2,
        wait_rate=21.687,
        departure=MARILIA_DEPARTURE,
        durations=durations,
        h_abs=MARILIA_DEPARTURE + 2 * DAY,
    )


def test_marilia_anticipation_4894_in_every_evaluator():
    inst = marilia()
    seq = list(range(25)) + ["B", 25, 26]
    route = route_of(inst, seq)
    assert_equal(route.time_warp(), 3)
    assert_equal(route.anticipation(), 4894)

    fwd = evaluate_forward_pass(inst.data, acts(inst, seq), 0)
    assert_equal(fwd.anticipation(), 4894)

    _, sroute = search_route(inst, seq)
    assert_equal(sroute.anticipation(), 4894)
    assert_equal(sroute.time_warp(), 3)

    # Proposal evaluator: moving the rest into place from the end prices the
    # same route (ShiftBreak's delta equals the update() difference), so its
    # anticipation is 4894 as well.
    start_seq = list(range(27)) + ["B"]
    sol, sroute = search_route(inst, start_seq)
    ce = inst.ce()
    before = ce.penalised_cost(sol.unload())
    op = ShiftBreak(inst.data)
    op.init(sol)
    rest_node = sroute[len(sroute) - 2]
    assert_(rest_node.is_custom_break())
    delta, _ = op.evaluate(rest_node, ce)
    op.apply(rest_node)
    sroute.update()
    after = ce.penalised_cost(sol.unload())
    assert_equal(after - before, delta)
    # Wherever ShiftBreak put it, the update() of the result agrees with the
    # fresh evaluation of the same sequence.
    moved = [str(node.activity) for node in list(sroute)[1:-1]]
    assert_equal(
        sroute.anticipation(),
        evaluate_forward_pass(
            inst.data, [Activity(a) for a in moved], 0
        ).anticipation(),
    )


def test_marilia_weights_with_10h_overtime_do_not_saturate():
    # OT 10h: duty_limit 24h. The router's weights still fit the saturation
    # bound, and the fork still reduces the excess second by second.
    inst = marilia(OT=10 * H)
    w = inst.w
    overnights = 2
    total = (overnights + 2) * w.limit * w.excess + 15 * w.unneeded
    assert_(total < SAT)

    # Same route, return 1 s longer: one more second of excess costs more.
    seq = list(range(27))
    a = route_of(inst, seq)
    assert_(a.duty_excess() > 0)
    longer = marilia(OT=10 * H)
    mat = np.array(longer.data.duration_matrix(0), dtype=np.int64)
    mat[27][0] += 1
    longer.data = longer.data.replace(duration_matrices=[mat])
    b = route_of(longer, seq)
    assert_equal(b.duty_excess(), a.duty_excess() + 1)
    assert_(pen(longer, b) - pen(inst, a) >= w.excess)


# ==================================================== late return and excess
def test_late_return_counts_in_final_stretch():
    # The end depot closes at 20:00 and the route returns at 21:00: the final
    # stretch's duty runs to the real return, its excess includes the late
    # hour, and the late hour itself is priced at the excess weight, not at
    # the time-warp penalty.
    durations = [[0, 6 * H], [6 * H, 0]]
    inst = build(
        [Stop(0, H)], durations=durations, departure=8 * H, n_rests=0,
        veh_tw_late=20 * H,
    )  # fmt: skip
    route = route_of(inst, [0])
    # 8:00 + 6h + 1h + 6h = 21:00: duty 13h, limit 10h -> excess 3h.
    assert_equal(route.time_warp(), H)
    assert_equal(route.duty_excess(), 3 * H)
    assert_equal(route.overtime_band(), 2 * H)

    w = inst.w
    expected = (
        UNIT_DURATION_COST * 13 * H
        + w.excess * H  # the late return, at the excess weight
        + w.excess * 3 * H  # the duty excess
        + w.overtime * 2 * H  # the final band
    )
    assert_equal(pen(inst, route), expected)


def test_classification_with_late_return():
    # A rest followed by a final stretch that returns late: the next stretch
    # "as it came out" includes the late return, so the rest is needed.
    inst = build(
        [Stop(0, H), Stop(0, H)],
        durations=[[0, 4 * H, 99 * H], [99 * H, 0, 4 * H], [5 * H, 99 * H, 0]],
        n_rests=1,
        veh_tw_late=8 * H + 5 * H + 11 * H + 9 * H,
    )
    r = route_of(inst, [0, "B", 1])
    # J1 = 5h; J2 = 4h + 1h + 5h = 10h, back 1h after the vehicle's end.
    assert_equal(r.time_warp(), H)
    assert_equal(r.duty_excess(), 0)
    assert_equal(r.rests_needed(), 1)  # 5h + 0 + 10h > 10h
    assert_equal(r.anticipation(), 3 * H)


def test_late_return_not_traded_for_travel_savings():
    # Two orders: 0-B-A returns on time over 2,000,000 distance units; 0-A-B
    # returns 60 s late but saves 1,000,000. With the time-warp penalty alone
    # (fork_late_rate) the late order would win; on the duty model the late
    # return is priced at the excess weight and never wins.
    durations = [
        [0, H, H],
        [H, 0, H],
        [H + 60, H, 0],
    ]
    distances = [
        [0, 0, 1_000_000],
        [1_000_000, 0, 0],
        [1_000_000, 0, 0],
    ]
    kwargs = dict(
        durations=durations,
        distances=distances,
        unit_distance_cost=1,
        n_rests=0,
        departure=8 * H,
        veh_tw_late=11 * H,
    )
    on_time_seq, late_seq = [1, 0], [0, 1]
    inst = build([Stop(0, 0), Stop(0, 0)], **kwargs)
    late = route_of(inst, late_seq)
    on_time = route_of(inst, on_time_seq)
    assert_equal(late.time_warp(), 60)
    assert_equal(on_time.time_warp(), 0)
    assert_(pen(inst, on_time) < pen(inst, late))

    legacy = build([Stop(0, 0), Stop(0, 0)], duty=False, **kwargs)
    legacy_late = pen(legacy, route_of(legacy, late_seq))
    assert_(legacy_late < pen(legacy, route_of(legacy, on_time_seq)))

    for seed in range(5):
        res = solve(inst, seed, iters=100)
        assert_equal(res.best_penalised.routes()[0].time_warp(), 0)


# ============================================= overtime before a rest (D2)
def _pre_rest_instance(n_rests=1):
    # Chain 0 -> A -> B -> C -> 0 (other arcs prohibitive); 30 min service.
    # Duty at A's departure 12h, at B's 15h; the whole chain 17.5h > 16h, so a
    # rest is unavoidable. T = 14h, OT = 2h.
    big = 99 * H
    durations = [
        [0, 11 * H + 30 * 60, big, big],
        [big, 0, 2 * H + 30 * 60, big],
        [big, big, 0, H],
        [H, big, big, 0],
    ]
    return build(
        [Stop(0, 30 * 60)] * 3,
        durations=durations,
        T=14 * H,
        OT=2 * H,
        n_rests=n_rests,
        departure=0,
    )


def test_pre_rest_overtime_rest_at_12h_not_15h():
    inst = _pre_rest_instance()
    at_12h = route_of(inst, [0, "B", 1, 2])
    at_15h = route_of(inst, [0, 1, "B", 2])
    assert_equal(at_12h.anticipation(), 2 * H)
    assert_equal(at_12h.overtime_band(), 0)
    assert_equal(at_15h.overtime_band(), H)  # 1h before the rest
    assert_equal(at_15h.anticipation(), 0)
    assert_(pen(inst, at_12h) < pen(inst, at_15h))

    for seed in range(3):
        route = solve(inst, seed).best.routes()[0]
        visits = [str(Activity(a.type, a.idx)) for a in route.schedule()[1:-1]]
        assert_equal(visits, ["C0", "B0", "C1", "C2"])


def test_pre_rest_overtime_when_it_avoids_a_second_rest():
    # Duty 14h at A, 15h at B; from B the rest of the route is 16h. A rest at
    # A (no overtime) leaves 17h: a second rest. A rest at B (1h of overtime
    # before it) leaves exactly 16h: one rest. One hour of overtime before a
    # rest costs less than a rest (rest_use_cost covers all of it).
    big = 99 * H
    durations = [
        [0, 13 * H + 30 * 60, big, big],
        [big, 0, 30 * 60, big],
        [big, big, 0, 13 * H],
        [2 * H + 30 * 60, big, big, 0],
    ]
    inst = build(
        [Stop(0, 30 * 60)] * 3,
        durations=durations,
        T=14 * H,
        OT=2 * H,
        n_rests=2,
        departure=0,
    )
    one_rest = route_of(inst, [0, 1, "B", 2])
    assert_equal(one_rest.duty_excess(), 0)
    assert_equal(served_rests(one_rest), 1)
    two_rests = route_of(inst, [0, "B", 1, 2, "B"])
    assert_equal(two_rests.duty_excess(), 0)
    assert_equal(served_rests(two_rests), 2)
    assert_(pen(inst, one_rest) < pen(inst, two_rests))


def test_unneeded_rest_vs_pre_rest_overtime_divergence_k():
    # In ONE stretch the fork agrees with the lexicographic criterion: an
    # unneeded rest costs more than the full overtime before a rest
    # (rest_unneeded_cost includes rest_use_cost). Summed over two or more
    # stretches the overtime can cost more than an unneeded rest: that is
    # divergence (k), corrected by the Python criterion.
    w = router_weights(21.687, 14 * H, 2 * H, 32, 14400 + 2 * DAY)
    assert_(w.unneeded > w.pre_rest * w.OT)
    assert_(2 * w.pre_rest * w.OT > w.unneeded)


# ============================================== dominance of the excess (D2)
def test_excess_dominates_unneeded_rests_and_overnights():
    w = router_weights(21.687, 14 * H, 2 * H, 32, 14400 + 2 * DAY)
    # One second of excess outweighs 15 unneeded rests and 15 rests, and any
    # anticipation plus overtime band of a day.
    assert_(w.excess > 15 * w.unneeded)
    assert_(w.excess > 15 * w.rest_use)
    assert_(w.excess > (w.overtime + w.early) * w.limit)

    # On a route: 1 s of excess against two served rests.
    durations = [[0, 8 * H, 99 * H], [99 * H, 0, H], [H + 1, 99 * H, 0]]
    inst = build([Stop(0, 0), Stop(0, 0)], durations=durations, n_rests=2)
    excess = route_of(inst, [0, 1])
    assert_equal(excess.duty_excess(), 1)
    rests = route_of(inst, [0, "B", "B", 1])  # two interior rests
    assert_equal(rests.duty_excess(), 0)
    assert_equal(served_rests(rests), 2)
    assert_(pen(inst, excess) > pen(inst, rests))


# ========================================== same-day return and waiting (solve)
def _same_day_instance(day2_groups=False, OT=2 * H, T=8 * H):
    # Departure 6:00. Stops at 1h, 1.5h, 2h, 2.5h from the depot, 30 min each;
    # the last one opens at 12:00 (2h of waiting before it). Back at 15:00:
    # 9h of duty, within T + OT.
    def windows(open_, close):
        out = [(open_, close)]
        if day2_groups:
            out.append((open_ + DAY, close + DAY))
        return out

    stops = [
        Stop(H, 30 * 60, windows(7 * H, 18 * H)),
        Stop(H + 30 * 60, 30 * 60, windows(7 * H, 18 * H)),
        Stop(2 * H, 30 * 60, windows(7 * H, 18 * H)),
        Stop(2 * H + 30 * 60, 30 * 60, windows(12 * H, 18 * H)),
    ]
    return build(stops, T=T, OT=OT, departure=6 * H, wait_rate=21.4)


@pytest.mark.parametrize("day2_groups", [False, True])
def test_same_day_return_with_overtime(day2_groups):
    inst = _same_day_instance(day2_groups)
    route = route_of(inst, [0, 1, 2, 3])
    assert_equal(route.wait_duration(), 2 * H)
    assert_equal(route.overtime_band(), H)
    for seed in range(10):
        res = solve(inst, seed, iters=300)
        for sol in (res.best, res.best_penalised):
            assert_(sol.is_feasible())
            route = sol.routes()[0]
            assert_equal(served_rests(route), 0)
            assert_equal(route.duty_excess(), 0)
            assert_(route.end_time() < DAY)


def test_one_day_route_with_two_hours_wait_and_no_rest():
    inst = _same_day_instance(OT=0, T=10 * H)
    for seed in range(10):
        res = solve(inst, seed, iters=300)
        route = res.best.routes()[0]
        assert_(res.best.is_feasible())
        assert_equal(served_rests(route), 0)


def test_fractional_wait_rate_never_buys_a_rest():
    # Wait rate 21.4, early_rate ceil(21.4) = 22: 13h20 of waiting in a
    # one-day route (duty exactly T) is never cheaper as anticipation, even
    # with free rests.
    stops = [
        Stop(10 * 60, 5 * 60),
        Stop(15 * 60, 5 * 60, [(17 * H + 40 * 60, BIG)]),
    ]
    inst = build(stops, T=14 * H, OT=2 * H, departure=4 * H, wait_rate=21.4)
    assert_equal(inst.w.early, 22)
    no_rest = route_of(inst, [0, 1])
    assert_equal(no_rest.wait_duration(), 13 * H + 20 * 60)
    assert_equal(no_rest.overtime_band(), 0)

    free = build(
        stops, T=14 * H, OT=2 * H, departure=4 * H, wait_rate=21.4,
        weights=_replace(inst.w, rest_use=0, unneeded=0),
    )  # fmt: skip
    rest = route_of(free, [0, "B", 1])
    assert_equal(rest.wait_duration(), 0)
    assert_equal(rest.anticipation(), 14 * H - 15 * 60)
    assert_(pen(free, route_of(free, [0, 1])) < pen(free, rest))

    for seed in range(10):
        route = solve(inst, seed, iters=200).best.routes()[0]
        assert_equal(served_rests(route), 0)


# ================================ lateness against the last window of a day
@pytest.mark.parametrize("late", [3, H])
def test_late_against_last_window_returns_same_day(late):
    # Stop 1's day window closes `late` seconds before the vehicle can be
    # there; its next window is only the next day. OT 2h. Waiting for the
    # next day needs a rest that is unneeded (a window had already opened on
    # arrival), so the fork prefers the lateness and returns the same day.
    arrive = 10 * H
    day_window = (8 * H, arrive - late)
    next_day = (DAY + 8 * H, DAY + 18 * H)
    stops = [
        Stop(H, 30 * 60, [(8 * H, 9 * H)]),  # first, or late itself
        Stop(H + 30 * 60, 30 * 60, [day_window, next_day]),
    ]
    inst = build(stops, OT=2 * H)
    no_rest = route_of(inst, [0, (1, 0)])
    assert_equal(no_rest.time_warp(), late)
    assert_(no_rest.end_time() < DAY)

    with_rest = route_of(inst, [0, "B", (1, 1)])
    assert_equal(with_rest.time_warp(), 0)
    assert_equal(with_rest.rests_unneeded(), 1)
    assert_(pen(inst, no_rest) < pen(inst, with_rest))

    for seed in range(5):
        res = solve(inst, seed, iters=200)
        route = res.best_penalised.routes()[0]
        assert_equal(served_rests(route), 0)
        assert_equal(route.time_warp(), late)
        assert_(route.end_time() < DAY)


def test_multiplier_one_does_not_prefer_late_to_wait():
    # Late multiplier 1: arriving at 10:00, the stop's first window closed at
    # 9:59:50 (10 s late) and its second opens at 10:00:09 (9 s of waiting).
    # The criterion waits (9 x rate < 10 x rate); the fork's late penalty,
    # max(21, ceil(rate) + 1, ceil(rate) + 20), keeps it from preferring the
    # lateness.
    arrive = 10 * H
    stops = [Stop(2 * H, 0, [(0, arrive - 10), (arrive + 9, 18 * H)])]
    inst = build(stops, late_mult=1, wait_rate=21.4, n_rests=0)
    assert_equal(inst.w.fork_late, 22 + 20)
    late = route_of(inst, [(0, 0)])
    wait = route_of(inst, [(0, 1)])
    assert_equal(late.time_warp(), 10)
    assert_equal(wait.wait_duration(), 9)
    assert_(pen(inst, wait) < pen(inst, late))

    for seed in range(5):
        res = solve(inst, seed, iters=100)
        assert_equal(res.best_penalised.routes()[0].time_warp(), 0)


# ======================================================= best_penalised (D1)
def _three_seconds_instance():
    # Stop 1 closes 3 s before the vehicle can reach it, and has no other
    # window: the lateness is unavoidable.
    arrive = 10 * H
    stops = [
        Stop(H, 30 * 60, [(8 * H, 9 * H)]),  # first, or late itself
        Stop(H + 30 * 60, 30 * 60, [(8 * H, arrive - 3)]),
    ]
    return build(stops, OT=2 * H)


def test_best_penalised_three_seconds_without_rest():
    inst = _three_seconds_instance()
    without = route_of(inst, [0, 1])
    with_rest = route_of(inst, [0, "B", 1])
    assert_equal(without.time_warp(), 3)
    assert_equal(with_rest.rests_unneeded(), 1)
    assert_(pen(inst, without) < pen(inst, with_rest))

    for seed in range(3):
        res = solve(inst, seed, iters=200)
        route = res.best_penalised.routes()[0]
        assert_equal(served_rests(route), 0)
        assert_equal(route.time_warp(), 3)


class _FakeSearch:
    """Returns the given solutions in turn and records what it was given."""

    def __init__(self, solutions):
        self.solutions = list(solutions)
        self.calls = []

    def __call__(self, solution, cost_evaluator, exhaustive=False):
        self.calls.append((solution, exhaustive))
        idx = min(len(self.calls) - 1, len(self.solutions) - 1)
        return self.solutions[idx]


class _RecordingStop:
    def __init__(self, inner):
        self.inner = inner
        self.values = []

    def __call__(self, best_cost):
        self.values.append(best_cost)
        return self.inner(best_cost)


def _late_solutions(duty=True):
    # Three stops whose windows all close before the vehicle can get there:
    # every order is late, with different total lateness.
    stops = [
        Stop(H, 0, [(0, 8 * H)]),
        Stop(2 * H, 0, [(0, 8 * H + 30 * 60)]),
        Stop(3 * H, 0, [(0, 9 * H)]),
    ]
    inst = build(stops, n_rests=0, duty=duty)
    ce = inst.ce()
    sols = {}
    for order in permutations(range(3)):
        sol = Solution(inst.data, [route_of(inst, list(order))])
        assert_(not sol.is_feasible())
        sols.setdefault(ce.penalised_cost(sol), sol)
    ordered = [sols[cost] for cost in sorted(sols, reverse=True)]
    return inst, ordered


def _ils(inst, search, init, penalised_best=False, **ils_kwargs):
    fl = inst.w.fork_late
    pm = PenaltyManager(
        ([], fl, fl, fl),
        PenaltyParams(min_penalty=fl, max_penalty=fl),
        inst.wait_rate,
    )
    params = IteratedLocalSearchParams(**ils_kwargs)
    return IteratedLocalSearch(
        inst.data, pm, search, init, params, penalised_best=penalised_best
    )


def test_unavoidable_lateness_does_not_stop_the_search_early():
    # NoImprovement(2) on the best FEASIBLE cost stops after two iterations
    # when nothing feasible exists; driven by the best penalised cost, the
    # search keeps going while that cost improves.
    inst, ordered = _late_solutions()
    assert_(len(ordered) >= 4)
    init, improving = ordered[0], ordered[1:]

    stop = _RecordingStop(NoImprovement(2))
    algo = _ils(inst, _FakeSearch(improving), init, exhaustive_on_best=False)
    assert_(algo.penalised_best)  # implied by duty_limit >= 0
    res = algo.run(stop, collect_stats=False)
    assert_(res.num_iterations >= len(improving))
    ce = inst.ce()
    best_pen = ce.penalised_cost(improving[-1])
    assert_equal(stop.values[-1], best_pen)  # the criterion's input
    assert_equal(ce.penalised_cost(res.best_penalised), best_pen)

    # Legacy vehicle (duty_limit -1), rule off: as before, two iterations.
    legacy, ordered = _late_solutions(duty=False)
    algo = _ils(legacy, _FakeSearch(ordered[1:]), ordered[0])
    assert_(not algo.penalised_best)
    res = algo.run(NoImprovement(2), collect_stats=False)
    assert_equal(res.num_iterations, 2)


def test_restart_from_best_penalised():
    inst, ordered = _late_solutions()
    s_init, s_a, s_b = ordered[0], ordered[1], ordered[-1]
    # init -> a (improves) -> b (improves) -> a (worse) -> restart: the next
    # call must start from b, the best penalised solution.
    search = _FakeSearch([s_a, s_b, s_a, s_a, s_a])
    algo = _ils(
        inst,
        search,
        s_init,
        num_iters_no_improvement=1,
        exhaustive_on_best=False,
    )
    algo.run(MaxIterations(5), collect_stats=False)
    inputs = [sol for sol, _ in search.calls]
    assert_(inputs[3] is s_b)
    # exhaustive_on_best off: no exhaustive search on any improvement.
    assert_(not any(exhaustive for _, exhaustive in search.calls))


def test_exhaustive_on_best_penalised():
    # With exhaustive_on_best, an improvement of the best penalised solution
    # gets the exhaustive search too; its result is kept if it is cheaper.
    inst, ordered = _late_solutions()
    s_init, s_a, s_b = ordered[0], ordered[1], ordered[-1]
    search = _FakeSearch([s_a, s_b, s_a])
    algo = _ils(inst, search, s_init, exhaustive_on_best=True)
    res = algo.run(MaxIterations(1), collect_stats=False)
    assert_(search.calls[0][0] is s_init and not search.calls[0][1])
    assert_(search.calls[1][0] is s_a and search.calls[1][1])  # exhaustive
    assert_(res.best_penalised is s_b)


def test_best_penalised_against_best():
    # A feasible solution exists only through an unneeded rest that waits for
    # the next day; the 3 s late one without a rest is cheaper once
    # penalised. `best` is the feasible one, `best_penalised` the late one.
    arrive = 10 * H
    stops = [
        Stop(H, 30 * 60),
        Stop(H + 30 * 60, 30 * 60, [(8 * H, arrive - 3), (DAY + 8 * H, DAY + 18 * H)]),
    ]
    inst = build(stops, OT=2 * H)
    late = Solution(inst.data, [route_of(inst, [0, (1, 0)])])
    rest = Solution(inst.data, [route_of(inst, [0, "B", (1, 1)])])
    assert_(rest.is_feasible() and not late.is_feasible())

    search = _FakeSearch([late, late])
    algo = _ils(inst, search, rest, exhaustive_on_best=False)
    res = algo.run(MaxIterations(2), collect_stats=False)
    assert_(res.best is rest)
    assert_(res.best_penalised is late)

    # Nothing infeasible is cheaper: best_penalised is best.
    feasible = build([Stop(H, 30 * 60), Stop(H + 30 * 60, 30 * 60)])
    res = solve(feasible, 0, iters=100)
    assert_(res.best_penalised is res.best)


def test_solve_params_penalised_best():
    params = SolveParams(penalised_best=True)
    assert_(params.penalised_best)
    assert_(not SolveParams().penalised_best)
    assert_(params != SolveParams())

    # The parameter turns the rule on for a legacy instance too.
    legacy, ordered = _late_solutions(duty=False)
    algo = _ils(
        legacy, _FakeSearch(ordered[1:]), ordered[0], penalised_best=True
    )
    assert_(algo.penalised_best)
    res = algo.run(NoImprovement(2), collect_stats=False)
    assert_(res.num_iterations >= len(ordered) - 1)

    # And solve() passes it on.
    res = solve(legacy, 0, iters=10, penalised_best=True)
    assert_(res.best_penalised is not None)


def test_solve_params_from_file(tmp_path):
    loc = tmp_path / "params.toml"
    loc.write_text("penalised_best = true\n")
    assert_(SolveParams.from_file(loc).penalised_best)


def test_no_penalty_bound_warning_with_fixed_penalties():
    inst, ordered = _late_solutions()
    fl = inst.w.fork_late
    params = PenaltyParams(
        min_penalty=fl, max_penalty=fl, solutions_between_updates=1
    )
    pm = PenaltyManager(([], fl, fl, fl), params, 0.0)
    with warnings.catch_warnings():
        warnings.simplefilter("error", PenaltyBoundWarning)
        for sol in ordered * 3:
            pm.register(sol)

    # Adaptive penalties at their maximum still warn, as before.
    params = PenaltyParams(max_penalty=1, solutions_between_updates=1)
    pm = PenaltyManager(([], 1, 1, 1), params, 0.0)
    with pytest.warns(PenaltyBoundWarning):
        for sol in ordered * 3:
            pm.register(sol)


# ============================================================ warm start (D1)
def test_warm_start_without_rests_parks_them():
    inst = build([Stop(H, 600), Stop(2 * H, 600), Stop(3 * H, 600)])
    sol = Solution(inst.data, [route_of(inst, [0, 1, 2])])
    ssol = SearchSolution(inst.data)
    ssol.load(sol)
    route = ssol.routes[0]
    kinds = [str(node.activity) for node in route]
    assert_equal(kinds, ["D0", "C0", "C1", "C2", "B0", "B1", "D0"])
    assert_equal(route.breaks_served(), [])  # parked: nothing served

    # And the search from that warm start keeps them parked on this short
    # route.
    res = solve(inst, 0, iters=50, init=sol)
    assert_equal(served_rests(res.best.routes()[0]), 0)


def _stuck_warm_start_instance():
    # Two stops; 0-A-B-0 takes 12h, 0-B-A-0 11h, both past the 10h duty
    # limit, and the vehicle's end closes at the limit (18:00). Every order
    # the search produces carries the rests and serves the one before the
    # return (divergence (n)), coming back about 11h late; the rest-less
    # warm start A-B pays only its 2h excess and 2h late return, so it is
    # cheaper than anything the search can produce.
    durations = [[0, 2 * H, 3 * H], [5 * H, 0, 4 * H], [6 * H, 3 * H, 0]]
    inst = build(
        [Stop(0, 0), Stop(0, 0)], durations=durations, veh_tw_late=18 * H
    )
    warm = Solution(inst.data, [route_of(inst, [0, 1])])
    return inst, warm


def test_warm_start_without_rests_is_not_the_incumbent():
    inst, warm = _stuck_warm_start_instance()
    ce = inst.ce()
    a_b = ce.penalised_cost(Solution(inst.data, [route_of(inst, [0, 1])]))
    b_a_rests = ce.penalised_cost(
        Solution(inst.data, [route_of(inst, [1, 0, "B", "B"])])
    )
    a_b_rests = ce.penalised_cost(
        Solution(inst.data, [route_of(inst, [0, 1, "B", "B"])])
    )
    # The trap: the rest-less warm start beats the best order with rests,
    # which beats the warm-start order with rests.
    assert_(a_b < b_a_rests < a_b_rests)

    # The ILS starts from the warm start with its rests seeded, as the search
    # sees it: with no iteration, that is what it reports.
    res = solve(inst, 0, iters=0, init=warm)
    for sol in (res.best, res.best_penalised):
        assert_equal(
            [str(act) for act in sol.routes()[0]],
            ["D0", "C0", "C1", "B0", "B1", "D0"],
        )
    assert_equal(ce.penalised_cost(res.best_penalised), a_b_rests)

    # And the search improves on it, instead of returning the warm start.
    for seed in range(3):
        res = solve(inst, seed, iters=200, init=warm)
        best = res.best_penalised
        assert_equal(ce.penalised_cost(best), b_a_rests)
        visits = [act.idx for act in best.routes()[0] if act.is_client()]
        assert_equal(visits, [1, 0])


@pytest.mark.parametrize("duty, n_rests", [(True, 0), (False, 2)])
def test_initial_solution_used_as_given_without_generated_rests(duty, n_rests):
    # Without duty-model rest rules (no rules, or legacy -1 rules) the ILS
    # starts from the very initial solution object: nothing is converted.
    stops = [Stop(H, 600), Stop(2 * H, 600), Stop(3 * H, 600)]
    inst = build(stops, duty=duty, n_rests=n_rests)
    init = Solution(inst.data, [route_of(inst, [2, 0, 1])])
    search = _FakeSearch([init])
    ils = _ils(inst, search, init)
    assert_(ils.initial_solution is init)
    res = ils.run(MaxIterations(3), collect_stats=False)
    assert_(all(given is init for given, _ in search.calls))
    assert_(res.best is init)
    assert_(res.best_penalised is init)


# ======================================================== evaluator parity
def _check_move_parity(inst, seq, op_cls):
    """
    For every improving move of the operator on the route of ``seq``, its
    delta equals the difference of the penalised costs before and after
    applying it (proposal evaluator against Route::update()).
    """
    ce = inst.ce()
    base = Solution(inst.data, [route_of(inst, seq)])
    checked = 0
    probe = SearchSolution(inst.data)
    probe.load(base)
    num_nodes = len(probe.routes[0])
    for u in range(1, num_nodes - 1):
        for v in range(0, num_nodes - 1):
            sol = SearchSolution(inst.data)
            sol.load(base)
            route = sol.routes[0]
            U, V = route[u], route[v]
            if U.is_custom_break() or V.is_custom_break() or u == v:
                continue
            op = op_cls(inst.data)
            op.init(sol)
            delta, _ = op.evaluate(U, V, ce)
            if delta >= 0:
                continue
            before = ce.penalised_cost(sol.unload())
            op.apply(U, V)
            route.update()
            after = ce.penalised_cost(sol.unload())
            assert_equal(after - before, delta)
            checked += 1
    return checked


def test_move_parity_crossing_duty_limit():
    # Six stops on a line, 1h30 apart: moving the last stops to the front and
    # the first ones to the back crosses duty_limit both ways.
    stops = [Stop(i * 5400, 30 * 60) for i in range(1, 7)]
    inst = build(stops, T=8 * H, OT=2 * H, n_rests=1)
    bad = [5, 0, 4, 1, 3, 2]
    checked = _check_move_parity(inst, bad, Relocate1)
    checked += _check_move_parity(inst, bad, Swap11)
    checked += _check_move_parity(inst, [0, 1, 2, 3, 4, 5, "B"], Relocate1)
    assert_(checked > 0)

    # Late return (end depot warp, priced apart from client warp): proposal
    # and update agree as well.
    late = build(stops, T=8 * H, OT=2 * H, n_rests=1, veh_tw_late=15 * H)
    assert_(route_of(late, bad).time_warp() > 0)
    checked = _check_move_parity(late, bad, Relocate1)
    checked += _check_move_parity(late, bad, Swap11)
    assert_(checked > 0)


def test_activities_constructor_omitted_rest_nodes():
    # The A7 manual candidate: a route of clients only, on a vehicle with
    # rest rules, gets every duty term from the same pass the search uses.
    inst = build([Stop(3 * H, H), Stop(6 * H, H)])
    route = route_of(inst, [0, 1])
    assert_equal(duty_terms(route), (4 * H, 2 * H, 0, 0, 0))
    assert_(not route.is_feasible())

    # A route that needs no rest: 2h + 1h + 1h + 1h + 3h = 8h, T = 7h.
    inst = build([Stop(2 * H, H), Stop(3 * H, H)], T=7 * H, OT=2 * H)
    seq = [0, 1]
    route = route_of(inst, seq)
    assert_equal(duty_terms(route), (0, H, 0, 0, 0))

    # Search route with the same clients and no rest node at all.
    sroute = SearchRoute(inst.data, 0)
    for act in acts(inst, seq):
        sroute.append(Node(act))
    sroute.update()
    assert_equal(duty_terms(sroute), duty_terms(route))

    # Cost parity with the search's own export of the same route (the export
    # gets the rests parked, which change nothing here).
    ssol = SearchSolution(inst.data)
    ssol.load(Solution(inst.data, [route]))
    ce = inst.ce()
    exported = ssol.unload()
    assert_equal(
        ce.penalised_cost(exported),
        ce.penalised_cost(Solution(inst.data, [route])),
    )


# ============================================================== saturation
def test_duty_terms_saturate():
    # Weights that would overflow int64 saturate at 2^62 per term and in the
    # sum, instead of wrapping around.
    huge = 2**61
    durations = [[0, 20 * H], [20 * H, 0]]
    inst = build(
        [Stop(0, 0)], durations=durations, n_rests=0,
        weights=Weights(
            T=8 * H, OT=2 * H, limit=10 * H, overtime=huge, early=1,
            fork_late=21, pre_rest=huge, rest_use=huge, unneeded=huge,
            excess=huge,
        ),
    )  # fmt: skip
    route = route_of(inst, [0])
    assert_equal(route.duty_excess(), 30 * H)
    cost = pen(inst, route)
    assert_(0 < cost <= SAT + UNIT_DURATION_COST * 40 * H)


# ============================================ breakDue and relaxable (2.6)
def test_duty_rests_have_no_break_due_and_ignore_relaxable():
    # Duty-model rests never fall due: no breakDue, no due mask, whatever the
    # route does; `relaxable` changes nothing (the duty excess is what makes
    # the route infeasible).
    rules = [
        CustomBreak(
            id=0,
            service=11 * H,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=8 * H,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=relaxable,
            eligible_from=0,
        )
        for relaxable in (False, True)
    ]
    for rule in rules:
        inst = build([Stop(3 * H, H), Stop(6 * H, H)], n_rests=0)
        vt = inst.data.vehicle_type(0).replace(custom_breaks=[rule])
        inst.data = inst.data.replace(vehicle_types=[vt])
        for seq in ([0, 1], [0, "B", 1], [0, 1, "B"]):
            route = route_of(inst, seq)
            assert_equal(route.break_due(), 0)
            assert_equal(route.break_due_mask(), 0)
            _, sroute = search_route(inst, seq)
            assert_equal(sroute.break_due(), 0)
            assert_equal(sroute.break_due_mask(), 0)

        excess = route_of(inst, [0, 1])
        assert_(excess.duty_excess() > 0)
        assert_(not excess.is_feasible())


# ============================================== legacy rules untouched (D1)
def test_legacy_result_best_penalised_defaults_to_best():
    inst = build([Stop(H, 600)], duty=False, n_rests=0)
    sol = Solution(inst.data, [route_of(inst, [0])])
    res = Result(sol, Statistics(), 0, 0.0)
    assert_(res.best_penalised is sol)


# ---------------------------------------------- duty clock from the departure
def _wait_before_first_client(start_late):
    # Departs 04:00, first client 1h away (arrival 05:00) opens 08:00; service
    # 1h, back 1h later (10:00). T = 3h, limit = 4h.
    return build(
        [Stop(H, H, [(8 * H, 12 * H)])],
        T=3 * H,
        OT=H,
        n_rests=0,
        departure=4 * H,
        start_late=start_late,
    )


def test_fixed_departure_counts_wait_before_first_client_as_duty():
    # start_late == tw_early: the duty clock starts at the 04:00 departure, so
    # the 3h wait for the window counts: J = 1h + 3h + 1h + 1h = 6h. Excess is
    # 6h - 4h = 2h and the overtime band min(6h, 4h) - 3h = 1h.
    inst = _wait_before_first_client(start_late=None)
    assert_all_evaluators(inst, [0], (2 * H, H, 0, 0, 0))


def test_flexible_departure_keeps_wait_before_first_client_out_of_duty():
    # Control: start_late > tw_early leaves the clock at the (latest) arrival
    # less the edge, 07:00: J = 3h, within T.
    inst = _wait_before_first_client(start_late=5 * H)
    assert_all_evaluators(inst, [0], (0, 0, 0, 0, 0))


def test_fixed_departure_with_open_first_client_is_unchanged():
    # Window already open on arrival: nothing to wait for, J = 1h + 1h + 1h.
    inst = build(
        [Stop(H, H, [(0, 12 * H)])],
        T=3 * H,
        OT=H,
        n_rests=0,
        departure=4 * H,
    )
    assert_all_evaluators(inst, [0], (0, 0, 0, 0, 0))
