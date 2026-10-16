"""
A route built from activities in Python (``Route(data, activities, type)``)
with CUSTOM_BREAK activities reports the rests it serves (``breaks_served``,
by position, as the search exports them; it used to come back empty) in both
rest models. Warm starts and the router's own emission rely on that.

Duty model: the route is the same route the search exports
(``search.Solution.unload``) for that sequence: the served rests, their
effective services, the schedule, the duration and its cost, the time warp,
the rest lateness and every duty term.

Legacy rules (no duty model): only ``breaks_served`` is taken from the search's
pass. Everything else stays what the fork computed before the duty model
(690fee3): no per-activity services, each break folds its configured service
(goldens in test_duty_model_legacy_parity and
test_legacy_overnight_characterization). It therefore equals the export
exactly when the export serves every break with its configured service.
"""

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal

from pyvrp import (
    Activity,
    ActivityType,
    Client,
    CostEvaluator,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Depot,
    Location,
    ProblemData,
    Route,
    Solution,
    VehicleType,
)
from pyvrp.search._search import Node
from pyvrp.search._search import Solution as SearchSolution
from tests.test_duty_model import H, Stop, acts, build, route_of, solve


def _fields(route):
    sched = [
        (
            str(act),
            act.trip,
            act.start_time,
            act.end_time,
            act.wait_duration,
            act.time_warp,
        )
        for act in route.schedule()
    ]
    return {
        "activities": [str(act) for act in route],
        "breaks_served": list(route.breaks_served()),
        "break_services": list(route.break_services()),
        "break_due": route.break_due(),
        "break_due_mask": route.break_due_mask(),
        "distance": route.distance(),
        "distance_cost": route.distance_cost(),
        "duration": route.duration(),
        "duration_cost": route.duration_cost(),
        "overtime": route.overtime(),
        "time_warp": route.time_warp(),
        "travel": route.travel_duration(),
        "service": route.service_duration(),
        "setup": route.setup_duration(),
        "wait": route.wait_duration(),
        "start_time": route.start_time(),
        "end_time": route.end_time(),
        "slack": route.slack(),
        "release_time": route.release_time(),
        "duty": (
            route.duty_excess(),
            route.overtime_band(),
            route.anticipation(),
            route.rests_needed(),
            route.rests_unneeded(),
        ),
        "feasible": route.is_feasible(),
        "schedule": sched,
    }


def _exported(data, activities, other):
    """
    The route the search exports for ``activities``: a different route is
    loaded first, so ``unload`` builds the route afresh from the search route
    (with the search's per-activity services and served rests) instead of
    copying the loaded one.
    """
    ssol = SearchSolution(data)
    ssol.load(Solution(data, [Route(data, other, 0)]))
    sroute = ssol.routes[0]
    sroute.clear()
    owned = []
    for act in activities:
        node = ssol[act]
        if node is None:
            node = Node(act)
            owned.append(node)
        sroute.append(node)
    sroute.update()
    exported = ssol.unload().routes()[0]
    assert_equal([str(a) for a in exported][1:-1], [str(a) for a in activities])
    return exported, list(sroute.breaks_served())


def _configured_services(data, activities):
    """The export's break services if every break is served unextended."""
    vt = data.vehicle_type(0)
    service = {brk.id: brk.service for brk in vt.custom_breaks}
    return [
        service[act.idx] if act.type == ActivityType.CUSTOM_BREAK else 0
        for act in activities
    ]


def _compare(data, activities, built, exported, ce):
    """
    Compares a Python-built route with the search's export of the same
    activities, per rest model (see the module docstring). Returns whether
    every field was compared.
    """
    fb, fe = _fields(built), _fields(exported)
    # Both models: the served rests are the export's.
    assert_equal(fb["breaks_served"], fe["breaks_served"])
    # The due bitmask is only set on export (unload): a Python-built route
    # keeps 0, so a late mandatory break leaves it feasible, priced through
    # break_due (the hows-router's legacy manual candidates rely on it).
    assert_equal(fb["break_due_mask"], 0)
    if fe["break_due_mask"] == 0:
        assert_equal(fb["feasible"], fe["feasible"])
    for key in ("break_due_mask", "feasible"):
        del fb[key], fe[key]

    if not data.vehicle_type(0).duty_limit >= 0:
        # Legacy rules: no per-activity services on a Python-built route; it
        # folds each break's configured service, as the previous fork did.
        assert_equal(fb["break_services"], [])
        has_breaks = any(
            act.type == ActivityType.CUSTOM_BREAK for act in activities
        )
        if has_breaks and fe["break_services"] != _configured_services(
            data, activities
        ):
            # The export leaves a break unserved or extends it (D5): the
            # Python-built route keeps the previous fork's schedule instead.
            return False
        del fb["break_services"], fe["break_services"]

    assert_equal(fb, fe)
    assert_equal(
        ce.penalised_cost(Solution(data, [built])),
        ce.penalised_cost(Solution(data, [exported])),
    )
    return True


def _check(data, activities, other, ce):
    exported, search_served = _exported(data, activities, other)
    built = Route(data, activities, 0)
    assert_equal(list(exported.breaks_served()), search_served)
    _compare(data, activities, built, exported, ce)
    return built


def _legacy_inst():
    # Legacy (-1) DUTY_TIME rests, no duty model.
    return build(
        [Stop(3 * H, H), Stop(6 * H, H), Stop(8 * H, H)], duty=False
    )


def _duty_inst():
    return build([Stop(3 * H, H), Stop(6 * H, H), Stop(8 * H, H)])


@pytest.mark.parametrize("make", [_legacy_inst, _duty_inst])
@pytest.mark.parametrize(
    "seq",
    [
        [0, "B", 1, 2, "B"],
        [0, 1, "B", 2, "B"],
        [0, 1, 2, "B", "B"],
        ["B", 0, 1, 2, "B"],
        [2, "B", 1, "B", 0],
        [0, 1, 2],
    ],
)
def test_route_from_activities_matches_search_export(make, seq):
    inst = make()
    other = acts(inst, [2, 1, 0])
    _check(inst.data, acts(inst, seq), other, inst.ce())


def test_route_from_activities_reports_served_rest():
    for make in (_legacy_inst, _duty_inst):
        inst = make()
        route = route_of(inst, [0, 1, "B", 2, "B"])
        assert_(0 in list(route.breaks_served()))


def test_legacy_extended_rest_matches_search_export():
    # D5: a served DUTY_TIME rest followed by a client whose window opens the
    # next morning absorbs the wait. On the duty model the Python route folds
    # the same extended service the search does; on legacy rules it reports
    # the served rest but keeps folding the configured service (690fee3).
    stops = [Stop(4 * H, H), Stop(5 * H, H, [(24 * H + 8 * H, 30 * 24 * H)])]
    for duty in (False, True):
        inst = build(stops, duty=duty, n_rests=1, T=4 * H)
        seq = acts(inst, [0, "B", 1])
        built = _check(inst.data, seq, acts(inst, [1, 0]), inst.ce())
        assert_equal(list(built.breaks_served()), [0])
        configured = inst.data.vehicle_type(0).custom_breaks[0].service
        rest = [a for a in built.schedule() if a.type == ActivityType.CUSTOM_BREAK]
        if duty:
            assert_(built.break_services()[1] > 11 * H)
            assert_(rest[0].end_time - rest[0].start_time > 11 * H)
        else:
            assert_equal(list(built.break_services()), [])
            assert_equal(rest[0].end_time - rest[0].start_time, configured)


def test_drive_time_break_matches_search_export():
    # A legacy DRIVE_TIME break (not a duty rest).
    n = 4
    dur = np.full((n + 1, n + 1), 2 * H, dtype=np.int64)
    np.fill_diagonal(dur, 0)
    brk = CustomBreak(
        id=0,
        service=45 * 60,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=4 * H,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
    )
    vt = VehicleType(capacity=[], unit_duration_cost=20, custom_breaks=[brk])
    data = ProblemData(
        [Location(i, 0) for i in range(n + 1)],
        [Client(location=i, service_duration=600) for i in range(1, n + 1)],
        [Depot(location=0)],
        [vt],
        [dur],
        [dur],
    )
    ce = CostEvaluator([], 100, 0)
    C = ActivityType.CLIENT
    B = Activity(ActivityType.CUSTOM_BREAK, 0)
    other = [Activity(C, i) for i in (3, 2, 1, 0)]
    for pos in range(n + 1):
        seq = [Activity(C, i) for i in range(n)]
        seq.insert(pos, B)
        _check(data, seq, other, ce)


@pytest.mark.parametrize("duty", [False, True])
def test_solved_routes_rebuilt_in_python_are_identical(duty):
    stops = [Stop(i * 5400, 30 * 60) for i in range(1, 7)]
    inst = build(stops, T=8 * H, OT=2 * H, n_rests=2, duty=duty)
    ce = inst.ce()
    for seed in range(3):
        res = solve(inst, seed, iters=200)
        for sol in (res.best, res.best_penalised):
            for route in sol.routes():
                activities = [act for act in route][1:-1]
                built = Route(inst.data, activities, route.vehicle_type())
                _compare(inst.data, activities, built, route, ce)
