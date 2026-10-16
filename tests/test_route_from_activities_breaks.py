"""
A route built from activities in Python (``Route(data, activities, type)``)
with CUSTOM_BREAK activities must be the same route the search exports
(``search.Solution.unload``) for that sequence: the served rests, their
effective services, the schedule, the duration and its cost, the time warp,
the rest lateness and every duty term. Warm starts and the router's own
emission rely on that (``breaks_served`` used to come back empty).
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


def _check(data, activities, other, ce):
    exported, search_served = _exported(data, activities, other)
    built = Route(data, activities, 0)
    assert_equal(list(exported.breaks_served()), search_served)
    fb, fe = _fields(built), _fields(exported)
    # The due bitmask is only set on export (unload): a Python-built route
    # keeps 0, so a late mandatory break leaves it feasible, priced through
    # break_due (the hows-router's legacy manual candidates rely on it).
    assert_equal(fb["break_due_mask"], 0)
    if fe["break_due_mask"] == 0:
        assert_equal(fb["feasible"], fe["feasible"])
    for key in ("break_due_mask", "feasible"):
        del fb[key], fe[key]
    if not any(act.type == ActivityType.CUSTOM_BREAK for act in activities):
        # A route without break activities is built as before: legacy rests
        # get no per-activity services (the export carries zeros; costs and
        # schedule agree regardless).
        if not data.vehicle_type(0).duty_limit >= 0:
            assert_equal(fb["break_services"], [])
            del fb["break_services"], fe["break_services"]
    assert_equal(fb, fe)
    assert_equal(
        ce.penalised_cost(Solution(data, [built])),
        ce.penalised_cost(Solution(data, [exported])),
    )
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
    # next morning absorbs the wait; the Python route folds the same
    # extended service the search does.
    stops = [Stop(4 * H, H), Stop(5 * H, H, [(24 * H + 8 * H, 30 * 24 * H)])]
    for duty in (False, True):
        inst = build(stops, duty=duty, n_rests=1, T=4 * H)
        seq = acts(inst, [0, "B", 1])
        built = _check(inst.data, seq, acts(inst, [1, 0]), inst.ce())
        assert_equal(list(built.breaks_served()), [0])
        assert_(built.break_services()[1] > 11 * H)


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
                fb, fe = _fields(built), _fields(route)
                assert_equal(fb.pop("break_due_mask"), 0)
                if fe.pop("break_due_mask") == 0:
                    assert_equal(fb["feasible"], fe["feasible"])
                del fb["feasible"], fe["feasible"]
                assert_equal(fb, fe)
                assert_equal(
                    ce.penalised_cost(Solution(inst.data, [built])),
                    ce.penalised_cost(Solution(inst.data, [route])),
                )
