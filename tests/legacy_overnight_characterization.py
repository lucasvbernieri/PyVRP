"""
Legacy (``eligible_from = -1``, ``duty_limit = -1``) characterization of a
multi-day route built in Python (``Route(data, activities, type)``) with an
overnight rest followed by a wait for the next morning's window: the case the
hows-router's legacy manual candidate hits (it is compared with the solver's
route by penalised cost). See test_legacy_overnight_characterization.py.

The measurements only use the API the fork had before the duty model, so the
same code runs against that build to produce the goldens:

    PYTHONPATH=<build of 690fee3> PYVRP_...=<mode> \\
        python tests/legacy_overnight_characterization.py \\
        tests/data/legacy_overnight_goldens.json

Each run adds (or replaces) the entry of the PYVRP_* mode it runs under. This
module is deliberately not a test module and imports nothing from ``tests``.
"""

import json
import os
import sys

import numpy as np

import pyvrp
from pyvrp import (
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

H = 3600
DAY = 24 * H
INT_MAX = 2**31 - 1
# Same variables (and key) as tests/duty_legacy_parity.py.
MODE_VARS = (
    "PYVRP_CLOCK_TRIGGER",
    "PYVRP_COMPOSE",
    "PYVRP_INERT_BREAK",
    "PYVRP_PROX_WAIT",
    "PYVRP_BREAK_FREE_DURATION",
)


def mode_key() -> str:
    """The PYVRP_* mode of this process, as a stable string key."""
    return ",".join(f"{var}={os.environ.get(var, '')}" for var in MODE_VARS)


def _overnights(n=2):
    # The router's legacy overnight rest: 11 h after 14 h on duty, from noon.
    return [
        CustomBreak(
            id=k,
            tws=[(12 * H + k * DAY, INT_MAX)],
            service=11 * H,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=14 * H,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
        )
        for k in range(n)
    ]


def instance(shift_start=6 * H):
    """
    Six clients over three days. Clients 0-2 (over 14 h on duty) have no
    window; client 3 opens the next morning at 10:00 and client 5 the morning
    after at 08:00, so the rest served after day 1 ends before client 3's
    window opens and is followed by a wait (which the search's export folds
    into the rest, D5; the previous fork's Python-built route does not).
    """
    services = [4 * H, 5 * H, 4 * H, 4 * H, 8 * H, 2 * H]
    windows = [
        (0, INT_MAX),
        (0, INT_MAX),
        (0, INT_MAX),
        (DAY + 10 * H, DAY + 18 * H),
        (0, INT_MAX),
        (2 * DAY + 8 * H, 2 * DAY + 18 * H),
    ]
    n = len(services)
    locs = [Location(0, 0)] + [Location(i + 1, 0) for i in range(n)]
    clients = [
        Client(
            location=i + 1,
            delivery=[1],
            service_duration=services[i],
            tw_early=windows[i][0],
            tw_late=windows[i][1],
        )
        for i in range(n)
    ]
    mat = np.zeros((n + 1, n + 1), dtype=np.int64)
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                mat[i, j] = 1800 + 900 * abs(i - j)
    vt = VehicleType(
        capacity=[100],
        tw_early=shift_start,
        tw_late=INT_MAX,
        unit_duration_cost=20,
        custom_breaks=_overnights(),
    )
    return ProblemData(locs, clients, [Depot(location=0)], [vt], [mat], [mat])


# Client indices and rest tokens ("B<k>"). The first two are the case of the
# review: an overnight rest, then a wait for the morning window (with and
# without the second night's rest). The others place the rests where the
# legacy pass does not serve them (too early) or late, and drop them.
SEQUENCES = {
    "overnight_then_morning_wait": [0, 1, 2, "B0", 3, 4, "B1", 5],
    "overnight_then_wait_one_rest": [0, 1, 2, "B0", 3, 4, 5],
    "rest_too_early": ["B0", 0, 1, 2, 3, 4, "B1", 5],
    "rests_late": [0, 1, 2, 3, "B0", 4, 5, "B1"],
    "rests_back_to_back": [0, 1, 2, "B0", "B1", 3, 4, 5],
    "no_rest_nodes": [0, 1, 2, 3, 4, 5],
}


def activities(seq):
    out = []
    for tok in seq:
        if isinstance(tok, str):
            out.append(
                pyvrp.Activity(pyvrp.ActivityType.CUSTOM_BREAK, int(tok[1:]))
            )
        else:
            out.append(pyvrp.Activity(pyvrp.ActivityType.CLIENT, tok))
    return out


def route_fields(data, ce, route):
    """Every cost, schedule and feasibility field of ``route``."""
    sol = Solution(data, [route])
    return {
        "schedule": [
            [
                str(act),
                int(act.start_time),
                int(act.end_time),
                int(act.wait_duration),
                int(act.time_warp),
            ]
            for act in route.schedule()
        ],
        "distance": int(route.distance()),
        "distance_cost": int(route.distance_cost()),
        "duration": int(route.duration()),
        "duration_cost": int(route.duration_cost()),
        "overtime": int(route.overtime()),
        "time_warp": int(route.time_warp()),
        "travel": int(route.travel_duration()),
        "service": int(route.service_duration()),
        "wait": int(route.wait_duration()),
        "start_time": int(route.start_time()),
        "end_time": int(route.end_time()),
        "slack": int(route.slack()),
        "break_due": int(route.break_due()),
        "break_due_mask": int(route.break_due_mask()),
        "break_services": [int(s) for s in route.break_services()],
        "route_feasible": bool(route.is_feasible()),
        "solution_feasible": bool(sol.is_feasible()),
        "penalised_cost": int(ce.penalised_cost(sol)),
        "cost": int(ce.cost(sol)),
    }


def cost_evaluator():
    return CostEvaluator([1000], 300, 0, 500, 21.4)


def measure():
    out = {}
    ce = cost_evaluator()
    for start_name, start in (("start_06h", 6 * H), ("start_00h", 0)):
        data = instance(start)
        for name, seq in SEQUENCES.items():
            route = Route(data, activities(seq), 0)
            out[f"{start_name}/{name}"] = route_fields(data, ce, route)
    return out


def main(path):
    goldens = {}
    if os.path.exists(path):
        with open(path) as fh:
            goldens = json.load(fh)
    goldens[mode_key()] = measure()
    with open(path, "w") as fh:
        json.dump(goldens, fh, indent=1, sort_keys=True)


if __name__ == "__main__":
    main(sys.argv[1])
