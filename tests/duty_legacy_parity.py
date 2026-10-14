"""
Legacy (``eligible_from = -1``, ``duty_limit = -1``) measurements for the duty
model's parity test (``test_duty_model_legacy_parity.py``).

The measurements only use the API the fork had before the duty model, so the
same code runs against that build to produce the goldens:

    PYTHONPATH=<build of 690fee3> PYVRP_...=<mode> \\
        python tests/duty_legacy_parity.py tests/data/duty_legacy_goldens.json

Each run adds (or replaces) the entry of the PYVRP_* mode it runs under. This
module is deliberately not a test module and imports nothing from ``tests``.
"""

import json
import os
import sys
from itertools import permutations

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
from pyvrp.PenaltyManager import PenaltyParams
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations

H = 3600
INT_MAX = 2**31 - 1
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


def _instance(rules, n=8, horizon=INT_MAX, windows=None):
    locs = [Location(0, 0)] + [Location(i + 1, 0) for i in range(n)]
    services = [3 * H, 2 * H, 4 * H, H, 3 * H, 2 * H, 6 * H, H][:n]
    clients = []
    for i in range(n):
        early, late = windows[i] if windows else (0, horizon)
        clients.append(
            Client(
                location=i + 1,
                delivery=[1],
                service_duration=services[i],
                tw_early=early,
                tw_late=late,
            )
        )
    mat = np.zeros((n + 1, n + 1), dtype=np.int64)
    for i in range(n + 1):
        for j in range(n + 1):
            mat[i, j] = 1800 * abs(i - j) + (900 if (i * j) % 3 == 1 else 0)
            if i == j:
                mat[i, j] = 0
    vt = VehicleType(
        capacity=[100],
        tw_early=0,
        tw_late=horizon,
        unit_duration_cost=20,
        custom_breaks=rules,
    )
    return ProblemData(
        locs, clients, [Depot(location=0)], [vt], [mat], [mat]
    )


def instances():
    """Legacy instances: overnight DUTY_TIME rules, a multi-rule vehicle, a
    vehicle without rules, and windows that force lateness."""
    overnights = [
        CustomBreak(
            id=k,
            tws=[(43200 + k * 86400, INT_MAX)],
            service=39600,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=50400,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
        )
        for k in range(2)
    ]
    multi = [
        CustomBreak(
            id=0,
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=4 * H,
            service=1800,
            reset=CustomBreakReset.DRIVE_TIMER,
            mandatory=True,
            priority=10,
        ),
        CustomBreak(
            id=1,
            service=39600,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=14 * H,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            priority=20,
        ),
    ]
    tight = [(k * 3 * H, k * 3 * H + 2 * H) for k in range(8)]
    return {
        "overnights": _instance(overnights),
        "multi_rule": _instance(multi),
        "no_rules": _instance([]),
        "tight_windows": _instance(overnights, windows=tight),
    }


def _route_stats(data, ce, visits_with_breaks):
    acts = []
    for tok in visits_with_breaks:
        if isinstance(tok, str):
            acts.append(pyvrp.Activity(pyvrp.ActivityType.CUSTOM_BREAK, int(tok[1:])))
        else:
            acts.append(pyvrp.Activity(pyvrp.ActivityType.CLIENT, tok))
    route = Route(data, acts, 0)
    sol = Solution(data, [route])
    return [
        int(route.duration()),
        int(route.time_warp()),
        int(route.break_due()),
        int(route.break_due_mask()),
        int(route.distance()),
        int(route.duration_cost()),
        [int(s) for s in route.break_services()],
        bool(route.is_feasible()),
        int(ce.penalised_cost(sol)),
    ]


def measure():
    out = {}
    for name, data in instances().items():
        ce = CostEvaluator([1000], 300, 0, 500, 21.4)
        n = data.num_clients
        has_rules = bool(data.vehicle_type(0).custom_breaks)

        routes = []
        for k, perm in enumerate(permutations(range(n))):
            if k % 997 != 0:  # a spread sample of the orders
                continue
            seq = list(perm)
            routes.append(_route_stats(data, ce, seq))
            if has_rules:
                with_breaks = seq[:3] + ["B0"] + seq[3:6] + ["B1"] + seq[6:]
                if name == "multi_rule":
                    with_breaks = seq[:2] + ["B0"] + seq[2:5] + ["B1"] + seq[5:]
                routes.append(_route_stats(data, ce, with_breaks))

        solves = []
        for seed in (1, 2):
            params = SolveParams(
                penalty=PenaltyParams(min_penalty=0.1, max_penalty=1e5)
            )
            res = pyvrp.solve(
                data,
                MaxIterations(150),
                seed=seed,
                params=params,
                wait_cost_rate=21.4,
                collect_stats=False,
            )
            best = res.best
            solves.append(
                [
                    float(res.cost()),
                    int(ce.penalised_cost(best)),
                    [str(route) for route in best.routes()],
                    int(res.num_iterations),
                ]
            )
        out[name] = {"routes": routes, "solves": solves}
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
