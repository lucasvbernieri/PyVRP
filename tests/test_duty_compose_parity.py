"""
The duty model's composed proposal evaluator (``PYVRP_DUTY_COMPOSE``) and its
lower bound in ``CostEvaluator::deltaCost`` (``PYVRP_DUTY_BOUND``) only change
how fast a proposal is evaluated, never the result: seeded solves give the
same routes, schedules, costs, best penalised solution and iteration
trajectory with both switched off, with either one on, and with both on (the
default).

The switches are read once, when the extension loads, so each combination
runs in its own interpreter.
"""

import json
import os
import pathlib
import subprocess
import sys

import pytest
from numpy.testing import assert_equal

ROOT = pathlib.Path(__file__).parent.parent

SCRIPT = """
import json
import random
from dataclasses import astuple

import pyvrp
from pyvrp.PenaltyManager import PenaltyParams
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxIterations

from tests.test_duty_model import DAY, H, Stop, build


def instances():
    rng = random.Random(4)

    # Multi-day windows (one client per day and window, as the router
    # models them), several rests, a fixed departure.
    stops = []
    for _ in range(14):
        open_ = rng.choice([7, 8, 9, 13]) * H
        close = open_ + rng.choice([4, 6, 9]) * H
        windows = [(open_ + d * DAY, close + d * DAY) for d in range(3)]
        stops.append(Stop(rng.randint(H // 2, 4 * H), 30 * 60, windows))
    yield "windows", build(stops, T=8 * H, OT=2 * H, n_rests=3, departure=6 * H)

    # No windows and too few rests: the excess is unavoidable.
    stops = [Stop(rng.randint(H, 5 * H), rng.randint(600, 3600)) for _ in range(16)]
    yield "excess", build(stops, T=8 * H, OT=2 * H, n_rests=1, departure=4 * H)

    # Asymmetric travel, windows on some stops, departure at midnight.
    n = 12
    durations = [[0 if i == j else rng.randint(20 * 60, 3 * H) for j in range(n + 1)]
                 for i in range(n + 1)]
    stops = []
    for _ in range(n):
        if rng.random() < 0.5:
            stops.append(Stop(0, 45 * 60))
        else:
            open_ = rng.choice([8, 10, 14]) * H
            stops.append(Stop(0, 45 * 60, [(open_, open_ + 5 * H),
                                            (open_ + DAY, open_ + DAY + 5 * H)]))
    yield "asym", build(stops, durations=durations, T=10 * H, OT=H, n_rests=2,
                        departure=0)


out = {}
for name, inst in instances():
    fl = inst.w.fork_late
    params = SolveParams(
        penalty=PenaltyParams(min_penalty=fl, max_penalty=fl),
        penalised_best=True,
    )
    for seed in range(3):
        res = pyvrp.solve(
            inst.data,
            MaxIterations(150),
            seed=seed,
            params=params,
            wait_cost_rate=inst.wait_rate,
            collect_stats=True,
            display=False,
        )

        def schedule(sol):
            return [[(str(a), a.start_time, a.end_time) for a in route]
                    for route in sol.routes()]

        out[f"{name}/{seed}"] = {
            "best": schedule(res.best),
            "best_penalised": schedule(res.best_penalised),
            "cost": str(res.cost()),
            "iterations": res.num_iterations,
            "stats": [astuple(datum) for datum in res.stats.data],
        }

print(json.dumps(out))
"""

MODES = [
    {"PYVRP_DUTY_COMPOSE": "0", "PYVRP_DUTY_BOUND": "0"},
    {"PYVRP_DUTY_COMPOSE": "0"},
    {"PYVRP_DUTY_BOUND": "0"},
    {},
]


def _run(mode):
    env = {
        key: value
        for key, value in os.environ.items()
        if key not in ("PYVRP_DUTY_COMPOSE", "PYVRP_DUTY_BOUND")
    }
    env.update(mode)
    path = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(p for p in (str(ROOT), path) if p)
    proc = subprocess.run(
        [sys.executable, "-c", SCRIPT],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


@pytest.fixture(scope="module")
def reference():
    return _run(MODES[0])


@pytest.mark.parametrize("mode", MODES[1:], ids=str)
def test_duty_compose_and_bound_do_not_change_the_search(reference, mode):
    actual = _run(mode)
    assert_equal(sorted(actual), sorted(reference))
    for key in reference:
        assert_equal(actual[key], reference[key], key)
