"""A local-search descent that does not converge under ``PYVRP_INERT_BREAK=2``.

See §45 of ``docs/BREAK_REGIME_EVAL_FINDINGS.md``. On the EU 3-day instance,
warm-started from a single route holding every client, one descent runs past
900 s with the shipped defaults and finishes in 0.001 s with
``PYVRP_INERT_BREAK`` set to 0 or 1. The vehicle carries a SINGLE break rule,
which is why 1 escapes: it makes breaks inert only on multi-rule types.

The condition is the long route, not the rule -- with the same clients split
across three routes every subset of the four rules finishes in milliseconds.

Production is not affected (three instances converge, ~30% slower with 2 and
one more client served on each) and the router stops on ``MaxRuntime``, which
arms the ``set_time_budget`` valve in ``pyvrp/solve.py``. The suite reaches the
pathology only because ``MaxIterations`` leaves that valve unarmed.

    python benchmarks/_inert_descent_repro.py
    PYVRP_INERT_BREAK=0 python benchmarks/_inert_descent_repro.py

``--budget`` bounds each descent so the script always terminates; a descent
that reports the budget back did not converge.
"""

from __future__ import annotations

import argparse
import itertools
import os
import pathlib
import sys
import time

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import pyvrp  # noqa: E402
from pyvrp import (  # noqa: E402
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
)
from pyvrp.PenaltyManager import PenaltyManager  # noqa: E402
from pyvrp.read import read  # noqa: E402
from pyvrp.search import (  # noqa: E402
    LocalSearch,
    PerturbationManager,
    compute_neighbours,
)
from pyvrp.search._search import CLOCK_TRIGGER, INERT_DISCARDED_BREAK  # noqa
from pyvrp.solve import SolveParams  # noqa: E402

# The four rules the EU 3-day regulatory test configures. ``lunch`` replaces
# the spec's CLOCK_TIME rule with its DUTY_TIME equivalent, and the two
# overnights only apply past their day boundary.
RULES = {
    "drive(1)": CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,
        service=2700,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=10,
    ),
    "lunch(2)": CustomBreak(
        id=2,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=21600,
        service=2700,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=5,
    ),
    "night1(11)": CustomBreak(
        id=11,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=32400,
        service=39600,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=1,
        condition_min_route_s=86400,
    ),
    "night2(12)": CustomBreak(
        id=12,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=32400,
        service=39600,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=1,
        condition_min_route_s=172800,
    ),
}


def descent(base, rules, num_routes, budget, seed):
    """Time one exhaustive descent from an ``num_routes``-way warm start."""
    vehicle = base.vehicle_type(0).replace(
        custom_breaks=rules, shift_duration=259200
    )
    data = base.replace(vehicle_types=[vehicle])

    params = SolveParams()
    rng = pyvrp.RandomNumberGenerator(seed=seed)
    ls = LocalSearch(
        data,
        rng,
        compute_neighbours(data, params.neighbourhood),
        PerturbationManager(data, params.perturbation),
    )
    ls.set_time_budget(budget)  # so a non-converging descent still returns
    for op in params.operators:
        if op.supports(data):
            ls.add_operator(op(data))

    penalties = params.penalty.midpoint_penalties(data)
    evaluator = PenaltyManager(penalties, params.penalty, 0.0).cost_evaluator()

    n = data.num_clients
    size = (n + num_routes - 1) // num_routes
    visits = [list(range(i, min(i + size, n))) for i in range(0, n, size)]
    warm = pyvrp.Solution(data, visits)

    start = time.perf_counter()
    ls(warm, evaluator, exhaustive=True)
    return time.perf_counter() - start


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--budget", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--instance",
        default=str(_HERE / "instances" / "multi_day_3day_shift_cap.vrp"),
    )
    args = parser.parse_args()

    base = read(args.instance, round_func="dimacs")
    print(
        f"CLOCK_TRIGGER={CLOCK_TRIGGER} "
        f"INERT_DISCARDED_BREAK={INERT_DISCARDED_BREAK} "
        f"COMPOSE={os.environ.get('PYVRP_COMPOSE', '(default)')} "
        f"clients={base.num_clients} budget={args.budget}s",
        flush=True,
    )

    names = list(RULES)
    for size in range(1, len(names) + 1):
        for combo in itertools.combinations(names, size):
            rules = [RULES[name] for name in combo]
            cells = []
            for num_routes in (1, 3):
                elapsed = descent(
                    base, rules, num_routes, args.budget, args.seed
                )
                stalled = elapsed >= args.budget * 0.95
                cells.append(
                    f"routes={num_routes}: {elapsed:7.3f}s"
                    f"{'  DID NOT CONVERGE' if stalled else ''}"
                )
            print(f"  {'+'.join(combo):32} " + "   ".join(cells), flush=True)


if __name__ == "__main__":
    main()
