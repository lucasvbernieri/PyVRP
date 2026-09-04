"""Does the break/nobreak ratio depend on route length?

Every conclusion in ``docs/BREAK_HW_OPT.md`` comes from one real instance whose
routes are ~23 activities long. The regime decomposition that the design in
``openspec/changes/break-regime-eval`` proposes is an O(#events) idea, so it
should only start paying once the node count dominates the per-event
bookkeeping. This builds the same synthetic instance at several route lengths
(same clients, fewer vehicles => longer routes) and reports the ratio at each,
so the conclusion can be checked against the regime the design assumes.

    python benchmarks/_hw_scale.py --clients 240 --iters 200
"""

from __future__ import annotations

import argparse
import math
import os
import random
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from _hw_fixed import pin_cpu  # noqa: E402

SPEED = 13.9  # metres per second, matching the other harnesses


def build(num_clients: int, num_vehicles: int, with_breaks: bool, seed: int):
    import numpy as np
    import pyvrp

    rng = random.Random(seed)
    pts = [(0, 0)] + [(rng.randint(-60000, 60000), rng.randint(-60000, 60000))
                      for _ in range(num_clients)]
    n = len(pts)

    dur = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(n):
            if i != j:
                d = math.dist(pts[i], pts[j])
                dur[i][j] = int(d / SPEED)
    dist = np.round(dur * SPEED).astype(int)

    locations = [pyvrp.Location(x=x, y=y) for x, y in pts]
    clients = [
        pyvrp.Client(location=i, delivery=[1], service_duration=900,
                     tw_early=0, tw_late=7 * 86400)
        for i in range(1, n)
    ]
    depots = [pyvrp.Depot(location=0, tw_early=0, tw_late=7 * 86400)]

    breaks = None
    if with_breaks:
        # One mandatory duty-time rest, the shape the fork's regulatory cases
        # use: fires after six hours of duty and resets the work timer.
        breaks = [pyvrp.CustomBreak(
            id=0, tws=[[0, 7 * 86400]], service=1800,
            trigger=pyvrp.CustomBreakTrigger.DUTY_TIME, trigger_value=21600,
            reset=pyvrp.CustomBreakReset.WORK_TIMER, mandatory=True,
            condition_min_route_s=0, priority=0, supersedes=[],
            tws_relative=False, relaxable=False)]

    vehicle_types = []
    for _ in range(num_vehicles):
        kw = dict(num_available=1, capacity=[10_000], start_depot=0,
                  end_depot=0, tw_early=0, tw_late=7 * 86400,
                  shift_duration=7 * 86400, unit_distance_cost=1, profile=0)
        if breaks is not None:
            kw["custom_breaks"] = breaks
        vehicle_types.append(pyvrp.VehicleType(**kw))

    data = pyvrp.ProblemData(locations, clients, depots, vehicle_types,
                             [dist], [dur], [])
    return pyvrp.Model.from_data(data)


def solve(model, iters: int, seed: int):
    from pyvrp.stop import MaxIterations

    t0 = time.perf_counter()
    res = model.solve(stop=MaxIterations(iters), seed=seed, display=False)
    wall = time.perf_counter() - t0
    routes = [r for r in res.best.routes() if len(r) > 0]
    mean_len = statistics.mean(len(r) for r in routes) if routes else 0.0
    return {
        "wall_s": wall,
        "cpp_s": sum(res.stats.runtimes),
        "routes": len(routes),
        "mean_route_len": mean_len,
        "distance": int(res.best.distance()),
        "feasible": bool(res.is_feasible()),
        "iters": res.num_iterations,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clients", type=int, default=240)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--seed", type=int, default=548585631)
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--vehicles", type=int, nargs="*",
                    default=[48, 20, 10, 5])
    args = ap.parse_args()

    print(pin_cpu(), flush=True)
    print(f"clients={args.clients} iters={args.iters} reps={args.reps}\n")
    print(f"{'vehicles':>9} {'nb len':>7} {'br len':>7} {'nobreak s':>10} "
          f"{'break s':>9} {'ratio':>7} {'nb feas':>8} {'br feas':>8}")

    for v in args.vehicles:
        best = {}
        for tag, brk in (("nobreak", False), ("break", True)):
            runs = [solve(build(args.clients, v, brk, args.seed), args.iters,
                          args.seed) for _ in range(args.reps)]
            best[tag] = min(runs, key=lambda r: r["cpp_s"])
        ratio = best["nobreak"]["cpp_s"] / best["break"]["cpp_s"]
        print(f"{v:>9} {best['nobreak']['mean_route_len']:>7.1f} "
              f"{best['break']['mean_route_len']:>7.1f} "
              f"{best['nobreak']['cpp_s']:>10.3f} {best['break']['cpp_s']:>9.3f} "
              f"{ratio:>7.3f} {str(best['nobreak']['feasible']):>8} "
              f"{str(best['break']['feasible']):>8}", flush=True)


if __name__ == "__main__":
    main()
