"""Upstream vs fork ??? BREAK-speed A/B on the SAME common model.

Same 510-client day-1 model as bench_upstream_vs_fork; when --breaks is set,
ONE synthetic DUTY_TIME break (30 min, trigger 6h duty, midday window) is added
to each of the 32 vehicle types ??? fork-only capability. Upstream runs the same
model WITHOUT breaks (all it can run). Direct ratio: fork-break / upstream-nobreak.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

REQUEST = sys.argv[1]
MATRIX = sys.argv[2]
SEED = 548585631
RUNTIME_S = 8.0\nMAX_ITERS = 1100\nUSE_MAX_ITERS = "--max-iters" in sys.argv
SPEED_M_S = 13.9
WITH_BREAKS = "--breaks" in sys.argv


def build_model():
    import numpy as np
    import pyvrp

    req = json.load(open(REQUEST, encoding="utf-8"))
    dur = np.array(json.load(open(MATRIX, encoding="utf-8")), dtype=int)
    np.fill_diagonal(dur, 0)
    n = len(dur)
    assert n == len(req["jobs"]) + 1, (n, len(req["jobs"]))
    dist = np.round(dur * SPEED_M_S).astype(int)
    np.fill_diagonal(dist, 0)

    depot = req["depot"]
    locations = [pyvrp.Location(x=depot["location"][0], y=depot["location"][1])]
    locations += [pyvrp.Location(x=j["location"][0], y=j["location"][1]) for j in req["jobs"]]

    clients = []
    for i, j in enumerate(req["jobs"], start=1):
        tw = (j.get("time_windows") or [[0, 86400]])[0]
        demand = j.get("demand") or [0]
        clients.append(pyvrp.Client(location=i, delivery=list(demand),
                                    service_duration=int(j.get("service") or 0),
                                    tw_early=int(tw[0]), tw_late=int(tw[1])))

    depots = [pyvrp.Depot(location=0, tw_early=0, tw_late=86400)]

    breaks = None
    if WITH_BREAKS:
        try:
            breaks = [pyvrp.CustomBreak(
                id=0, tws=[[21600, 64800]], service=1800,
                trigger=pyvrp.CustomBreakTrigger.DUTY_TIME, trigger_value=21600,
                reset=pyvrp.CustomBreakReset.WORK_TIMER, mandatory=True,
                condition_min_route_s=0, priority=0, supersedes=[], tws_relative=False,
                relaxable=False,
            )]
        except Exception as e:
            print(f"CustomBreak construction failed (fork-only API): {e}")
            raise

    vehicle_types = []
    for v in req["vehicles"]:
        cap = list(v.get("capacity") or [1])
        tw = v.get("time_window")
        tw_early = int(tw[0]) if tw else 0
        tw_late = int(tw[1]) if tw else 86400
        kw = dict(num_available=1, capacity=cap, start_depot=0, end_depot=0,
                  tw_early=tw_early, tw_late=tw_late, shift_duration=86400,
                  unit_distance_cost=1, profile=0)
        if breaks is not None:
            kw["custom_breaks"] = breaks  # fork-only kwarg
        vehicle_types.append(pyvrp.VehicleType(**kw))

    data = pyvrp.ProblemData(locations, clients, depots, vehicle_types,
                             [dist], [dur], [])
    return pyvrp.Model.from_data(data)


def solve_once(model, seed, runtime_s):
    from pyvrp.stop import MaxIterations, MaxRuntime

    t0 = time.perf_counter()
    stop = MaxIterations(MAX_ITERS) if USE_MAX_ITERS else MaxRuntime(runtime_s)\n    result = model.solve(stop=stop, seed=seed, display=False)
    wall = time.perf_counter() - t0
    cpp = sum(result.stats.runtimes)
    best = result.best
    return {
        "iters": result.num_iterations,
        "wall_s": round(wall, 2),
        "iters_per_s": result.num_iterations / cpp if cpp > 0 else 0.0,
        "distance": best.distance(),
        "feasible": result.is_feasible(),
    }


def main():
    import pyvrp

    try:
        import importlib.metadata as im
        version = im.version("pyvrp")
    except Exception:
        version = "unknown"

    model = build_model()
    print(f"engine: {version} | breaks={'YES' if WITH_BREAKS else 'no'} | pyvrp={pyvrp.__file__}")

    reps = 5
    rows = []
    for r in range(reps):
        m = solve_once(model, SEED, RUNTIME_S)
        m["rep"] = r
        rows.append(m)
        print(f"[{version}{'-brk' if WITH_BREAKS else ''}] rep={r} iters={m['iters']} "
              f"iters/s={m['iters_per_s']:.1f} dist={m['distance']} feasible={m['feasible']}")

    ips = [m["iters_per_s"] for m in rows]
    dists = sorted(set(m["distance"] for m in rows))
    print(f"SUMMARY iters/s median={statistics.median(ips):.1f} "
          f"p25={sorted(ips)[1]:.1f} p75={sorted(ips)[3]:.1f} "
          f"dists={dists} feasible_all={all(m['feasible'] for m in rows)}")


if __name__ == "__main__":
    main()

