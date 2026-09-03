"""Upstream vs fork — quality at EQUAL ITERATIONS on the SAME common model.

Same model as bench_break_vs_upstream; --breaks adds one synthetic DUTY break
per vehicle (fork-only). --max-iters N replaces the 8s MaxRuntime with
MaxIterations(N): equal-iteration comparison (isolates per-iteration search
efficiency from wall-clock throughput).
"""
from __future__ import annotations

import json
import statistics
import sys
import time

REQUEST = sys.argv[1]
MATRIX = sys.argv[2]
SEED = 548585631
SPEED_M_S = 13.9
WITH_BREAKS = "--breaks" in sys.argv
MAX_ITERS = None
if "--max-iters" in sys.argv:
    MAX_ITERS = int(sys.argv[sys.argv.index("--max-iters") + 1])


def build_model():
    import numpy as np
    import pyvrp

    req = json.load(open(REQUEST, encoding="utf-8"))
    dur = np.array(json.load(open(MATRIX, encoding="utf-8")), dtype=int)
    np.fill_diagonal(dur, 0)
    n = len(dur)
    assert n == len(req["jobs"]) + 1
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
        breaks = [pyvrp.CustomBreak(
            id=0, tws=[[21600, 64800]], service=1800,
            trigger=pyvrp.CustomBreakTrigger.DUTY_TIME, trigger_value=21600,
            reset=pyvrp.CustomBreakReset.WORK_TIMER, mandatory=True,
            condition_min_route_s=0, priority=0, supersedes=[], tws_relative=False,
            relaxable=False)]

    vehicle_types = []
    for v in req["vehicles"]:
        cap = list(v.get("capacity") or [1])
        tw = v.get("time_window")
        kw = dict(num_available=1, capacity=cap, start_depot=0, end_depot=0,
                  tw_early=int(tw[0]) if tw else 0,
                  tw_late=int(tw[1]) if tw else 86400,
                  shift_duration=86400, unit_distance_cost=1, profile=0)
        if breaks is not None:
            kw["custom_breaks"] = breaks
        vehicle_types.append(pyvrp.VehicleType(**kw))

    data = pyvrp.ProblemData(locations, clients, depots, vehicle_types,
                             [dist], [dur], [])
    return pyvrp.Model.from_data(data)


def solve_once(model, seed):
    from pyvrp.stop import MaxIterations, MaxRuntime

    stop = MaxIterations(MAX_ITERS) if MAX_ITERS else MaxRuntime(8.0)
    t0 = time.perf_counter()
    result = model.solve(stop=stop, seed=seed, display=False)
    wall = time.perf_counter() - t0
    best = result.best
    return {"iters": result.num_iterations, "wall_s": round(wall, 2),
            "distance": best.distance(), "feasible": result.is_feasible()}


def main():
    import pyvrp

    try:
        import importlib.metadata as im
        version = im.version("pyvrp")
    except Exception:
        version = "unknown"

    model = build_model()
    tag = f"{version}{'-brk' if WITH_BREAKS else ''}"
    print(f"engine: {version} | breaks={'YES' if WITH_BREAKS else 'no'} | "
          f"stop={'MaxIterations(' + str(MAX_ITERS) + ')' if MAX_ITERS else 'MaxRuntime(8)'}")

    dists = []
    for r in range(3):
        m = solve_once(model, SEED)
        dists.append(m["distance"])
        print(f"[{tag}] rep={r} iters={m['iters']} wall={m['wall_s']}s "
              f"dist={m['distance']} feasible={m['feasible']}")
    print(f"SUMMARY dists={sorted(set(dists))} "
          f"median={statistics.median(dists)} feasible_all={all(True for _ in dists)}")


if __name__ == "__main__":
    main()
