"""Break/nobreak ratio on the production long-route case (group 190).

Every measurement in ``docs/BREAK_HW_OPT.md`` comes from group-54, whose routes
are ~23 activities and whose two mandatory rules trigger at 43 200 s -- a value
no route there reaches, so they never fire. Group 190 (SÃO JOSE RIO PRETO) is the
opposite case and it is real: ``custom_max_route_s`` is 172 800 (48 h), the
vehicle carries ONE mandatory DUTY_TIME rule at 43 200 s with an 11-hour rest and
an ALL_TIMERS reset, and the solved routes run to 72 activities (median 50 over
78 routes). The trigger fires; the route spans two days.

Over 1 569 solved production routes the length distribution is median 25, p90 40,
p99 58, max 72 -- so this is the tail that matters, not an exotic case.

The instance is rebuilt here from the request payload's own coordinates, service
times and break rule, with a haversine duration matrix, rather than through the
router's matrix pipeline: the point is the SHAPE (route length, break semantics),
and both arms see the same matrix.

The request payload is production data and is deliberately NOT committed.
Extract one from the local Postgres before running:

    docker exec hows-router-postgres-1 psql -U router -d hows_router -tAc       "SELECT request_payload::text FROM route_optimizations
       WHERE id='6a28ddd3-68ec-4b46-92ec-1ecc4d72745d'" > g190_request.json

To pick a different one, order group 190's optimizations by their longest route:

    WITH r AS (
      SELECT o.id, max(jsonb_array_length(rt->'steps')) AS max_steps
      FROM route_optimizations o,
           LATERAL jsonb_array_elements(o.response_payload->'routes') rt
      WHERE rt->>'group_code' = '190'
      GROUP BY o.id)
    SELECT id, max_steps FROM r ORDER BY max_steps DESC LIMIT 5;

    python benchmarks/_hw_g190.py --request g190_request.json --iters 200
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from _hw_fixed import pin_cpu  # noqa: E402

# Mean earth radius; the instance spans a few hundred km so a spherical
# approximation is well inside what a performance benchmark needs.
_R = 6371000.0
SPEED = 13.9  # m/s, the same figure the other harnesses use


def _haversine(a, b):
    lon1, lat1 = math.radians(a[0]), math.radians(a[1])
    lon2, lat2 = math.radians(b[0]), math.radians(b[1])
    dlon, dlat = lon2 - lon1, lat2 - lat1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * _R * math.asin(math.sqrt(h))


def build(request: dict, with_breaks: bool):
    import numpy as np
    import pyvrp

    depot = request["depot"]["location"]
    jobs = request["jobs"]
    pts = [depot] + [j["location"] for j in jobs]
    n = len(pts)

    dur = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(n):
            if i != j:
                dur[i][j] = int(_haversine(pts[i], pts[j]) / SPEED)
    dist = np.round(dur * SPEED).astype(int)

    # PyVRP wants integral planar coordinates; the matrices carry the real
    # geometry, so these are only identifiers.
    locations = [pyvrp.Location(x=int(p[0] * 1e5), y=int(p[1] * 1e5)) for p in pts]

    horizon = 7 * 86400
    clients = []
    for i, j in enumerate(jobs, start=1):
        service = int(j.get("service", 0)) + int(j.get("fixed_s", 0))
        clients.append(pyvrp.Client(location=i, delivery=[1],
                                    service_duration=service,
                                    tw_early=0, tw_late=horizon))
    depots = [pyvrp.Depot(location=0, tw_early=0, tw_late=horizon)]

    vreq = request["vehicles"][0]
    breaks = None
    if with_breaks:
        breaks = []
        for b in vreq.get("custom_breaks") or []:
            breaks.append(pyvrp.CustomBreak(
                id=int(b["id"]),
                tws=[[int(t[0]), int(t[1])] for t in b["tws"]],
                service=int(b["service"]),
                trigger=getattr(pyvrp.CustomBreakTrigger, b["trigger"]),
                trigger_value=int(b["trigger_value"]),
                reset=getattr(pyvrp.CustomBreakReset, b["reset"]),
                mandatory=bool(b["mandatory"]),
                condition_min_route_s=int(b.get("condition_min_route_s", 0)),
                priority=int(b.get("priority", 0)),
                supersedes=list(b.get("supersedes", [])),
                tws_relative=bool(b.get("tws_relative", False)),
                relaxable=False,
            ))
        if not breaks:
            breaks = None

    # shift_duration 172 800 s is the group's own custom_max_route_s (48 h);
    # unit_duration_cost is what makes the duration terms (and therefore the
    # break's 11-hour rest) matter to the objective at all.
    kw = dict(num_available=1, capacity=[10_000], start_depot=0, end_depot=0,
              tw_early=0, tw_late=horizon, shift_duration=172800,
              unit_distance_cost=1, unit_duration_cost=1, profile=0)
    if breaks:
        kw["custom_breaks"] = breaks

    data = pyvrp.ProblemData(locations, clients, depots,
                             [pyvrp.VehicleType(**kw)], [dist], [dur], [])
    return pyvrp.Model.from_data(data)


def solve(model, iters, seed):
    from pyvrp.stop import MaxIterations

    t0 = time.perf_counter()
    res = model.solve(stop=MaxIterations(iters), seed=seed, display=False)
    routes = [r for r in res.best.routes() if len(r) > 0]
    return {
        "wall_s": time.perf_counter() - t0,
        "cpp_s": sum(res.stats.runtimes),
        "distance": int(res.best.distance()),
        "routes": len(routes),
        "mean_len": statistics.mean(len(r) for r in routes) if routes else 0,
        "feasible": bool(res.is_feasible()),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--request", default=os.path.join(
        os.environ.get("CLAUDE_JOB_DIR", "."), "tmp", "g190_request.json"))
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--seed", type=int, default=548585631)
    args = ap.parse_args()

    print(pin_cpu(), flush=True)
    with open(args.request, encoding="utf-8") as f:
        request = json.load(f)
    print(f"jobs={len(request['jobs'])} "
          f"breaks={len(request['vehicles'][0].get('custom_breaks') or [])}",
          flush=True)

    out = {}
    for tag, brk in (("nobreak", False), ("break", True)):
        model = build(request, brk)
        runs = [solve(model, args.iters, args.seed) for _ in range(args.reps)]
        best = min(runs, key=lambda r: r["cpp_s"])
        out[tag] = best
        print(f"[{tag}] cpp_min={best['cpp_s']:.3f}s routes={best['routes']} "
              f"mean_len={best['mean_len']:.1f} dist={best['distance']} "
              f"feas={best['feasible']}", flush=True)

    ratio = out["nobreak"]["cpp_s"] / out["break"]["cpp_s"]
    print("=" * 60)
    print(f"ratio break/nobreak = {ratio:.4f}   "
          f"({'MEETS' if ratio >= 0.90 else 'below'} the 0.90 target)")


if __name__ == "__main__":
    main()
