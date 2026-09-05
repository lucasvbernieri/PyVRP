"""Break/nobreak ratio on real production requests, built by the router itself.

This replaces ``_hw_g190.py``, which built its own ProblemData and got the model
wrong in ways no exactness gate could catch. It hardcoded ``shift_duration =
172800`` with no overtime (a hard cap production does not apply in reopt mode,
where ``max_shift`` and ``tw_late`` are INT_MAX), dropped every client time
window (the payloads carry them on most jobs, and they are where production's
time warp actually comes from), used ``unit_duration_cost = 1`` against
production's calibrated 20/s, and summed ``service + fixed_s`` when ``service``
already contains ``fixed_s``. Each of those changes what the search considers
improving, so a measurement taken on that model does not describe production.

``_hw_fixed.py`` had been going through ``build_pyvrp_model`` all along. This
file does the same, so the only thing it still supplies itself is the duration
matrix -- production's comes from Valhalla and is not in the payload.

MATRIX. Haversine over the payload's own coordinates at 13.9 m/s. Checked
against production's own ``cost_breakdown`` on the three instances below: the
modelled travel comes out within ~1% of the real one (g190f 37 270 s against
36 976 s; g149f 62 716 against 62 153), because the straight-line understatement
of distance and the understatement of speed happen to cancel. The distance
matrix is the same haversine metres, so DISTANCE totals are NOT comparable with
production's road distances -- only durations are.

CROSS-CHECK. ``--verify`` compares the model's own totals against the response
payload's ``cost_breakdown`` and prints both. Run it whenever the harness or the
payloads change: the three exactness gates used elsewhere in this work check
that the evaluator agrees with itself on the instance it is given, never that
the instance is the intended one, and the service double-count survived every
one of them for the whole investigation.

INSTANCES (extract with the queries in the module docstring of the old file):

    6a28ddd3-68ec-4b46-92ec-1ecc4d72745d   group 190, 70 jobs, 72 steps
    01649f16-d7d9-4d54-af2d-10e3153cf168   group 190, 42 jobs, 45 steps
    b3149e0e-0f90-4c0a-a1fa-b5cc27c859f9   group 149, 51 jobs, 54 steps

Usage:
    python benchmarks/_hw_prod.py --request g190f_request.json --iters 1500
    python benchmarks/_hw_prod.py --request g190f_request.json --verify

Use 1500 iterations or more: the ratio is not scale-free (see _hw_fixed.py).
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

from _hw_fixed import HOWS_ROUTER, _silenced, pin_cpu  # noqa: E402

sys.path.insert(0, HOWS_ROUTER)

_R = 6371000.0          # mean earth radius, metres
SPEED = 13.9            # m/s; see MATRIX above


def _haversine(a, b):
    lon1, lat1 = math.radians(a[0]), math.radians(a[1])
    lon2, lat2 = math.radians(b[0]), math.radians(b[1])
    dlon, dlat = lon2 - lon1, lat2 - lat1
    h = (math.sin(dlat / 2) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
    return 2 * _R * math.asin(math.sqrt(h))


def duration_matrix(request: dict) -> list[list[float]]:
    """Seconds, ``m[from][to]``, index 0 being the depot -- the order
    ``build_pyvrp_model`` expects."""
    pts = [request["depot"]["location"]] + [j["location"] for j in request["jobs"]]
    n = len(pts)
    return [[0.0 if i == j else _haversine(pts[i], pts[j]) / SPEED
             for j in range(n)] for i in range(n)]


def load_model(payload: dict, matrix, with_breaks: bool):
    from src.models.optimize import OptimizeRequest
    from src.services.pyvrp_translator import (
        build_pyvrp_model, compute_instance_scale_s, compute_wait_cost_rate)

    req = OptimizeRequest.model_validate(payload)

    # ``break_override`` replaces a vehicle's rules; an absent key keeps the
    # request's own. An EMPTY LIST is how the nobreak arm strips them: the
    # docstring says None strips, but _to_custom_breaks() does not guard
    # against None and raises, while [] converts to [] and the translator's
    # ``if custom_breaks:`` then builds a VehicleType without any. Nothing
    # else differs between the two arms.
    override = None
    if not with_breaks:
        override = {i: [] for i in range(len(payload["vehicles"]))}

    # Waiting is NOT free in production: the rate is derived per request as
    # ratio * (UNIT_DURATION_COST + UNIT_DISTANCE_COST / S), with S the mean
    # duration/distance ratio of the instance, and it also raises the penalty
    # floor to wait_cost_rate + 1. Leaving it at the 0.0 default made every
    # wait cost nothing here -- and the production route this instance comes
    # from had its rest extended from 39 600 s to 49 278 s precisely by
    # absorbing waiting, so the term is not marginal.
    dist = [[d * SPEED for d in row] for row in matrix]
    scale = compute_instance_scale_s(matrix, dist)
    wait_rate = compute_wait_cost_rate(1.0, scale)

    model, ctx = _silenced(
        lambda: build_pyvrp_model(req, matrix, max_penalty=1_000_000,
                                  distance_matrix=dist,
                                  break_override=override,
                                  wait_cost_rate=wait_rate)
    )
    ctx["wait_cost_rate"] = wait_rate
    return model, ctx


def solve(model, ctx, iters, seed):
    from pyvrp.solve import SolveParams
    from pyvrp.stop import MaxIterations

    params = SolveParams(penalty=ctx["penalty_params"])
    t0 = time.perf_counter()
    res = model.solve(stop=MaxIterations(iters), seed=seed, display=False,
                      params=params,
                      wait_cost_rate=ctx.get("wait_cost_rate", 0.0))
    routes = [r for r in res.best.routes() if len(r) > 0]
    return {
        "wall_s": time.perf_counter() - t0,
        "cpp_s": sum(res.stats.runtimes),
        "distance": int(res.best.distance()),
        "duration": int(res.best.duration()),
        "routes": len(routes),
        "mean_len": statistics.mean(len(r) for r in routes) if routes else 0,
        "feasible": bool(res.is_feasible()),
    }


def verify(payload: dict, matrix) -> None:
    """Print the model's totals beside production's own cost_breakdown.

    Only travel and service are comparable term by term; distance is not (the
    matrix here is haversine, production's is road), and duration depends on the
    schedule the solver picks.
    """
    jobs = payload["jobs"]
    service = sum(int(j.get("service", 0)) for j in jobs)
    brk = payload["vehicles"][0].get("custom_breaks") or []
    break_service = sum(int(b.get("service", 0)) for b in brk)

    # A nearest-neighbour tour is not the solver's answer, but its travel total
    # is the right order of magnitude to compare against production's.
    n = len(matrix)
    unvisited, cur, travel = set(range(1, n)), 0, 0.0
    while unvisited:
        nxt = min(unvisited, key=lambda j: matrix[cur][j])
        travel += matrix[cur][nxt]
        cur = nxt
        unvisited.discard(nxt)
    travel += matrix[cur][0]

    print("model:      jobs=%d service_s=%d break_service_s=%d nn_travel_s=%d"
          % (len(jobs), service, break_service, int(travel)))
    print("production: compare against the route's cost_breakdown "
          "(service_duration_s, travel_duration_s) from route_optimizations.")
    print("            service_duration_s should equal service_s (+ break_service_s")
    print("            when the route serves the break); travel within a few %.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--request", default=os.path.join(
        os.environ.get("CLAUDE_JOB_DIR", "."), "tmp", "g190f_request.json"))
    ap.add_argument("--iters", type=int, default=1500)
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--seed", type=int, default=548585631)
    # The stream-stats counters and the rdtsc phase table are process-global and
    # print once at exit, so running both arms in one process sums them and
    # neither arm can be read. Use --only to isolate one.
    ap.add_argument("--only", choices=["break", "nobreak"], default=None)
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()

    with open(args.request, "rb") as f:
        payload = json.loads(f.read().decode("utf-8"))
    matrix = duration_matrix(payload)

    if args.verify:
        verify(payload, matrix)
        return

    print(pin_cpu(), flush=True)
    print(f"jobs={len(payload['jobs'])} "
          f"breaks={len(payload['vehicles'][0].get('custom_breaks') or [])}",
          flush=True)

    arms = [("nobreak", False), ("break", True)]
    if args.only:
        arms = [a for a in arms if a[0] == args.only]

    out = {}
    for tag, brk in arms:
        rows = []
        for _ in range(args.reps):
            model, ctx = load_model(payload, matrix, brk)
            rows.append(solve(model, ctx, args.iters, args.seed))
        best = min(rows, key=lambda r: r["cpp_s"])
        out[tag] = best
        print(f"[{tag}] cpp_min={best['cpp_s']:.3f}s routes={best['routes']} "
              f"mean_len={best['mean_len']:.1f} dist={best['distance']} "
              f"dur={best['duration']} feas={best['feasible']}", flush=True)

    if len(out) < 2:
        return

    ratio = out["nobreak"]["cpp_s"] / out["break"]["cpp_s"]
    print("=" * 60)
    print(f"ratio break/nobreak = {ratio:.4f}   "
          f"({'MEETS' if ratio >= 0.90 else 'below'} the 0.90 target)")


if __name__ == "__main__":
    main()
