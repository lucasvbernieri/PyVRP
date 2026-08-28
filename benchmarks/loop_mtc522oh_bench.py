"""Loop benchmark: iters/s for the group-54 (32d73c97) instance, with/without breaks.

Builds the PyVRP Model in-process from the hows-router resolved request +
real duration matrix (both already materialised on disk), then times repeated
``model.solve(...)`` runs under a fixed seed and MaxRuntime stop.

Scenarios:
    A "nobreak": all vehicles' custom_breaks stripped (pure routing).
    B "break"  : natural custom_breaks kept (5 per vehicle, DUTY_TIME).

Metrics per rep: iters, sum(stats.runtimes), wall solve time, iters/s
(iters / sum(runtimes)), and objective distance (best.distance()).
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import statistics
import sys
import time

HOWS_ROUTER = r"C:\Users\lupi_\projetos\zanella\CascadeProjects\hows-router"
REQUEST_JSON = (
    HOWS_ROUTER
    + r"\optimization_exports\32d73c97-9258-4a39-8d69-e6e47367c048\request.json"
)
MATRIX_JSON = HOWS_ROUTER + r"\bench\real_matrix.json"

sys.path.insert(0, HOWS_ROUTER)


def _silenced(fn):
    """Run fn with fd-level stdout/stderr suppression (C-level prints)."""
    devnull = open(os.devnull, "w")
    saved = os.dup(1), os.dup(2)
    os.dup2(devnull.fileno(), 1)
    os.dup2(devnull.fileno(), 2)
    try:
        return fn()
    finally:
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        os.close(saved[0])
        os.close(saved[1])
        devnull.close()


def load_model(break_override=None):
    from src.models.optimize import OptimizeRequest
    from src.services.pyvrp_translator import build_pyvrp_model

    with open(REQUEST_JSON, encoding="utf-8") as f:
        req = OptimizeRequest.model_validate(json.load(f))
    with open(MATRIX_JSON, encoding="utf-8") as f:
        matrix = json.load(f)

    def _build():
        return build_pyvrp_model(
            req, matrix, max_penalty=1_000_000, break_override=break_override
        )

    model, ctx = _silenced(_build)
    return model, ctx


def solve_once(model, ctx, seed, runtime_s):
    from pyvrp.solve import SolveParams
    from pyvrp.stop import MaxRuntime

    params = SolveParams(penalty=ctx["penalty_params"])
    t0 = time.perf_counter()
    result = model.solve(
        stop=MaxRuntime(runtime_s), seed=seed, display=False, params=params
    )
    wall = time.perf_counter() - t0
    runtimes = list(result.stats.runtimes)
    iters = result.num_iterations
    cpp_time = sum(runtimes)
    iters_per_s = iters / cpp_time if cpp_time > 0 else 0.0
    iters_per_s_wall = iters / wall if wall > 0 else 0.0
    best = result.best
    return {
        "iters": iters,
        "cpp_time_s": cpp_time,
        "wall_s": wall,
        "iters_per_s": iters_per_s,
        "iters_per_s_wall": iters_per_s_wall,
        "distance": best.distance(),
        "cost": result.cost(),
        "feasible": result.is_feasible(),
    }


def run_scenario(name, break_override, seed, runtime_s, reps):
    model, ctx = load_model(break_override=break_override)
    rows = []
    for r in range(reps):
        # fresh model per rep for clean isolation (matches prior harness)
        if r > 0:
            model, ctx = load_model(break_override=break_override)
        m = solve_once(model, ctx, seed, runtime_s)
        m["rep"] = r
        rows.append(m)
        print(
            f"[{name}] rep={r} iters={m['iters']} "
            f"iters/s={m['iters_per_s']:.1f} wall={m['wall_s']:.2f}s "
            f"dist={m['distance']} feasible={m['feasible']}"
        )
    ips = [m["iters_per_s"] for m in rows]
    dists = [m["distance"] for m in rows]
    return {
        "scenario": name,
        "seed": seed,
        "runtime_s": runtime_s,
        "reps": reps,
        "iters_per_s_median": statistics.median(ips),
        "iters_per_s_p25": _quantile(ips, 0.25),
        "iters_per_s_p75": _quantile(ips, 0.75),
        "distance_median": statistics.median(dists),
        "distance_unique": sorted(set(dists)),
        "rows": rows,
    }


def _quantile(xs, q):
    s = sorted(xs)
    if not s:
        return 0.0
    k = (len(s) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=548585631)
    ap.add_argument("--runtime", type=float, default=8.0)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    n_vehicles = 32  # group-54 resolved request has 32 vehicles

    results = {}
    # Scenario A: no breaks
    results["nobreak"] = run_scenario(
        "nobreak",
        break_override={i: [] for i in range(n_vehicles)},
        seed=args.seed,
        runtime_s=args.runtime,
        reps=args.reps,
    )
    # Scenario B: natural breaks
    results["break"] = run_scenario(
        "break",
        break_override=None,
        seed=args.seed,
        runtime_s=args.runtime,
        reps=args.reps,
    )

    print("\n" + "=" * 70)
    for name, res in results.items():
        print(
            f"{name:8s} iters/s median={res['iters_per_s_median']:.1f} "
            f"p25={res['iters_per_s_p25']:.1f} p75={res['iters_per_s_p75']:.1f} "
            f"dist_median={res['distance_median']} dists={res['distance_unique']}"
        )

    if args.out:
        payload = {
            "request_json": REQUEST_JSON,
            "matrix_json": MATRIX_JSON,
            "seed": args.seed,
            "runtime_s": args.runtime,
            "reps": args.reps,
            "results": {k: _strip_rows(v) for k, v in results.items()},
        }
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"\nSaved -> {args.out}")


def _strip_rows(res):
    out = {k: v for k, v in res.items() if k != "rows"}
    out["rows"] = [
        {kk: vv for kk, vv in r.items() if kk != "rep"} for r in res["rows"]
    ]
    return out


if __name__ == "__main__":
    main()
