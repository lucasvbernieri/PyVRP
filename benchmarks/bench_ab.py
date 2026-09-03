"""Loop benchmark A/B: iters/s group-54 instance, with/without breaks.

Same code runs against the OLD binary (host .venv, pyvrp .pyd 8/23 pre-cache)
and the NEW binary (docker container, pyvrp built from fork 96e16aa).
Paths are env-overridable; metric iters/s uses C++-side stats.runtimes so the
comparison is binary-fair across python versions.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import statistics
import sys
import time

# Pin to one performance core: this is a hybrid CPU, and letting the solve
# migrate to an E-core changes iters/s by up to 2x, swamping any real effect.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _hw_fixed import pin_cpu  # noqa: E402

DEFAULT_HOWS_ROUTER = r"C:\Users\lupi_\projetos\zanella\CascadeProjects\hows-router"
HOWS_ROUTER = os.environ.get("HOWS_ROUTER", DEFAULT_HOWS_ROUTER)
REQUEST_JSON = os.environ.get(
    "REQUEST_JSON",
    HOWS_ROUTER + r"\optimization_exports\32d73c97-9258-4a39-8d69-e6e47367c048\request.json",
)
MATRIX_JSON = os.environ.get("MATRIX_JSON", HOWS_ROUTER + r"\bench\real_matrix.json")

sys.path.insert(0, HOWS_ROUTER)


def _silenced(fn):
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
    best = result.best
    return {
        "iters": iters,
        "cpp_time_s": cpp_time,
        "wall_s": wall,
        "iters_per_s": iters_per_s,
        "distance": best.distance(),
        "cost": result.cost(),
        "feasible": result.is_feasible(),
    }


def run_scenario(name, break_override, seed, runtime_s, reps):
    model, ctx = load_model(break_override=break_override)
    rows = []
    for r in range(reps):
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
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=548585631)
    ap.add_argument("--runtime", type=float, default=8.0)
    ap.add_argument("--reps", type=int, default=5)
    args = ap.parse_args()

    print(pin_cpu(), flush=True)

    n_vehicles = 32
    results = {}
    results["nobreak"] = run_scenario(
        "nobreak", {i: [] for i in range(n_vehicles)}, args.seed, args.runtime, args.reps
    )
    results["break"] = run_scenario(
        "break", None, args.seed, args.runtime, args.reps
    )

    print("\n" + "=" * 70)
    for name, res in results.items():
        print(
            f"{name:8s} iters/s median={res['iters_per_s_median']:.1f} "
            f"p25={res['iters_per_s_p25']:.1f} p75={res['iters_per_s_p75']:.1f} "
            f"dist_median={res['distance_median']} dists={res['distance_unique']}"
        )
    print(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
