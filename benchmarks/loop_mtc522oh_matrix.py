"""Extended benchmark matrix: iters/s + objective distance across instances,
scenarios (no-break vs break), and seeds. Used to verify the Route.h
forward-sequence cache does not regress iters/s (ratio >= 0.90) nor quality
(distance bit-identical per seed) vs the pre-change baseline.

Instances:
  g54         510 jobs, 32 veh (production matrix)   -> nobreak / natural (51 breaks)
  g54sub300   300-job subsample of g54               -> nobreak / synthetic (2 breaks)
  g54sub100   100-job subsample of g54               -> nobreak / synthetic (2 breaks)
  g54sub50     50-job subsample of g54               -> nobreak / synthetic (2 breaks)
  g54synth    510 jobs + synthetic 2 breaks          -> nobreak / synthetic (2 breaks)

The synthetic break set uses a CLOCK_TIME lunch window (12:00-13:00) plus a
DUTY_TIME rest, mirroring the hows-router CustomBreak dict format.

Usage:
  python loop_mtc522oh_matrix.py --runtime 5 --seeds "548585631,42,123,999,7777" --out out.json
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

HOWS_ROUTER = r"C:\Users\lupi_\projetos\zanella\CascadeProjects\hows-router"
G54_REQUEST = (
    HOWS_ROUTER
    + r"\optimization_exports\32d73c97-9258-4a39-8d69-e6e47367c048\request.json"
)
G54_MATRIX = HOWS_ROUTER + r"\bench\real_matrix.json"

N_VEHICLES = 32
INT64_MAX = 9223372036854775807

# Two custom breaks with different windows: a CLOCK_TIME lunch (12:00-13:00)
# and a DUTY_TIME rest (12h). Same dict shape pyvrp_translator._to_custom_breaks
# consumes.
SYNTH_BREAKS = [
    {
        "id": 1,
        "tws": [[43200, 46800]],          # 12:00-13:00 local
        "service": 1800,
        "trigger": "CLOCK_TIME",
        "trigger_value": 43200,
        "reset": "NONE",
        "mandatory": False,
        "priority": 5,
        "supersedes": [],
        "condition_min_route_s": 0,
    },
    {
        "id": 2,
        "tws": [[0, INT64_MAX]],
        "service": 3600,
        "trigger": "DUTY_TIME",
        "trigger_value": 43200,
        "reset": "ALL_TIMERS",
        "mandatory": False,
        "priority": 20,
        "supersedes": [1],
        "condition_min_route_s": 0,
    },
]

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


def load_request_matrix(subsample=None):
    from src.models.optimize import OptimizeRequest

    with open(G54_REQUEST, encoding="utf-8") as f:
        req = OptimizeRequest.model_validate(json.load(f))
    with open(G54_MATRIX, encoding="utf-8") as f:
        matrix = json.load(f)

    if subsample is not None:
        n = subsample
        req.jobs = req.jobs[:n]
        matrix = [row[: n + 1] for row in matrix[: n + 1]]

    return req, matrix


def build_model(req, matrix, break_override):
    from src.services.pyvrp_translator import build_pyvrp_model

    def _build():
        return build_pyvrp_model(
            req, matrix, max_penalty=1_000_000, break_override=break_override
        )

    return _silenced(_build)


def solve_cell(model, ctx, seeds, runtime_s):
    from pyvrp.solve import SolveParams
    from pyvrp.stop import MaxRuntime

    params = SolveParams(penalty=ctx["penalty_params"])
    rows = []
    for seed in seeds:
        t0 = time.perf_counter()
        result = model.solve(
            stop=MaxRuntime(runtime_s), seed=seed, display=False, params=params
        )
        wall = time.perf_counter() - t0
        runtimes = list(result.stats.runtimes)
        iters = result.num_iterations
        cpp_time = sum(runtimes)
        rows.append(
            {
                "seed": seed,
                "iters": iters,
                "cpp_time_s": cpp_time,
                "wall_s": wall,
                "iters_per_s": iters / cpp_time if cpp_time > 0 else 0.0,
                "distance": result.best.distance(),
                "cost": result.cost(),
                "feasible": result.is_feasible(),
            }
        )
    return rows


def run_cell(name, break_override, seeds, runtime_s, subsample=None):
    req, matrix = load_request_matrix(subsample=subsample)
    model, ctx = build_model(req, matrix, break_override)
    rows = solve_cell(model, ctx, seeds, runtime_s)
    ips = [r["iters_per_s"] for r in rows]
    print(
        f"[{name}] iters/s median={statistics.median(ips):.1f} "
        f"dists={sorted(set(r['distance'] for r in rows))} "
        f"feasible={all(r['feasible'] for r in rows)}"
    )
    return {"cell": name, "rows": rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runtime", type=float, default=5.0)
    ap.add_argument("--seeds", default="548585631,42,123,999,7777")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]

    cells = [
        # (name, break_override, subsample)
        ("g54_nobreak", {i: [] for i in range(N_VEHICLES)}, None),
        ("g54_natural", None, None),
        ("g54_synth", {i: SYNTH_BREAKS for i in range(N_VEHICLES)}, None),
        ("g54sub300_nobreak", {i: [] for i in range(N_VEHICLES)}, 300),
        ("g54sub300_synth", {i: SYNTH_BREAKS for i in range(N_VEHICLES)}, 300),
        ("g54sub100_nobreak", {i: [] for i in range(N_VEHICLES)}, 100),
        ("g54sub100_synth", {i: SYNTH_BREAKS for i in range(N_VEHICLES)}, 100),
        ("g54sub50_nobreak", {i: [] for i in range(N_VEHICLES)}, 50),
        ("g54sub50_synth", {i: SYNTH_BREAKS for i in range(N_VEHICLES)}, 50),
    ]

    results = {}
    for name, brk, subsample in cells:
        results[name] = run_cell(
            name, brk, seeds, args.runtime, subsample=subsample
        )

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "runtime_s": args.runtime,
                    "seeds": seeds,
                    "results": results,
                },
                f,
                indent=2,
            )
        print(f"\nSaved -> {args.out}")


if __name__ == "__main__":
    main()
