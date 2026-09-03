"""Fixed-iteration A/B harness for the break-path hardware/perf loop.

Runs the real group-54 instance for a FIXED number of iterations (so the work
is deterministic) and reports wall time + C++-side runtime for the break and
nobreak scenarios.  Because the iteration count is pinned, wall time is a
direct measure of cost per iteration and the break/nobreak ratio is
    ratio = nobreak_time / break_time
(the same quantity as ``break_iters_per_s / nobreak_iters_per_s``).

Usage:
    python benchmarks/_hw_fixed.py --iters 800 --reps 3 --tag base
"""

from __future__ import annotations

import json
import os
import statistics
import sys
import time

_MINGW = r"C:\Users\lupi_\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
if os.path.isdir(_MINGW):
    os.add_dll_directory(_MINGW)

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _HERE)


def pin_cpu(logical_cpu: int = 2) -> str:
    """Pin this process to one performance core and raise its priority.

    This is a hybrid CPU (Raptor Lake: 8 P-cores as logical 0-15, then E-cores).
    Letting the scheduler migrate the solve between a P-core and an E-core made
    repeated runs of the *same* binary differ by up to 2x, which is far larger
    than any optimisation being measured. Pinning removes that.
    """
    if not sys.platform.startswith("win"):
        return "not pinned (non-Windows)"
    import ctypes

    kernel32 = ctypes.windll.kernel32
    # Without an explicit restype ctypes truncates the returned HANDLE to a
    # 32-bit int and both calls below fail silently.
    kernel32.GetCurrentProcess.restype = ctypes.c_void_p
    kernel32.SetProcessAffinityMask.argtypes = [ctypes.c_void_p,
                                                ctypes.c_size_t]
    kernel32.SetPriorityClass.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    handle = kernel32.GetCurrentProcess()
    ok_affinity = kernel32.SetProcessAffinityMask(handle, 1 << logical_cpu)
    ok_priority = kernel32.SetPriorityClass(handle, 0x00000080)  # HIGH
    return f"pinned cpu={logical_cpu} affinity={bool(ok_affinity)} high_prio={bool(ok_priority)}"

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

    return _silenced(
        lambda: build_pyvrp_model(
            req, matrix, max_penalty=1_000_000, break_override=break_override
        )
    )


def solve_fixed(model, ctx, seed, iters):
    from pyvrp.solve import SolveParams
    from pyvrp.stop import MaxIterations

    params = SolveParams(penalty=ctx["penalty_params"])
    t0 = time.perf_counter()
    result = model.solve(
        stop=MaxIterations(iters), seed=seed, display=False, params=params
    )
    wall = time.perf_counter() - t0
    return {
        "iters": result.num_iterations,
        "wall_s": wall,
        "cpp_s": sum(result.stats.runtimes),
        "distance": int(result.best.distance()),
        "cost": int(result.cost()),
        "feasible": bool(result.is_feasible()),
    }


def run(name, break_override, seed, iters, reps):
    rows = []
    for r in range(reps):
        model, ctx = load_model(break_override=break_override)
        m = solve_fixed(model, ctx, seed, iters)
        m["rep"] = r
        rows.append(m)
        print(
            f"[{name}] rep={r} iters={m['iters']} wall={m['wall_s']:.3f}s "
            f"cpp={m['cpp_s']:.3f}s dist={m['distance']} feas={m['feasible']}",
            flush=True,
        )
    return {
        "scenario": name,
        "iters": iters,
        "reps": reps,
        "wall_median": statistics.median(m["wall_s"] for m in rows),
        "wall_min": min(m["wall_s"] for m in rows),
        "cpp_median": statistics.median(m["cpp_s"] for m in rows),
        "cpp_min": min(m["cpp_s"] for m in rows),
        "distances": sorted({m["distance"] for m in rows}),
        "rows": rows,
    }


def main():
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=548585631)
    ap.add_argument("--iters", type=int, default=800)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--tag", default="run")
    ap.add_argument("--only", choices=["break", "nobreak", "both"], default="both")
    ap.add_argument("--out", default="")
    ap.add_argument("--cpu", type=int, default=2,
                    help="logical CPU to pin to (must be a P-core)")
    args = ap.parse_args()

    print(pin_cpu(args.cpu), flush=True)

    n_vehicles = 32
    res = {"tag": args.tag, "seed": args.seed, "iters": args.iters}
    if args.only in ("nobreak", "both"):
        res["nobreak"] = run(
            "nobreak", {i: [] for i in range(n_vehicles)}, args.seed, args.iters, args.reps
        )
    if args.only in ("break", "both"):
        res["break"] = run("break", None, args.seed, args.iters, args.reps)

    if "break" in res and "nobreak" in res:
        for key in ("wall_median", "wall_min", "cpp_median", "cpp_min"):
            res[f"ratio_{key}"] = res["nobreak"][key] / res["break"][key]
        print("=" * 66)
        print(
            f"[{args.tag}] ratio(cpp_min)   = {res['ratio_cpp_min']:.4f}\n"
            f"[{args.tag}] ratio(cpp_median)= {res['ratio_cpp_median']:.4f}\n"
            f"[{args.tag}] ratio(wall_min)  = {res['ratio_wall_min']:.4f}\n"
            f"[{args.tag}] break cpp_min={res['break']['cpp_min']:.3f}s "
            f"nobreak cpp_min={res['nobreak']['cpp_min']:.3f}s"
        )
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
