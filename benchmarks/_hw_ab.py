"""Interleaved A/B driver for the break-path perf loop.

The machine is noisy (~+-15% run to run), so a single before/after pair means
nothing. This driver alternates the order of the two binaries across pairs and
reports the paired ratio distribution, which cancels slow drift (thermal
throttling, background load).

Each variant's built extensions live in ``artifacts/pyd/<tag>/``; the driver
copies the selected variant into ``pyvrp/`` and runs the fixed-iteration
benchmark in a fresh subprocess (so the DLL is loaded anew every time).

Usage:
    python benchmarks/_hw_ab.py --tags BASE HW1 --pairs 4 --iters 300
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYD = "_pyvrp.cp312-win_amd64.pyd"
SEARCH_PYD = "_search.cp312-win_amd64.pyd"


def _copy_retry(src: str, dst: str, tries: int = 30) -> None:
    """Copy, retrying while the target is locked.

    A freshly written .pyd is briefly held by the on-access virus scanner (and
    by any interpreter still shutting down), which surfaces as PermissionError.
    """
    for attempt in range(tries):
        try:
            shutil.copyfile(src, dst)
            return
        except PermissionError:
            if attempt == tries - 1:
                raise
            time.sleep(1.0)


def install(tag: str) -> None:
    src = os.path.join(ROOT, "artifacts", "pyd", tag)
    if not os.path.isdir(src):
        raise SystemExit(f"no such variant: {src}")
    _copy_retry(os.path.join(src, PYD), os.path.join(ROOT, "pyvrp", PYD))
    _copy_retry(
        os.path.join(src, "search", SEARCH_PYD),
        os.path.join(ROOT, "pyvrp", "search", SEARCH_PYD),
    )


def measure(tag: str, iters: int, scenario: str, reps: int = 2) -> dict:
    """One measurement of ``tag``: ``reps`` solves in a fresh process, best kept.

    Two reps and the minimum, because the first solve in a process is
    consistently the slowest (cold caches, first-touch page faults) and that
    cold-start penalty is larger than the effects being measured.
    """
    out = os.path.join(tempfile.gettempdir(), f"hwab_{tag}.json")
    cmd = [
        sys.executable,
        os.path.join(ROOT, "benchmarks", "_hw_fixed.py"),
        "--iters", str(iters),
        "--reps", str(reps),
        "--tag", tag,
        "--only", scenario,
        "--out", out,
    ]
    res = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if res.returncode != 0:
        print(res.stdout[-3000:], res.stderr[-3000:], file=sys.stderr)
        raise SystemExit(f"measure failed for {tag}")
    with open(out, encoding="utf-8") as f:
        data = json.load(f)
    rows = data[scenario]["rows"]
    best = min(rows, key=lambda r: r["cpp_s"])
    return {"cpp_s": best["cpp_s"], "wall_s": best["wall_s"],
            "distance": best["distance"], "iters": best["iters"]}


def ratio_mode(tag: str, pairs: int, iters: int, out: str) -> None:
    """Measure the break/nobreak ratio for ONE variant, alternating scenarios.

    The goal metric is break speed relative to nobreak speed on the same
    instance. Alternating the two scenarios within each pair cancels the slow
    thermal drift that made the sequential ``_hw_fixed`` measurement swing by
    a factor of two.
    """
    install(tag)
    rows = []
    for p in range(pairs):
        order = ["nobreak", "break"] if p % 2 == 0 else ["break", "nobreak"]
        got = {s: measure(tag, iters, s) for s in order}
        ratio = got["nobreak"]["cpp_s"] / got["break"]["cpp_s"]
        rows.append({"pair": p, "order": order, "ratio": ratio, **got})
        print(
            f"pair {p} nobreak={got['nobreak']['cpp_s']:.3f}s "
            f"break={got['break']['cpp_s']:.3f}s ratio={ratio:.4f}",
            flush=True,
        )
    ratios = [r["ratio"] for r in rows]
    print("=" * 66)
    print(f"[{tag}] break/nobreak ratio median = {statistics.median(ratios):.4f}")
    print(f"[{tag}] ratios = {[round(r, 4) for r in ratios]}")
    print(f"[{tag}] target 0.90 -> {'MET' if statistics.median(ratios) >= 0.90 else 'not met'}")
    if out:
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"tag": tag, "iters": iters, "rows": rows,
                       "ratio_median": statistics.median(ratios)}, f, indent=1)
        print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tags", nargs=2, metavar=("BASE", "NEW"))
    ap.add_argument("--ratio", metavar="TAG",
                    help="measure break/nobreak ratio for a single variant")
    ap.add_argument("--pairs", type=int, default=4)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--scenario", choices=["break", "nobreak"], default="break")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if args.ratio:
        ratio_mode(args.ratio, args.pairs, args.iters, args.out)
        return
    if not args.tags:
        raise SystemExit("pass either --tags BASE NEW or --ratio TAG")

    base, new = args.tags
    rows = []
    for p in range(args.pairs):
        order = [base, new] if p % 2 == 0 else [new, base]
        got = {}
        for tag in order:
            install(tag)
            got[tag] = measure(tag, args.iters, args.scenario)
            print(
                f"pair {p} {tag:>6s} cpp={got[tag]['cpp_s']:.3f}s "
                f"wall={got[tag]['wall_s']:.3f}s dist={got[tag]['distance']}",
                flush=True,
            )
        ratio = got[base]["cpp_s"] / got[new]["cpp_s"]
        rows.append({"pair": p, "order": order, base: got[base], new: got[new],
                     "speedup": ratio})
        print(f"pair {p} speedup(base/new) = {ratio:.4f}", flush=True)

    speedups = [r["speedup"] for r in rows]
    dists = {t: sorted({r[t]["distance"] for r in rows}) for t in (base, new)}
    print("=" * 66)
    print(f"scenario={args.scenario} iters={args.iters} pairs={args.pairs}")
    print(f"speedup median = {statistics.median(speedups):.4f}")
    print(f"speedups       = {[round(s, 4) for s in speedups]}")
    print(f"wins (>1.00)   = {sum(s > 1.0 for s in speedups)}/{len(speedups)}")
    print(f"distances      = {dists}")
    if dists[base] != dists[new]:
        print("!! DISTANCE MISMATCH — trajectory changed, not a pure perf change")

    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump({"tags": [base, new], "scenario": args.scenario,
                       "iters": args.iters, "rows": rows,
                       "speedup_median": statistics.median(speedups)}, f, indent=1)
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
