"""
A/B benchmark for the native ``setup`` primitive (task 1.11 / design D15).

Compares the PyVRP fork with ``setup_durations`` populated against a baseline
that uses the *same build* with ``setup_durations`` all-zero, on the same seed.

Two bars are produced:

(a) **no-regression**: on instances without co-located (shared-location)
    clients, solutions must be *strictly identical* to the all-zero-setup
    baseline (same seed), with no measurable runtime regression.
(b) **quality**: on multi-compartment instances with co-located pairs, measure
    the number of stops, total service time and cost vs. the baseline.

Methodology follows the VROOM ``benchmarks/`` practice: N runs per
configuration, discarding the fastest and slowest extremes, reporting the
median of the remainder.

Usage (full run, ~1000 jobs, 5 runs):
    python benchmarks/stop_setup_ab.py --jobs 1000 --runs 5

Smoke run (small, for CI/sanity):
    python benchmarks/stop_setup_ab.py --smoke

Artifacts (JSON) are written to ``benchmarks/results/stop_setup/``.
"""
import argparse
import json
import os
import random
import statistics
import time

import numpy as np

from pyvrp import Model, solve
from pyvrp.stop import MaxRuntime

HORIZON = 8 * 3600  # 8h planning horizon
FIXED_S = 900       # fixed setup/service seconds per stop
SERVICE_PER_M3 = 250


# ---------------------------------------------------------------------------
# Instance generator
# ---------------------------------------------------------------------------

def generate_instance(num_jobs: int, seed: int, *, co_located: bool):
    """
    Generates a random instance with ``num_jobs`` clients.

    When ``co_located`` is True, ~30% of clients are paired with a sibling at
    the same (x, y) location — modelling multi-compartment deliveries to a
    single stop that the solver may merge (paying setup once).
    """
    rng = random.Random(seed)
    model = Model()

    depot = model.add_location(x=0.0, y=0.0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=HORIZON)

    locations = [depot]
    clients = []

    while len(clients) < num_jobs:
        x = round(rng.uniform(-100, 100), 2)
        y = round(rng.uniform(-100, 100), 2)
        loc = model.add_location(x=x, y=y)

        # First client at this location.
        model.add_client(
            loc,
            delivery=[rng.randint(1, 5)],
            service_duration=300,
            tw_early=rng.randint(0, 4 * 3600),
            tw_late=HORIZON,
        )
        clients.append(loc)

        # Possibly a co-located sibling (multi-compartment pair).
        if co_located and len(clients) < num_jobs and rng.random() < 0.3:
            model.add_client(
                loc,
                delivery=[rng.randint(1, 5)],
                service_duration=300,
                tw_early=rng.randint(0, 4 * 3600),
                tw_late=HORIZON,
            )
            clients.append(loc)

    # Euclidean travel between all locations (truncated to ints).
    for frm in model.locations:
        for to in model.locations:
            if frm is to:
                continue
            dist = round(((frm.x - to.x) ** 2 + (frm.y - to.y) ** 2) ** 0.5)
            model.add_edge(frm, to, distance=dist, duration=dist)

    model.add_vehicle_type(
        num_available=max(1, num_jobs // 10),
        capacity=[max(10, num_jobs)],
        unit_duration_cost=1,
    )

    return model.data()


def with_setup(data, fixed_s):
    """Returns a copy of ``data`` with setup durations on client locations."""
    locs = data.locations()
    clients = data.clients()

    setup = [0] * len(locs)
    if fixed_s > 0:
        client_locs = {c.location for c in clients}
        for loc in client_locs:
            setup[loc] = fixed_s

    return data.replace(setup_durations=setup)


# ---------------------------------------------------------------------------
# Solve + metrics
# ---------------------------------------------------------------------------

def solve_once(data, seed, max_runtime):
    start = time.perf_counter()
    result = solve(data, stop=MaxRuntime(max_runtime), seed=seed)
    runtime = time.perf_counter() - start

    best = result.best
    return {
        "cost": result.cost(),
        "num_routes": best.num_routes(),
        "distance": best.distance(),
        "duration": best.duration(),
        "service": sum(r.service_duration() for r in best.routes()),
        "setup": sum(r.setup_duration() for r in best.routes()),
        "runtime": runtime,
        "routes": [
            [sa.idx for sa in r if sa.is_client()] for r in best.routes()
        ],
    }


def run_config(data, seed, runs, max_runtime):
    """Runs ``runs`` times, discards extremes, returns median metrics."""
    samples = [solve_once(data, seed + i, max_runtime) for i in range(runs)]

    if runs >= 5:
        # Discard fastest and slowest extremes by runtime.
        samples = sorted(samples, key=lambda s: s["runtime"])[1:-1]

    keys = [
        "cost", "num_routes", "distance", "duration", "service", "setup",
        "runtime",
    ]
    medians = {k: statistics.median(s[k] for s in samples) for k in keys}
    medians["routes"] = samples[len(samples) // 2]["routes"]
    return medians


# ---------------------------------------------------------------------------
# Bars
# ---------------------------------------------------------------------------

def bar_no_regression(seed, runs, max_runtime):
    """(a) no-setup instance: solutions must be identical to the baseline."""
    data = generate_instance(80, seed, co_located=False)

    baseline = run_config(data, seed, runs, max_runtime)
    zero_setup = run_config(with_setup(data, 0), seed, runs, max_runtime)

    identical = sorted(baseline["routes"]) == sorted(zero_setup["routes"])
    return {
        "bar": "no-regression",
        "identical_solutions": identical,
        "baseline": baseline,
        "zero_setup": zero_setup,
    }


def bar_quality(seed, runs, max_runtime):
    """(b) multi-compartment instance: quality vs. baseline."""
    data = generate_instance(80, seed, co_located=True)

    baseline = run_config(with_setup(data, 0), seed, runs, max_runtime)
    with_setup_res = run_config(with_setup(data, FIXED_S), seed, runs, max_runtime)

    return {
        "bar": "quality",
        "baseline": baseline,
        "with_setup": with_setup_res,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--jobs", type=int, default=1000)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--max-runtime", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument(
        "--out-dir",
        default=os.path.join(os.path.dirname(__file__), "results", "stop_setup"),
    )
    args = parser.parse_args()

    if args.smoke:
        args.jobs = 60
        args.runs = 2
        args.max_runtime = 2.0

    os.makedirs(args.out_dir, exist_ok=True)

    results = {
        "config": vars(args),
        "no_regression": bar_no_regression(
            args.seed, args.runs, args.max_runtime
        ),
        "quality": bar_quality(args.seed, args.runs, args.max_runtime),
    }

    out_path = os.path.join(args.out_dir, "stop_setup_ab.json")
    with open(out_path, "w") as fh:
        json.dump(results, fh, indent=2)

    print(json.dumps(results, indent=2))
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
