"""Why does the break search apply ~2.6x more moves per iteration?

At 1500 iterations the break path calls ``Route::update()`` 119 times per
``LocalSearch::operator()`` against the nobreak path's 47, and that gap is
roughly half of the whole break/nobreak cost difference. Per-call cost is not
the cause (15 050 cycles on break against 15 969 on nobreak), so it is purely
call count, i.e. accepted moves.

Two candidate explanations:

  H1  phantom moves -- the streaming evaluator's delta disagrees with the cost
      recomputed after the move is applied, so moves are accepted on stale
      deltas and the search churns. ``LocalSearch`` counts these directly, and
      the move-cap safety valve bounds the resulting oscillation.
  H2  legitimate chains of tiny improvements -- with break service, waiting and
      duty measured in seconds, the converged landscape is fine-grained, so
      there really are many strictly-improving moves left to take.

``LocalSearch``'s statistics describe a single invocation (they reset per
call), so this subclasses it to accumulate across the whole solve. Subclassing
rather than patching the instance is deliberate: Python looks ``__call__`` up on
the type, so an instance attribute would never be used.

    python benchmarks/_hw_moves.py --iters 250 1500
"""

from __future__ import annotations

import argparse
import importlib
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from _hw_fixed import load_model, pin_cpu  # noqa: E402

_CAPTURED: list = []


def _install_probe():
    """Make solve() build an accumulating LocalSearch instead."""
    # ``import pyvrp.solve`` binds the *function* the package re-exports, not
    # the module, so reach the module object explicitly.
    solve_mod = importlib.import_module("pyvrp.solve")
    real = solve_mod.LocalSearch

    class Probe(real):  # type: ignore[misc, valid-type]
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.acc = {"calls": 0, "moves": 0, "improving": 0,
                        "updates": 0, "parity": 0, "valve": 0}
            _CAPTURED.append(self)

        def __call__(self, *args, **kwargs):
            out = super().__call__(*args, **kwargs)
            st = self.statistics
            a = self.acc
            a["calls"] += 1
            a["moves"] += st.num_moves
            a["improving"] += st.num_improving
            a["updates"] += st.num_updates
            a["parity"] += self.parity_violations()
            a["valve"] += 1 if self.valve_triggered() else 0
            return out

    solve_mod.LocalSearch = Probe


def run(scenario: str, iters: int, seed: int) -> dict:
    from pyvrp.solve import SolveParams
    from pyvrp.stop import MaxIterations

    _CAPTURED.clear()
    override = {i: [] for i in range(32)} if scenario == "nobreak" else None
    model, ctx = load_model(break_override=override)
    params = SolveParams(penalty=ctx["penalty_params"])
    res = model.solve(
        stop=MaxIterations(iters), seed=seed, display=False, params=params
    )

    ls = _CAPTURED[-1]
    return {
        "scenario": scenario,
        "iters": res.num_iterations,
        "distance": int(res.best.distance()),
        **ls.acc,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, nargs="+", default=[250, 1500])
    ap.add_argument("--seed", type=int, default=548585631)
    args = ap.parse_args()

    print(pin_cpu(), flush=True)
    _install_probe()

    for iters in args.iters:
        for scenario in ("nobreak", "break"):
            r = run(scenario, iters, args.seed)
            c = max(r["calls"], 1)
            print(
                f"\n=== {scenario} @ {r['iters']} iters  dist={r['distance']}\n"
                f"    LocalSearch invocations = {r['calls']}\n"
                f"    parity_violations = {r['parity']}   "
                f"valve triggered in {r['valve']} invocation(s)\n"
                f"    per invocation: moves={r['moves'] / c:8.1f}  "
                f"improving={r['improving'] / c:6.2f}  "
                f"updates={r['updates'] / c:6.2f}",
                flush=True,
            )


if __name__ == "__main__":
    main()
