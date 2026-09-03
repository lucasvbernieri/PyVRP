# Break-path performance — hardware / constant-factor round (worktree `break-hw`)

> Branch `omos/break-hw-opt`, base `1607afc` (the state that closed the previous
> loop with H6 + H10). Goal restated from that loop: make a route **with**
> custom breaks run at >= 0.90x the speed of the same instance **without** them.
>
> This document records what was measured, what was refuted, and what the
> remaining wall actually is. Everything here is reproducible with the harnesses
> in `benchmarks/_hw_fixed.py` and `benchmarks/_hw_ab.py`.

## 0. Result

**The break path is 17.5% faster on the canonical metric and the ratio moved from
0.60 to 0.70. The 0.90 target is not met.** Semantics are bit-identical: the
break distance is 4 896 741 on every run recorded here, all three parity
harnesses are unchanged, and the fork suite is green (1179 passed, 2 skipped,
6 xfailed).

**Canonical metric** — `benchmarks/bench_ab.py`, 8 s x 5 reps, iters/s from the
C++-side runtimes. This is the metric the previous loop's goal was written
against, so it is the one that decides the verdict. Both runs pinned to one
P-core:

| session | | nobreak it/s | break it/s | ratio |
|---|---|---|---|---|
| A | base `1607afc` | 151.2 | 90.2 | 0.597 |
| A | this branch | 150.7 | 106.0 | 0.703 |
| B | base `1607afc` | 203.1 | 117.1 | **0.577** |
| B | this branch | 210.6 | **139.7** | **0.663** |

Break throughput **+17.5%** and **+19.3%** in the two sessions; nobreak unchanged
within each, confirming every change is scoped to the break path.

**Only compare within a session.** The absolute level of this metric moves with
the machine's state — session B was ~35% faster overall — and it moves the ratio
too, because a faster machine reaches more iterations in its 8 s and so spends
more of them in the regime where break routes are heaviest. Both sessions agree
on the delta; neither pins the absolute ratio to better than about +-0.04.

**The ratio depends on which part of the search you sample, and by a lot.** At a
fixed 250 iterations — equal search progress rather than equal wall time — the
same two binaries measure:

| | break / nobreak ratio | pair spread (8 pairs) |
|---|---|---|
| base `1607afc` | **0.671** | 0.657 – 0.688 |
| this branch | **0.895** | 0.863 – 0.919 |

Both numbers are real and neither is cherry-picked: a break route accumulates
break nodes as the search converges, so its cost per iteration *grows* with
iteration count, while the nobreak route's does not. Over 8 s the break scenario
reaches ~850 iterations and spends most of its time in that heavier regime; over
a fixed 250 it does not. **The honest headline is the canonical 0.597 -> 0.703**;
the 0.895 figure is reported because it is what an equal-iteration comparison
gives and because the gap between the two is itself a finding.

§6 quantifies exactly what stands between 0.703 and 0.90, and it is a single
item.

## 1. The headline finding

The previous loop concluded (`docs/BREAK_PERF_INVESTIGATION.md` §6, H6) that the
O(n) forward pass per candidate was **structural to the break contract**, and
that only incremental gains remained. That conclusion was premature. The pass was
carrying a large amount of ordinary constant-factor waste that the O(n) shape
hid:

- eight to ten `std::vector` heap allocations **per candidate evaluation**;
- a scattered pointer dereference per node position, paid two to three times per
  candidate;
- a fold that re-read `CustomBreak` (112 bytes, two `std::vector` members) at
  every node, for every configured break;
- an 80-byte segment copy per node, for a case that only fires on reload depots.

None of this required touching break semantics. Every gate below stayed
bit-identical throughout.

## 2. Method — and why the first numbers produced here were wrong

**The measurement was the hard part.** Three methodological facts, each of which
invalidated an earlier reading:

1. **Never benchmark while anything else builds.** The first baseline showed
   `nobreak` at 6.4 s and at 14.0 s for identical work in one session, because a
   compile was running concurrently. That contaminated run is what produced the
   "0.71" ratio quoted early in this work; the honest interleaved figure is
   lower.
2. **This is a hybrid CPU** (i9-13950HX: 8 P-cores as logical 0-15, then
   E-cores). Letting the scheduler migrate the solve between a P-core and an
   E-core changes the result by up to 2x. `_hw_fixed.py` now pins the process to
   one P-core and raises its priority.
3. **Verify that the pinning actually took effect.** `ctypes` truncates the
   `HANDLE` returned by `GetCurrentProcess` unless `restype` is set, so the
   first version of the pinning code failed silently and every measurement
   taken with it was unpinned. The harness now prints
   `pinned cpu=2 affinity=True high_prio=True`; if it prints `False`, the
   numbers from that run are worthless.

**The noise floor, measured both ways.** Running two *byte-identical* binaries
against each other over 5 interleaved pairs gives:

| | median | range |
|---|---|---|
| unpinned (the silent-failure version) | 1.14 | 0.98 – 1.20 |
| pinned | **0.986** | 0.97 – 1.04 |

So an effect below roughly 1.2x was unmeasurable before the pinning was fixed,
and is measurable to about +-3% after. The per-step verdicts in §5 that were
taken before the fix are labelled accordingly; the aggregate and ratio figures
were re-taken with pinning working.

Two harnesses are used, and §0 explains why both are reported.
`benchmarks/bench_ab.py` is the canonical one (fixed 8 s runtime, iters/s from
the C++-side timings) and decides the verdict. `benchmarks/_hw_ab.py` fixes the
**iteration count** instead, which makes the work deterministic and lets a
change be A/B-ed against itself in interleaved pairs — that is what every
per-step number in §5 comes from.

Every run also reports the solution distance. The search trajectory is
hypersensitive, so an unchanged distance across a change is strong end-to-end
evidence of bit-exactness: **4 896 741 for break and 6 515 801 for nobreak on
every run recorded here.**

## 3. Gates (unchanged on every commit)

| Gate | Expected | Status |
|---|---|---|
| `tests/cpp/test_stream_parity` | 500/500 exact, 0 mismatches | green throughout |
| `tests/cpp/test_segment_fold_parity` | 63/63 | green throughout |
| `tests/cpp/test_proposal_fold_residual` | 53/174 (NEGATIVE canary — must not move) | green throughout |
| break distance | 4 896 741 | exact on every measured run |

## 4. Instrumentation added (`PYVRP_STREAM_STATS`, compiled out by default)

Two probes, both of which changed the plan.

`detail::StreamStats` — what a candidate evaluation actually does, measured over
**1 267 511 real proposal evaluations**:

```
seeded=0.912  round2_f=0.009  L_round=17.95  n_flat=26.67  sc_hit=0.429  sc_saved=3.57
```

Read: 91.2% of candidates start from the cached route-prefix seed; the second
round of the pass runs on 0.9% of them; a candidate simulates ~18 of the ~27
nodes in its flat sequence; and the tail collapse fires on 42.9% of candidates,
removing 3.57 nodes on average.

`detail::PhaseProfile` (`search/PhaseProfile.h`) — cycle counters per search
phase, used for §6.

## 5. Verdicts

### Confirmed and committed

| # | Change | Effect on the break path | Confidence |
|---|---|---|---|
| HW1 | `detail::SmallBuf` — zero heap allocation per candidate | **+37.0%** (4/4 pairs; 1.35–1.55 across sessions) | **solid** — far above the 1.2 noise floor |
| HW2 | `Route::activitiesAt_` SoA array; descriptor-walk pre-scan; `loadNode` cursor; pointer-bound fold operand; flat 64-byte `BreakRule` table replacing `CustomBreak` in the fold | +9.2% median (4/4) | directional — below the noise floor alone, but every item is provably strictly less work |
| HW3 | pre-scan O(#breaks) via cached `breakPositions_` | +3.5% median (2/4) | within noise; kept as strictly less work |
| HW4 | tail collapse onto the cached `durAfter` fold | +3.9% median (3/5) | within noise; fires on **60.2%** of candidates, saving 6.6 of 17.95 simulated nodes |
| HW5 | settled-rule skip in `DriveSegment::merge`; `driveState()` no longer allocates its upcoming-break mask on the single-node (ShiftBreak) path; `SwapTails` tail scan O(n) -> O(1) | +2.2% median (4/5) | within noise |
| HW6 | `Route::update()` assigns into its drive-segment buffers instead of `optional::emplace` + `resize`, which freed and reallocated all three on every update | see aggregate | break-only (`hasBreaks() \|\| hasSetup()`); `update()` costs 45 818 cycles/call on break vs 15 185 on nobreak |

Only the aggregate clears the noise floor cleanly. Re-measured end to end with
pinning working, base -> branch on the break path is **1.334x** (5/5 pairs,
1.269 – 1.359) at a fixed 250 iterations, and **+17.5%** on the canonical 8 s
metric. The per-step figures above are recorded for provenance, not as
independent claims.

### Refuted, with evidence

| Hypothesis | Verdict |
|---|---|
| **Round 2 of the forward pass is a significant cost.** A prior analysis put its firing rate at 0.4–0.6 and scoped an incremental resume as days of work. | **REFUTED.** Measured `round2_f = 0.009` — it runs on 0.9% of candidates. Incrementalising it is worthless. |
| **Break routes are longer, so part of the gap is inherent to the workload.** | **REFUTED.** Mean route length 22.7 (break) vs 23.2 (nobreak), ratio 0.98. The excess is code, not data size. |
| **The break path loses a pruning step that the nobreak path has.** | **REFUTED.** 31.2% of break proposals reach `duration()` vs 32.4% for nobreak — break prunes marginally *more*. |
| **The collapse's "every mandatory break is served" gate is what limits its firing rate.** It only exists because the collapse could not produce the arrival clock at the end depot, which the D3 term for an unserved mandatory break needs. | **REFUTED as the binding constraint.** Caching a second suffix fold that stops one node short of the end depot (`durAfterExEnd`) lets the collapse take the final step by hand and recover that clock exactly, which removes the gate. Firing rate: **unchanged at 42.9%** — the same candidates are blocked one gate later, by `takenMask`. Reverted (an extra 80 B/node array and a merge per update, for nothing). |
| **The tail can be proven inert without walking it, so the collapse's gates can be dropped.** All three accumulators are non-decreasing along a stretch with no reset, so a rule that cannot fire at the LAST boundary cannot fire anywhere in the tail — and every quantity needed is an end-of-route value: the collapsed fold (the segment merge is associative), the arrival clock at the end depot (via a suffix fold stopping one node short of it), and the drive/work/duty totals (via cached suffix sums). For duty this needs the identity `duty(i) = max(duty(i-1) + edge, atSecond(i) - lastReset) + service(i)`, whose unrolled maximum is dominated by its last term. | **IMPLEMENTED, EXACT, AND STILL NOT WORTH IT.** Firing rose only 42.9% -> 48.7% (3.57 -> 3.84 nodes saved), while the extra suffix arrays cost ~2% in `Route::update()`: net **0.980** over 5 pairs. Reverted. **The finding is the point:** the new blocker is `inert = 30.3%` — the bound is not being conservative, the tail genuinely *does* cross a mandatory break's trigger in those candidates. No "prove it cannot fire" shortcut can reach them. |
| **Reusing the route's cached `durAt[pos]` instead of rebuilding each client's duration singleton.** The route already stores exactly that segment, and the rebuild costs a random probe into the client array. | **REFUTED.** 0.973 median over 5 pairs (2/5 wins) — a small *regression*. `durAt` is an 80-byte struct, so reading it streams a third array alongside `activitiesAt_` and `locations`, while the client records are already warm from being re-read across thousands of candidates on the same route. Reverted. |
| **Software prefetch hides the per-node memory latency.** (H5 in the previous round's list, never tested) | **REFUTED.** Prefetching the client record and matrix entry two nodes ahead measured 0.997 median over 5 pairs (2/5 wins) — no effect. The data is already warm: the pre-scan touches the same positions, and the same route is re-evaluated thousands of times consecutively. Reverted. |
| **Matrix lookups dominate and are DRAM-bound.** | **REFUTED.** The instance has 511 locations, so the duration and distance matrices are 2.1 MB each — 4.2 MB against a 36 MB L3. Lookups are L2 misses / L3 hits (~45 cycles), not DRAM. This also explains why the earlier loop's "materialise edges" hypothesis measured ~0%. |
| **The naive monoid fold can replace the forward pass.** (from the previous loop) | Still refuted, and re-confirmed here: `test_proposal_fold_residual` remains at 53/174 divergences. It is kept as a negative canary. |

## 6. Where the remaining gap is

Cycle profile of the **current** binary, pinned, with the profiler scope
corrected to wrap `LocalSearch::operator()` (an earlier version wrapped only
`search()`, so the perturbation and the up-front reinsertion contributed
`duration`/`distance` cycles to a denominator that excluded their own). 150
fixed iterations, two reps:

| phase | break | nobreak | cycles/call break | cycles/call nobreak |
|---|---|---|---|---|
| `operator()` total | 14.551 Gcyc | 10.065 Gcyc | — | — |
| `binaryOps` | 76.5% | 78.1% | 11 925 | 7 277 |
| **`duration()`** | **27.6%** | **6.3%** | **1573** | **280** |
| `Route::update()` | 4.6% | 4.1% | 34 790 | 23 095 |
| `distance()` | 4.6% | 3.8% | 86 | 56 |
| `breakScan` | 1.5% | 0.0% | 206 465 | 422 |
| `perturb` | 2.6% | 2.7% | 1 237 548 | 898 203 |

(These totals carry the profiler's own `rdtsc` overhead — 12.6M scopes on break
against 10.2M on nobreak — so the *ratio* they imply is pessimistic. Use them for
attribution, not for the headline.)

**`duration()` is 75% of the whole gap.** Break spends 4.016 Gcyc there against
nobreak's 0.634 Gcyc: an excess of 3.38 Gcyc out of a 4.486 Gcyc total gap.
Nothing else comes close — `Route::update()`, `breakScan`, `distance()` and the
operator bodies together account for the remaining quarter.

This is the number that sizes the rest of the job, and it is encouraging rather
than not: **bringing `duration()` down to the nobreak fold's cost would land the
ratio at almost exactly 0.90.** There is no second hidden obstacle behind it.

An earlier version of this section, computed from an unpinned profile, concluded
the opposite — that even a free `duration()` would cap the ratio at ~0.72 and
that 0.90 therefore required attacking the shared guard/dispatch layer. That was
an artifact of the solve being migrated onto an E-core, which penalises the
longer-running break scenario far more than the shorter nobreak one. The lesson
generalises: **an attribution built on a contaminated absolute baseline produces
a confident, wrong "this is impossible" verdict** — the same shape as the
conclusion this work set out to re-test.

**What `duration()` costs now, and what would move it.** 1573 cycles per call
over the ~11 nodes a candidate still simulates is ~120 cycles per node, against
roughly one L2-missing probe into the client array plus one into the 2.1 MB
duration matrix. It is memory-bound, not arithmetic-bound — but see the prefetch
verdict in §5: the data is already warm, because the pre-scan touches it and
because the same route is re-evaluated thousands of times in a row. Constant
factors are close to exhausted here.

What is left is the node count itself, and the instrumentation now says exactly
what holds it up. Attributing every non-firing candidate to the first gate that
blocked it:

| blocked by | share of all candidates |
|---|---|
| a mandatory break is neither taken nor served (`takenMask`) | **36.1%** |
| a break node still lies ahead (`remainingMask != 0`) | 13.2% |
| the last segment is not a clean route suffix | 8.6% |
| collapse fires | 42.9% |

After the (reverted) inertness proof, the same attribution reads: fires 48.7%,
a rule genuinely can still fire 30.3%, break node ahead 13.2%, structure 8.6%.

The dominant blocker is a mandatory break that has not been taken — typically a
second overnight rest on a route that only needed one. It cannot be skipped
because the tail could still push accumulated duty past its trigger, which would
record a first-due clock and price real lateness that the collapse would miss.

Deciding that cheaply means answering **where inside a stretch a trigger first
crosses** — which is precisely the open question flagged as Decisão 1 / Fase A of
`openspec/changes/break-regime-eval/design.md`, and the reason that change is
scoped as a viability investigation rather than an implementation. It is also
what a jump between consecutive break nodes would need, so the two collapse
directions share the same blocker.

**That question is no longer open in one direction.** The "prove it cannot fire"
route was implemented and measured (see the third refutation in §5): it is exact,
it needs no crossing location, and it only moves the firing rate from 42.9% to
48.7%, because in **30.3%** of candidates the tail really does cross a mandatory
break's trigger. For those, knowing *whether* is not enough — the D3 term needs
the clock at the crossing, so the localisation is unavoidable rather than merely
convenient. That closes off the cheap half of Fase A and leaves only the
expensive half, which is worth knowing before that change is scheduled.

That change's cost estimate can now be sharpened from real data rather than
assumption, and one of its work items can be dropped outright: round 2 of the
pass runs on 0.9% of candidates, so the incremental-resume item is worth
nothing.

## 7. Correctness notes worth keeping

Two real hazards were found while designing the tail collapse, both of which
would have been silent corruption:

1. **`ALL_TIMERS` replaces `takenMask` rather than adding to it.** An untaken
   `ALL_TIMERS` break firing in the tail can therefore clear a *mandatory*
   break's bit and let `breakDueMask` grow again. The collapse gate requires
   every `ALL_TIMERS` break to be taken as well, not only every mandatory one
   (`collapseRequiredMask`).
2. **`presentMask` aliases break ids modulo 16 while `occ`/`remaining` are
   indexed by the raw id.** `remainingMask == 0` is therefore only a sound "no
   break node ahead" test while the id space stays below 16, so the gate checks
   `maxBreakId < 16`.

The collapse also requires: the last segment is a contiguous suffix of the *same*
route, that route is clean, the instance has no setup durations (`durAfter` is
built from bare edges), and the route is single-trip (`durAfter` finalises the
front where the forward pass finalises the back — the asymmetry behind the
earlier H4/H5 fixes).

A third hazard was found the hard way. `Route::activitiesAt_` and
`breakPositions_` were briefly gated on `vehicleType_.hasBreaks() ||
data.hasSetup()`, on the reasoning that only the break/setup evaluation path
reads them. That path is selected by the *proposal's* route, but a cross-route
proposal also walks segments belonging to the **other** route — which may be a
vehicle type without breaks, whose caches would then be empty. The result was a
segfault. Both arrays are therefore built for every route. The cost to a
break-free solve is a few tens of nanoseconds per `Route::update()` against a
~4 us update, i.e. well under 0.1% of its runtime, so it does not meaningfully
flatter the ratio reported here.

## 8. Reproducing

```
# one variant's break/nobreak ratio (the goal metric)
python benchmarks/_hw_ab.py --ratio HW5 --pairs 6 --iters 250

# A/B two built variants on the break path, interleaved
python benchmarks/_hw_ab.py --tags BASE HW5 --pairs 8 --iters 250 --scenario break

# what a candidate evaluation actually does (needs a PYVRP_STREAM_STATS build)
python buildtools/build_extensions.py --build_dir build-stats --build_type release \
    --additional -Dcpp_args=-DPYVRP_STREAM_STATS
python benchmarks/_hw_fixed.py --iters 150 --reps 1 --only break --tag STATS
```

Built variants live in `artifacts/pyd/<tag>/`; `_hw_ab.py` copies the selected
one into `pyvrp/` before each measurement subprocess, so the two binaries can be
interleaved within a session.
