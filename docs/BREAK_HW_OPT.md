# Break-path performance — hardware / constant-factor round (worktree `break-hw`)

> Branch `omos/break-hw-opt`, base `1607afc` (the state that closed the previous
> loop with H6 + H10). Goal restated from that loop: make a route **with**
> custom breaks run at >= 0.90x the speed of the same instance **without** them.
>
> This document records what was measured, what was refuted, and what the
> remaining wall actually is. Everything here is reproducible with the harnesses
> in `benchmarks/_hw_fixed.py` and `benchmarks/_hw_ab.py`.

## 0. Result

**Read §6d first.** The single most consequential thing measured here is that
**this ratio is a steep function of route length**: on one synthetic instance it
falls from 0.32 to 0.033 as break routes grow from 12 to 123 activities, and at
equal route length the break path costs **28x** the nobreak path. The 0.663
reported below is from an instance whose routes are ~23 activities — the
favourable end of that curve, not a typical value. It also reverses the verdict
in §6c: the event decomposition does not pay at 23-node routes but is exactly
the right instrument at 60+, so the design in
`openspec/changes/break-regime-eval` should be argued from long-route instances
rather than this one. Whether 0.90 is the right target at all depends on which
route lengths actually occur in production.

**On the instance this work optimised: the break path is ~24% faster and the
ratio moved from 0.564 to 0.671. The 0.90 target is not met.** A build
configuration (§5c) takes the ratio further, to 0.745, but roughly half of that
extra comes from the nobreak side slowing down rather than the break side
speeding up — read §5c before quoting it. §6b turns the
remaining distance into two concrete requirements rather than a verdict: `duration()` from 1573 to roughly 500-600
cycles per call (the event decomposition — research-grade, and this work has
narrowed what it must solve), *together with* the 0.669 ms/iteration of
break-only cost that sits outside the evaluator (ordinary engineering). Neither
lever reaches 0.90 alone; both together model at 0.926. Semantics are bit-identical: the
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
| B | base `1607afc` | 203.1 | 117.1 | 0.577 |
| B | this branch | 210.6 | 139.7 | 0.663 |
| C | base `1607afc` | 218.2 | 120.9 | 0.554 |
| C | this branch, incl. the ported volume filter | 215.1 | 150.4 | 0.699 |
| D | base `1607afc` | 227.9 | 128.5 | **0.564** |
| D | this branch (source only) | 237.7 | 159.6 | 0.671 |
| D | this branch **built with break-trained PGO** | 226.5 | **168.7** | **0.745** |

Break throughput **+24.2%** over the base in session D from the source changes
alone. The break-trained PGO row adds a further +5.7% on break but costs
**-4.7% on nobreak** against the same source without PGO, so about half of its
ratio movement is the denominator dropping — see §5c.

An earlier revision of this section claimed nobreak was "unchanged throughout"
under PGO. That was wrong: it compared the PGO build's nobreak (226.5) against
the *base's* (227.9) instead of against the same source without PGO (237.7).
The correct comparison is the one in §5c.

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

### 5b. The axis this work missed, and how it was found

Everything above makes each candidate evaluation cheaper. **Nothing above
reduces how many evaluations happen** — and that is an independent factor that
multiplies with the first.

The gap was found by a question, not by analysis: *did you take into account the
worktree that had already evolved with other optimisations?* The repository
carries several sibling branches. One of them, `omos/loop-mtkrjm48-y8p6oe`,
forked from `e5bc9d7` — before the streaming pass — and never merged, so it does
not appear in the base's history and was never inspected here. It carries two
commits, one of which attacks exactly the missing axis.

**Ported (`00df023`).** A persistent evaluation-volume filter: the same
exactness argument H10 uses, but carried across `operator()` invocations instead
of being reset on each one. If neither route in a candidate pair has been
modified since that pair was last tested and found non-improving, re-testing it
must give the same answer, so it is skipped. Generations are epoch-scoped (reset
when the cost evaluator changes or on an exhaustive call), and a per-route
content snapshot catches changes that bypass `update()` — solution loads,
perturbation, crossovers. It supersedes H10's per-invocation `lastBreakScan_`.

Exact, so the trajectory is untouched: distance still 4 896 741, all parity
harnesses and the fork suite unchanged. **+3.0% on the fixed-iteration A/B (5/5
pairs) and +5.4 points of ratio on the canonical 8 s metric** — worth more over
a longer run, because a persistent filter has more to skip. Notably it removes
only 1.6% of `runStreamForward` calls: it prunes whole candidates before
`duration()` is reached, taking the operator body and `distance()` with them.

**Not ported (`6af9347`, workspace reuse in `evaluateForwardPass`).** On that
branch it also removed a per-candidate `atSecond` allocation, but this branch's
streaming pass already bypasses `evaluateForwardPass` for proposals. What is
left is the four allocations that path makes per `Route::update()`: 4 x 67
updates per iteration x ~60 ns = **16 us of a 7160 us iteration, 0.22%**. Below
the noise floor, and the port would have to be reconciled against a
substantially diverged `DriveSegment.cpp`. Skipped deliberately, with the
number.

**The process lesson is the durable part.** The base was chosen by checking that
`main` was contained in it. That is not sufficient in a repository with parallel
loop branches: work can exist that is in neither. Enumerate every branch and
diff each against the chosen base before starting.

### 5c. PGO, and why what you train it on decides everything

Profile-guided optimisation was never tried by the earlier loops, and the build
script has had a `--use_pgo` flag all along whose stock training workload
(`X-n101`, `RC208`) contains no breaks at all — it would train only the nobreak
path. Trained instead on **both** scenarios of the real instance, so that any
ratio movement is genuine rather than a degraded denominator:

| | nobreak it/s | break it/s | ratio |
|---|---|---|---|
| no PGO | 214.9 | 146.7 | **0.683** |
| PGO | 230.6 | **151.9** | **0.659** |

Same session, canonical metric, distance 4 896 741 on both, all parity harnesses
green. Break **+3.5%**, nobreak **+7.3%** — so PGO is a genuine speed win on both
sides and a **regression on the ratio**, because it helps the shared path more
than the break-specific one. On the fixed-iteration A/B it measures +10.0% on the
break path (5/5 pairs); the difference from +3.5% is the same phase sensitivity
described in §0.

**But training it on the break scenario alone changes the answer.** The
production workload has breaks, so that is the binary one would actually ship;
the nobreak column below is then simply "the same binary, run without breaks":

| | nobreak it/s | break it/s | ratio |
|---|---|---|---|
| no PGO | 237.7 | 159.6 | 0.671 |
| PGO, trained on break **and** nobreak | 230.6 | 151.9 | 0.659 |
| PGO, trained on **break only** | 226.5 | **168.7** | **0.745** |

Against the same source **without** PGO — which is the only fair comparison, and
not the one an earlier revision of this document made — break is **+5.7%** and
nobreak is **-4.7%** (237.7 -> 226.5). Decomposing the +0.073 of ratio:

| source of the gain | |
|---|---|
| break getting faster | +0.038 |
| **nobreak getting slower** | **+0.035 (48%)** |

So roughly half of it is the denominator dropping. The earlier claim that
nobreak was "unchanged" came from comparing against the *base's* 227.9 instead
of against the same source without PGO. It was wrong, and the correction matters:
this is a real but much smaller break-path win than 0.745 suggests, wrapped in a
metric artefact.

**It also does not reproduce reliably.** Rebuilding through the scripted recipe
of the same flow, trained the same way, produced a binary measuring **0.87** of
the hand-built one (0/4 pairs) — a 13% spread between two supposedly identical
PGO builds. Profile-driven layout on this codebase is not a stable artefact.

That asymmetry is itself a finding: **the break path's code layout is bad enough
that telling the compiler which branches it actually takes is worth more than
any single source change made here except removing the allocations.** It points
at a structural issue — the rule loop and the D5/gate branches are laid out for
the wrong case — which a hot/cold split or explicit branch hints might reach
without needing PGO at all.

**Verdict: not shipped, and not counted towards the goal.** It is exact
(distance 4 896 741, all gates green) and the +5.7% on break is real, so it
remains an option if break throughput is what matters. But half its apparent
ratio gain is the denominator, it costs 4.7% of nobreak, and it does not
reproduce to better than 13% between builds. **The number this branch stands
behind is the source-only 0.671.**

Two things will block anyone who tries to reproduce this:

1. **Ninja does not recompile when `b_pgo` changes.** Reconfiguring from
   `generate` to `use` leaves the instrumented objects in place, so the "PGO"
   build is silently still the instrumented one — it measured 6x *slower* and
   the `.pyd` stayed byte-identical at 12.8 MB. Run `ninja -C <dir> -t clean`
   first; the `.gcda` files are not ninja outputs and survive it.
2. **`-Wmissing-profile` is fatal under this project's `-Werror`.** Translation
   units that the training run never executes (all of spdlog) have no profile,
   and the build dies. Pass `-Dcpp_args=-Wno-missing-profile`.

Recipe, for the absolute-speed option:

```
python buildtools/build_extensions.py --build_dir build-pgo --build_type release     --additional -Db_pgo=generate
python benchmarks/_hw_fixed.py --iters 60 --reps 1 --only break   --tag train
python benchmarks/_hw_fixed.py --iters 60 --reps 1 --only nobreak --tag train
ninja -C build-pgo -t clean
python buildtools/build_extensions.py --build_dir build-pgo --build_type release     --additional -Db_pgo=use "-Dcpp_args=-Wno-missing-profile"
```

### 5d. What a technical review found, and what came of each item

The review looked for correctness holes and for opportunities the measurements
had not reached. Three items were worth acting on and one was worth declining.

**Acted on — dead O(n^2) work in `Route::update` (shipped, +1.3%).** The
`driveAfter` suffix array was built by a nested loop calling
`DriveSegment::merge` on the slow `CustomBreak` overload, ~300 merges per
update at ~67 updates per iteration, and its only reader
(`SegmentAfter::driveState`) has no callers. Removed; distance unchanged.

**Acted on — the PGO claim (corrected, see §5c).** The review caught that the
"nobreak unchanged" claim used the wrong baseline. Half the apparent ratio gain
was the denominator. That correction is the most valuable thing the review
produced.

**Tested and refuted — that PGO's win was mostly undoing `[[unlikely]]`.** The
break branches of `distance()` and `duration()` were marked cold, which for this
fork's workload is exactly backwards. Removing them measured 1.004 (4/5 pairs),
so the annotation was not the explanation. Removed anyway, for being wrong.

**Declined — "closing the volume filter's exactness hole".** The review is
right that the filter is not exact for a node that is *unplanned* at the time of
the test: `uUpdate` is 0 in that case, so the pair is skipped unless V's route
changed, even though that configuration was never tested. Two things decide
against changing it:

1. **It is not a regression this branch introduced.** The `uUpdate = 0` pattern
   is in the base; persistence widens its reach but does not create it.
2. **Closing it costs 10.6% of solution quality on the measured seed.** With the
   conservative fix (always test when U is unplanned), the break distance goes
   from 4 896 741 to **5 418 230** — outside the quality gate this work is held
   to (4 651 458 – 4 940 363). All parity harnesses and the fork suite stay
   green, so this is a trajectory change rather than a correctness failure: the
   search is first-improvement, and evaluating a differently-ordered candidate
   set lands in a different local optimum.

So it is a **heuristic filter with a documented blind spot**, not a proof, and
the blind spot is load-bearing for the search's current quality. Anyone
tightening it must re-tune against the quality gate, on more than one seed. The
patch is kept in the branch's stash under `exactness-fix-under-review` rather
than discarded.

**Checked and found inactive — the "wrong reference" for round 2.** The review
flagged that `Route::update` calls `evaluateForwardPass` *with* an
`extendedBreakServices` buffer, in which mode the second drive pass reuses the
D5 service frozen by the first, while `test_stream_parity` compares against the
variant *without* the buffer — so the 500/500 gate would be validating an
evaluator that `update()` never runs.

The harness now evaluates **both** modes on every recipe and reports any
disagreement. Result: **0 disagreements over all 500 proposals**. The two modes
coincide on the current recipe set, so the gate does cover what `update()` runs.
The check is kept permanently rather than removed, because the concern is sound
in principle and would otherwise reappear silently the first time a recipe or a
change makes the modes diverge. Worth closing before this branch is
trusted on a workload with absolute break windows.

### Refuted, with evidence

| Hypothesis | Verdict |
|---|---|
| **Round 2 of the forward pass is a significant cost.** A prior analysis put its firing rate at 0.4–0.6 and scoped an incremental resume as days of work. | **REFUTED.** Measured `round2_f = 0.009` — it runs on 0.9% of candidates. Incrementalising it is worthless. |
| **Break routes are longer, so part of the gap is inherent to the workload.** | **REFUTED.** Mean route length 22.7 (break) vs 23.2 (nobreak), ratio 0.98. The excess is code, not data size. |
| **The break path loses a pruning step that the nobreak path has.** | **REFUTED.** 31.2% of break proposals reach `duration()` vs 32.4% for nobreak — break prunes marginally *more*. |
| **The collapse's "every mandatory break is served" gate is what limits its firing rate.** It only exists because the collapse could not produce the arrival clock at the end depot, which the D3 term for an unserved mandatory break needs. | **REFUTED as the binding constraint.** Caching a second suffix fold that stops one node short of the end depot (`durAfterExEnd`) lets the collapse take the final step by hand and recover that clock exactly, which removes the gate. Firing rate: **unchanged at 42.9%** — the same candidates are blocked one gate later, by `takenMask`. Reverted (an extra 80 B/node array and a merge per update, for nothing). |
| **The tail can be proven inert without walking it, so the collapse's gates can be dropped.** All three accumulators are non-decreasing along a stretch with no reset, so a rule that cannot fire at the LAST boundary cannot fire anywhere in the tail — and every quantity needed is an end-of-route value: the collapsed fold (the segment merge is associative), the arrival clock at the end depot (via a suffix fold stopping one node short of it), and the drive/work/duty totals (via cached suffix sums). For duty this needs the identity `duty(i) = max(duty(i-1) + edge, atSecond(i) - lastReset) + service(i)`, whose unrolled maximum is dominated by its last term. | **IMPLEMENTED, EXACT, AND STILL NOT WORTH IT.** Firing rose only 42.9% -> 48.7% (3.57 -> 3.84 nodes saved), while the extra suffix arrays cost ~2% in `Route::update()`: net **0.980** over 5 pairs. Reverted. **The finding is the point:** the new blocker is `inert = 30.3%` — the bound is not being conservative, the tail genuinely *does* cross a mandatory break's trigger in those candidates. No "prove it cannot fire" shortcut can reach them. |
| **Reusing the route's cached `durAt[pos]` instead of rebuilding each client's duration singleton.** The route already stores exactly that segment, and the rebuild costs a random probe into the client array. | **REFUTED.** 0.973 median over 5 pairs (2/5 wins) — a small *regression*. `durAt` is an 80-byte struct, so reading it streams a third array alongside `activitiesAt_` and `locations`, while the client records are already warm from being re-read across thousands of candidates on the same route. Reverted. |
| **Lever 2 is memory-latency waste too, so the same SoA treatment will pay again.** `distance()`'s break branch and six of `Route::update`'s per-node predicates still dereference `nodes[i]` (pointers into three different owners) where the maintained `activitiesAt_` array would answer sequentially. | **REFUTED.** 1.001 median over 5 pairs (3/5 wins) — no effect. Reverted. |
| **PGO's win can be captured in the source with explicit branch hints.** Break-trained PGO buys ~11% on the break path, which says the default layout assumes the wrong case; stating the measured case with `[[likely]]`/`[[unlikely]]` on the settled-rule skip in `DriveSegment::merge` and on the client branch of the fold should recover part of it. | **REFUTED.** 0.995 median over 5 pairs (1/5 wins). PGO's value here is not a handful of branch probabilities — it is global block reordering and inlining decisions across the heavily templated `Proposal<Segments...>` instantiations, which no annotation reaches. Reverted; the win is only available by actually running the profile. |
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

## 6b. What 0.90 actually requires

An earlier version of this section claimed 0.876 was a *hard ceiling* and that
0.90 was therefore unreachable while break evaluation stays bit-exact. **That was
wrong, and the error is worth stating plainly: it held everything except
`duration()` fixed.** There is another 0.669 ms/iteration of break-only cost
outside the evaluator, and it is attackable too. Corrected below.

Canonical metric, same session: nobreak 4.748 ms/iteration, break 7.158 ms,
ratio 0.663. Splitting the 2.410 ms excess with the phase profile (rdtsc
overhead discounted at ~50 cycles per scope):

| | ms/iteration |
|---|---|
| break excess over nobreak | 2.410 |
| of which `duration()` | 1.741 (**72%**) |
| of which everything else, break-only | 0.669 |

Reaching 0.90 means removing 1.882 ms of the 2.410. Two levers, and **both are
needed** — neither suffices alone:

| | resulting ratio |
|---|---|
| remove the whole `duration()` excess, touch nothing else | 0.877 |
| remove the whole non-evaluator excess, touch `duration()` not at all | ~0.72 |
| `duration()` at 500 cyc/call **and** the non-evaluator excess removed | **0.926** |
| `duration()` at 700 cyc/call and the non-evaluator excess removed | 0.882 |

So the requirement is concrete: **`duration()` from 1573 down to roughly 500-600
cycles per call, together with essentially all of the 0.669 ms of break-only
cost outside it.**

**Lever 1 — `duration()` to ~500 cyc/call.** That is 3-5 merges per candidate
instead of the ~11 nodes it still simulates: the event/regime decomposition of
`openspec/changes/break-regime-eval/design.md`. **It was built and measured; see
§6c. It does not pay at this route length**, so Lever 1 has no known route left. This work has sharpened its
scope considerably — round 2 is a non-issue (0.9%), the tail collapse already
handles 42.9% of candidates, and §5 shows that the "prove the tail cannot fire"
shortcut only reaches 48.7% because in 30.3% of candidates the tail genuinely
crosses a trigger. So the crossing localisation is unavoidable, and it is the
one genuinely research-grade piece left.

One caution for whoever picks it up: at ~23-node routes, an O(log n) localisation
query costing O(log^2 n) merges is not obviously cheaper than walking the ~8-node
tail it replaces. The asymptotic win needs routes far longer than these, so Fase
A should measure the constant before committing to the design.

**Lever 2 — the 0.669 ms outside the evaluator.** No open questions here, but it
is not free either: `Route::update()` (34 790 cycles/call on break against
23 095 on nobreak, six passes over the route), the ShiftBreak scan (break-only),
the break branch of `distance()` (86 against 56 cycles/call, and called three
times more often than `duration()`), and the operator guard layer.

Do **not** expect this to be latency waste. Four independent experiments say the
break path is compute-bound, not memory-bound, once the heap traffic is gone:
software prefetch (0.997), reusing the cached `durAt` singleton (0.973),
replacing the remaining `nodes[i]->` predicates in `distance()` and
`Route::update` with sequential `activitiesAt_` reads (1.001), and the fact that
the one locality change that *did* pay — `activitiesAt_` itself — paid inside
the evaluator's hot loop rather than in these callers. Lever 2 will have to come
from doing less work, not from arranging the same work better.

**What remains true from the earlier version.** A break evaluator as cheap as the
*nobreak fold* (280 cyc/call) would be one doing O(#segments) work with no
per-node simulation — the naive cached monoid fold, which
`tests/cpp/test_proposal_fold_residual` measures diverging on 53 of 174 mutated
proposals with waiting off by up to 26 595 s. So `duration()` cannot go
arbitrarily low while staying bit-exact. But ~500 cycles is well above that
floor and is not excluded by the harness — the fold is refuted, the event
decomposition is not.

Caveats. These numbers come from one instance (group-54: 511 locations, ~23-node
routes, 5 break rules per vehicle, 2 of them mandatory overnight rests). Route
length in particular moves everything: see §6d, which measures the ratio falling
from 0.32 to 0.033 as break routes grow from 12 to 123 activities.

## 6c. The event decomposition was built and measured. It does not pay here.

§6b named Lever 1 as research-grade. It was then implemented, in its cheapest
correct form, and **measured neutral-to-negative**. This is the Fase 0 answer
`openspec/changes/break-regime-eval/design.md` asks for, obtained by building the
thing rather than by argument.

### The mechanism

The design's premise is that a stretch between decision points can be skipped.
Two forms were built:

**A jump between consecutive break nodes**, using per-route cached folds of each
stretch (`durToBreak`) plus closed forms for the accumulators. It is correct, and
it fires on **0.1%** of candidates — because **86.8% of candidates have no break
node in the evaluated region at all.** The seed already skips the prefix, and the
break nodes are almost always in it. There is nothing to jump between. This alone
retires the design's central picture of "k+1 regimes separated by k break nodes"
for this instance class.

**A light scalar replay of the break-free tail**, which is what the shape of the
problem actually calls for. It rests on an identity of `DurationSegment::merge`
worth recording, because it is what makes any such replay possible:

> While no time warp occurs, with `S(i)` the prefix fold's
> `duration() + startEarly()`,
>
>     atSecond(i) = max(S(i-1) + edge(i), twEarly(i))
>     S(i)        = atSecond(i) + service(i)
>
> — an identity, not an approximation. The schedule clock follows a plain scalar
> recurrence, so the tail can be replayed from contiguous per-node arrays and the
> 80-byte segment merges replaced by the one fold `durAfter` already caches.

That replay was built, mirroring `DriveSegment::merge` rule for rule, gated on
zero time warp (checked per node, falling back to the walk otherwise). It is
**bit-exact**: a differential harness that runs the scan and the node walk side
by side and compares `duration`, `timeWarp`, `waiting`, `dueMask`, `firstDue[]`
and the end-depot clock reports **0 mismatches over 60 iterations**, and the
solve distance stays 4 896 741.

### The measurement

| variant | speedup vs the shipped evaluator |
|---|---|
| scan before the tail collapse | 0.982 (0/5 pairs) |
| tail collapse first, scan only for what it misses | 0.989 (2/5) |
| scan skipping settled rules via a bitmask, not touching their `BreakRule` | 0.974 (1/5) |

All three reverted.

### Why it loses, which is the part worth keeping

1. **The routes are too short.** ~23 nodes, of which the scan replaces 8-15. An
   O(#events) win needs the node count to dominate the per-event bookkeeping;
   here they are the same order.
2. **The scan does not remove the contract's work, only its packaging.** Every
   boundary still has to evaluate the break rules — that is what the semantics
   require. The scan removes the segment merge and the matrix lookup (~50 cycles
   of ~110), not the rule loop.
3. **The cheap case was already cheap.** The tail collapse settles 42.9% of
   candidates in two merges; the scan can only compete for what it misses.
4. **The caches are not free.** Five per-node arrays rebuilt in every
   `Route::update()`, and there are ~67 updates per iteration.

### What this means for the design

The premise that survives is that the tail can be short-circuited — that is
shipped and pays. The premise that does not is that decomposing the *interior*
into regimes buys anything at this problem size. A debugging note also worth
carrying over: the drive state's `lastResetAt_` is initialised part-way through
the `idx == 1` iteration, so anything reading the drive state earlier in that
iteration sees a zero duty clock and fires every trigger far too early. That cost
one wrong-distance run to find, and only the end-to-end distance caught it — the
500/500 parity harness did not.

## 6d. The ratio is a function of route length, and 0.663 is the good end

Every number above comes from group-54, whose routes are ~23 activities. That is
not a neutral choice. `benchmarks/_hw_scale.py` builds one synthetic instance and
varies how many vehicles it may use, so the same clients are served by routes of
different lengths, and measures the ratio at each (120 clients, 120 fixed
iterations, best of two, pinned; both scenarios feasible throughout):

| break route length | nobreak s | break s | ratio |
|---|---|---|---|
| 12 | 0.069 | 0.215 | **0.321** |
| 42 | 0.054 | 1.212 | 0.045 |
| 63 | 0.066 | 1.583 | 0.042 |
| 123 | 0.055 | 1.688 | **0.033** |

The nobreak side is the *same* 122-activity single route in every row and its cost
barely moves. The break side grows with its own route length. At equal route
length — 122 against 123 activities, bottom row — the break path costs **28x**
the nobreak path.

This is the O(n)-per-candidate signature, and it is the shape the fork's
evaluator has by construction: `runStreamForward` simulates the re-evaluated
span node by node, while the nobreak path merges two or three cached segments
regardless of how long the route is.

**Three consequences, in order of importance.**

1. **group-54's 0.663 is the favourable end of this curve, not a typical value.**
   Any production instance with longer routes is dramatically worse. If routes of
   40+ activities occur in practice, the break path there is ~20x slower rather
   than ~1.5x, and no amount of the constant-factor work in this document
   changes that — it is the O(n) term.
2. **It reverses §6c's verdict outside group-54.** The event decomposition was
   measured as not paying at 23-node routes, for the good reason that the node
   count and the per-event bookkeeping are the same order there. At 60-120 nodes
   they are not, and the decomposition is exactly the right instrument. The
   design in `openspec/changes/break-regime-eval` is not wrong; it is aimed at a
   regime group-54 does not occupy. **Its business case should be argued from
   long-route instances, not from this one.**
3. **The 0.90 target is only even conceivable at short route lengths.** At 23
   activities it is out of reach for the reasons in §6b/§6c. At 60+ it is not in
   the same universe. Whether the target is the right one to hold the fork to
   therefore depends on which instances matter — a question this document cannot
   settle, but which it can now put numbers on.

Caveat on the comparison. The two scenarios do not produce the same routing: the
nobreak solver collapses onto one long route because capacity and time windows
are unconstrained here, while the break solver splits according to the vehicles
available. The bottom row is the clean comparison — near-equal route lengths on
both sides — and the trend across rows is monotone and large enough that the
looser rows still carry the point.

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
