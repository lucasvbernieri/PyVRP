# Findings for `openspec/changes/break-regime-eval`

Drop-in appendix for that change's `design.md`. Everything here was measured on
branch `omos/break-hw-opt`; the harnesses and full write-up are in
`docs/BREAK_HW_OPT.md`.

The change is scoped as a viability investigation with an explicit go/no-go, and
its Fase 0 asks for exactly the numbers below. Several of its open questions can
be answered now, and one of its work items can be struck.

## 1. The "métrica-matadora" has an answer, and it is not the one the design expects

The design names the frequency of interior trigger-crossings as the metric that
decides viability (Decisão 1, Fase 0). Measured over **1 267 511 real proposal
evaluations** on the production instance (group-54):

```
seeded=0.912  round2_f=0.009  L_round=17.95  n_flat=26.67
tail-collapse fires=0.429  nodes saved when firing=8.3
```

But the load-bearing number is a different one:

> **86.8% of candidates contain no CUSTOM_BREAK node at all in the region they
> re-evaluate.**

The prefix seed already skips the part of the route where the break nodes live.
So the design's picture — `k+1` pure regimes separated by `k` break nodes, with
cost `O(#breaks in the tail)` — does not describe what the evaluator actually
sees. There is usually nothing to decompose *between*; there is one break-free
stretch, and the cost is walking it.

**Consequence for the plan:** Fase A should not begin by characterising crossing
localisation between break nodes. It should begin from the shape above.

## 2. Round 2 is a non-issue. Strike it from the plan. **[WRONG -- see §9.4]**

The design (and the prior analysis it inherits) treats the second round of the
forward pass as a cost worth incrementalising, scoped at days of work.

    round2_f = 0.009

It runs on **0.9%** of candidates.

> **This measurement was taken at 250 iterations and does not hold.** At 1500
> iterations `round2_f` is 0.384 and at 3000 it is 0.442. Round 2 is the largest
> open lever on the break path, not a non-issue. See §9.4.

## 3. The decomposition was built. At this route length it does not pay.

Both forms reverted. Neither showed observable divergence (end-to-end distance
unchanged at 4 896 741 -- single seed 548585631, 250-400 iterations, one
instance; that is consistent with exactness, not a proof):

| form | firing | measured |
|---|---|---|
| jump between consecutive break nodes, via cached stretch folds | **0.1%** | n/a |
| light scalar replay of the break-free stretch | 30.8% | **0.974 – 0.989** |

The jump fires on 0.1% for the reason in §1. The replay reports **0 mismatches
in a differential harness** (run beside the node walk, comparing `duration`,
`timeWarp`, `waiting`, `dueMask`, `firstDue[]` and the end-depot clock; 60
iterations, ~150k candidates) -- and still loses.

**Note what was built.** The scalar replay is still **O(n)**: it removes the
segment merge and the matrix lookup but keeps evaluating the break rules at
every boundary, because the contract requires it. **The O(#events) evaluator
this change's thesis asks for -- skipping a whole stretch in O(1) -- was not
built.** Why the replay loses here:

- routes are ~23 activities and the replay covers 8-15 of them;
- it removes the segment merge and the matrix lookup (~50 of ~110 cycles per
  node) but **not** the break-rule evaluation, which the contract requires at
  every boundary;
- the tail collapse already settles the cheap 42.9% in two merges;
- its per-node arrays must be rebuilt in each of ~67 `Route::update()` calls per
  iteration.

## 4. …but that verdict is specific to this instance, and reverses at scale

| break route length | break s / 120 iters | ratio break/nobreak |
|---|---|---|
| 12 | 0.215 | 0.321 |
| 42 | 1.212 | 0.045 |
| 63 | 1.583 | 0.042 |
| 123 | 1.688 | 0.033 |

Same instance, vehicle count varied so route length varies; the nobreak side is
the same 122-activity route in every row. At **equal** route length the break
path costs **~30x** the nobreak path.

**Three caveats stop this from being read as an O(n)-per-candidate signature:**

1. **The curve saturates.** From 63 to 123 nodes the length doubles and the
   break-side time rises 7%. An O(n)-per-candidate cost with a similar candidate
   count would double. Something beyond the pass is in play (fewer routes =>
   fewer cross-route candidates, fewer `Route::update` calls), and **there is no
   profile and no `StreamStats` on this instance** to attribute it.
2. **This is not the same curve as group-54.** The synthetic instance has **one
   rule** (DUTY_TIME 6h, mandatory, 7-day window, no capacity/TW pressure);
   group-54 has five, two of them mandatory. At 12 nodes the synthetic already
   measures 0.321 while group-54 measures 0.733 at ~23. group-54 is not "the
   favourable end of this curve" -- it is a different curve.
3. **The decomposition was never measured on long routes.** What was measured
   there is that the ratio falls, not that the decomposition recovers it. And
   the form that was built, being O(n) with a smaller constant, would give at
   most ~2x on `duration()` at any length -- it does not close a 30x gap.

**Consequence for the plan: the go/no-go must be taken against the route lengths
that actually occur in production.** On group-54 it is a measured no-go. On long
routes the problem is real (ratio 0.03-0.05) but **the hypothesis that the
decomposition solves it is untested**. The first step of any Fase A is to
profile the break side on a long-route instance (`PYVRP_STREAM_STATS` +
`PhaseProfile`, both already on the branch) before writing any evaluator.

## 5. An identity worth keeping, whoever implements this

While no time warp occurs, with `S(i)` the prefix fold's
`duration() + startEarly()`:

    atSecond(i) = max(S(i-1) + edge(i), twEarly(i))
    S(i)        = atSecond(i) + service(i)

This is an identity of `DurationSegment::merge`, not an approximation (it falls
out of `diffWait` cancelling between `duration_` and `startEarly_`). It is what
makes any scalar replay of a stretch possible: the schedule clock needs no
segment algebra. Warp is detected per node as `S(i-1) + edge > startLate(i)`,
which is exactly the merge's own `diffTw` condition, so a replay can fall back
to the walk precisely when the identity stops holding.

All three accumulators are non-decreasing along a stretch with no reset, and for
duty specifically

    duty(i) = max(duty(i-1) + edge, atSecond(i) - lastReset) + service(i)

whose unrolled maximum is dominated by its last term, because
`atSecond(i) >= atSecond(j) + service(j) + travel(j..i)`. So a rule that cannot
fire at a stretch's last node cannot fire anywhere in it — testing the far end
settles the whole stretch in O(1).

## 6. Two correctness traps, both of which cost a wrong-distance run to find

1. **`ALL_TIMERS` replaces `takenMask`, it does not add to it.** An untaken
   `ALL_TIMERS` break firing in a skipped stretch can clear a *mandatory*
   break's bit and let `breakDueMask` grow again. Any gate must require every
   `ALL_TIMERS` break to be taken, not only every mandatory one.
2. **`lastResetAt_` is initialised part-way through the `idx == 1` iteration.**
   Anything reading the drive state earlier in that same iteration sees a zero
   duty clock and fires every trigger far too early. Gate on
   `driveNode0Ready`.

Also worth knowing before trusting a gate: `presentMask` aliases break ids
modulo 16 while `occ`/`remaining` are indexed by the raw id, so
`remainingMask == 0` is only a sound "no break node ahead" test while the id
space stays below 16.

## 7. What the parity harnesses do and do not cover

`test_stream_parity` (500/500) passed on an implementation that changed the
solve distance from 4 896 741 to 5 622 635. It does not cover the gate space of
a short-circuit. **The end-to-end distance on a real solve is the gate that
catches these** — it is hypersensitive, because one divergent evaluation changes
the search trajectory. But it covers a short prefix of the trajectory (250-400
iterations), on **one seed, one instance**. For the jump between break nodes,
which fires on 0.1% of candidates, that gate exercised ~1 300 evaluations —
close to untested there. Any implementation of this change should treat an
unchanged distance over a few hundred iterations as the primary gate, and build
a differential harness (run both paths, compare the full result tuple) rather
than relying on the recomposition harnesses.

## 8. Added after the first draft — what changed the picture

The sections above were written when the branch stood at ratio 0.663 and the
conclusion was that the decomposition was the only path left. Three things
happened afterwards that change what this change should be scoped to do.

### 8.1 The ratio is now 0.733, and the decomposition did not do it

Canonical metric, base and branch in the same session:

    base 1607afc   nobreak 232.5 it/s   break 127.6 it/s   ratio 0.549
    branch         nobreak 235.7 it/s   break 172.7 it/s   ratio 0.733

Replicated in a second session: 0.561 -> 0.736. Only same-session pairs are
controlled; this machine varies by up to ~50% between sessions.

+35.3% on the break path, none of it from the regime decomposition. It came from
three things this change does not contemplate: removing per-candidate heap
allocations, a persistent evaluation-volume filter, and an admissible lower
bound that prunes 33% of proposals before `duration()` runs.

### 8.2 `duration()` is now 84% of the gap, up from 72-75%

Everything around it shrank, so the evaluator is now almost the whole story:

Both columns are pinned profiles; "before" is the pre-lower-bound binary.

| | pre-lower-bound | current |
|---|---|---|
| `duration()` cycles/call | 1573 | **966** (nobreak: 158) |
| cycles per simulated node (~14) | ~110 | **~69** |
| `duration()` calls | 2.55M | 1.74M |
| `Route::update` cycles/call | 34 790 | 16 522 |
| share of the gap in `duration()` | 72-75% | **84%** |
| `binaryOps` body vs nobreak | worse | **better** |

The break path now makes **fewer** `duration()` calls than the nobreak path
(1.74M against 2.27M) and its operator bodies are cheaper. What is left is the
per-call cost: **966 cycles against 158**. If that excess went away the ratio
would be ~0.96. So this change's target is the right one — it is now the *only*
one.

### 8.3 But the reason it is expensive is not what a bound can reach

Of the proposals that survive the lower bound and pay for the pass, **99.4% are
non-improving anyway**. The obvious response is a tighter bound. Three were
built and measured; all failed, for one shared reason:

| attempt | result |
|---|---|
| the cached monoid fold as a bound | **inadmissible** — 0.88% violations over 1.0M checks, max overshoot 580 200 |
| extend the bound to the cross-route `deltaCost` | 0.99 — the existing `out >= 0` early-out already catches what is catchable |
| abort the pass incrementally on the partial value | 0.982 (0/5) — 2.9% abort rate, at 85% of the walk |

> **What makes a break candidate non-improving lives in the penalty terms — time
> warp, break lateness, overtime, waiting — and computing those is the forward
> pass.** The duration-cost term is exactly `c_d * (travel + service)`; it is
> cheaply boundable, already bounded, and it is not the discriminator.

The fold's failure is worth knowing precisely: it treats a CUSTOM_BREAK as an
ordinary node carrying its own window, so `merge` clamps arrival to that window
and charges wait or warp *whether or not the break is eligible there*. The real
pass decides eligibility first.

### 8.4 Nothing is left on the memory side

**Four** independent experiments say this path is compute-bound once the heap
traffic is gone: software prefetch (0.997), reusing the cached `durAt` singleton
(0.973), replacing the remaining `nodes[i]->` predicates with sequential
`activitiesAt_` reads (1.001), and reading the route's cached duration edges
instead of probing the 2.1 MB matrix (0.990). (An earlier draft counted the
aggregate rule-loop skip as a fifth; it is a *compute* experiment -- less work,
not better locality -- and says nothing about memory.) The matrix rows are hot — the same route is re-evaluated thousands of
times in a row.

### 8.5 What this means for the change

The scope narrows to one sentence: **cut the ~14 nodes a candidate simulates**
with an evaluator that skips a stretch in O(1) — not the scalar replay, which
was built and measured. Everything else on the break path has been taken; the
constant factor per node is ~69 cycles and five attempts to reduce it failed.

Whether that is worth doing: **measured no on group-54; unproven on long
routes.** §4's ratio collapse says the problem exists there; nothing measured
says this change is the fix, and §4's saturation says part of the cost is not
even in the pass. Profile a long-route instance first.

**Limitations of everything above:** one seed (548585631); one instance for
every profile and `StreamStats`; Windows/UCRT (the allocation win is attributed
to malloc+free ~50 ns and may be smaller on glibc/jemalloc — nothing was
measured on the production platform); and the volume filter of §8.1 is a
*heuristic* with a documented blind spot (`uUpdate = 0` for unplanned nodes),
whose closure moves the distance to 5 418 230 — patch parked in
`stash@{0}: exactness-fix-under-review`.

## 9. Added after the audit: the ratio depends on how far the search has run

Every fixed-iteration A/B in this investigation was run at **250 iterations**.
Running the **current** binary (HEAD, post-lower-bound) at several lengths --
same session, pinned, break distance invariant at **4 896 741** at every point:

| iterations | nobreak ms/it | break ms/it | ratio |
|---|---|---|---|
| 250 | 7.51 | 7.03 | **1.069** |
| 600 | 5.83 | 6.64 | 0.878 |
| 1000 | 5.34 | 7.11 | 0.751 |
| 1500 | 5.33 | 7.05 | 0.756 |
| 3000 | 5.25 | **7.60** | **0.691** |

(`cpp_min` over 2-5 reps, clean release binary; artefacts `hw_ratio_HEAD*.json`.
Break distance invariant at 4 896 741 at every point; nobreak converges to
6 499 539 from 1500 onward.)

### 9.1 There is no plateau -- the ratio keeps falling

The ratio decreases monotonically and had **not** stabilised by 3000 iterations.
For production this is the headline: **the longer the solve runs, the worse the
ratio gets.** The canonical 8 s metric (0.733) sits between the 1500 and 3000
points, exactly where its iteration count puts it. The two metrics never
disagreed -- one was simply read at 250 iterations and the other much further
along.

### 9.2 The break path gets more expensive; the nobreak path gets cheaper

**Break rises from 7.03 to 7.60 ms/it. Nobreak falls 30% (7.51 -> 5.25) and
flattens.** Both movements feed the gap. And the break path is not "doing more
work because it is still improving": both searches have converged in distance by
1500 iterations (4 896 741 and 6 499 539, unchanged at 3000).

That fits §8.3's wall exactly. As the search converges almost every candidate
becomes non-improving. On the nobreak side that is cheap -- the folds and the
`out >= 0` early-out settle it with no work. On the break side **every candidate
still pays the full forward pass to discover it is non-improving**, because the
discriminator lives in the penalty terms. Hence the flat line.

> **This reframes the target.** It is not "cut the ~14 nodes a candidate
> simulates" in the abstract. It is **make the break path's per-iteration cost
> decay with convergence, the way the nobreak path's does.** Same wall, different
> angle -- and it points at a different attack: rather than making the pass
> faster, **avoid paying it** once the search is stable. The evaluation-volume
> filter is exactly that idea and returned the least of the three axes (+3.0%)
> because it is very conservative. Worth reopening there.

### 9.3 Methodological consequence -- it changes how to read this document

Every per-step verdict in `BREAK_HW_OPT.md` §5 (1.03, 1.09, 1.11, and the
0.97-1.00 refutations) was measured at 250 iterations, i.e. in the regime where
the break path is at its best. **Those verdicts are not necessarily the effect
in the converged regime**, which is what the canonical metric -- and production
-- see. No reverted experiment was re-tested at 1000+ iterations, and some of
the "neutral" ones may not be neutral there.

Unfortunately most reverted experiments were measured from the working tree and
never committed, so they are not cheaply recoverable. **The cheap, high-value
next step is therefore: re-run the surviving levers at 1000+ iterations before
writing anything new**, and make 1000 iterations (or the 8 s metric) the default
for any future A/B on this path.

### 9.4 The mechanism, localised -- and §2 of this document was wrong

Instrumented profile per `operator()`, 250 against 3000 iterations:

| | nobreak 250->3000 | break 250->3000 |
|---|---|---|
| total (Mcyc/it) | 20.7 -> 14.5 (-30%) | 20.4 -> 20.1 (flat) |
| `duration()` calls/it | 7093 -> 5232 (-26%) | 4285 -> 4837 (**+13%**) |
| `duration()` cyc/call | 167 -> 159 | 1006 -> **1371 (+36%)** |
| `Route::update` calls/it | 51.5 -> 46.9 | 53.1 -> **133.4 (+151%)** |

At 3000 iterations `duration()` costs 6.63 Mcyc/it on break against 0.83 on
nobreak -- an excess of **5.80 Mcyc/it** against a total gap of 5.57. So
**`duration()` is the entire gap**; everything else nets out.

And the +36% per call is localised. Counters added in this session:

    round2_f = 0.007  at 250 iterations
    round2_f = 0.384  at 1500
    round2_f = 0.442  at 3000
    round2-why: absorb=0.003  cleared=0.378  both=0.003   (at 1500)

> **§2 of this document said round 2 "runs on 0.9% of candidates, is worth
> nothing, strike it from the plan". That was measured at 250 iterations and is
> wrong. In the regime production runs in, 44% of candidates execute the forward
> pass twice** -- roughly 31% of all `duration()` time, which is the entire gap.

The trigger is `absorbedWaiting > 0 || clearedWindows` (`Route.h`, next to
`runRound(true)`), and the attribution says it is **almost entirely
`clearedWindows`**: a non-due break with an absolute window has its close
cleared in the fold, and today that costs a full second pass from scratch. The
D5 absorption case is negligible.

### 9.4b The counters double-count round-2 candidates -- divide before reading

`StreamStats` increments its per-node and per-gate counters once **per round**,
while `calls` increments once **per candidate**. So every rate in a converged-
regime dump is inflated by `1 + round2_f`. At 250 iterations that factor is
1.007 and the historical numbers in this document are unaffected; at 1500 it is
1.384 and at 3000 it is 1.442.

Correcting for it turns the picture sharper rather than softer:

| per candidate | 250 iters | 1500 iters |
|---|---|---|
| nodes walked per round (`L_round`) | 16.7 | 14.7 |
| `duration()` cyc/call per round | ~999 | ~960 |
| tail collapse fires (`sc_hit`) | 0.349 | **0.226** |

**A single round costs the same as it always did** -- slightly less, if anything.
The entire +36% in `duration()` per call is the second round, and nothing else.
That makes round 2 worth its full nominal value as a lever: removing it takes
`duration()` from ~1371 back to ~950 cycles per call.

It also means **the tail collapse gets less effective as the search converges**
(0.349 -> 0.226), which is one more figure in this document that only holds at
250 iterations. The same applies to the `why-no-collapse` attribution: `inert`
blocks 0.000 of candidates at 250 iterations but 0.211 at 1500, so the reverted
inertness shortcut (§5 of `BREAK_HW_OPT.md`, judged "does not pay") was judged
in a regime where the gate it widens never binds. It deserves a re-test.

### 9.5 How much is left, and what round 2 is worth

At 3000 iterations nobreak costs 5.25 ms/it and break 7.60. For ratio 0.90 the
break path must fall to 5.83 ms/it -- a **23% cut of its total cost**, or ~68%
of the `duration()` excess.

If a round-2 candidate costs ~2x a single-round one, round 2 accounts for
`0.442/1.442 ~= 31%` of `duration()` time, about 1.8 Mcyc/it. Removing it
entirely would move the ratio from 0.691 to **~0.79**. It does not reach the
goal alone, but it is the largest single lever found since the lower bound, and
the first one that attacks the term which **grows** with convergence.

## 10. Round 2 removed. Ratio 0.744 -> 0.805.

§9.4 identified round 2 as the largest open lever. It was taken.

### 10.1 What changed

Round 2 existed to undo a clamp the first round should never have applied. When
a break turns out not to be due its absolute window does not apply -- but the
pass had already folded the singleton with that window as `late`, charging wait
or warp against it. The clearing was recorded in the per-break store and the
whole pass re-ran to fold it again without the clamp.

Nothing forced that order. `earlyArrival` comes from the previous node, the
drive merge reads only `atSecond`, and the eligibility decision reads only the
drive state and `atSecond`. None of them touch the post-merge `durBefore`. So
the drive fold and the decision now run **first**, and the singleton is built
with the window already cleared -- one round instead of two.

Why it is exact: clearing changes only this node's `startLate_`. Downstream,
`startLate_` reaches `merge()` solely through `diffWait`, which adds to
`duration_` and subtracts from `startEarly_`, leaving `duration() +
startEarly()` -- the clock this pass runs on -- invariant. Every downstream
arrival, drive state and decision is therefore unchanged, so the in-line fold is
exactly what the second round produced.

That argument needs the clamp not to have warped, so `pastWinClose` (a
conservative superset of "the clamp warped", since `diffTw > 0` requires arrival
past the close) falls back to the two-round path. Two structural gates fall back
the same way, because the invariance argument does not survive them: a release
time makes `startEarly()` clamp to `releaseTime_`, and a reload depot routes the
fold through `finaliseBack()`, which reads `startLate_` directly.

### 10.2 Measured

    round2_f              0.384 -> 0.006   (only the D5 absorb case is left)
    round2 cleared        0.378 -> 0.000
    pastClose fallback             0.000   (never fires on this instance)
    duration() cyc/call  1328.7 -> 1014.1
    L_round               20.33 -> 14.85

| metric | before | after |
|---|---|---|
| paired A/B, 5 interleaved pairs, 1500 iters | — | **+8.00%** (5/5, 1.076-1.093) |
| ratio at 1500 iters, same session, 3 pairs each | 0.7439 | **0.8052** |
| canonical 8 s metric | 0.733 | **0.781** (break 179.6 it/s) |

Against the `1607afc` base the canonical break path is now **127.6 -> 179.6
it/s, +40.8%**, ratio 0.549 -> 0.781. (The base figure is from an earlier
session; the nobreak side moved less than 1% between them, 232.5 -> 230.0, so
the comparison is close to controlled but is not a paired measurement.)

Gates: distance 4 896 741 unchanged; `test_stream_parity` 500/500 exact with 0
evaluator-mode disagreements; `test_segment_fold_parity` 63/63;
`test_proposal_fold_residual` 53/174 unchanged; pytest 1181 passed, 2 skipped,
6 xfailed. `duration()` was called 6 727 873 times before and after, so the
search followed the same trajectory candidate for candidate.

### 10.3 One lever tested and refuted: the drive fold in `Route::update()`

`driveState()` is not part of the `Segment` concept, and the only
`.driveState(` call site in the **library** was inside ShiftBreak's own segment
adapter calling the very method being removed.

> **Correction.** An earlier version of this section said deleting both
> "compiles clean, which is the proof". It is not: `tests/cpp/`
> `test_segment_fold_parity.cpp` calls `SegmentBefore::driveState`, and the
> harness failed to link. The library builds without it; the test suite does
> not. `SegmentBefore::driveState` is restored, with a comment saying why it has
> no cost-path caller and stays anyway. Grep the tests, not just the library.
That left `breaksServed()` as the sole reader of `driveAt`/`driveBefore`, called
once per route at solution export against ~133 `Route::update()` calls per
iteration -- the same dead-work shape as the removed `driveAfter`.

Disabling the whole drive-array block measured **10.18 s against 10.05 s** for
the shipped build at 1500 iterations: no gain, slightly negative, and the solve
distance did not move. The block stays. Recorded because it looked obviously
right beforehand.

### 10.4 What is left between 0.805 and 0.90

At 1500 iterations the break path must fall another **7.2%**. `duration()` is
still the whole gap: 4.52 Mcyc/it against the nobreak path's 0.88, an excess of
3.64 against a total gap of 2.16 -- everything else nets negative, because the
break path makes fewer `binaryOps` calls. Closing 0.90 means cutting ~28% more
out of `duration()`.

The tail collapse is the visible handle. It now fires on 0.286 of candidates,
and the blockers are:

    struct=0.076   remain=0.223   taken=0.240   inert=0.173

**`inert` was 0.000 at 250 iterations and is 0.173 here.** The inertness
widening recorded in `BREAK_HW_OPT.md` §5 as "does not pay" was therefore judged
in a regime where the gate it widens never binds. It is the first thing to
re-test. The same applies to `remain`, which nearly doubled (0.175 -> 0.223).

How far the tail collapse could go, as an upper bound. It walks `L_round` =
14.85 nodes and skips `sc_saved` = 2.27, so a candidate is 17.12 nodes and the
collapse currently removes **13.3%** of the evaluator's node work. Unblocking
*both* movable gates -- `inert` (0.173) and `remain` (0.223) -- would take firing
from 0.286 to 0.682. At the same 7.94 nodes saved per firing that is 5.42 of
17.12, or **31.6%**: a gain of ~18 points of node work over today.

Taking node work as a proxy for `duration()` time (an upper bound -- there is
fixed per-call overhead that does not scale with nodes), that is roughly 5% off
the break path's total, so **the ratio would land near 0.85, not 0.90** -- and
that assumes both gates go to zero, which nothing suggests is possible.

> **So 0.90 is not reachable by widening the collapse.** It needs the O(#events)
> evaluator -- skipping a stretch in O(1) -- and that still has the open blocker
> from §0.2: in 30.3% of candidates the crossing genuinely has to be located,
> and localisation was never built. That is the honest state of the go/no-go.

What *did* move the number twice now is the same shape of finding both times:
work the evaluator does that it does not have to do (heap allocations, a
redundant second round), found by measuring in the regime production runs in
rather than assuming. That is where a third look should start.

## 11. The rest of the gap is not the evaluator -- it is more search work

§10.4 said `duration()` is "the whole gap". That was read off a decomposition
that did not separate cost-per-unit-work from amount-of-work. Decomposing the
2.16 Mcyc/it difference at 1500 iterations properly:

| component | break excess |
|---|---|
| `duration()` | +3.64 |
| `Route::update` | **+1.06** |
| `unaryOps` | +0.44 |
| rest of `binaryOps` | -3.78 |

`Route::update` alone is ~49% of the gap, and **not** through per-call cost --
15 050 cycles on break against 15 969 on nobreak, so the break path's update is
slightly *cheaper*. It is purely call count: 119 per iteration against 46.6.

### 11.1 It is not phantom moves

The obvious worry was H1: the streaming evaluator's delta disagreeing with the
cost recomputed after a move is applied, so moves get accepted on stale deltas
and the search churns. `benchmarks/_hw_moves.py` accumulates the counters that
settle it across a whole solve:

| per `LocalSearch` invocation, 1500 iterations | nobreak | break |
|---|---|---|
| `parity_violations` | **0** | **0** |
| safety valve triggered | never | never |
| evaluated moves | 31 063 | 43 281 |
| improving moves (= updates) | 26.65 | **55.33** |

Zero parity violations and no valve trips in either scenario. **H1 is refuted.**
Every update corresponds to a genuinely improving move (improving == updates in
both), and the break search simply finds **2.08x more of them per invocation**.
That is real search work, not waste: removing it would change what the search
does, not how fast it does it. (55.33 improving moves x up to 2 routes each
matches the 119 `Route::update` calls.)

### 11.2 Per unit of search work, the break path is already the faster one

    break     6.33 ms/it    43 281 moves/it  ->  146.3 ns/move
    nobreak   5.10 ms/it    31 063 moves/it  ->  164.2 ns/move

    ratio, iterations/second   0.805
    ratio, ns per moved evaluated   1.122   (>1 = break is faster)

**The break path evaluates a candidate move 12% faster than the nobreak path.**
The whole of the 0.805 shortfall is that a break iteration evaluates 39% more
moves and applies 2.08x more improving ones, because break nodes are extra
activities in the route and the duty/waiting landscape is finer-grained.

This is not a claim that the goal is met. The goal is written in iterations per
second, and by that metric the number is **0.781 canonical / 0.805 at 1500
iterations**. But it does change what the remaining 19.5% *is*: it is not
evaluator overhead that better C++ can remove. Roughly half of it is the
`Route::update` and `unaryOps` cost of moves the search legitimately wants to
apply, and that half is only reachable by changing the search, not by making the
evaluator faster.

### 11.3 What that leaves

Of the 2.16 Mcyc/it gap:

- **~1.5 Mcyc/it (update + unaryOps + their share of binaryOps)** is more search
  work. Not addressable as a speed problem.
- **~3.6 Mcyc/it of `duration()` excess, offset by -3.8 elsewhere** is the
  evaluator. Widening the tail collapse to its ceiling is worth ~5% of the break
  path (§10.4), landing near 0.85 on the it/s metric.

So on the it/s metric, 0.90 needs the O(#events) evaluator, and its blocker from
§0.2 is still open. On a work-normalised metric the fork is already ahead. **The
first thing the next person should settle is which of those the business
actually cares about**, because they point at completely different work: one is
a multi-week evaluator redesign with an unsolved prerequisite, the other is
already done.

## 12. The tail collapse's ceiling is lower than §10.4 estimated

§10.4 priced widening the tail collapse at "~0.85 if both movable gates go to
zero". Two attempts at the natural proof say they do not go to zero, and the
reason is structural rather than a matter of effort.

### 12.1 What the instance actually looks like

Measured on the converged solution (18 non-empty routes, 1500 iterations):

    clients per route          20.44
    breaks SERVED per route     0.17     (about 3 across all 18 routes)
    rules per vehicle              5     (2 mandatory DUTY_TIME at 43 200 s,
                                          3 optional DUTY_TIME at 21 600 s
                                          with condition_min_route_s 21 600)
    route duration        min 4 348   median 33 454   max 43 187
    routes reaching 21 600 s      15 of 18
    routes reaching 43 200 s       0 of 18

So the two **mandatory** rules never become due -- the longest route stops 13
seconds short of their trigger -- and the optional ones become due on 15 of 18
routes but are almost never served, which costs nothing because they carry no
D3 term. The whole break machinery runs on every candidate and, in this
instance, produces almost nothing.

That is what suggested the lever: if a rule provably cannot fire, the tail can
be collapsed even when the masks say otherwise.

### 12.2 The proof, and why the bound is too loose

Every accumulator (drive, work, duty) is elapsed time since the last reset, so
all three are bounded by `endClock - lastResetAt_`, and all are non-decreasing
while nothing fires. Folding the route's cached `durAfter` gives that end clock
in one merge. If the bound is below every unsettled rule's trigger then nothing
fires -- self-consistent rather than circular, since firing is the only thing
that could push the clock higher, so the walk follows exactly the no-fire
trajectory.

Built and measured. It is exact -- distance 4 896 741 unchanged -- and it is
useless:

    clock-inert: tried=2.197  hit=0.062     (per candidate)

Tried 2.2 times per candidate, each attempt paying a speculative
`DurationSegment::merge`, and settling 6%. The bound is `endClock -
lastResetAt`, which is essentially the whole route's duration -- a median of
33 454 s against triggers at 21 600. **The rules are reachable; they are simply
not served.** Only routes shorter than the smallest trigger can pass, and there
are three of them. A useful bound would need the actual duty at the tail's
start, and getting that is the walk.

### 12.3 And dropping `remainingMask == 0` is not available

The first version of the same idea also dropped the "no break node ahead" gate,
on the reasoning that a break which cannot fire cannot be served either. That is
true of the proposal's own verdicts but not of the cache it folds: `durAfter`
carries the **route's** verdicts, and a break node ahead that the route served
(with its D5 extension and window clearing) is a different singleton from the
one the proposal would fold. The search diverged badly enough to grind against
the move cap -- a 1500-iteration solve that normally takes 11 s had not finished
after 25 minutes. `remainingMask == 0` is load-bearing, not inherited.

### 12.4 Revised ceiling

§10.4's estimate assumed the movable gates could be opened. The natural proof
opens 6% of them, so the realistic gain from this direction is under a point of
ratio, not the ~5 that estimate implied. **Widening the tail collapse is not a
route to 0.90.** Both reverted; recorded because both looked right beforehand,
and the second one was exact -- just worthless.

## 13. The per-rule loop: measured, and its cheap exits are already cheap

`duration()` costs ~1014 cycles per call on the break path against ~167 on the
nobreak path, over ~14.85 simulated nodes. The suspect for that 6x was the loop
in `DriveSegment::merge` that evaluates every configured break rule at every
node boundary. It was instrumented (counters under `PYVRP_STREAM_STATS`):

    [rule-stats] iters=284 783 209  settled=0.385 cond=0.286
                 reached-trigger=0.329 of-which-no-fire=0.106

**42.3 rule iterations per `duration()` call** — 2.85 per node, since vehicle
types differ in how many rules they carry. Where they exit:

| exit | share |
|---|---|
| settled (taken, with a first-due clock recorded) | 38.5% |
| `conditionMinRouteS` not met | 28.6% |
| reached the trigger test | 32.9% |

A rule that fires becomes taken *and* settled, so it is skipped ever after. The
permanent cost is the rules that never fire — here the two mandatory ones, whose
43 200 s trigger the longest route misses by 13 seconds.

### 13.1 Removing the loop is not measurable this way

The obvious ceiling experiment — skip the loop entirely and time it — is
useless: the results change, the search follows a different trajectory, and the
solve took 27.4 s against 9.5 s. It measures a different search, not the loop.

### 13.2 Turning the most-taken exit into a register test buys nothing

The settled exit (38.5% of iterations) reached its verdict through a dependent
load into `firstDueClock[rule.id]`. Replacing that with a bitmask the caller
maintains — exact, distance unchanged — measured **0.9998 (2/5 pairs)**. Nothing.

The reason is the earlier compute-bound finding restated: `firstDueClock` is a
handful of bytes, permanently in L1, so the "dependent load" was free, and the
extra branch cancels whatever was saved.

That also condemns the companion idea (partition rules by
`conditionMinRouteS` and break out early, targeting the 28.6% exit): it removes
the same kind of already-cheap branch. **The loop's cheap exits are already
cheap.** Its cost sits in the 32.9% that reach the trigger test and do real
work, and that fraction is not compressible by rearranging the loop.

### 13.3 What this closes

Reverted; the counters stay, since they are what make the loop's shape legible.
Together with §12 this exhausts the evaluator levers that do not change the
algorithm:

| lever | verdict |
|---|---|
| round 2 (decide before clamping) | **shipped, +8.0%** |
| drive fold in `Route::update` | no gain (10.18 s vs 10.05 s) |
| phantom moves / parity churn | none exist (0 violations) |
| tail collapse, dropping `remainingMask == 0` | unsound (search diverged) |
| tail collapse, end-clock inertness proof | exact, 6% hit rate |
| rule loop, settled exit as a bitmask | 0.9998 (2/5) |

What remains is the O(#events) evaluator: stop visiting every node. Its blocker
from §0.2 is unchanged — in 30.3% of candidates the crossing has to be located,
and for DUTY_TIME triggers duty depends on waiting, which depends on the
schedule, which is the walk. That is the problem to solve, and nothing smaller
substitutes for it.

## 14. The gap in one line: 8.5x the merges, at 0.7x the cost each

Every micro-optimisation in §13 measured zero, and the same decomposition
explains all of them at once.

| | nobreak | break |
|---|---|---|
| `duration()` cycles per call | 167 | 1014 |
| merges per call | ~3.5 | ~29.7 |
| **cycles per merge** | **47.7** | **34.1** |

The nobreak path folds the proposal's **cached segment** folds: a handful of
`DurationSegment::merge` calls, one per segment descriptor. The break path walks
**nodes**, and each node costs one `DurationSegment::merge` plus one
`DriveSegment::merge`.

> **Per merge the break path is already 1.40x cheaper than the nobreak path. It
> simply performs 8.5x as many.** The entire gap is merge *count* -- O(nodes)
> against O(segments) -- and nothing that shaves cycles off an individual merge
> can touch it.

**Correction, measured afterwards: "merge" is the wrong unit.** Timing the two
merges directly (rdtsc phases around each, overhead ~45 cycles per pair) gives
`DurationSegment::merge` at 25.1 and `DriveSegment::merge` at 49.7 cycles
measured -- i.e. roughly **free** and **~5 cycles** once the instrument's own
cost is subtracted. Together they are about 10 of the ~80 cycles a node costs.

So the break path is not paying for merges. It is paying for the **loop body**
that surrounds them: walking the segment descriptors, the duration-matrix
lookup, building the node singleton out of `ProblemData`, the tail-collapse gate
evaluated at every node, and the mask/counter bookkeeping. None of it is
individually dominant -- attempts to shave each piece are the refutations in
§13, all inside the noise floor.

The conclusion survives the correction and is sharper for it: **the nobreak path
executes ~48 cycles per SEGMENT; the break path executes ~80 cycles per NODE.**
The unit of work is the difference, not the cost of any operation inside it.
Finer instrumentation cannot resolve further -- at this scale rdtsc is two
thirds of the measurement.

That is why the settled-exit bitmask, the upcoming skip, the rule-loop exit
profile and the drive-fold removal all landed inside the noise floor: each
attacks the per-merge cost, which is the half of the comparison the break path
already wins.

It also settles which half of the original thesis in
`STRUCTURAL_BREAK_EVAL_REDESIGN.md` survived. "Only a regime redesign can help"
was wrong -- constant-factor work took the ratio from 0.549 to 0.781, +40.8% on
the break path. "The cost is O(nodes) and structural" is **right**, and this
table is the cleanest evidence of it produced so far.

### 14.1 Why the O(#events) evaluator is hard, stated precisely

Folding a break-free stretch in one merge instead of walking it needs the
**drive** state at the far end, not just the duration fold. `DriveSegment::merge`
is not composable from cached pieces: whether a rule fires depends on the
absolute schedule clock at each boundary, not on accumulated deltas. That is the
same fact Decisão 2 found from the other side (the cached monoid fold diverges
on 53 of 174 proposals, and is inadmissible as a bound at 0.88% violations).

The escape is to prove nothing fires in the stretch, which makes the fold
composable. §12 built that proof and measured a 6% hit rate, because with 86.8%
of candidates carrying no break node in the region they re-evaluate, "the
stretch" is the whole remaining walk and the bound is the whole route's
duration.

Which leaves locating the crossing and splitting the stretch there. Duty is
monotone along a stretch, so the crossing is binary-searchable in O(log n) --
**if** the fold of an arbitrary interior range is available in O(1). PyVRP
caches prefix (`durBefore`) and suffix (`durAfter`) folds, not arbitrary ranges;
an interior range needs a sparse table or equivalent, rebuilt in each of the
~133 `Route::update()` calls per iteration. At ~15 nodes, the walk is 30 merges
and the binary-searched version is roughly 3 crossings x 4 probes plus the
folds, i.e. the same order -- before paying for the structure that makes the
probes O(1).

**That is the wall, stated in terms that can be argued with.** It is not "the
evaluator is slow"; it is that O(1) access to interior range folds costs more to
maintain than the walk it would replace, at this route length. The scaling data
in §4 is what would change that verdict: the walk grows with route length while
the number of crossings does not.

## 15. The long-route profile the go/no-go was waiting on

`design.md` made profiling the break side on a long-route instance the explicit
prerequisite for opening Fase A: the ratio collapse in §4 says the problem is
real there, but nothing said the decomposition addresses it. That profile had
never been run. It has now.

Synthetic instance (`benchmarks/_hw_scale.py`, 120 clients, one mandatory
DUTY_TIME rule), 5 vehicles so routes reach 123 activities, against group-54's
~23:

| | group-54 (~23 nodes) | synthetic (123 nodes) |
|---|---|---|
| `duration()` share of break total | 26% | **62.9%** |
| `duration()` cycles per call | 1014 | 1048.7 |
| `L_round` (nodes walked) | 14.85 | **41.5** |
| tail collapse fires | 0.286 | **0.788** |
| nodes saved when it fires | 7.9 | **22.8** |
| `seeded` | 0.86 | 0.947 |
| `round2_f` | 0.006 | 0.000 |
| collapse blockers (struct/remain/taken/inert) | .076/.223/.240/.173 | **.093/.012/.019/.079** |
| break/nobreak ratio | 0.809 | 0.056 |

### 15.1 The thesis holds on long routes, and now there is a profile behind it

`duration()` goes from a quarter of the break path's time to **nearly two
thirds**, and the walked node count nearly triples. That is the O(n) term
dominating, which is exactly what `STRUCTURAL_BREAK_EVAL_REDESIGN.md` claimed
and what §4 could only infer from the ratio. **The "GO hipotético" in
`design.md` item 4 can be upgraded: the mechanism is now measured, not
extrapolated.**

### 15.2 The existing machinery already scales — which narrows the work

Three of the four things this branch shipped behave *better* on long routes, not
worse:

- The **tail collapse** fires on 78.8% of candidates and saves 22.8 nodes each,
  against 28.6% and 7.9 on group-54. Its blockers nearly vanish: the largest is
  `struct` at 0.093, and `taken`/`remain` — the two that dominate on group-54 —
  drop to 0.019 and 0.012.
- The **prefix seed** covers 94.7% of candidates, against 86%.
- **Round 2 never fires at all** (`round2_f` = 0.000), so the fix in §10 buys
  nothing here. It is a group-54 win, and harmless elsewhere.

So Fase A does not start from zero on long routes: the suffix is already
collapsed on four candidates in five, and the prefix is already seeded on
nineteen in twenty. **What is left is the middle**, and that is precisely the
O(#events) evaluator's target.

### 15.3 What this does not say

One rule, one seed, one synthetic instance, no capacity or time-window
pressure — group-54 carries five rules and real windows. Per-node cost differs
accordingly (25.3 cycles here against 68.3 on group-54), so the two are not on a
common curve and this profile cannot be used to predict group-54 or vice versa.
It answers one question only, and it is the question the go/no-go was blocked
on: **on long routes the break path's cost really is concentrated in walking
nodes, and the machinery that avoids walking them already works there.**

The remaining prerequisite from §0.2 is unchanged — locating the crossing — but
its value is now bounded from below by a measurement rather than a guess.

## 16. Crossing localisation is NOT blocked — the identity makes it O(1)

Every document in this investigation records crossing localisation as the
unsolved prerequisite for the O(#events) evaluator, and §14.1 gave the reason:
locating where a trigger fires needs the fold of an arbitrary interior range,
which PyVRP does not cache and which would cost more to maintain than the walk
it replaces. **That reasoning is wrong, and the identity this branch already
documented is what breaks it.**

### 16.1 The unrolling

The evaluator's clock obeys, at every node,

    atSecond(i) = max( S(i-1) + edge(i) + setup(i),  twEarly(i) )
    S(i)        = atSecond(i) + service(i)

with `S = duration() + startEarly()`. Unrolling the recurrence over a stretch
`[q, m]` gives a **max-plus** form:

    atSecond(m) = max( S(q-1) + cum(q..m),
                       max_{j in (q,m]} ( twEarly(j) + cum(j..m) ) )

where `cum(a..b)` is the travel plus service between them. The first term is the
only one carrying the prefix. **The second term is a pure function of the route
suffix** -- it does not depend on what the proposal did before `q` at all.

So it can be cached per route in one backward O(n) pass in `Route::update()`,
and then `atSecond(m)` costs **O(1) for any m**: one prefix-sum difference, one
array read, one max. No interior range fold, no sparse table.

And `atSecond` is non-decreasing in `m`, so the first `m` at which
`atSecond(m) - lastResetAt >= trigger` -- the crossing -- is a **binary search**,
O(log n) per crossing, over quantities that are O(1) each.

### 16.2 Validated, not argued

The unrolling follows by induction from the local identity, so the local
identity is what needs checking. Instrumented in `Route::update()` under
`PYVRP_STREAM_STATS`, comparing `S(i)` against `atSecond(i) + service(i)` at
every node of every route update on group-54:

    [identity] checked=374 744  warp=36 219 (0.097)  mismatch=0 (0.000000)

**Zero mismatches over 374 744 checks.** The 9.7% carrying time warp are
excluded, which is exactly the condition the identity is stated under -- warp is
detected per node as `S(i-1) + edge > startLate(i)`, the merge's own `diffTw`,
so a localiser knows precisely when it must fall back to walking.

### 16.3 What this changes

The prerequisite in §0.2 -- "in 30.3% of candidates the crossing genuinely has
to be located" -- is no longer a blocker. Locating it costs one cached array
(built in the same `update()` pass that already computes `atSecondVec`) plus a
binary search per crossing.

That reopens the two tail-collapse gates §12 could not open. `taken` (0.240) and
`inert` (0.173) both fail because a rule's verdict in the tail is unknown;
localisation computes it exactly, including the `firstDue` clock the D3 term
needs. Together they are 0.413 of candidates against the 0.286 that collapse
today.

Sizing it from the measured numbers: a candidate is 17.12 nodes, the collapse
saves 7.9 when it fires, and it fires on 0.286. At 0.70 firing the saving goes
from 2.27 to 5.53 nodes, so the walk drops from 14.85 to ~11.6 -- about 22%
fewer nodes, worth roughly 5-6% of the break path, i.e. **ratio 0.809 to about
0.85**. Removing the walk entirely (the full O(#events) evaluator, which the
same primitive enables for the 86.8% of candidates whose re-evaluated region has
no break node at all) is worth substantially more.

**This is the first time in this investigation that the structural path has a
concrete, validated primitive rather than an open question.** It is also where
this session stops: building it is real surgery in the code that has produced
wrong distances more than once, and it deserves its own run with the differential
harness in place from the start.

## 17. The localiser was built. Time warp is the real blocker, not localisation.

§16 validated the primitive and stopped there. It was then built, and the
result relocates the obstacle.

### 17.1 What was built

`Route::update()` now fills two tables (`cumT_`, `cumM_`) and exposes
`clockAt(q, clockQ, m)`, which returns the arrival clock at any position from
the clock at any earlier one in O(1). On top of that, a collapse path that
binary-searches each unsettled DUTY_TIME rule's first crossing, records the
`firstDue` clock D3 needs, and folds the tail onto the cached `durAfter` even
when the mask gates refuse -- which is the case on exactly the vehicles that
carry mandatory ALL_TIMERS rules, whose bits are never set because those rules
never fire.

The tables cost nothing measurable: **1.0041 over 4 interleaved pairs** against
the build without them.

### 17.2 The differential harness earned its keep immediately

The first version passed the distance gate (4 896 741) and passed
`test_stream_parity` 500/500 -- and was wrong. The tell was the evaluation
count: 1 928 891 against 1 598 411 with the localiser disabled, a **21%**
difference on a change that must be semantically neutral.

So the localiser was made to record its answer and let the walk run on anyway,
comparing the two afterwards. That found three separate defects, in order:

1. **A rule already due but not taken fires at the very first tail boundary**,
   reset and all. Skipping rules whose clock was already recorded missed it.
2. **`cumM_` is a prefix max from position 0.** The claim in §16 that
   `atSecond(q)` dominates earlier clamps holds for the *route's* trajectory,
   not for a proposal that reaches `q` **earlier** than the route did -- it
   would inherit clamps its own prefix never saw. Fixed by requiring
   `clockQ >= ownClockAt(q)`. `arrivalEnd` had been wrong on 45% of cases.
3. **Time warp.** The guard only excluded warp *added by the tail*; making it
   exclude warp outright fixed the remaining mismatches. The stream's clock is
   `duration() + startEarly()` (warp not subtracted) while
   `DurationSegment::merge` computes its own arrival as
   `duration_ - timeWarp_ + edge`, so the two part company once warp appears.

   > **Correction, measured afterwards.** "Any warp breaks the identity" is
   > what the fix implied, and it is **false**. The validation had *excluded*
   > warped nodes rather than testing them. Testing them:
   >
   >     [identity] under-warp: checked=36 219  mismatch=522   (1.44%)
   >     [clock]    under-warp: checked=36 219  mismatch=1 193 (3.29%)
   >
   > The identity holds on **98.6%** of warped nodes and the clock table on
   > **96.7%**. `timeWarp == 0` is therefore a far heavier guard than the
   > mathematics requires -- and it is what drove the hit rate to 1.7%.

With all three fixed the localiser is exact on everything the harness compares
except one residue: `dur=0 warp=0 due=0 end=0`, `first` differing on 9.6%. That
last one is understood -- `S(m) = clock(m) + dutyTime_(m)` is **not** monotone
(the end depot carries zero drive-side duty against a non-zero duration-side
service), so the binary search is not valid at that node.

### 17.3 And then the hit rate collapsed

    localiser: tried=0.412  hit=0.017        (1500 iterations)

**1.7%.** The warp-free precondition is what does it: 9.7% of nodes carry time
warp, and in the converged regime almost every candidate's fold has some.

> **This relocates the obstacle, and the correction above sharpens where to
> push.** Locating a crossing is solved -- O(1) probes, binary search,
> validated. The formula is also very nearly right under warp (96.7% of warped
> nodes). What actually cost the hit rate is the *guard*: `timeWarp == 0` is
> a blunt instrument standing in for a condition that bites on ~3% of warped
> nodes.
>
> **The next attempt should replace the guard, not the formula.** The cheapest
> candidate is an O(1) endpoint verification: the merged fold already gives the
> exact final clock, so comparing it against
> `clockAt(q0, clockQ, lastPos) + service(lastPos)` catches a table that did not
> reproduce this fold, at the cost of one comparison. Allowing warp in the
> prefix while checking the endpoint is what turns 1.7% back into a useful
> fraction — measured with the differential harness, which is already in place
> and which caught every defect above.

Reverted, keeping the tables, `clockAt`/`ownClockAt`, and both validations --
the primitive is correct and the next implementation needs it. What is gone is
the collapse path that used it, because 1.7% does not pay for a speculative
merge plus a binary search per candidate.

**Do not repeat these without the differential harness.** The distance gate and
`test_stream_parity` both passed on a version that changed a fifth of the
evaluations.

## 18. The localiser, shipped: neutral at 23 nodes, +4.0% at 123

§17's correction said to replace the guard, not the formula. Done, and the
result is the first structural win in this investigation.

### 18.1 The three changes

- **Endpoint check instead of a warp ban.** The merged fold is exact, so it is
  asked to confirm the tables reproduced it: `clockAt(q0, clockQ, lastPos) +
  service(lastPos)` against `merged.duration() + merged.startEarly()`. One
  comparison. It rejects 4.5% of candidates -- the ones the blunt guard was
  standing in for.
- **Anchor bound.** `clockQ >= ownClockAt(q0)`, because `cumM_` is a prefix max
  from position 0 and a proposal arriving earlier than the route would inherit
  clamps its own prefix never saw.
- **Monotone search range.** `S(m) = clock(m) + dutyTime_(m)` is searched over
  the clients only; the end depot carries zero drive-side duty against a
  non-zero duration-side service and is tested directly.

### 18.2 Exact where it counts, and the harness says so

Differential mode, 1500 iterations, **895 551 comparisons**:

    loc: tried=0.412  hit=0.136  endpoint-reject=0.045
    loc-diff: dur=0  warp=0  due=0  first=10 914 (mandatory=0)  end=0

Everything that reaches the returned value is exact. The `first` residue is
**entirely on non-mandatory rules**, which the D3 tail skips -- and the shipped
path now only writes mandatory clocks, so that is an invariant of the code
rather than a property to re-derive.

Coverage went from 1.7% under the warp ban to **13.6%**.

### 18.3 What it is worth, and where

| instance | paired A/B | distance |
|---|---|---|
| group-54, ~23-node routes, 5 pairs | **1.0000** (2/5) | 4 896 741 both |
| synthetic, 123-node routes, 3 pairs | **1.0397** (3/3) | 1 219 572 both |

**Neutral where the route is short, +4.0% where it is long.** That is exactly
the shape §4 and §15 predicted and nothing had yet demonstrated: the walk grows
with route length while the number of crossings does not, so machinery that
replaces walking with locating only pays once there is enough walking to
replace.

It ships on that basis -- no measurable cost where it does not help, a
consistent gain (3 of 3 pairs) where it does.

### 18.4 What this does and does not settle

It does **not** move the goal on group-54: the ratio there is unchanged at
0.819-0.839, and 0.90 is not reached. What it settles is the open question the
whole investigation carried: **the structural approach works, is exactly
implementable, and its payoff scales with route length.** The first instance of
it is now in the branch with a differential harness around it.

The remaining coverage limits, in order of size: candidates where the mask gates
already pass (the localiser defers to the existing collapse), the 4.5% the
endpoint check rejects, and rules whose firing carries a reset (deferred to the
walk). Widening any of them is incremental work on a path that is now open
rather than blocked.
