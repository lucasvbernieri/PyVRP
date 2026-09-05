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

## 19. Where the localiser's coverage goes, and what is left to widen it

§18 shipped it at 13.6% coverage on group-54 and 5.3% on long routes. Rejection
counters now say where the rest goes. Of the 0.412 of candidates it is tried on
(1500 iterations, group-54):

| outcome | share of candidates |
|---|---|
| **hit** | **0.136** |
| structural — tail too short, or a non-DUTY_TIME rule | 0.061 |
| endpoint check | 0.045 |
| anchor bound (`clockQ < ownClockAt(q0)`) | 0.028 |
| a rule whose firing carries a reset | 0.024 + ~0.118 |

Two things this settles, both against a guess:

- **The anchor bound is not the bottleneck.** It was the obvious suspect --
  `cumM_` is a prefix max, and lifting it to a proper range max needs a sparse
  table, O(n log n) per `Route::update()`. It is worth **2.8%** of candidates.
  Not worth the structure.
- **Reset handling is the bottleneck**, at ~0.14 combined. A rule that fires in
  the tail and carries a reset is deferred to the walk, and on this instance
  those are the two mandatory ALL_TIMERS rules. Modelling them means
  reproducing the documented trap -- ALL_TIMERS *replaces* `takenMask` rather
  than adding to it, so a fire rewrites the state every later rule reads.

### 19.1 Two structural changes that measured as no-ops

Both looked right and both changed nothing, byte for byte:

- **Running the localiser after the mask-gated collapse instead of before it**,
  so it sees candidates the masks pass but the inertness check refuses (`inert`
  blocks 0.173 on group-54, 0.079 on long routes). Identical `tried`/`hit`:
  those candidates were already being reached at an earlier node.
- **Allowing up to three attempts per candidate instead of one.** The crossings
  are absolute but the guards are not, so a later anchor could succeed where an
  early one failed. Identical again.

Both reverted. Recorded because the reasoning behind each still looks sound --
the measurement is what says otherwise.

### 19.2 The honest ceiling from here

On group-54 the localiser is already neutral (1.0000 over 5 pairs), so widening
its coverage cannot move that ratio; the route is too short for the machinery to
matter. On long routes it is +4.0% at 5.3% coverage, and the reset work is what
would raise that -- worth roughly doubling it, on the arithmetic above.

Neither path reaches 0.90 on group-54. What they do is make the long-route case
better, which is where §15's profile says the structural work belongs.

## 20. The go/no-go measurement: production routes are 15-24 activities

Every version of this document ended by handing back the same question -- the
real distribution of route lengths in production -- because §4 and §15 make the
structural work pay only on long routes. The data was in the repository.

`hows-router/optimization_exports/` holds 381 exports; two carry a solved
`response.json`, and one of them is the break-configured instance this whole
investigation benchmarks:

| export | routes | activities per route | >= 40 | >= 60 |
|---|---|---|---|---|
| `32d73c97…` (with breaks) | 9 | min 7, **median 15**, max 34 | 0 | 0 |
| `7c79b458…` | 27 | min 3, **median 24**, max 41 | 1 of 27 | 0 |

Step types on the break instance: 9 start, 140 job, **17 break**, 9 end.

> **Routes of 60+ activities do not occur.** The median is 15-24 and the
> longest route seen is 41. The regime where the O(#events) evaluator pays
> (measured +4.0% at 123 nodes, neutral at 23) **is not present in this
> production data.**

### 20.1 What that decides

The no-go on group-54 is the final answer, not a provisional one. Fase A is not
worth opening: the machinery it would build has been shown to be neutral at the
route lengths production actually runs, and the profile that made it look
attractive (§15) belongs to a synthetic instance three to eight times longer
than anything here.

What ships instead is the constant-factor work — **0.549 -> 0.794 canonical, the
break path from 127.6 to 157.9-179.6 it/s** depending on the session — plus the
localiser, which costs nothing where it does not help and is there if route
lengths ever grow.

### 20.2 Caveat, stated plainly

Two solved responses, not 381: the other exports carry the input model but no
solution, and 135 of 143 model pickles fail to unpickle against the current
build. Both available responses agree, and one is the instance the goal is
measured on, but this is a sample of two. If a tenant with genuinely long routes
exists, it is not in these exports — checking `route_optimizations` directly
would settle it beyond doubt.

## 21. The interior jump: built, made exact, and it still does not pay

With clockAt() locating a crossing in O(1) probes and foldRange() folding an
interior range in O(log n), the O(#events) evaluator's two halves both existed.
This wires them together: at the first walked node, binary-search the first
position where any unsettled DUTY_TIME rule reaches its trigger, fold everything
before it with the tree, and land the walk directly on it.

### 21.1 Two defects, and which tool caught each

- **The differential harness reported zero differences on a version that was
  wrong**, because the harness itself assigned `prevAct`/`prevLoc`/`arrivalCur`
  before splitting and never restored them: it compared the jump against a walk
  it had corrupted. Fixed by computing the jump's answer into locals and
  touching the walk only on the real path. The tell was the evaluation count —
  the differential build had to reproduce the no-jump build's 1 047 576 calls
  exactly, and once it did, the comparison meant something.
- **The distance gate caught what the differential harness could not.** With no
  crossing anywhere, `stop` lands on the last position, and a jump that consumes
  it skips the iteration that assigns `arrivalEnd` — which the D3 tail then
  reads as zero. Every field compared at `stop` still matched; the damage was
  downstream. Distance moved 896 884 -> 916 148. Capping `stop` at `lastPos - 1`
  fixes it.

After both, the jump is exact across everything comparable: **105 559 checks,
zero differences** in duration, time warp, startEarly, startLate, drive, work,
duty, clock, dueMask, firstDue and takenMask — and the solve distance is back to
896 884.

### 21.2 The measurement

| variant | production long-route A/B | hit rate | nodes skipped |
|---|---|---|---|
| warp-free guard | 0.9961 | 7.9% | 2.50 |
| fold-consistency guard | **0.9783** | 10.3% | 3.20 |

Replacing the warp ban with a consistency check (the tree's fold of the tail
against the route's cached `durAfter`) raised coverage, as it did for the
localiser — and made the result *worse*. Skipping 3.2 of ~48 walked nodes is
6.7% of the walk, and it does not cover an O(log n) fold plus a consistency
check plus a binary search per rule, paid on the 14.7% of candidates the jump is
tried on.

### 21.3 What that settles

The structural evaluator is not blocked by anything conceptual any more. Both
primitives exist, both are validated, and an implementation on top of them is
exact. It simply loses to walking at the route lengths that exist: 48 nodes is
not enough walk to amortise O(log n) machinery over.

Reverted. The tree stays — it is validated, it costs nothing at the 60-node
threshold, and it is what any future attempt needs. What that attempt would have
to change is the ratio between what a jump skips and what it costs: either much
higher coverage (the guards, again) or much longer routes than the 72 production
tops out at.

## 22. The ceiling on the production case: 0.684, and 0.90 is arithmetically out

Profiling BOTH arms of the group-190 instance — which no earlier section did —
closes the question the whole investigation was chasing.

Per `LocalSearch::operator()` at 100 iterations:

| | nobreak | break | ratio |
|---|---|---|---|
| total | 6.02 Mcyc | 21.36 Mcyc | 3.55x |
| `duration()` calls | 8 396 | 10 372 | 1.24x |
| `duration()` cycles/call | **210.9** | **1382.7** | 6.6x |
| `duration()` share | 29.4% | 67.1% | |
| `binaryOps` calls | 2 137 | 2 605 | 1.22x |
| `binaryOps` cycles/call | 2 651 | 7 672 | 2.89x |
| `Route::update` cycles/call | 5 411 | 20 662 | 3.82x |

Decomposed, in Mcyc per iteration:

    duration()                nobreak 1.77   break 14.34   excess 12.57
    binaryOps minus duration          3.90         5.65           1.75
    Route::update                     0.18         0.81           0.63
    TOTAL                             6.02        21.36          15.35

### 22.1 The ceiling

`duration()` is 82% of the gap, exactly as on group-54 — but here it is 67% of
the break path's own time rather than 26%, which is why the ratio is 0.28 and not
0.79.

Suppose the evaluator became perfect: `duration()` on the break path costs what
it costs on the nobreak path, which is what an O(1) fold of cached segments
costs. The break path drops to **8.79 Mcyc/it and the ratio reaches 0.684.**

For 0.90 the break arm would have to shed **14.68 Mcyc/it**, and the entire
`duration()` excess is **12.57**. **The target is arithmetically out of reach by
evaluator work alone on this instance**, whatever the evaluator does. Closing the
rest means removing the extra `binaryOps` and `Route::update` work — and that is
the break arm evaluating 22% more moves and applying more of them, which is
search behaviour, not overhead. §11 measured the same thing on group-54: zero
parity violations, every update a genuine improvement.

### 22.2 What is actually reachable

The 25 nodes a candidate still walks here (73 flat, 24.5 skipped by the prefix
seed, 23 by the tail collapse) cost ~55 cycles each — the same per-node cost the
nobreak path pays per *segment* merge. An interior evaluator that folded all 25
would take the ratio from 0.28 toward 0.68. §21's jump does exactly that when it
fires, skipping ~31 nodes, and it fires on 10.3% of candidates for a net 2.2%
loss — the per-attempt cost of the guards, not the idea, is what beats it.

So the honest statement of what remains:

- **0.90 on the production long-route case: not reachable**, and the arithmetic
  above says so without needing another experiment.
- **0.68 on that case: reachable in principle**, by raising the interior jump's
  coverage from 10% toward saturation. Every attempt so far to widen a guard has
  cost either exactness or more than it bought, and that is where the next
  effort belongs.
- **On the typical route (median 25 activities), the ratio is 0.79-0.84**, and
  the same arithmetic there leaves 0.90 much closer — but the levers that would
  close it have all measured neutral, because 23-node routes are too short for
  the machinery to amortise.

## 23. Why the interior jump breaks even, and what that closes

§21 measured the jump at 10.3% coverage and a 2.2% loss. Rejection counters say
where the coverage went, and the answer moves the design rather than the tuning.

### 23.1 The walked nodes are not in the suffix

    jump-gate: ok=0.148  no-idx=0.842  (everything else 0.000)

**84% of candidates end their walk before passing the first node of the final
descriptor.** Cross-referenced with the walk's own numbers -- the prefix seed
starts it at position 25, the tail collapse ends it at 50 -- the 25 walked nodes
sit in the descriptors *before* the last one.

That is a single-route instance: the moves that dominate reorder large spans
(SwapTails and the perturbation), so the proposal is several long route ranges
rather than a prefix, a moved node and a suffix. Restricting the jump to the
final descriptor aimed it at the part of the walk that was already gone.

### 23.2 Generalising it works, and still breaks even

A descriptor does not have to be the suffix to be foldable -- it only has to be
a contiguous range of the same route, which those spans are. Generalised:

    tried  0.147 -> 0.939        hit  0.103 -> 0.130
    rejections: short=0.505  anchor=0.252  too-near=0.052  fold/break-in=0.000

and the evaluation count stays at 1 047 576, exactly the no-jump build's.

Measured on the production long-route A/B: **1.0020** — neutral, distance
identical at 896 884.

The economics are now legible. The jump pays a `foldRange` (~7 merges), a binary
search per rule, and its guards on **94%** of candidates, against skipping 2.47
of the 25 nodes walked (10%). At ~820 cycles per `duration()` call the overhead
and the saving are the same size.

Half of what is left is descriptors shorter than the 9-node minimum, which no
amount of work fixes. The other quarter is the anchor bound, liftable with a
sparse table over `twEarly(j) - cumT_[j]` (that max IS idempotent, unlike the
duration fold, so O(1) queries are available for ~4 KB and ~500 ops per update).
It would take the hit rate to roughly 0.38 and the skip to ~28% of the walk —
against the same overhead. It does not obviously flip the sign.

### 23.3 What this closes

Combined with §22's ceiling, the picture is complete and consistent:

- **0.90 on the production long-route case needs two things, not one.**
  A perfect O(1) evaluator alone lands at 0.684. Adding `Route::update`'s excess
  gives 0.737, and adding `binaryOps`' gives **0.938**. So the target is
  reachable in principle — an earlier version of this section said
  "arithmetically out", which was overstated: it is out *by evaluator work
  alone*.

  The second half is evaluation volume, and it does not come from problem size:
  the break route carries 73 activities against 72 (1.01x) while applying 18%
  more moves per iteration (3 956 against 3 362) and testing 22% more pairs
  (2 605 against 2 137). §11 measured those moves to be genuine improvements
  with zero parity violations, so removing them means changing what the search
  accepts — a quality trade, not an overhead removal. Whether a finer-grained
  invalidation in the evaluation filter can cut the *testing* without touching
  the *accepting* is the open question, and it is a different problem from the
  evaluator.
- **The structural evaluator is built, exact, and breaks even.** Both primitives
  are validated (clock O(1), 0 mismatches in 338 525; range fold O(log n), 0 in
  15 780), an implementation on top of them is exact across eleven compared
  fields, and at 73-node routes with 25 walked nodes it costs what it saves.
- **What actually moved the number was volume, not asymptotics**: removing
  per-candidate heap traffic, a persistent evaluation filter, an admissible
  lower bound, and eliminating a redundant second pass — +40.8% on the break
  path, none of it structural.

Reverted. The tree and the clock tables stay: validated, free at the 60-node
threshold, and the only things a future attempt would not have to rebuild.

## 24. Last measurement: the drive fold, re-tested where it should have mattered

§10.3 refuted the drive-array block in `Route::update()` on group-54, whose
routes are ~23 nodes. On group 190 the route is 73 and `Route::update` costs
20 662 cycles per call against the nobreak path's 5 411, so the same block was
worth re-testing where it is three times bigger.

Disabling it measures **0.9504** — slower, not faster. But the experiment is
confounded: the localiser gates on `r->driveAt.has_value()`, so removing the
block also removes the localiser and its +6.0%. The number is that loss, not the
block's cost.

Left as is. The `Route::update` excess is 0.63 of the 15.35 Mcyc/it gap — 4% —
so even a clean win there moves the ratio from 0.684 to 0.737 in the ceiling
arithmetic, and nothing suggests the block is most of it.

### 24.1 Where this investigation ends

Fifteen hypotheses, twelve refuted with a number. What moved the ratio:

| | |
|---|---|
| per-candidate heap allocations removed | +37% (unpinned; ~1.2-1.33x pinned) |
| persistent evaluation-volume filter | +3.0% |
| admissible lower bound before `duration()` | +11.2% |
| dead `driveAfter` fold removed | +1.3% |
| round 2 eliminated (decide before clamping) | +8.0% |
| crossing localiser | +6.0% on production long routes, neutral elsewhere |

**0.549 -> 0.794 canonical; the break path 127.6 -> 179.6 it/s, +40.8%.**

What does not move it, each with the measurement that says so: the regime
decomposition at 23-node routes (0.974-0.989), a cached monoid fold as a bound
(inadmissible, 0.88% violations), a tighter cross-route bound (0.99), aborting
the pass incrementally (0.982), software prefetch (0.997), the cached `durAt`
singleton (0.973), sequential `activitiesAt_` reads (1.001), cached duration
edges (0.990), the drive fold in `update()` (twice), the settled-exit bitmask
(0.9998), the upcoming skip (1.0043), the end-clock inertness proof (6% hit),
and the interior jump (1.0020 at 94% coverage).

And the target itself: **0.90 is not reachable by evaluator work alone.** A
perfect O(1) evaluator lands at 0.684 on the production case; the rest is
evaluation volume, which is a filter problem and a quality trade, not an
overhead removal.

## 25. Correction: the work-normalised framing does not survive the production case

§11.2 measured group-54 and found the break path **faster** per evaluated move —
146.3 ns against the nobreak path's 164.2 — and concluded that the it/s deficit
there measures the problem being harder, not the code being slower. That
conclusion was offered as a reason the metric itself might be the wrong one.

On the production long-route case it is false:

| | nobreak | break |
|---|---|---|
| moves evaluated (100 iterations) | 1 089 085 | 1 320 339 (1.21x) |
| improving moves | 735 | 1 330 (1.81x) |
| parity violations | 0 | 0 |
| **ns per evaluated move** | **98.2** | **391.4** (4.0x slower) |

**Four times slower per unit of search work**, not 12% faster. The break arm
does evaluate 21% more moves and accept 81% more of them, but that is a small
part of a 4.8x wall-clock difference; the rest is per-move cost, and per-move
cost is the evaluator.

So the honest statement of the two instances is not the same statement:

- **group-54 (~23-node routes):** the break path is already cheaper per unit of
  work. Its it/s deficit is the problem being harder, and the levers that would
  close it have all measured neutral because the routes are too short for the
  machinery to amortise.
- **group 190 (50-72-node routes, production):** the break path is genuinely 4x
  slower per unit of work. There *is* overhead to remove, it is `duration()`,
  and §23 measured the structural approach breaking even on it at this length —
  not because the idea is wrong but because the guards cost what the skip saves.

Both are true, and quoting either one alone misrepresents the fork.

## 26. The regime decomposition finally pays — a prefix max was the whole problem

§23 closed the interior jump as break-even: exact, 94% coverage, 13% hit, and
1.0020 on the production A/B. The rejection counters said 25% of its candidates
died on the anchor bound. That bound existed for one reason: `cumM_` is a
**prefix** max from position 0, so a proposal reaching the anchor earlier than
the route did would inherit window clamps its own prefix never saw.

`max` is idempotent. The duration fold is not — which is why §14.1's range-fold
needed a segment tree — but a range **maximum** takes a sparse table with
overlapping windows, O(n log n) to build and O(1) to query. So the bound is not
a necessary guard, it is an artefact of the wrong data structure.

### 26.1 What removing it did

    jump hit          0.130 -> 0.280
    nodes skipped     2.47  -> 5.16   per candidate
    anchor rejects    0.252 -> 0.000
    production A/B    1.0020 -> 1.0345

**+3.45% on the production long-route case, distance identical at 896 884**, and
`calls` unchanged at 1 047 576 — the same evaluation count as the build with no
jump at all.

### 26.2 It also corrects §17

§17 concluded that time warp breaks the clock identity, and §17's correction
softened that to "it breaks on ~3% of warped nodes". Both were wrong about the
cause. Under the range max:

    [clock] under-warp: checked=10 015  mismatch=0 (0.000000)

**Zero.** What the prefix max got wrong 3.3% of the time was never warp — it was
clamps from before the anchor. Warp was blamed for a data-structure artefact
through three sections of this document.

### 26.3 Each route pays only for what it uses

Building both tables everywhere cost ~1.6% on group-54, whose ~23-node routes
have nothing to consult them. The final shape:

- **Short routes:** no fold tree, no range-max table. `clockAt()` takes a fast
  path — one `cumM_` read — which is exact precisely when the anchor bound
  holds, and the localiser still requires that bound there. Nothing is built and
  nothing is paid.
- **Long routes (>= 60 nodes):** both tables. The jump drops the anchor bound
  and reaches 28% of candidates.

Measured, paired:

| | group-54 | production (group 190) |
|---|---|---|
| A/B | **1.002** (4/8) — neutral | **+3.45%** (6 pairs) |
| distance | 4 896 741 unchanged | 896 884 unchanged |

Gates: `test_stream_parity` 500/500 exact with 0 evaluator-mode disagreements;
`test_segment_fold_parity` 63/63; `test_proposal_fold_residual` 53/174;
`test_custombreak` passed; pytest 1181 passed, 2 skipped, 6 xfailed.
(`tests/cpp/test_drivesegment.cpp` does not compile — it calls a six-argument
`DriveSegment` constructor that has five in `1607afc` too, so it was already
broken before this branch.)

### 26.4 What it does and does not change

**This is the first time in fifteen hypotheses that the regime decomposition —
this change's original thesis — measures positive.** It took both primitives,
the differential harness, and finding that a guard everyone (including this
document, repeatedly) attributed to time warp was a prefix max.

It does not move the target. 0.90 needs the evaluator *and* the evaluation
volume; a perfect evaluator alone caps at 0.684 on the production case, and
+3.45% is a step inside that, not past it.

### 26.5 Lowering the minimum span: more coverage, no gain

With the anchor bound gone, the jump's dominant rejection became "descriptor
shorter than the 9-node minimum" at 0.505. The threshold was a guess: a fold
costs ~7 merges plus the searches, walking k nodes costs ~55 cycles each, so
break-even looked closer to 5.

At 5 the coverage moved as expected and the payoff did not:

    hit            0.280 -> 0.385
    nodes skipped  5.16  -> 5.80
    production A/B         1.0025   (neutral)
    group-54 A/B           0.958    (0/6 — a 4% loss)

Reverted. Two things it says: the marginal skipped nodes are the cheap ones, so
coverage past ~28% buys little; and group-54 has transient routes long enough to
build the tables during the search, which a lower threshold lets attempt the
jump more often for nothing. 9 stays.

## 27. The evaluation-volume excess is search behaviour, not a discarded filter

§22 left 0.90 needing two things: the evaluator and the evaluation volume. The
volume side had one hypothesis that would not have cost quality — that the
persistent filter is thrown away every iteration on the break arm, because the
solution is infeasible, the penalty manager retunes, the cost evaluator changes,
and `LocalSearch::operator()` calls `resetFilters()` on a parameter change.

Measured, as a fraction of invocations:

| instance / arm | calls | reset on changed parameters |
|---|---|---|
| g190 break (infeasible, 73-node route) | 101 | **0.0099** (1 of 101) |
| g190 nobreak | 102 | 0.0098 |
| group-54 break | 1510 | 0.0020 |
| group-54 nobreak | 1514 | 0.0020 |

If the hypothesis held, the break arm would sit near 1.0. It sits at 0.0099, and
identical to the nobreak arm. The filter survives in ≥97% of invocations
everywhere.

The reason is in `pyvrp/PenaltyManager.py:305-311`: penalties are recomputed
only once every `solutions_between_updates` (default **500**), and hows-router
overrides only the min/max bounds, not the cadence. Infeasibility changes the
*direction* of the adjustment, not how often it happens — so a 100-iteration
g190 solve has zero penalty updates, and the single parameter reset is the
`max_cost_evaluator()` used for the exhaustive initial descent handing over to
the ILS. The safety valve never fires in any arm.

**So the 22% more pairs tested and 18% more moves applied are genuinely the
search doing more work**, exactly as §11 measured from the other side (zero
parity violations, every update a real improvement). Closing that half of the
gap means changing what the search accepts.

One exact refinement exists and does not pay: a cached "non-improving" verdict
survives a penalty change when both routes were feasible at test time and the
penalties only rose, since each `floor(v * p)` term with `v >= 0` is monotone in
`p` (`CostEvaluator.h:226-258`). A partial reset invalidating only infeasible
routes would be exact — and would save under 1% of invocations. Not implemented.

## 28. The lower-bound prefilter prunes nothing on the production case

`CostEvaluator::deltaCost` gates the expensive `duration()` term behind
`Proposal::durationLowerBound()` (`CostEvaluator.h:368-381`): if the proposal is
already non-improving at a lower bound on what duration and the terms after it
can add, it cannot become improving, so the fold is skipped. Measured on the
production instance (group 190, 73 nodes, one rule), 1500 iterations, stats
build:

```
[bound-stats] reached=64668585 pruned=0.000 paid_but_rejected=0.999
[bound-stats][lbfold] checked=64668585 violations=0 (rate=0.000000)
```

**It prunes 0.000%.** Not "a little" — nothing. And of the candidates that go on
to pay for `duration()`, 99.9% turn out non-improving anyway, which is the
population a working bound would have caught. On group-54 the same bound prunes
21.6%, so this is instance-dependent looseness, not a construction bug. The
plausible reason, not yet measured: on group 190 there is one long route and
every move is intra-route, so `out` starts at `-penalisedCost(route)`, and a
bound built only from travel and service prefix sums recovers nothing close to
that on a route carrying this much waiting and time warp.

The filter is not free. §lane-4's phase attribution puts `durationLowerBound()`
at 261 cyc/call in the break arm and 164 in the nobreak arm, over 4.2 ranges per
call, reached on 94% of break-arm candidates. On production that is pure
subtraction: a filter that costs 261 cycles and rejects nobody.

Two things follow, and they point in opposite directions:

An exact ceiling closes the waste without changing the search. If
`out + unitDurationCost * SUM(route totals of the proposal's segments)` is still
below zero, the bound provably cannot prune, and computing it is wasted; each
prefix slice is non-decreasing and bounded by its route's total, so the test is
exact by construction. It skips 100% of arrivals on g190 and 37.5% on group-54,
with `calls` identical on both. Worth about -215 cyc/call on the break arm — but
the nobreak arm gains -120 too, so the effect on the *ratio* is roughly 2%.

The second is the interesting one, and it is unbuilt. `durationLowerBoundFold()`
(`Route.h:3730`) — the ordinary no-break monoid fold, `duration() - waiting()` —
has been computed alongside the real increment **64.7 million times with zero
violations** and has never been wired into the gate. That is precisely the
technique the literature calls the standard prefilter (§29): the time-window
relaxation without breaks, used to reject moves before the break scheduler runs.
Goel and Vidal report it eliminating 70-95% of moves. Ours eliminates 0%.

What is missing is not admissibility but *pruning power*, which nobody measured:
the counter records violations, never how many candidates `out + lbFold >= 0`
would have caught. Until that number exists the change cannot be judged, because
the bound is not cheap either — it is a full `foldDuration()`, 600-800 cyc/call,
against `duration()`'s ~1096. It has to prune well over half to pay on g190.

A methodological note that invalidates part of the cycle attribution taken
before this was found: under `PYVRP_STREAM_STATS`, `deltaCost<false>` was
computing `durationLowerBoundFold()` on *every* proposal reaching the duration
term — 3.7 per `binaryOps` call, in **both** arms, outside `PH_DURATION`. Counter
readings survive; any cycle attribution from a stats build that did not isolate
that term does not. It is opt-in now.

## 29. What the literature says: the composable summary does not exist

Two independent lines were run against the same question — whether the
break-constrained forward pass can be replaced by a composable per-segment
summary, the way `DurationSegment` replaces the time-window walk. Both came back
negative, and they agree on why.

Vidal, Crainic, Gendreau and Prins — the authors of the O(1) concatenation
framework this codebase inherits — classify hours-of-service as an attribute
*outside* that framework, and implement it with forward-only labels at O(|s2|)
per evaluation. Tilk and Goel (EJOR 283(1), 2020) state that "no polynomial
complexity bound is known for route evaluation subject to hours of service
regulations" — not for concatenating two segments, for evaluating one route.
Goel and Vidal (Transportation Science 48(3), 2014) say the O(1) time-window
evaluation "is not the case when hours of service regulations must be complied
with", and cap the schedules kept per subsequence at 5 with heuristic dominance,
giving up exactness to bound the cost.

The one documented O(1) shortcut — Vidal et al.'s lunch break, two data sets per
segment plus a three-case concatenation — works because the trigger is the
**absolute clock**, a fixed window. It does not generalise to an accumulator
trigger, because the *position* of the break inside a segment then depends on
the entry state.

The structural analysis reaches the same wall from inside this code. The arrival
to departure map of a break-free stretch is genuinely piecewise linear, as
hypothesised. But one term ruins it: `firstDue` (D3) is the arrival at the
*first node* where the rule fires (`DriveSegment.cpp:299-300`), so as the entry
clock slides, the crossing node changes, and the function is a sawtooth with one
tooth per node — O(nodes x rules) pieces, not O(rules). It does not explode; it
simply does not compress. Composing two blocks is O(n_A + n_B). Carried to its
conclusion, the hypothesis becomes exactly the binary-search crossing localiser
already built and measured at +3.45% (§26).

So the ceiling is not an implementation failure. What the state of the art
actually offers is O(n) per move with a smaller constant, via four levers:
memoisation of partial schedules per subsequence (2-10x, per Goel and Vidal),
a lower-bound prefilter (70-95% of moves — §28, ours prunes 0%), a cap on labels
per node (inexact), and lazy break evaluation only on solutions about to be
accepted (Ostermeier 2024). The first two are open here. The third trades
exactness. The fourth is a different architecture.

One reading worth recording because it is a product decision, not an engineering
one: if `firstDue` were defined as the continuous instant the limit is crossed
(`lastResetAt + tv`) rather than the arrival at the next node past it, the
sawtooth collapses to two pieces, the binary search disappears, and the block is
O(1) per rule. It is arguably *more* correct for CLT — the driver exceeds 12h at
the instant, not at the next stop. It changes the objective, so both the
evaluator and `evaluateForwardPass` would have to move together, and there is no
parity with today's answer. Not proposed; recorded.

## 30. The production benchmark was the optimistic case

`_hw_g190.py` has been the production yardstick for this whole investigation,
and §20's go/no-go, §22's ceiling and §25's correction all rest on it. It is a
real production route — the longest one there is, 72 activities. It is also the
most deformed one, and that turns out to matter more than its length.

Production's own answer for it carries `time_warp_s = 1070992`, serves no break
at all, and came from a manual assignment rather than the solver. So every
evaluation on it starts from a route whose penalty term is around 2.28e9 while
any duration lower bound is around 2.0e5 — four orders of magnitude apart. In
that regime no duration bound can prune anything, which is exactly what §28
measured and mistook for a property of the bound.

The regime is half of production, not all of it. Of the 99 production routes
whose vehicle carries break rules: 49 have time warp and 50 do not, 43 actually
serve a break, the median is 30 steps and the maximum 72.

Two feasible, break-serving counterparts were extracted from the same database —
`01649f16` (group 190, 42 jobs, 45 steps) and `b3149e0e` (group 149, 51 jobs,
54 steps), both solved by production with zero time warp and one break served.
They are not the easy cases:

| instance | steps | production time warp | break/nobreak ratio |
|---|---|---|---|
| `6a28ddd3` (the yardstick) | 72 | 1 070 992 | 0.20-0.26 |
| `01649f16` | 45 | 0 | **0.0975** |
| `b3149e0e` | 54 | 0 | **0.0663** |

**The case this project has been optimising against is the optimistic one.** On a
feasible route that actually serves its break, the break arm is 10-15x slower per
iteration, not 4x. Every ratio quoted before §30 describes the saturated regime.

Two things follow. The 0.90 target is further away than any measurement here has
shown — the ceiling arithmetic of §22 was computed on the friendlier instance and
needs redoing. And the levers that only work without penalty saturation, the
lower-bound prefilter first among them, were being evaluated where they cannot
possibly show a result; §28's "prunes 0.000%" is a fact about the instance, not
about the filter.

A methodological correction that belongs here rather than in §28: the "zero
violations in 64.7M checks" recorded there was vacuous. With `actual` around
3.4e9 per candidate, any bound whatsoever passes the admissibility test. Where
the test has teeth — group-54, in the unsaturated regime — the same candidate
bound violates 68 320 times and would prune 746 genuinely improving moves. It is
inadmissible, and the counter that said otherwise was measuring nothing. What is
admissible, and does pay, is the existing bound with the boundary edges it was
discarding: pruning goes from 21.6% to 61.3% on group-54.

## 31. On the real production regime the ceiling is 0.27, and it is search volume

With §30's feasible instance (`01649f16`, group 190, 45 activities, one break
served) the two arms can finally be profiled separately — `_hw_g190.py --only`
exists now, because the stream-stats counters and the rdtsc phase table are
process-global and print once at exit, so running both arms in one process sums
them and neither can be read.

1500 iterations, one rep, same binary, break arm distance 588997:

| | nobreak | break | ratio |
|---|---|---|---|
| search, total cycles | 3.03e9 | 2.35e10 | **7.76x** |
| `binaryOps` calls | 1 342 504 | 4 984 780 | **3.71x** |
| `binaryOps` cyc/call | 2 221 | 4 630 | 2.08x |
| `duration()` calls | 5 690 955 | 12 594 536 | 2.21x |
| `duration()` cyc/call | 158.2 | 1 238.1 | 7.82x |
| `Route::update` calls | 41 396 | 74 336 | 1.80x |
| `Route::update` cyc/call | 2 639 | 9 232 | 3.50x |
| duration share of search | 27.4% | 63.7% | |
| lower bound pruned | 0.000 | **0.359** | |

3.71 x 2.08 = 7.7, which is the whole gap. So it factors cleanly into a volume
half and a cost half — and **the volume half is the bigger one**.

`search()` is entered the same number of times in both arms (1510 vs 1504). What
differs is how much local search happens inside each entry: the break arm runs
3.71x more neighbourhood scans while applying only 1.80x more moves. It is not
the neighbourhood being bigger — 45 activities against 44 — and it is not more
improvements causing more rescans, or the two factors would move together. With
breaks the landscape simply keeps yielding improving moves for longer.

**The arithmetic that decides the mission.** Even with a `duration()` that costs
nothing, the break arm still pays 3.71x more `binaryOps` invocations at whatever
the per-call cost is. If every per-call cost were driven down to the nobreak
arm's, the ratio would stop at **1/3.71 = 0.27**. Removing `duration()` entirely
and leaving everything else as measured gives 3.03e9 / 8.54e9 = **0.355**.

0.90 is not reachable by making the evaluator faster. It requires the break arm
to stop searching 3.7x as much — which is a change to what the search accepts,
not to how fast it evaluates, and it trades solution quality for speed. §27
reached the same conclusion on group-54, where the volume excess is 22%; here it
is 271%, and it dominates.

This supersedes §22's ceiling of 0.684, which was computed on the saturated
instance where the volume excess is much smaller.

What remains worth doing on the cost half, in descending order of measured size:
`duration()` at 63.7% of the break arm (1238 cyc/call, ~44 cyc per node actually
walked after the tail collapse, plus 235-260 cyc/call of fixed overhead before
the loop); the lower-bound prefilter, which prunes 35.9% here against 0% on the
saturated instance and which the boundary-edge fix takes to 61.3% on group-54;
and `Route::update` at 3.50x per call. None of them can reach 0.90 alone or
together.

## 32. Correction: the harness was double-counting service, and there was no saturated regime

§30 and §31 rest on measurements taken with a defect in `_hw_g190.py`, and the
defect is the reason the "penalty-saturated regime" of §28 and §30 existed at
all.

The harness built each client's service as `service + fixed_s`. The router emits
both, but `service` **already contains** `fixed_s`: it is
`compute_service_seconds(volume, fixed_s) = fixed + variable`
(`hows-router/src/services/service_time.py:90`), and `fixed_s` is emitted
alongside only so a caller can see the split
(`hows-router/src/services/order_service.py:704`). Adding them double-counts the
fixed component — 95% inflation of total service on these instances (g149f:
79 854 s used against 40 854 s real).

That inflation is what pushed the routes past the 48-hour cap. With it removed,
**all three production instances solve feasibly**, including the one §30 called
the most deformed case in production:

| instance | before | after |
|---|---|---|
| `6a28ddd3` (72 steps) | feas=False, dist 896884, ratio 0.20-0.26 | **feas=True, dist 860327, ratio 0.1242** |
| `01649f16` (45 steps) | feas=True, dist 588997, ratio 0.0975 | feas=True, **dist 582033**, ratio 0.0924 |
| `b3149e0e` (54 steps) | feas=False, dist 1005387, ratio 0.0663 | **feas=True, dist 813930, ratio 0.0465** |

So §28's "the bound prunes 0.000%" and §30's "half of production is penalty
saturated" were both describing an artefact of the benchmark, not production.
The database evidence in §30 about production's *own* answers still stands — 49
of 99 break-configured routes really do carry time warp, and `6a28ddd3` really
does carry 1 070 992 s of it. What does not stand is the claim that the harness
was reproducing that regime; it was manufacturing its own.

§31's factorisation survives the correction and gets slightly stronger. Same
instance, same binary, 1500 iterations, arms isolated with `--only`:

| | nobreak | break | ratio |
|---|---|---|---|
| search, total cycles | 2.98e9 | 2.83e10 | **9.52x** |
| `binaryOps` calls | 1 342 504 | 5 417 412 | **4.04x** |
| `binaryOps` cyc/call | 2 180 | 5 149 | 2.36x |
| `duration()` calls | 5 690 955 | 13 270 879 | 2.33x |
| `duration()` cyc/call | 155.2 | 1 474.6 | 9.50x |
| `Route::update` calls | 41 396 | 80 601 | 1.95x |
| duration share of search | 27.4% | 66.2% | |
| lower bound pruned | 0.000 | 0.375 | |

4.04 x 2.36 = 9.53, the whole gap. The volume half grew from 3.71x to **4.04x**,
so **the ceiling with a free evaluator is 1/4.04 = 0.248**, and removing
`duration()` entirely while leaving everything else measured gives 0.310. The
conclusion of §31 is unchanged and firmer: 0.90 cannot be reached by making the
evaluator faster.

The distance gates used throughout this work change with the instance
definition. The valid ones are now g190 break **860327**, `01649f16` break
**582033**, `b3149e0e` break **813930**, group-54 break **4896741** (unaffected —
different harness), and `[stream-stats] calls=13270879` on `01649f16 --only
break` at 1500 iterations, 1 rep.

Worth stating plainly: this defect had been in place for every production
measurement in this document, and none of the three exactness gates could catch
it — they check that the evaluator agrees with itself on whatever instance it is
given, not that the instance is the one intended. It was found by comparing the
harness's totals against production's own `cost_breakdown`, which is a check
nothing here was doing.
