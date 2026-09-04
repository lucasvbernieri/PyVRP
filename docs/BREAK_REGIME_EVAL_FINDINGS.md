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
`.driveState(` call site in the tree was inside ShiftBreak's own segment
adapter calling the very method being removed. Deleting both compiles clean.
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
