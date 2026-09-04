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

## 2. Round 2 is a non-issue. Strike it from the plan.

The design (and the prior analysis it inherits) treats the second round of the
forward pass as a cost worth incrementalising, scoped at days of work.

    round2_f = 0.009

It runs on **0.9%** of candidates. Incrementalising it is worth nothing.

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

(`cpp_min` over 3-5 reps; artefacts `hw_ratio_HEAD*.json`.)

### 9.1 The two metrics never actually disagreed

The ratio converges to ~0.75 and stays there, which matches the canonical 8 s
metric (0.733, ~1400-1900 iterations). The apparent conflict between "0.895 at
fixed iterations" and "0.733 canonical" was an artefact of reading one at 250
iterations and the other in the converged regime. There is one number, and it
is ~0.74.

### 9.2 The break path does not degrade -- the nobreak path improves

**Break stays flat at ~7.0 ms/it at every length. Nobreak falls 29% (7.51 ->
5.33) and then plateaus.** The gap is not the break path getting worse; it is
the break path **failing to collect** the speedup that convergence hands the
nobreak path.

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

### 9.4 How much is left, in converged-regime numbers

At the plateau nobreak costs 5.33 ms/it and break 7.05. For ratio 0.90 the break
path must fall to 5.92 ms/it -- a **16% cut of the break path's total cost**.
With `duration()` accounting for 84% of the excess, that means removing ~78% of
the `duration()` excess. That is large, and it is precisely §8.3's wall.
