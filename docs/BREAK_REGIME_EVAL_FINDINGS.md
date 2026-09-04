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

Both forms, both bit-exact, both reverted:

| form | firing | measured |
|---|---|---|
| jump between consecutive break nodes, via cached stretch folds | **0.1%** | n/a |
| light scalar replay of the break-free stretch | 30.8% | **0.974 – 0.989** |

The jump fires on 0.1% for the reason in §1. The replay is correct — a
differential harness running it beside the node walk and comparing `duration`,
`timeWarp`, `waiting`, `dueMask`, `firstDue[]` and the end-depot clock reports
**0 mismatches over 60 iterations**, with the solve distance unchanged — and
still loses, because:

- routes are ~23 activities and the replay covers 8-15 of them;
- it removes the segment merge and the matrix lookup (~50 of ~110 cycles per
  node) but **not** the break-rule evaluation, which the contract requires at
  every boundary;
- the tail collapse already settles the cheap 42.9% in two merges;
- its per-node arrays must be rebuilt in each of ~67 `Route::update()` calls per
  iteration.

## 4. …but that verdict is specific to this instance, and reverses at scale

| break route length | ratio break/nobreak |
|---|---|
| 12 | 0.321 |
| 42 | 0.045 |
| 63 | 0.042 |
| 123 | 0.033 |

Same instance, vehicle count varied so route length varies; the nobreak side is
the same 122-activity route in every row. At **equal** route length the break
path costs **28x** the nobreak path.

So the change's thesis is right — for routes of 60+ activities, where the node
count finally dominates the per-event bookkeeping. It is simply not right for
group-54, whose ~23-activity routes put both terms at the same order.

**Consequence for the plan: the go/no-go must be taken against the route lengths
that actually occur in production, and the change's business case should be
argued from long-route instances.** On group-54 alone it is a no-go; on
60+-activity routes the break path is ~20x slower and nothing in the
constant-factor space touches it, because it is the O(n) term.

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
the search trajectory. Any implementation of this change should treat an
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

+35.3% on the break path, none of it from the regime decomposition. It came from
three things this change does not contemplate: removing per-candidate heap
allocations, a persistent evaluation-volume filter, and an admissible lower
bound that prunes 33% of proposals before `duration()` runs.

### 8.2 `duration()` is now 84% of the gap, up from 47%

Everything around it shrank, so the evaluator is now almost the whole story:

| | before | after |
|---|---|---|
| `duration()` cycles/call | 1573 | **966** |
| `duration()` calls | 2.55M | 1.74M |
| `Route::update` cycles/call | 34 790 | 16 522 |
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
| abort the pass incrementally on the partial value | 0.978 — 2.9% abort rate, at 85% of the walk |

> **What makes a break candidate non-improving lives in the penalty terms — time
> warp, break lateness, overtime, waiting — and computing those is the forward
> pass.** The duration-cost term is exactly `c_d * (travel + service)`; it is
> cheaply boundable, already bounded, and it is not the discriminator.

The fold's failure is worth knowing precisely: it treats a CUSTOM_BREAK as an
ordinary node carrying its own window, so `merge` clamps arrival to that window
and charges wait or warp *whether or not the break is eligible there*. The real
pass decides eligibility first.

### 8.4 Nothing is left on the memory side

Five independent experiments say this path is compute-bound once the heap
traffic is gone: software prefetch (0.997), reusing the cached `durAt` singleton
(0.973), replacing the remaining `nodes[i]->` predicates with sequential
`activitiesAt_` reads (1.001), an aggregate skip of the rule loop (0.981), and
reading the route's cached duration edges instead of probing the 2.1 MB matrix
(0.990). The matrix rows are hot — the same route is re-evaluated thousands of
times in a row.

### 8.5 What this means for the change

The scope narrows to one sentence: **cut the ~14 nodes a candidate simulates.**
Everything else on the break path has been taken. The constant factor per node
is ~69 cycles and five attempts to reduce it have failed, so the node count is
the only remaining variable — which is exactly what this change proposes, and
why §3 above (measured neutral at 23-activity routes) and §4 (the ratio falling
to 0.033 at 123-activity routes) together decide whether it is worth doing:
**not on group-54, and very much so on long routes.**
