// Residual harness: naive cached-summary recomposition vs the full forward
// pass on ROUTE-MUTATING proposals (loop do CASO COM BREAK — Estágio 2, step 1).
//
// Motivation
// ----------
// Estágio 1 proved that recomposing a whole route from cached segment
// summaries (SegmentBefore/SegmentBetween/SegmentAfter) reproduces the shared
// forward pass EXACTLY when the route is unchanged (59/59). The question this
// harness answers is: what happens to that parity when a PROPOSAL changes the
// route (removal, relocation, break shift)? Those proposals are exactly what
// ShiftBreak/Relocate/Remove evaluate on break-configured routes.
//
// For every mutation of a break-configured base route we compare:
//   * ground truth: the full flat forward pass over the MUTATED sequence
//     (this is what Proposal::duration() runs today — semantics contract);
//   * candidate: a DurationSegment monoid fold over maximal *unchanged* runs
//     (reusing the cached route summaries) plus a leaf fold of the changed
//     run. The changed-run leaves are re-derived from the data (break
//     service/window singletons); the fold has NO drive-state transfer, so it
//     cannot re-derive D5 rest extensions or due-window clearing inside the
//     changed span.
//
// The output quantifies the structural residual: how many proposals diverge
// and by how much, split by mutation kind. This is the evidence for whether a
// cached-monoid fold can ever replace the forward pass on break routes, and
// therefore which incremental strategy (if any) attempt 4 should implement.
//
// Run with: meson test -C <builddir> --suite cpp
// (registered in meson.build; `meson test ... test_proposal_fold_residual` runs just this one)
//
// NOTE: divergence is the EXPECTED finding here. The test passes when it
// is present and fails if it ever disappears — see the exit logic at the
// bottom for why.
#include "CustomBreak.h"
#include "DriveSegment.h"
#include "ProblemData.h"
#include "search/Route.h"

#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

using namespace pyvrp;
using namespace pyvrp::search;

namespace
{
// ---------------------------------------------------------------------------
// Chain data builder (same convention as the Estágio-1 harness)
// ---------------------------------------------------------------------------

ProblemData buildChain(std::vector<Duration> const &twEarly,
                       std::vector<CustomBreak> const &breaks,
                       Duration travel = Duration(1000),
                       Duration svc = Duration(600),
                       std::optional<Duration> startLate = std::nullopt)
{
    size_t const nClients = twEarly.size();
    size_t const numLoc = nClients + 1;

    std::vector<Location> locations;
    for (size_t i = 0; i != numLoc; ++i)
        locations.emplace_back(static_cast<Coordinate>(i), 0.0);

    Duration const MAX = std::numeric_limits<Duration>::max();

    std::vector<Client> clients;
    for (size_t i = 0; i != nClients; ++i)
        clients.emplace_back(Client(i + 1, {0}, {}, svc, twEarly[i], MAX));

    std::vector<Depot> depots;
    depots.emplace_back(Depot(0, Duration(0), MAX));

    Matrix<Distance> distMat(numLoc, numLoc, Distance(0));
    Matrix<Duration> durMat(numLoc, numLoc, Duration(0));
    for (size_t i = 0; i != numLoc; ++i)
        for (size_t j = 0; j != numLoc; ++j)
            if (i != j)
            {
                distMat(i, j) = Distance(travel.get());
                durMat(i, j) = travel;
            }

    std::vector<VehicleType> vts;
    vts.emplace_back(1, std::vector<Load>{9999}, 0, 0, 0, 0, MAX, MAX,
                     std::numeric_limits<Distance>::max(), 0, 0, 0, startLate,
                     std::vector<Load>{}, std::vector<size_t>{},
                     std::numeric_limits<size_t>::max(), 0, 0, breaks, false,
                     "vt");

    return ProblemData(std::move(locations), std::move(clients),
                       std::move(depots), std::move(vts),
                       std::vector<Matrix<Distance>>{distMat},
                       std::vector<Matrix<Duration>>{durMat});
}

Activity cl(size_t idx) { return {Activity::ActivityType::CLIENT, idx}; }
Activity brk(size_t id) { return {Activity::ActivityType::CUSTOM_BREAK, id}; }
Activity depot() { return {Activity::ActivityType::DEPOT, 0}; }

// Location of activity ``act`` in a chain whose previous location is ``prev``.
// CUSTOM_BREAK inherits the previous node's location (update() convention).
size_t locOf(Activity const &act, ProblemData const &data, size_t prev)
{
    if (act.isCustomBreak())
        return prev;
    if (act.isDepot())
        return data.depot(act.idx()).location;
    return data.client(act.idx()).location;
}

std::vector<size_t> locsOf(std::vector<Activity> const &acts,
                           ProblemData const &data)
{
    std::vector<size_t> locs;
    locs.reserve(acts.size());
    size_t prev = 0;
    for (auto const &act : acts)
    {
        locs.push_back(locOf(act, data, prev));
        prev = locs.back();
    }
    return locs;
}

// Raw (pre-D5) per-node DurationSegment leaf, mirroring the evaluator's
// Step 1 (and update()'s durAt builder). Location of the node is not needed
// for the leaf itself (setup is only charged on edges, never in leaves).
DurationSegment leafOf(Activity const &act,
                       ProblemData const &data,
                       VehicleType const &vt)
{
    if (act.isCustomBreak())
    {
        auto const breakId = act.idx();
        Duration svc(0);
        Duration early = 0;
        Duration late = std::numeric_limits<Duration>::max();
        for (auto const &brk : vt.custom_breaks)
            if (brk.id == static_cast<size_t>(breakId))
            {
                svc = brk.service;
                if (!brk.tws.empty())
                {
                    if (brk.twsRelative)
                    {
                        early = brk.tws.front().first + vt.twEarly;
                        late = brk.tws.back().second + vt.twEarly;
                    }
                    else
                    {
                        early = brk.tws.front().first;
                        late = brk.tws.back().second;
                    }
                }
                break;
            }
        return DurationSegment(svc, Duration(0), early, late);
    }
    if (act.isDepot())
        return {data.depot(act.idx()), 0};
    return {data.client(act.idx())};
}

// A contiguous run of a PROPOSED route, with its summary:
//   ``first``/``last`` : inclusive indices in the proposed activity vector.
//   ``cached``         : when true, summary is read from the ORIGINAL route's
//                        cached segments (unchanged content); else it is a
//                        leaf fold (changed run, raw leaves).
struct Run
{
    size_t first;
    size_t last;
    bool cached;
    DurationSegment summary;
};

// Fold of the whole proposed route from its maximal unchanged/raw runs,
// merged left-to-right across the inter-run boundary edges (no reload depots
// in these recipes, so the plain merge is exact for a DurationSegment fold).
DurationSegment foldRuns(std::vector<Run> const &runs,
                         std::vector<Activity> const &acts,
                         std::vector<size_t> const &locs,
                         ProblemData const &data,
                         VehicleType const &vt,
                         size_t profile)
{
    auto const &durMat = data.durationMatrix(profile);

    auto segOf = [&](Run const &run) -> DurationSegment
    {
        if (run.cached)
            return run.summary;
        auto ds = leafOf(acts[run.first], data, vt);
        for (size_t i = run.first + 1; i <= run.last; ++i)
        {
            auto const edge = durMat(locs[i - 1], locs[i]);
            ds = DurationSegment::merge(edge, ds, leafOf(acts[i], data, vt));
        }
        return ds;
    };

    auto ds = segOf(runs.front());
    for (size_t i = 1; i < runs.size(); ++i)
    {
        auto const prevLast = runs[i - 1].last;
        auto const curFirst = runs[i].first;
        auto const edge = durMat(locs[prevLast], locs[curFirst]);
        ds = DurationSegment::merge(edge, ds, segOf(runs[i]));
    }
    return ds;
}

struct GT
{
    Duration dur;
    Duration warp;
    Duration wait;
    int64_t breakDue;
};

GT groundTruth(std::vector<Activity> const &acts, ProblemData const &data,
               VehicleType const &vt)
{
    auto const locs = locsOf(acts, data);
    std::vector<Duration> atSecond(acts.size());
    auto const res = evaluateForwardPass(acts, locs, atSecond, nullptr, nullptr,
                                         data, vt);
    return {res.duration, res.timeWarp, res.waiting, res.breakDue};
}

enum class Mut
{
    REMOVE_CLIENT,
    RELOCATE,
    SHIFT_BREAK,
};

struct MutCase
{
    Mut kind;
    size_t a;  // moved/removed position (proposed-original space)
    size_t b;  // target position (relocate/shift)
    std::string desc;
};

struct Totals
{
    size_t total = 0;
    size_t diverge = 0;
    // Absolute divergences, summed and counted only over diverging cases.
    long long sumDur = 0;
    long long sumWarp = 0;
    long long sumWait = 0;
};

void report(std::vector<std::string> const &names,
            std::vector<Totals> const &totals)
{
    for (size_t k = 0; k < names.size(); ++k)
    {
        auto const &t = totals[k];
        std::printf("  [%s] %zu/%zu diverge", names[k].c_str(), t.diverge,
                    t.total);
        if (t.diverge)
        {
            std::printf("  avg |ddur|=%.1f  avg |dwarp|=%.1f  avg |dwait|=%.1f",
                        static_cast<double>(t.sumDur) / t.diverge,
                        static_cast<double>(t.sumWarp) / t.diverge,
                        static_cast<double>(t.sumWait) / t.diverge);
        }
        std::printf("\n");
    }
}

}  // namespace

int main()
{
    std::printf("test_proposal_fold_residual — cached-fold vs forward pass on "
                "mutated break routes (Estágio 2)\n");
    std::fflush(stdout);

    Duration const TRAVEL = Duration(1000);
    Duration const SVC = Duration(600);
    Duration const REST = Duration(3600);

    struct Case
    {
        char const *name;
        ProblemData data;
        std::vector<Activity> acts;
    };

    std::vector<Case> cases;
    // Recipe A: overnight DUTY rest followed by clients whose windows open far
    // in the future. Whenever the served rest is followed by such a client,
    // the forward pass D5-extends the rest to absorb the wait (effSvc is a
    // function of the ARRIVAL at the rest — which changes with every mutation
    // before it). A cached monoid fold over raw leaves cannot re-derive that.
    {
        std::vector<Duration> twEarly;
        for (size_t i = 0; i < 12; ++i)
            twEarly.push_back(i < 4 ? Duration(0) : Duration(43200));
        cases.push_back(
            {"D5-absorb",
             buildChain(twEarly,
                        {CustomBreak(1, {}, REST, CustomBreakTrigger::DUTY_TIME,
                                     Duration(1), CustomBreakReset::ALL_TIMERS,
                                     true)},
                        TRAVEL, SVC, Duration(0)),
             {depot(), cl(0), cl(1), cl(2), cl(3), brk(1), cl(4), cl(5),
              cl(6), cl(7), cl(8), cl(9), cl(10), cl(11), depot()}});
    }

    // Recipe B: like A, but TWO DUTY rests (two overnight stops), so a client
    // relocated across the second rest changes that rest's own D5 absorption.
    {
        std::vector<Duration> twEarly;
        for (size_t i = 0; i < 14; ++i)
            twEarly.push_back(i < 4 ? Duration(0)
                                    : (i < 9 ? Duration(28800)
                                             : Duration(57600)));
        cases.push_back(
            {"D5-two-rests",
             buildChain(twEarly,
                        {CustomBreak(1, {}, REST, CustomBreakTrigger::DUTY_TIME,
                                     Duration(1), CustomBreakReset::ALL_TIMERS,
                                     true),
                         CustomBreak(2, {}, REST, CustomBreakTrigger::DUTY_TIME,
                                     Duration(1), CustomBreakReset::ALL_TIMERS,
                                     true)},
                        TRAVEL, SVC, Duration(0)),
             {depot(), cl(0), cl(1), cl(2), cl(3), brk(1), cl(4), cl(5),
              cl(6), cl(7), cl(8), brk(2), cl(9), cl(10), cl(11), cl(12),
              cl(13), depot()}});
    }

    // Recipe C: absolute-window CLOCK_TIME rest (due-ness gate + window-close
    // warp). A non-due rest must NOT warp on its (already passed) absolute
    // close; a due one must. Relocating clients shifts the arrival across the
    // gate, flipping the cleared-window state that only the drive-aware pass
    // reproduces.
    cases.push_back(
        {"clock-abs-gate",
         buildChain({Duration(0), Duration(0), Duration(0), Duration(0),
                     Duration(0), Duration(0), Duration(0), Duration(0),
                     Duration(0), Duration(0), Duration(0), Duration(0)},
                    {CustomBreak(1,
                                 {std::make_pair(Duration(4000),
                                                 Duration(9000))},
                                 Duration(1800),
                                 CustomBreakTrigger::CLOCK_TIME, Duration(0),
                                 CustomBreakReset::DRIVE_TIMER, true)},
                    TRAVEL, SVC, Duration(0)),
         {depot(), cl(0), cl(1), cl(2), cl(3), brk(1), cl(4), cl(5), cl(6),
          cl(7), cl(8), cl(9), cl(10), cl(11), depot()}});

    std::vector<std::string> mutNames = {"REMOVE_CLIENT", "RELOCATE",
                                         "SHIFT_BREAK"};
    std::vector<Totals> mutTotals(mutNames.size());

    for (auto &cs : cases)
    {
        std::printf("case %s: n=%zu ... ", cs.name, cs.acts.size());
        std::fflush(stdout);
        // Build the search route in-place (Route is pointer-based and must
        // never be moved after nodes point back at it). Nodes are collected
        // first so the backing vector never reallocates mid-push.
        Route route(cs.data, 0);
        std::vector<Route::Node> nodeStore;
        for (auto const &act : cs.acts)
            if (!act.isDepot())
                nodeStore.emplace_back(act);
        for (auto &node : nodeStore)
            route.push_back(&node);
        route.update();

        auto const &orig = cs.acts;
        size_t const n = orig.size();
        auto const profile = route.profile();
        auto const &durMat = cs.data.durationMatrix(profile);
        VehicleType const &vt = cs.data.vehicleType(route.vehicleType());

        // ---- enumerate mutations ----
        std::vector<std::vector<Activity>> muts;
        std::vector<MutCase> meta;
        for (size_t i = 1; i + 1 < n; ++i)
        {
            if (!orig[i].isClient())
                continue;

            // REMOVE_CLIENT: remove client at i.
            {
                std::vector<Activity> m = orig;
                m.erase(m.begin() + i);
                muts.push_back(m);
                meta.push_back({Mut::REMOVE_CLIENT, i, 0, "rm c"});
            }

            // RELOCATE: move client at i just after position j (i < j), and
            // also just after every break position (cross-break moves).
            for (size_t j = i + 1; j + 1 < n; ++j)
            {
                bool const targetBreak = orig[j].isCustomBreak();
                bool const adjacent = j == i + 1;
                if (!targetBreak && !adjacent && (j - i) % 3 != 0)
                    continue;  // thin out the plain intra-span moves
                std::vector<Activity> m = orig;
                auto it = m.begin() + i;
                Activity moved = *it;
                m.erase(it);
                m.insert(m.begin() + j, moved);
                muts.push_back(m);
                meta.push_back({Mut::RELOCATE, i, j, "reloc"});
            }
        }

        // SHIFT_BREAK: move each break to a handful of positions.
        for (size_t i = 1; i + 1 < n; ++i)
        {
            if (!orig[i].isCustomBreak())
                continue;
            for (size_t step = 1; step <= 3; ++step)
            {
                size_t j = i + 2 * step;
                if (j >= n - 1)
                    break;
                if (orig[j].isCustomBreak())
                    continue;
                std::vector<Activity> m = orig;
                auto it = m.begin() + i;
                Activity moved = *it;
                m.erase(it);
                m.insert(m.begin() + j, moved);
                muts.push_back(m);
                meta.push_back({Mut::SHIFT_BREAK, i, j, "shift"});
            }
            for (size_t step = 1; step <= 3; ++step)
            {
                if (i < 2 * step + 1)
                    break;
                size_t j = i - 2 * step;
                if (orig[j].isCustomBreak())
                    continue;
                std::vector<Activity> m = orig;
                auto it = m.begin() + i;
                Activity moved = *it;
                m.erase(it);
                m.insert(m.begin() + j, moved);
                muts.push_back(m);
                meta.push_back({Mut::SHIFT_BREAK, i, j, "shiftL"});
            }
        }

        size_t caseDiv = 0;
        for (size_t mi = 0; mi < muts.size(); ++mi)
        {
            auto const &acts = muts[mi];
            auto const &mc = meta[mi];
            auto const locs = locsOf(acts, cs.data);
            size_t const m = acts.size();

            // Ground truth: full flat forward pass on the mutated sequence.
            auto const gt = groundTruth(acts, cs.data, vt);

            // Maximal unchanged prefix/suffix relative to the original.
            size_t L = 0;
            while (L < m && L < n && acts[L] == orig[L])
                ++L;
            size_t R = m;
            while (R > L && R <= n && acts[R - 1] == orig[R - 1 + (n - m)])
                --R;

            // Build runs.
            std::vector<Run> runs;
            size_t cursor = 0;
            if (L > 0)
            {
                // unchanged prefix [0..L-1]
                DurationSegment seg;
                if (L == n)
                    seg = route.before(L - 1).duration(profile);
                else
                    seg = route.between(0, L - 1).duration(profile);
                runs.push_back({0, L - 1, true, seg});
                cursor = L;
            }
            if (cursor < R)
            {
                // changed run [cursor..R-1]: raw leaf fold (no drive state).
                DurationSegment raw = leafOf(acts[cursor], cs.data, vt);
                for (size_t i = cursor + 1; i < R; ++i)
                    raw = DurationSegment::merge(durMat(locs[i - 1], locs[i]),
                                                 raw,
                                                 leafOf(acts[i], cs.data, vt));
                runs.push_back({cursor, R - 1, false, raw});
                cursor = R;
            }
            if (cursor < m)
            {
                // unchanged suffix: original [n-(m-cursor) .. n-1]
                size_t const oStart = n - (m - cursor);
                DurationSegment seg;
                if (oStart == 0)
                    seg = route.before(n - 1).duration(profile);
                else
                    seg = route.between(oStart, n - 1).duration(profile);
                runs.push_back({cursor, m - 1, true, seg});
            }

            auto const fold = foldRuns(runs, acts, locs, cs.data, vt, profile);
            auto const foldDur = fold.duration().get();
            auto const foldWarp = fold.timeWarp(vt.maxDuration).get();
            auto const foldWait = fold.waiting().get();

            bool const ok = foldDur == gt.dur.get()
                            && foldWarp == gt.warp.get()
                            && foldWait == gt.wait.get();
            auto &tot = mutTotals[static_cast<size_t>(mc.kind)];
            tot.total++;
            if (!ok)
            {
                tot.diverge++;
                tot.sumDur += llabs(foldDur - gt.dur.get());
                tot.sumWarp += llabs(foldWarp - gt.warp.get());
                tot.sumWait += llabs(foldWait - gt.wait.get());
                caseDiv++;
            }
        }

        std::printf("  [%s] %zu mutations, %zu diverge from the forward pass\n",
                    cs.name, muts.size(), caseDiv);
        std::fflush(stdout);
    }

    std::printf("Residual by mutation kind:\n");
    report(mutNames, mutTotals);

    size_t const total = mutTotals[0].total + mutTotals[1].total
                         + mutTotals[2].total;
    size_t const div = mutTotals[0].diverge + mutTotals[1].diverge
                       + mutTotals[2].diverge;
    std::printf("TOTAL %zu/%zu mutated proposals diverge under a cached "
                "monoid fold.\n", div, total);

    // Divergence here is the EXPECTED, documented finding: a cached monoid
    // fold cannot reproduce the forward pass on break routes, which is why the
    // evaluator walks. This used to exit 1 whenever divergence was found,
    // which made the harness permanently "failing" once it was wired into
    // `meson test`.
    //
    // The regression worth guarding is the opposite one: if divergence ever
    // disappears, the premise the break evaluator is built on has changed and
    // the cheaper fold-based path should be reconsidered. So that is what
    // fails here — loudly, with an explanation, rather than silently passing.
    if (total == 0)
    {
        std::fprintf(stderr,
                     "FAIL: no proposals were mutated; the harness measured "
                     "nothing.\n");
        return 1;
    }

    if (div == 0)
    {
        std::fprintf(stderr,
                     "FAIL: %zu mutated proposals, none diverging. A cached "
                     "monoid fold now reproduces the forward pass, which "
                     "contradicts the assumption the break evaluator's walk "
                     "is built on. Re-read the header of this file.\n",
                     total);
        return 1;
    }

    return 0;
}
