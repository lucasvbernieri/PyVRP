// Parity harness: segment-fold recomposition vs the shared forward pass on
// break-configured routes (loop do CASO COM BREAK — Estágio 1).
//
// For a break route R (already ``update()``d), split the whole route into
// contiguous cached segment summaries (``SegmentBetween``/``SegmentBefore``/
// ``SegmentAfter`` ``duration()``) and recompose them with
// ``DurationSegment::merge`` across the boundaries. If the cached summaries
// were a faithful monoid of the forward pass, the recomposition must
// reproduce the route's own ``duration()/timeWarp()/waiting()`` exactly
// (route-level parity). The forward pass is the ground truth here (it is what
// ``Route::update()`` and ``Proposal::duration()`` use on break routes).
//
// Why parity matters: it is the property the Estágio-1 fold composition must
// satisfy before break-route proposals can be evaluated incrementally from
// cached prefixes instead of a full O(n) ``evaluateForwardPass`` per
// candidate.
//
// Compile (this branch; not wired into meson, like the other tests/cpp):
//   g++ -std=c++20 -O1 -I pyvrp/cpp -I pyvrp/cpp/search \
//       tests/cpp/test_segment_fold_parity.cpp \
//       -L build-release -lsearch -lpyvrp -o build-release/test_segment_fold_parity.exe
//   build-release\test_segment_fold_parity.exe
#include "CustomBreak.h"
#include "DriveSegment.h"
#include "ProblemData.h"
#include "search/Route.h"

#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

using namespace pyvrp;
using namespace pyvrp::search;

namespace
{
// ---------------------------------------------------------------------------
// Chain data builder (depot loc 0; client i at loc i+1; travel ``travel``)
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

Route makeRoute(ProblemData const &data, std::vector<Activity> const &acts)
{
    Route route(data, 0);  // already has start and end depot
    std::vector<Route::Node> nodes;
    for (auto const &act : acts)
        if (!act.isDepot())  // start/end depots are implicit
            nodes.emplace_back(act);
    for (auto &node : nodes)
        route.push_back(&node);
    route.update();
    return route;
}

Activity cl(size_t idx) { return {Activity::ActivityType::CLIENT, idx}; }
Activity brk(size_t id) { return {Activity::ActivityType::CUSTOM_BREAK, id}; }
Activity depot() { return {Activity::ActivityType::DEPOT, 0}; }

// Location of node at ``pos``, replicating Route::update()'s convention:
// CUSTOM_BREAK inherits the previous node's location.
size_t nodeLoc(ProblemData const &data, Route const &route, size_t pos)
{
    if (route[pos]->isCustomBreak())
        return pos == 0 ? 0 : nodeLoc(data, route, pos - 1);
    if (route[pos]->isDepot())
        return data.depot(route[pos]->idx()).location;
    return data.client(route[pos]->idx()).location;
}

// One cached-range summary kind.
enum class Kind
{
    Before,    // [0 .. end]
    Between,   // [start .. end]
    After,     // [start .. n-1]
};

struct Seg
{
    Kind kind;
    size_t a;
    size_t b;
};

DurationSegment segSummary(Route const &route, Seg const &s)
{
    switch (s.kind)
    {
    case Kind::Before:
        return route.before(s.a).duration(route.profile());
    case Kind::Between:
        return route.between(s.a, s.b).duration(route.profile());
    case Kind::After:
        return route.after(s.a).duration(route.profile());
    }
    return {};
}

size_t segLastPos(Route const &route, Seg const &s)
{
    switch (s.kind)
    {
    case Kind::Before:
        return s.a;
    case Kind::Between:
        return s.b;
    case Kind::After:
        return route.size() - 1;
    }
    return 0;
}

size_t segFirstPos(Seg const &s)
{
    return s.kind == Kind::Before ? 0 : s.a;
}

// Whole-route no-op segmentation from a sorted list of cut positions. Each
// cut is the index of the LAST node of the current prefix segment. The first
// segment uses the cached ``durBefore`` (SegmentBefore), the last uses the
// cached ``durAfter`` (SegmentAfter), and interior segments use
// ``SegmentBetween``. The union covers [0 .. n-1] exactly.
void splitInto(Route const &route,
               std::vector<size_t> const &cuts,
               std::vector<Seg> &out)
{
    size_t const n = route.size();
    out.clear();
    size_t cursor = 0;  // first node of the current open segment
    for (size_t i = 0; i < cuts.size(); ++i)
    {
        auto const c = cuts[i] < n ? cuts[i] : n - 1;
        if (c < cursor)  // bad cut; skip
            continue;
        if (i == 0)
            out.push_back({Kind::Before, c, c});
        else
            out.push_back({Kind::Between, cursor, c});
        cursor = c + 1;
    }
    if (cursor < n)
        out.push_back({Kind::After, cursor, cursor});
}

}  // namespace
int main()
{
    std::printf("test_segment_fold_parity — cached-segment recomposition vs "
                "forward pass (Estágio 1)\n");

    Duration const TRAVEL = Duration(1000);
    Duration const SVC = Duration(600);
    Duration const REST = Duration(39600);

    struct Case
    {
        char const *name;
        ProblemData data;
        std::vector<Activity> acts;
    };

    std::vector<Case> cases;
    cases.push_back(
        {"plain-drive",
         buildChain({Duration(0), Duration(0), Duration(0), Duration(0)},
                    {CustomBreak(1, {}, REST, CustomBreakTrigger::DRIVE_TIME,
                                 Duration(5000), CustomBreakReset::DRIVE_TIMER,
                                 true)}),
         {depot(), cl(0), cl(1), brk(1), cl(2), cl(3), depot()}});

    // C1 opens at 45800; the DUTY rest must extend to absorb the 3600s wait
    // (D5). Rest ends 1600+39600 = 41200, +travel 1000 = 42200 < 45800.
    cases.push_back(
        {"overnight-D5",
         buildChain({Duration(0), Duration(45800)},
                    {CustomBreak(1, {}, REST, CustomBreakTrigger::DUTY_TIME,
                                 Duration(1), CustomBreakReset::ALL_TIMERS,
                                 true)},
                    TRAVEL, SVC, Duration(0)),
         {depot(), cl(0), brk(1), cl(1), depot()}});

    cases.push_back(
        {"clock-window",
         buildChain({Duration(0), Duration(0), Duration(0)},
                    {CustomBreak(1,
                                 {std::make_pair(Duration(0), Duration(15000))},
                                 Duration(1800),
                                 CustomBreakTrigger::CLOCK_TIME, Duration(0),
                                 CustomBreakReset::DRIVE_TIMER, true)},
                    TRAVEL, SVC, Duration(0)),
         {depot(), cl(0), brk(1), cl(1), cl(2), depot()}});

    cases.push_back(
        {"multi-reset",
         buildChain({Duration(0), Duration(0), Duration(0), Duration(0),
                     Duration(0)},
                    {CustomBreak(1, {}, Duration(3600),
                                 CustomBreakTrigger::DRIVE_TIME, Duration(7000),
                                 CustomBreakReset::DRIVE_TIMER, true),
                     CustomBreak(2, {}, Duration(5400),
                                 CustomBreakTrigger::DUTY_TIME, Duration(12000),
                                 CustomBreakReset::ALL_TIMERS, true)},
                    TRAVEL, SVC, Duration(0)),
         {depot(), cl(0), cl(1), brk(1), cl(2), brk(2), cl(3), cl(4),
          depot()}});

    // Enumerate single-interior-split and two-interior-split recompositions.
    size_t total = 0;
    size_t ok = 0;

    for (auto &cs : cases)
    {
        // Build the search route in-place (Route is pointer-based and not
        // movable, so it must never be returned from a helper).
        Route route(cs.data, 0);
        std::vector<Route::Node> nodes;
        for (auto const &act : cs.acts)
            if (!act.isDepot())  // start/end depots are implicit
                nodes.emplace_back(act);
        for (auto &node : nodes)
            route.push_back(&node);
        route.update();

        size_t const n = route.size();

        // (Estágio 2) Drive-state parity: the aligned SegmentBetween::driveState
        // over the WHOLE route span (start depot .. end depot), recomputed from
        // the cached per-node leaves, must reproduce the route's own final
        // forward drive state driveBefore.back() exactly — same clock, same
        // eligibility/reset/D5 decisions. This validates the absolute-clock
        // alignment of SegmentBetween::driveState against evaluateForwardPass.
        if (route.hasBreaks())
        {
            auto const fullSpan = route.between(0, n - 1).driveState(route.profile());
            auto const &ref = route.before(n - 1).driveState(route.profile());
            bool const driveOk
                = fullSpan.driveTime_ == ref.driveTime_
                  && fullSpan.workTime_ == ref.workTime_
                  && fullSpan.dutyTime_ == ref.dutyTime_
                  && fullSpan.lastResetAt_ == ref.lastResetAt_
                  && fullSpan.breaksTakenMask_ == ref.breaksTakenMask_;
            std::printf("  [%s] driveState(0,n-1) %s the forward pass "
                        "(drv=%lld/%lld wrk=%lld/%lld dty=%lld/%lld "
                        "rst=%lld/%lld msk=%u/%u)\n",
                        cs.name,
                        driveOk ? "reproduces" : "DIVERGES from",
                        fullSpan.driveTime_, ref.driveTime_,
                        fullSpan.workTime_, ref.workTime_,
                        fullSpan.dutyTime_, ref.dutyTime_,
                        fullSpan.lastResetAt_, ref.lastResetAt_,
                        fullSpan.breaksTakenMask_, ref.breaksTakenMask_);
            total++;
            if (driveOk)
                ok++;
        }

        auto const &durMat = cs.data.durationMatrix(route.profile());
        auto const groundDur = route.duration().get();
        auto const groundWarp = route.timeWarp().get();
        auto const groundWait = route.waiting().get();

        std::vector<size_t> splitSet;
        for (size_t k = 1; k + 1 < n; ++k)
            splitSet.push_back(k);

        std::vector<std::vector<size_t>> splitLists;
        for (size_t a : splitSet)
            splitLists.push_back({a});
        for (size_t i = 0; i < splitSet.size(); ++i)
            for (size_t j = i + 1; j < splitSet.size(); ++j)
                splitLists.push_back({splitSet[i], splitSet[j]});

        size_t caseOk = 0;
        size_t caseTotal = 0;
        for (auto const &splits : splitLists)
        {
            std::vector<Seg> segs;
            splitInto(route, splits, segs);
            ++caseTotal;

            DurationSegment ds = segSummary(route, segs[0]);
            for (size_t i = 1; i < segs.size(); ++i)
            {
                auto const from = segLastPos(route, segs[i - 1]);
                auto const to = segFirstPos(segs[i]);
                auto const edgeDur = durMat(nodeLoc(cs.data, route, from),
                                            nodeLoc(cs.data, route, to));
                ds = DurationSegment::merge(edgeDur, ds,
                                            segSummary(route, segs[i]));
            }

            long long const dur = ds.duration().get();
            long long const warp = ds.timeWarp(route.maxDuration()).get();
            long long const wait = ds.waiting().get();

            bool const good = dur == groundDur && warp == groundWarp
                              && wait == groundWait;
            total++;
            if (good)
            {
                ok++;
                caseOk++;
            }
            else
            {
                std::printf("  [%s] split {", cs.name);
                for (size_t s : splits)
                    std::printf("%zu,", s);
                std::printf("}: fold(dur=%lld warp=%lld wait=%lld) vs "
                            "pass(dur=%lld warp=%lld wait=%lld)\n",
                            dur, warp, wait, groundDur, groundWarp, groundWait);
            }
        }
        std::printf("  [%s] %zu/%zu recompositions exact (n=%zu)\n", cs.name,
                    caseOk, caseTotal, n);
    }

    std::printf("TOTAL %zu/%zu recompositions reproduce the forward pass.\n",
                ok, total);
    return ok == total ? 0 : 1;
}
