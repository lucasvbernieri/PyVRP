// Stream-parity probe (Alternativa A): Proposal::duration()/breakDue()/waiting()
// computed by the STREAMING forward pass (runStreamForward) must be bit-identical
// to the array-based evaluateForwardPass() over the same flat sequence.
//
// For every break-configured proposal we enumerate (whole-route recompositions
// at arbitrary cuts, client relocations across breaks, and break shifts via a
// generic single-node segment), we compare:
//   * proposal.duration()  cost + timeWarp
//   * proposal.breakDue()  (cached by duration())
//   * proposal.waiting()   (cached by duration())
// against evaluateForwardPass() on the materialised flat (acts, locs).
//
// Compile (same convention as the other tests/cpp harnesses):
//   g++ -std=c++20 -O1 -I pyvrp/cpp -I pyvrp/cpp/search \
//       tests/cpp/test_stream_parity.cpp \
//       -L build-release -lsearch -lpyvrp -o build-release/test_stream_parity.exe
//   build-release\test_stream_parity.exe
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
// Chain data builder (same convention as the other harnesses). unitDurationCost
// is 1 so duration == cost + waiting for proposals (overtime rate 0).
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
                     std::numeric_limits<Distance>::max(), 0, 1, 0, startLate,
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

// Ground truth: array-based shared forward pass.
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

// Generic single-node segment (forces the Proposal generic single-node branch,
// the path used by ShiftBreak's BreakSegment in the real operators).
struct NodeSeg
{
    Route const &route_;
    size_t pos_;

    NodeSeg(Route const &route, size_t pos) : route_(route), pos_(pos) {}

    Route const *route() const { return &route_; }
    SegmentProxy front() const { return route_.at(pos_).front(); }
    SegmentProxy back() const { return front(); }
    size_t size() const { return 1; }
    size_t numClients() const { return route_[pos_]->isClient() ? 1u : 0u; }
    size_t numPickups() const { return 0; }
    bool startsAtReloadDepot() const { return false; }
    bool endsAtReloadDepot() const { return false; }
    Distance distance(size_t profile) const
    {
        return route_.at(pos_).distance(profile);
    }
    DurationSegment duration(size_t profile) const
    {
        return route_.at(pos_).duration(profile);
    }
    LoadSegment load(size_t dimension) const
    {
        return route_.at(pos_).load(dimension);
    }
    DriveSegment driveState(size_t profile) const
    {
        return route_.at(pos_).driveState(profile);
    }
};

// Compares a proposal (whose flat content equals ``acts``) with the GT.
template <typename ProposalT>
bool checkProposal(ProposalT const &proposal,
                   std::vector<Activity> const &acts,
                   ProblemData const &data,
                   VehicleType const &vt,
                   Duration const &shiftDuration,
                   Cost const &unitDurationCost,
                   size_t &mismatch)
{
    auto const gt = groundTruth(acts, data, vt);

    auto const [cost, warp] = proposal.duration();
    auto const breakDue = proposal.breakDue();
    auto const wait = proposal.waiting();

    auto const costGT = unitDurationCost
                        * static_cast<Cost>(gt.dur - gt.wait);

    bool ok = cost == costGT && warp == gt.warp && wait == gt.wait
              && breakDue == gt.breakDue;
    if (!ok)
    {
        ++mismatch;
        std::printf("    MISMATCH dur_cost %lld vs %lld | warp %lld vs %lld "
                    "| wait %lld vs %lld | breakDue %lld vs %lld\n",
                    cost.get(), costGT.get(), warp.get(), gt.warp.get(),
                    wait.get(), gt.wait.get(), breakDue, gt.breakDue);
    }
    return ok;
}

}  // namespace

int main()
{
    std::printf("test_stream_parity — Proposal streaming vs evaluateForwardPass "
                "(Alternativa A)\n");
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
    // Recipe A: D5-absorbing overnight rest (mandatory DUTY_TIME, ALL_TIMERS).
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

    // Recipe B: two DUTY rests.
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

    // Recipe C: absolute-window CLOCK_TIME rest (due-ness gate).
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

    // Recipe D: DRIVE_TIME + DUTY_TIME resets.
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

    size_t total = 0;
    size_t mismatch = 0;

    for (auto &cs : cases)
    {
        // Build the search route in place (Route is not movable).
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
        VehicleType const &vt = cs.data.vehicleType(route.vehicleType());
        Duration const shiftDur = route.shiftDuration();
        Cost const unitDurCost = route.unitDurationCost();

        auto check = [&](auto const &proposal, std::vector<Activity> const &acts)
        {
            ++total;
            return checkProposal(proposal, acts, cs.data, vt, shiftDur,
                                 unitDurCost, mismatch);
        };

        // 1) whole-route recompositions at 1 and 2 cuts (all interior cuts).
        {
            size_t ok = 0;
            size_t cnt = 0;
            std::vector<size_t> interior;
            for (size_t k = 1; k + 1 < n; ++k)
                interior.push_back(k);
            for (size_t a : interior)
            {
                ++cnt;
                if (check(Route::Proposal(route.before(a), route.after(a + 1)),
                          orig))
                    ++ok;
                for (size_t b = a + 1; b + 1 < n; ++b)
                {
                    ++cnt;
                    if (check(Route::Proposal(route.before(a),
                                              route.between(a + 1, b),
                                              route.after(b + 1)),
                              orig))
                        ++ok;
                }
            }
            std::printf("  [%s] recompositions %zu/%zu exact\n", cs.name, ok,
                        cnt);
            std::fflush(stdout);
        }

        // 2) relocate clients across cuts / breaks (REMOVE+INSERT shape used by
        //    the residual harness and the Relocate operators).
        {
            size_t ok = 0;
            size_t cnt = 0;
            for (size_t i = 1; i + 1 < n; ++i)
            {
                if (!orig[i].isClient())
                    continue;
                for (size_t j = i + 1; j + 1 < n; ++j)
                {
                    bool const targetBreak = orig[j].isCustomBreak();
                    bool const adjacent = j == i + 1;
                    if (!targetBreak && !adjacent && (j - i) % 3 != 0)
                        continue;  // thin out plain intra-span moves

                    std::vector<Activity> m = orig;
                    m.erase(m.begin() + i);
                    m.insert(m.begin() + j, orig[i]);
                    ++cnt;
                    if (check(Route::Proposal(route.before(i - 1),
                                              route.between(i + 1, j),
                                              route.at(i),
                                              route.after(j + 1)),
                              m))
                        ++ok;
                }
            }
            std::printf("  [%s] relocations %zu/%zu exact\n", cs.name, ok, cnt);
            std::fflush(stdout);
        }

        // 3) break shifts through the generic single-node segment path.
        {
            size_t ok = 0;
            size_t cnt = 0;
            for (size_t i = 1; i + 1 < n; ++i)
            {
                if (!orig[i].isCustomBreak())
                    continue;
                // rightwards
                for (size_t step = 1; step <= 4; ++step)
                {
                    size_t j = i + 2 * step;
                    if (j >= n - 1)
                        break;
                    if (orig[j].isCustomBreak())
                        continue;
                    std::vector<Activity> m = orig;
                    m.erase(m.begin() + i);
                    m.insert(m.begin() + j, orig[i]);
                    ++cnt;
                    if (check(Route::Proposal(route.before(i - 1),
                                              route.between(i + 1, j),
                                              NodeSeg(route, i),
                                              route.after(j + 1)),
                              m))
                        ++ok;
                }
                // leftwards
                for (size_t step = 1; step <= 4; ++step)
                {
                    if (i < 2 * step + 1)
                        break;
                    size_t j = i - 2 * step;
                    if (orig[j].isCustomBreak())
                        continue;
                    std::vector<Activity> m = orig;
                    m.erase(m.begin() + i);
                    m.insert(m.begin() + j, orig[i]);
                    ++cnt;
                    if (check(Route::Proposal(route.before(j - 1),
                                              NodeSeg(route, i),
                                              route.between(j, i - 1),
                                              route.after(i + 1)),
                              m))
                        ++ok;
                }
            }
            std::printf("  [%s] shift-break %zu/%zu exact\n", cs.name, ok, cnt);
            std::fflush(stdout);
        }
    }

    std::printf("TOTAL %zu proposals, %zu mismatch(es).\n", total, mismatch);
    return mismatch ? 1 : 0;
}
