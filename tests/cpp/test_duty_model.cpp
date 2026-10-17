// Duty model (vehicle types with duty_limit >= 0; rests with eligible_from >=
// 0). Checks, in C++, what the Python suite cannot reach directly:
//
//   1. The fold: DriveSegment::merge (both overloads) never fires a duty
//      rest -- same state, untouched due mask and first-due clocks -- and
//      inertBreaksFor() holds for every duty-model vehicle type.
//   2. One evaluator: evaluateForwardPass, Route::update() and the solution
//      route (activities constructor) agree on every duty term; a proposal's
//      dutyTerms() and deltaCost<true> equal a fresh update() of the proposed
//      sequence (so neither the composed evaluator nor the streaming walk
//      answers for a duty-model route), for rest moves into the final
//      stretch, client moves crossing duty_limit, and a vehicle without
//      rules.
//   3. SegmentBetween::driveState (the CustomBreak overload of merge) resets
//      at a served rest and not at a parked one.
//   4. ShiftBreak parks an unneeded interior rest after the last stop.
//   5. CostEvaluator::dutyPenalty: the excess weight and saturation at 2^62.
#include "CostEvaluator.h"
#include "CustomBreak.h"
#include "DriveSegment.h"
#include "ProblemData.h"
#include "Route.h"
#include "search/Route.h"
#include "search/ShiftBreak.h"
#include "search/Solution.h"

#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#define CHECK(expr)                                                            \
    do                                                                         \
    {                                                                          \
        if (!(expr))                                                           \
        {                                                                      \
            std::fprintf(stderr, "FAIL: %s at %s:%d\n", #expr, __FILE__,       \
                         __LINE__);                                            \
            std::exit(1);                                                      \
        }                                                                      \
    } while (0)

using pyvrp::Activity;
using pyvrp::Client;
using pyvrp::Coordinate;
using pyvrp::Cost;
using pyvrp::CostEvaluator;
using pyvrp::CustomBreak;
using pyvrp::CustomBreakReset;
using pyvrp::CustomBreakTrigger;
using pyvrp::Depot;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::DurationSegment;
using pyvrp::DutyRates;
using pyvrp::DutyTerms;
using pyvrp::Load;
using pyvrp::LoadSegment;
using pyvrp::Location;
using pyvrp::Matrix;
using pyvrp::ProblemData;
using pyvrp::VehicleType;
using pyvrp::search::DriveSegment;
using pyvrp::search::SegmentProxy;
using Route = pyvrp::search::Route;
using Solution = pyvrp::search::Solution;

namespace
{
int64_t const H = 3600;
Duration const MAX = std::numeric_limits<Duration>::max();
int64_t const T = 8 * H;
int64_t const OT = 2 * H;

CustomBreak dutyRest(size_t id)
{
    return CustomBreak(id, {}, Duration(11 * H),
                       CustomBreakTrigger::DUTY_TIME, Duration(T),
                       CustomBreakReset::ALL_TIMERS, true, Duration(0), 20, {},
                       false, false, Duration(0));
}

// Depot at x = 0, client i at x = i + 1 (hours); 30 min of service each. A
// duty-model vehicle type (T = 8h, OT = 2h) with `rules` rests, or a legacy
// one when `duty` is false.
ProblemData makeData(size_t rules, bool duty = true)
{
    size_t const nClients = 6;
    size_t const numLoc = nClients + 1;

    std::vector<Location> locations;
    for (size_t i = 0; i != numLoc; ++i)
        locations.emplace_back(static_cast<Coordinate>(i), 0.0);

    std::vector<Client> clients;
    for (size_t i = 0; i != nClients; ++i)
        clients.emplace_back(Client(i + 1, {0}, {}, Duration(H / 2),
                                    Duration(0), MAX));

    std::vector<Depot> depots;
    depots.emplace_back(Depot(0, Duration(0), MAX));

    Matrix<Distance> distMat(numLoc, numLoc, Distance(0));
    Matrix<Duration> durMat(numLoc, numLoc, Duration(0));
    for (size_t i = 0; i != numLoc; ++i)
        for (size_t j = 0; j != numLoc; ++j)
        {
            auto const d = static_cast<int64_t>(i > j ? i - j : j - i) * H;
            distMat(i, j) = Distance(d);
            durMat(i, j) = Duration(d);
        }

    std::vector<CustomBreak> breaks;
    for (size_t k = 0; k != rules; ++k)
        breaks.push_back(dutyRest(k));

    std::vector<VehicleType> vts;
    if (duty)
        vts.emplace_back(1, std::vector<Load>{9999}, 0, 0, 0, 0, MAX, MAX,
                         std::numeric_limits<Distance>::max(), 0, 20, 0,
                         Duration(0), std::vector<Load>{},
                         std::vector<size_t>{},
                         std::numeric_limits<size_t>::max(), 0, 0, breaks,
                         false, "duty", Duration(T), Duration(T + OT),
                         Cost(1'000'000'000), Cost(21), Cost(1000), Cost(21),
                         Cost(1'000'000), Cost(100'000'000));
    else
        vts.emplace_back(1, std::vector<Load>{9999}, 0, 0, 0, 0, MAX, MAX,
                         std::numeric_limits<Distance>::max(), 0, 20, 0,
                         Duration(0), std::vector<Load>{},
                         std::vector<size_t>{},
                         std::numeric_limits<size_t>::max(), 0, 0,
                         std::vector<CustomBreak>{}, false, "legacy");

    return ProblemData(std::move(locations), std::move(clients),
                       std::move(depots), std::move(vts),
                       std::vector<Matrix<Distance>>{distMat},
                       std::vector<Matrix<Duration>>{durMat});
}

Activity cl(size_t idx) { return {Activity::ActivityType::CLIENT, idx}; }
Activity brk(size_t id) { return {Activity::ActivityType::CUSTOM_BREAK, id}; }

// A single node of a route, as a proposal segment.
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
};

struct Built
{
    Solution sol;
    std::vector<Route::Node> store;

    Built(ProblemData const &data, std::vector<Activity> const &acts)
        : sol(data)
    {
        store.reserve(acts.size());
        for (auto const &act : acts)
            store.emplace_back(act);
        auto &route = sol.routes[0];
        for (auto &node : store)
            route.push_back(&node);
        route.update();
    }

    Route const &route() const { return sol.routes[0]; }
};

bool sameTerms(DutyTerms const &a, DutyTerms const &b) { return a == b; }

pyvrp::search::ForwardEvalResult reference(ProblemData const &data,
                                           std::vector<Activity> const &acts)
{
    std::vector<Activity> flat;
    std::vector<size_t> locs;
    flat.emplace_back(Activity::ActivityType::DEPOT, 0);
    locs.push_back(0);
    for (auto const &act : acts)
    {
        flat.push_back(act);
        locs.push_back(act.isClient() ? data.client(act.idx()).location
                                      : locs.back());
    }
    flat.emplace_back(Activity::ActivityType::DEPOT, 0);
    locs.push_back(0);
    std::vector<Duration> atSecond(flat.size());
    return pyvrp::search::evaluateForwardPass(flat, locs, atSecond, nullptr,
                                              nullptr, data,
                                              data.vehicleType(0));
}

void checkFold()
{
    static_assert(sizeof(DriveSegment) == 40, "DriveSegment layout changed");

    auto const data = makeData(2);
    auto const &vt = data.vehicleType(0);
    CHECK(vt.hasDutyModel());
    CHECK(pyvrp::search::inertBreaksFor(vt));
    CHECK(!pyvrp::search::inertBreaksFor(makeData(0, false).vehicleType(0))
          || pyvrp::search::inertDiscardedBreak != 0);

    std::vector<pyvrp::BreakRule> const none;
    std::vector<CustomBreak> const noBreaks;
    for (int64_t duty : {int64_t(0), T - 1, T, T + 1, 3 * T})
        for (uint16_t taken : {uint16_t(0), uint16_t(1), uint16_t(3)})
        {
            DriveSegment const first(duty / 2, duty, duty, taken, 1000);
            DriveSegment const second(0, H / 2, H / 2, 0, 0);
            Duration const at(1000 + duty + 2 * H);

            uint16_t mask1 = 0, mask2 = 0;
            int64_t due1[2] = {-1, -1}, due2[2] = {-1, -1};
            auto const a = DriveSegment::merge(Duration(H), first, second,
                                               vt.breakRules, at, 0,
                                               Duration(0), &mask1, due1);
            auto const b = DriveSegment::merge(Duration(H), first, second,
                                               none, at, 0, Duration(0),
                                               &mask2, due2);
            CHECK(a.driveTime_ == b.driveTime_ && a.workTime_ == b.workTime_
                  && a.dutyTime_ == b.dutyTime_
                  && a.lastResetAt_ == b.lastResetAt_
                  && a.breaksTakenMask_ == b.breaksTakenMask_);
            CHECK(mask1 == 0 && due1[0] == -1 && due1[1] == -1);

            auto const c = DriveSegment::merge(Duration(H), first, second,
                                               vt.custom_breaks, at, 0,
                                               Duration(0), &mask1, due1);
            auto const d = DriveSegment::merge(Duration(H), first, second,
                                               noBreaks, at, 0, Duration(0),
                                               &mask2, due2);
            CHECK(c.dutyTime_ == d.dutyTime_ && c.lastResetAt_ == d.lastResetAt_
                  && c.breaksTakenMask_ == d.breaksTakenMask_);
            CHECK(mask1 == 0 && due1[0] == -1 && due1[1] == -1);
        }

    // The decision helpers.
    DriveSegment const drs(0, 0, 100, 0, 0);
    CHECK(pyvrp::search::isBreakServable(drs, vt.breakRules[0], Duration(0),
                                         false));
    CHECK(!pyvrp::search::isBreakServable(drs, vt.breakRules[0], Duration(0),
                                          true));
    std::printf("  fold: duty rests never fire; helpers ok\n");
}

// Every evaluator on the same sequence.
void checkEvaluators(ProblemData const &data,
                     std::vector<Activity> const &acts,
                     DutyTerms const &expected)
{
    auto const ref = reference(data, acts);
    CHECK(sameTerms(ref.duty, expected));

    Built built(data, acts);
    CHECK(sameTerms(built.route().dutyTerms(), expected));

    pyvrp::Route const solRoute(data, acts, 0);
    CHECK(sameTerms(solRoute.dutyTerms(), expected));

    CostEvaluator const ev({0}, 300, 0, 0, 21.4);
    CHECK(ev.penalisedCost(built.route()) == ev.penalisedCost(solRoute));
}

void checkTerms()
{
    auto const data = makeData(2);

    // Rest after C3: J1 = 4h + 4 x 30min = 6h (anticipation 2h); J2 = 1h +
    // 30min + 1h + 30min + 6h = 9h (band 1h). Needed: 6 + 0 + 9 > 10.
    DutyTerms expected;
    expected.anticipation = 2 * H;
    expected.bandDepot = H;
    expected.restsNeeded = 1;
    checkEvaluators(data,
                    {cl(0), cl(1), cl(2), cl(3), brk(0), cl(4), cl(5), brk(1)},
                    expected);

    // No rest at all: 15h, excess 5h, band 2h.
    DutyTerms noRest;
    noRest.excessCapped = 5 * H;
    noRest.bandDepot = 2 * H;
    checkEvaluators(data, {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5)}, noRest);

    // Zero rules on a duty-model vehicle: the same fold.
    checkEvaluators(makeData(0), {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5)},
                    noRest);
    std::printf("  evaluators agree\n");
}

// The proposal of `proposed` (built from `base`'s segments) prices exactly as
// a fresh update() of `proposed`.
template <typename Proposal>
void checkProposal(ProblemData const &data,
                   Route const &base,
                   Proposal const &proposal,
                   std::vector<Activity> const &proposed)
{
    CostEvaluator const ev({0}, 300, 0, 0, 21.4);
    Built fresh(data, proposed);

    CHECK(sameTerms(proposal.dutyTerms(), fresh.route().dutyTerms()));
    Cost delta = 0;
    ev.deltaCost<true>(delta, proposal);
    CHECK(delta == ev.penalisedCost(fresh.route()) - ev.penalisedCost(base));
}

void checkProposals()
{
    auto const data = makeData(2);
    std::vector<Activity> const acts
        = {cl(0), cl(1), cl(2), cl(3), brk(0), cl(4), cl(5), brk(1)};
    Built built(data, acts);
    auto const &R = built.route();
    CHECK(R.hasDutyModel());

    // Positions: 0 depot, 1..4 C0..C3, 5 B0, 6 C4, 7 C5, 8 B1, 9 depot.
    // Rest B0 to the final stretch (after C5, next to B1).
    checkProposal(data, R,
                  Route::Proposal(R.before(4), R.between(6, 7),
                                  NodeSeg(R, 5), R.after(8)),
                  {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5), brk(0), brk(1)});
    // Rest B0 to the very end, after B1.
    checkProposal(data, R,
                  Route::Proposal(R.before(4), R.between(6, 8),
                                  NodeSeg(R, 5), R.after(9)),
                  {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5), brk(1), brk(0)});
    // Rest B0 earlier (after C1).
    checkProposal(data, R,
                  Route::Proposal(R.before(2), NodeSeg(R, 5), R.between(3, 4),
                                  R.after(6)),
                  {cl(0), cl(1), brk(0), cl(2), cl(3), cl(4), cl(5), brk(1)});
    // Client moves crossing duty_limit: C5 to the front, C0 to the back.
    checkProposal(data, R,
                  Route::Proposal(R.before(0), NodeSeg(R, 7), R.between(1, 6),
                                  R.after(8)),
                  {cl(5), cl(0), cl(1), cl(2), cl(3), brk(0), cl(4), brk(1)});
    checkProposal(data, R,
                  Route::Proposal(R.before(0), R.between(2, 7), NodeSeg(R, 1),
                                  R.after(8)),
                  {cl(1), cl(2), cl(3), brk(0), cl(4), cl(5), cl(0), brk(1)});
    // Remove the rest: the route passes duty_limit.
    checkProposal(data, R, Route::Proposal(R.before(4), R.after(6)),
                  {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5), brk(1)});

    // Zero rules on a duty-model vehicle.
    auto const zero = makeData(0);
    Built z(zero, {cl(0), cl(1), cl(2), cl(3), cl(4), cl(5)});
    auto const &Z = z.route();
    checkProposal(zero, Z,
                  Route::Proposal(Z.before(0), NodeSeg(Z, 6), Z.between(1, 5),
                                  Z.after(7)),
                  {cl(5), cl(0), cl(1), cl(2), cl(3), cl(4)});
    checkProposal(zero, Z, Route::Proposal(Z.before(3), Z.after(5)),
                  {cl(0), cl(1), cl(2), cl(4), cl(5)});
    std::printf("  proposals == update()\n");
}

void checkDriveState()
{
    auto const data = makeData(2);
    auto const &mat = data.durationMatrix(0);
    (void)mat;

    // Served interior rest: the state over the whole route restarts there.
    Built served(data, {cl(0), cl(1), cl(2), cl(3), brk(0), cl(4), cl(5)});
    auto const &S = served.route();
    auto const s = S.between(0, S.size() - 1).driveState(0);
    CHECK(s.breaksTakenMask_ & 1u);
    CHECK(s.dutyTime_ == 9 * H);  // the final stretch

    // Parked rest (not needed before the return): no reset.
    Built parked(data, {cl(0), cl(1), brk(0)});
    auto const &P = parked.route();
    auto const p = P.between(0, P.size() - 1).driveState(0);
    CHECK((p.breaksTakenMask_ & 1u) == 0);
    CHECK(p.dutyTime_ == 2 * H + H + 2 * H);  // 1h, 30min, 1h, 30min, 2h
    CHECK(P.breaksServed().empty());
    CHECK(S.breaksServed().size() == 1);
    std::printf("  driveState: served resets, parked does not\n");
}

void checkShiftBreak()
{
    // A short route (5h of duty) with an interior rest: unneeded and costly.
    auto const data = makeData(1);
    Built built(data, {cl(0), brk(0), cl(1)});
    auto &route = built.sol.routes[0];
    CHECK(route.restsUnneeded() == 1);

    CostEvaluator const ev({0}, 300, 0, 0, 21.4);
    auto const before = ev.penalisedCost(route);

    pyvrp::search::ShiftBreak op(data);
    auto *rest = route[2];
    CHECK(rest->isCustomBreak());
    auto const [delta, apply] = op.evaluate(rest, ev);
    CHECK(apply && delta < 0);
    op.apply(rest);
    route.update();
    CHECK(route[route.size() - 2]->isCustomBreak());  // parked at n - 1
    CHECK(route.restsUnneeded() == 0 && route.restsNeeded() == 0);
    CHECK(ev.penalisedCost(route) - before == delta);
    std::printf("  ShiftBreak parks the rest after the last stop\n");
}

void checkPenalty()
{
    DutyRates rates{10 * H, 5000, 21, 1000, 21, 1'000'000, 100'000'000};
    DutyTerms terms;
    terms.excessCapped = 2;
    terms.depotWarp = 3;

    // Excess weight max(10 x tw, duty_excess_rate).
    CostEvaluator const low({0}, 1, 0, 0, 0);
    CHECK(low.dutyExcessWeight(rates.dutyExcessRate) == 5000);
    CHECK(low.dutyPenalty(Duration(3), terms, rates) == 5 * 5000);
    CostEvaluator const high({0}, 1000, 0, 0, 0);
    CHECK(high.dutyExcessWeight(rates.dutyExcessRate) == 10'000);

    // Client warp at the tw penalty, the late return at the excess weight.
    CHECK(low.dutyPenalty(Duration(10), terms, rates) == 7 + 5 * 5000);

    // Saturation: products and sum at 2^62.
    DutyRates huge = rates;
    huge.dutyExcessRate = int64_t(1) << 61;
    huge.restUnneededCost = int64_t(1) << 61;
    DutyTerms many;
    many.excessCapped = 1000;
    many.restsUnneeded = 1000;
    CHECK(low.dutyPenalty(Duration(0), many, huge).get()
          == CostEvaluator::DUTY_SAT);
    std::printf("  dutyPenalty weights and saturation\n");
}
}  // namespace

int main()
{
    std::printf("test_duty_model\n");
    checkFold();
    checkTerms();
    checkProposals();
    checkDriveState();
    checkShiftBreak();
    checkPenalty();
    std::printf("test_duty_model: PASS\n");
    return 0;
}
