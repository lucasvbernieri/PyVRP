#ifndef PYVRP_SEARCH_ROUTE_H
#define PYVRP_SEARCH_ROUTE_H

#include "Activity.h"
#include "CostEvaluator.h"
#include "DriveSegment.h"
#include "DurationSegment.h"
#include "LoadSegment.h"
#include "PhaseProfile.h"
#include "ProblemData.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdio>
#include <concepts>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace pyvrp::search
{
class SegmentProxy;  // forward declaration

// This defines the minimal interface required for a segment of activities.
template <typename T>
concept Segment
    = requires(T arg, size_t profile, size_t dimension, size_t idx) {
          { arg.route() };
          { arg.front() } -> std::convertible_to<SegmentProxy>;
          { arg.back() } -> std::convertible_to<SegmentProxy>;
          { arg.size() } -> std::same_as<size_t>;
          { arg.numClients() } -> std::same_as<size_t>;
          { arg.numPickups() } -> std::same_as<size_t>;
          { arg.startsAtReloadDepot() } -> std::same_as<bool>;
          { arg.endsAtReloadDepot() } -> std::same_as<bool>;
          { arg.distance(profile) } -> std::convertible_to<Distance>;
          { arg.duration(profile) } -> std::convertible_to<DurationSegment>;
          { arg.load(dimension) } -> std::convertible_to<LoadSegment>;
      };

namespace detail
{
template <class Tuple, std::size_t... Indices>
auto constexpr reverse_impl(Tuple &&tuple, std::index_sequence<Indices...>)
{
    return std::forward_as_tuple(std::get<sizeof...(Indices) - 1 - Indices>(
        std::forward<Tuple>(tuple))...);
}

template <class Tuple> auto constexpr reverse(Tuple &&tuple)
{
    auto constexpr size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
    auto constexpr indices = std::make_index_sequence<size>{};
    return reverse_impl(std::forward<Tuple>(tuple), indices);
}

/**
 * Small-buffer array used by the per-candidate break bookkeeping in
 * Route::Proposal::runStreamForward(). The number of distinct break ids per
 * vehicle is capped at 16 by the uint16_t taken/due masks, so in practice the
 * stack buffer always wins and the proposal path performs zero heap
 * allocations; the vector fallback keeps the code correct for hypothetical
 * larger id spaces (where the masks already alias via ``id & 0xF``).
 */
template <typename T, std::size_t N> class SmallBuf
{
    T stack_[N];
    std::vector<T> heap_;
    T *ptr_;
    std::size_t size_;

public:
    SmallBuf(std::size_t n, T const init) : size_(n)
    {
        if (n <= N) [[likely]]
        {
            ptr_ = stack_;
            for (std::size_t i = 0; i != n; ++i)
                ptr_[i] = init;
        }
        else
        {
            heap_.assign(n, init);
            ptr_ = heap_.data();
        }
    }

    SmallBuf(SmallBuf const &) = delete;
    SmallBuf &operator=(SmallBuf const &) = delete;

    [[nodiscard]] T &operator[](std::size_t i) { return ptr_[i]; }
    [[nodiscard]] T const &operator[](std::size_t i) const { return ptr_[i]; }
    [[nodiscard]] T *data() { return ptr_; }
    [[nodiscard]] std::size_t size() const { return size_; }

    void fill(T const value)
    {
        for (std::size_t i = 0; i != size_; ++i)
            ptr_[i] = value;
    }

    void copyFrom(SmallBuf const &other)
    {
        for (std::size_t i = 0; i != size_; ++i)
            ptr_[i] = other.ptr_[i];
    }
};

#ifdef PYVRP_STREAM_STATS
/**
 * Opt-in counters for the break-path proposal evaluator. Compiled out unless
 * PYVRP_STREAM_STATS is defined, so the shipped binary pays nothing. They
 * answer the two questions that decide where the remaining cost lives: how
 * many nodes a candidate actually simulates (L) and how often the second
 * round has to run (f).
 */
struct StreamStats
{
    unsigned long long calls = 0;
    unsigned long long seeded = 0;
    unsigned long long shortCircuit = 0;
    unsigned long long round2 = 0;
    // Which of round 2's two triggers fired. They are not exclusive; both is
    // counted separately so the three add up to round2.
    unsigned long long r2Absorb = 0;   // D5 rest absorbed waiting only
    unsigned long long r2Cleared = 0;  // a break window was cleared only
    unsigned long long r2Both = 0;
    unsigned long long r2Warp = 0;    // clearing that had to fall back to the
                                      // legacy two-round path (past close)
    unsigned long long clrInline = 0; // clearing settled in a single round
    unsigned long long locTried = 0, locHits = 0, locEndBad = 0;
    unsigned long long locChecked = 0, locDiffDur = 0, locDiffWarp = 0;
    unsigned long long locDiffDue = 0, locDiffFirst = 0, locDiffEnd = 0;
    unsigned long long locDiffMand = 0;
    unsigned long long prescanNodes = 0;
    unsigned long long roundNodes = 0;
    unsigned long long flatNodes = 0;
    unsigned long long scHits = 0;       // candidates whose tail is inert
    unsigned long long scSaved = 0;      // nodes such a candidate could skip
    unsigned long long scSuffixOk = 0;   // ... and the tail is a route suffix
    // Why the collapse did NOT fire, attributed to the first gate that failed
    // at the last position where firing was still possible.
    unsigned long long scNoStruct = 0;   // last segment is not a clean suffix
    unsigned long long scNoRemain = 0;   // a break node still lies ahead
    unsigned long long scNoServed = 0;   // a mandatory break is not served
    unsigned long long scNoTaken = 0;    // a mandatory/ALL_TIMERS bit is unset
    unsigned long long scNoInert = 0;    // a served break has no first-due yet

    ~StreamStats()
    {
        if (!calls)
            return;
        std::fprintf(stderr,
                     "[stream-stats] calls=%llu seeded=%.3f short=%.3f "
                     "round2_f=%.3f L_prescan=%.2f L_round=%.2f n_flat=%.2f "
                     "sc_hit=%.3f sc_saved=%.2f\n"
                     "[stream-stats] round2-why: absorb=%.3f cleared=%.3f "
                     "both=%.3f | clear-inline=%.3f clear-warp=%.3f\n"
                     "[stream-stats] loc: tried=%.3f hit=%.3f endpoint-reject=%.3f\n"
                     "[stream-stats] loc-diff: checked=%llu dur=%llu warp=%llu "
                     "due=%llu first=%llu (mandatory=%llu) end=%llu\n"
                     "[stream-stats] why-no-collapse: struct=%.3f "
                     "remain=%.3f served=%.3f taken=%.3f inert=%.3f\n",
                     calls,
                     double(seeded) / double(calls),
                     double(shortCircuit) / double(calls),
                     double(round2) / double(calls),
                     double(prescanNodes) / double(calls),
                     double(roundNodes) / double(calls),
                     double(flatNodes) / double(calls),
                     double(scHits) / double(calls),
                     double(scSaved) / double(calls),
                     double(r2Absorb) / double(calls),
                     double(r2Cleared) / double(calls),
                     double(r2Both) / double(calls),
                     double(clrInline) / double(calls),
                     double(r2Warp) / double(calls),
                     double(locTried) / double(calls),
                     double(locHits) / double(calls),
                     double(locEndBad) / double(calls),
                     locChecked, locDiffDur, locDiffWarp,
                     locDiffDue, locDiffFirst, locDiffMand, locDiffEnd,
                     double(scNoStruct) / double(calls),
                     double(scNoRemain) / double(calls),
                     double(scNoServed) / double(calls),
                     double(scNoTaken) / double(calls),
                     double(scNoInert) / double(calls));
    }
};

inline StreamStats streamStats{};
#define PYVRP_STAT(field, by) (::pyvrp::search::detail::streamStats.field += (by))
#else
#define PYVRP_STAT(field, by) ((void)0)
#endif
}  // namespace detail

/**
 * Simple proxy that can be queried for activity and location attributes.
 */
class SegmentProxy
{
    Activity activity_;
    size_t location_;

public:
    inline SegmentProxy(Activity activity, size_t location);
    inline Activity activity() const;
    inline size_t location() const;
};

/**
 * Returns the tw_early of the client or depot underlying the given activity.
 * Custom break activities return 0 (the break's own tw_early is stored in
 * the DurationSegment at its position and used via atSecond computation).
 */
inline Duration twEarlyFromActivity(Activity const &activity,
                                    ProblemData const &data)
{
    if (activity.isClient())
        return data.client(activity.idx()).twEarly;
    if (activity.isDepot())
        return data.depot(activity.idx()).twEarly;
    return 0;  // CUSTOM_BREAK
}

/**
 * This ``Route`` class supports fast delta cost computations and in-place
 * modification. It can be used to implement move evaluations.
 *
 * A ``Route`` object tracks a full route, including the depots. The clients
 * and depots on the route can be accessed using ``Route::operator[]`` on a
 * ``route`` object.
 *
 * .. note::
 *
 *    Modifications to the ``Route`` object do not immediately propagate to its
 *    statistics like time window, load and distance data. To make that happen,
 *    ``Route::update()`` must be called!
 */
class Route
{
public:
    /**
     * A simple class that tracks a proposed, non-empty route structure. This
     * new structure can be efficiently evaluated by calling appropriate member
     * functions, detailing the newly proposed route's statistics.
     */
    template <Segment... Segments> class Proposal
    {
        std::tuple<Segments...> segments_;

        // Cached breakDue value, computed during duration() when hasBreaks()
        // is true. -1 means not yet computed; breakDue() does its own fold
        // as a fallback.
        mutable int64_t breakDue_ = -1;

        // Cached waiting value, computed during duration(). -1 means not yet
        // computed; waiting() does its own fold as a fallback.
        mutable int64_t waiting_ = -1;

        // Break-aware forward evaluation computed by STREAMING over the
        // proposal's flat sequence (no fwdActs_/fwdLocs_/atSecond arrays are
        // materialised per candidate). Shares the evaluateForwardPass
        // semantics bit-for-bit (same D5 due-gate / wait / reload rules); used
        // by duration() when hasBreaks() or hasSetup() routes.
        ForwardEvalResult runStreamForward() const;

        // Composes the proposal's cached per-segment DurationSegments into a
        // single DurationSegment via the associative merge/finaliseFront
        // monoid -- exactly the fold duration() itself performs on its
        // no-break/no-setup path. Shared by that path and by
        // durationLowerBoundFold(), which reuses the same composition as a
        // (candidate, unverified) lower bound on break-configured routes,
        // where it does NOT apply break decisions (D5 rest extension, due
        // gating, window narrowing) and so is not exact there.
        DurationSegment foldDuration() const;

    public:
        Proposal(Segments &&...segments);

        /**
         * The proposal's route. This is the route associated with the first
         * and last segments, and determines the vehicle type and route profile
         * used when evaluating the proposal.
         */
        Route const *route() const;

        /**
         * Returns the fixed vehicle cost incurred by the proposed route.
         */
        Cost fixedVehicleCost() const;

        /**
         * Returns whether the proposed route is empty (no clients and no
         * shipments). Empty proposals can arise when a move removes the last
         * visit from a route; their statistics are trivially zero.
         */
        bool empty() const;

        /**
         * Returns the (distance cost, excess distance) attributes of the
         * proposed route.
         */
        std::pair<Cost, Distance> distance() const;

        /**
         * Returns the (duration cost, time warp) attributes of the proposed
         * route.
         */
        std::pair<Cost, Duration> duration() const;

        /**
         * Returns a cheap, admissible lower bound on what ``duration()``
         * will add to the objective's duration-related cost terms (see
         * ``CostEvaluator::deltaCost``): ``unitDurationCost * (unavoidable
         * travel + minimum service)`` over the proposed sequence. This is
         * always at most ``duration()``'s actual ``(duration - waiting)``
         * contribution, so it is safe to use to skip calling ``duration()``
         * outright when the running cost delta already cannot recover.
         * Returns 0 -- itself a trivially valid, if not tight, lower bound
         * -- when the underlying route(s) have no cached prefix sums (i.e.
         * outside the break/setup path, where ``duration()`` is cheap
         * anyway).
         */
        Duration durationLowerBound() const;

        /**
         * EXPERIMENTAL -- candidate lower bound, NOT verified admissible.
         *
         * Returns ``(duration - waiting)`` from folding the proposal's
         * cached per-segment DurationSegments via the same associative
         * merge/finaliseFront composition ``duration()`` uses on its
         * no-break/no-setup path (see ``foldDuration()``). That composition
         * does not apply break decisions (D5 rest extension, due gating,
         * window narrowing), so on break-configured routes this is only a
         * *candidate* bound -- its admissibility (never overestimating what
         * ``duration()`` and the terms that follow it actually add) must be
         * verified empirically (see the ``PYVRP_STREAM_STATS`` violation
         * counters in ``CostEvaluator::deltaCost``) before it is used to
         * prune anything.
         */
        Duration durationLowerBoundFold() const;

        /**
         * Returns the excess load of the proposed route.
         */
        Load excessLoad(size_t dimension) const;

        /**
         * Returns the number of mandatory break violations (breakDue) of the
         * proposed route. Returns 0 when no breaks are configured. If
         * ``duration()`` was called first, the cached value is used; otherwise
         * a dedicated fold over the segment chain is performed.
         */
        int64_t breakDue() const;

        /**
         * Returns the total idle waiting time of the proposed route. If
         * ``duration()`` was called first, the cached value is used; otherwise
         * a dedicated fold over the segment chain is performed.
         */
        Duration waiting() const;
    };

    template <Segment... Segments>  // deduct guide for forward reference pack
    Proposal(Segments &&...) -> Proposal<Segments...>;

    /**
     * Light wrapper class around an activity. This class tracks the route it
     * is in, and the position in that route.
     */
    class Node
    {
        friend class Route;

        Activity activity_;
        size_t pos_;    // Position in the route.
        size_t trip_;   // Trip index.
        Route *route_;  // Indicates membership of a route, if any

    public:
        Node(Activity::ActivityType type, size_t idx);
        Node(Activity activity);

        /**
         * Returns the activity modelled with this node.
         */
        [[nodiscard]] inline Activity activity() const;

        /**
         * Index of the activity modelled with this node.
         */
        [[nodiscard]] inline size_t idx() const;

        /**
         * Type of activity modelled with this node.
         */
        [[nodiscard]] inline Activity::ActivityType type() const;

        /**
         * Returns this node's position in a route. This value is ``0`` when
         * the node is *not* in a route.
         */
        [[nodiscard]] inline size_t pos() const;

        /**
         * Returns this node's assigned trip number.  This value is ``0`` when
         * the node is *not* in a route.
         */
        [[nodiscard]] inline size_t trip() const;

        /**
         * Returns the route this node is currently in. If the node is not in
         * a route, this returns ``None`` (C++: ``nullptr``).
         */
        [[nodiscard]] inline Route *route() const;

        /**
         * Returns whether this node is a client.
         */
        [[nodiscard]] inline bool isClient() const;

        /**
         * Returns whether this node is a depot.
         */
        [[nodiscard]] inline bool isDepot() const;

        /**
         * Returns whether this node is a start depot.
         */
        [[nodiscard]] inline bool isStartDepot() const;

        /**
         * Returns whether this node is an end depot.
         */
        [[nodiscard]] inline bool isEndDepot() const;

        /**
         * Returns whether this node is a custom break.
         */
        [[nodiscard]] inline bool isCustomBreak() const;

        /**
         * Returns whether this node is a reload depot.
         */
        [[nodiscard]] inline bool isReloadDepot() const;

        /**
         * Returns whether this node is part of a shipment.
         */
        [[nodiscard]] inline bool isShipment() const;

        /**
         * Returns whether this node is a pickup step of a shipment.
         */
        [[nodiscard]] inline bool isPickup() const;

        /**
         * Returns whether this node is a delivery step of a shipment.
         */
        [[nodiscard]] inline bool isDelivery() const;

        /**
         * Assigns the node to the given route, at the given position, in the
         * given trip.
         */
        void assign(Route *route, size_t pos, size_t trip);

        /**
         * Removes the node from its assigned route, if any.
         */
        void unassign();
    };

    /**
     * Class storing data related to the route segment starting at ``start``,
     * and ending at the end depot (inclusive).
     */
    class SegmentAfter
    {
        Route const &route_;
        size_t const start;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;  // at start
        inline SegmentProxy back() const;   // at end depot

        inline size_t size() const;
        inline size_t numClients() const;
        inline size_t numPickups() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentAfter(Route const &route, size_t start);

        /** Index of the first node in this segment. */
        inline size_t startIdx() const;

        /** Index of the last node in this segment (end depot, always
         *  route.size() - 1). */
        inline size_t endIdx() const;

        inline Distance distance(size_t profile) const;
        inline DurationSegment const &duration(size_t profile) const;
        inline LoadSegment const &load(size_t dimension) const;
        inline DriveSegment driveState(size_t profile) const;
    };

    /**
     * Class storing data related to the route segment starting at the start
     * depot, and ending at ``end`` (inclusive).
     */
    class SegmentBefore
    {
        Route const &route_;
        size_t const end;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;  // at start depot
        inline SegmentProxy back() const;   // at end

        inline size_t size() const;
        inline size_t numClients() const;
        inline size_t numPickups() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBefore(Route const &route, size_t end);

        /** Index of the first node in this segment (always 0, the start
         *  depot). */
        inline size_t startIdx() const;

        /** Index of the last node in this segment. */
        inline size_t endIdx() const;

        inline Distance distance(size_t profile) const;
        inline DurationSegment const &duration(size_t profile) const;
        inline LoadSegment const &load(size_t dimension) const;
        inline DriveSegment driveState(size_t profile) const;
    };

    /**
     * Class storing data related to the route segment starting at ``start``,
     * and ending at ``end`` (inclusive). The segment must consist of a single
     * trip, possibly including its ending depot.
     */
    class SegmentBetween
    {
    protected:
        Route const &route_;
        size_t start;
        size_t end;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;  // at start
        inline SegmentProxy back() const;   // at end

        inline size_t size() const;
        inline size_t numClients() const;
        inline size_t numPickups() const;

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBetween(Route const &route, size_t start, size_t end);

        /** Index of the first node in this segment. */
        inline size_t startIdx() const;

        /** Index of the last node in this segment. */
        inline size_t endIdx() const;

        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
        inline LoadSegment load(size_t dimension) const;
        inline DriveSegment driveState(size_t profile) const;
    };

private:
    using LoadSegments = std::vector<LoadSegment>;

    ProblemData const &data;

    VehicleType const &vehicleType_;

    Distance distance_;  // Separately cached cost components
    Cost distanceCost_;
    Distance excessDistance_;
    Duration duration_;
    Cost durationCost_;
    Duration timeWarp_;
    Duration waiting_;
    uint16_t breakDueMask_ = 0;  // bitmask of violated (due) break ids
    int64_t breakDue_ = 0;       // mandatory-break lateness, in SECONDS

    // Effective (possibly extended) service duration of each CUSTOM_BREAK
    // node, indexed by node position (0 for non-break nodes).
    std::vector<Duration> breakServicesAt_;

    std::vector<Node> depots_;  // start, end, and reload depots (in that order)
    std::vector<Node> breaks_;  // CUSTOM_BREAK activities owned by this route

    std::vector<Node *> nodes;      // Nodes in this route
    std::vector<size_t> locations;  // Visited locations in this route

    // Activity of each node, by position. ``nodes`` holds pointers into
    // several different owners, so reading ``nodes[i]->activity()`` is a
    // scattered dereference. The proposal evaluator touches every position of
    // the re-simulated span two to three times per candidate, and there are
    // orders of magnitude more candidates than updates — so the dereference is
    // paid once here and read back sequentially from this array.
    std::vector<Activity> activitiesAt_;

    // Positions of this route's CUSTOM_BREAK nodes, ascending. Break nodes are
    // sparse (one to three per route), so the proposal evaluator's pre-scan
    // reads this instead of walking every position looking for them.
    std::vector<uint32_t> breakPositions_;

    // Max-plus schedule tables, filled by update() alongside atSecondVec.
    //
    //   cumT_[m] = sum of edges up to m + sum of services before m
    //   cumM_[m] = max over j <= m of ( twEarly(j) - cumT_[j] )
    //
    // Together they give the arrival clock at any position from the clock at
    // any earlier one, in O(1) -- see clockAt(). This is the primitive that
    // makes locating a break trigger's crossing a binary search over O(1)
    // probes rather than something needing interior range folds. Validated to
    // 0 mismatches; see docs/BREAK_REGIME_EVAL_FINDINGS.md section 16.
    std::vector<int64_t> cumT_;
    std::vector<int64_t> cumM_;

    // True when any client on this route has a non-zero release time. The
    // single-round break evaluation's clock-invariance argument does not hold
    // under release times (startEarly() clamps to releaseTime_), so it falls
    // back to the two-round scheme when this is set. Refreshed by update().
    bool hasReleaseTimes_ = false;

    std::vector<size_t> numClients_;     // Clients on start -> node (incl.)
    std::vector<size_t> numPickups_;     // Pickups on start -> node (incl.)
    std::vector<size_t> numDeliveries_;  // Deliveries on start -> node (incl.)

    std::vector<Distance> cumDist;  // Dist of start -> node (incl.)

    // Load data, for each load dimension. These vectors form matrices, where
    // the rows index the load dimension, and the columns the nodes.
    std::vector<LoadSegments> loadAt;      // Load data at each node
    std::vector<LoadSegments> loadAfter;   // Load of node -> end (incl)
    std::vector<LoadSegments> loadBefore;  // Load of start -> node (incl)

    std::vector<Load> load_;        // Route loads (for each dimension)
    std::vector<Load> excessLoad_;  // Route excess load (for each dimension)

    // Duration data, for singleton, suffix, and prefix segments. If a segment
    // *ends* at a depot, that depot's service duration is not included, since
    // end depots have no service. In particular, a singleton reload or end
    // depot segment does *not* include service.
    std::vector<DurationSegment> durAt;      // Duration data at each node
    std::vector<DurationSegment> durAfter;   // Dur of node -> end (incl.)
    std::vector<DurationSegment> durBefore;  // Dur of start -> node (incl.)

    // Drive data, for singleton, suffix, and prefix segments. These are only
    // populated when the vehicle type has break rules configured. For users
    // without breaks, all three remain std::nullopt (zero overhead).
    std::optional<std::vector<DriveSegment>> driveAt;
    std::optional<std::vector<DriveSegment>> driveBefore;

    // Prefix sums of, respectively, DURATION edges (start -> node, excl.)
    // and MINIMUM per-node service (start -> node, incl.), mirroring
    // ``cumDist`` above but for the duration side. Populated under the same
    // condition as ``driveAt``/``driveBefore`` (only routes that actually
    // pay for the expensive break/setup-aware Proposal::duration() fold
    // need them). Used by Proposal::durationLowerBound() to compute a
    // cheap, admissible lower bound on duration()'s contribution to the
    // objective, so CostEvaluator::deltaCost can skip that fold for
    // provably non-improving candidates. ``cumSvcLB`` uses the MINIMUM
    // (unextended) service per node -- e.g. a CUSTOM_BREAK's configured
    // ``service``, not any D5 rest extension, since D5 only ever increases
    // it and the bound must never overestimate.
    std::optional<std::vector<Duration>> cumDurEdge;
    std::optional<std::vector<Duration>> cumSvcLB;

    // ---- Alternativa D: per-position forward-pass seed cache -------------
    // ``fwdDrive_`` mirrors the FINAL (post D5 re-run) per-position drive
    // state produced by the shared forward pass (evaluateForwardPass). It is
    // only populated for break-configured routes and used by
    // Proposal::runStreamForward() to fast-forward over an intact route
    // prefix instead of re-simulating it for every candidate.
    std::optional<std::vector<DriveSegment>> fwdDrive_;

    // Per-break-id D3/mutation facts of the CURRENT route (as evaluated by the
    // shared forward pass in update()). Used to seed the streaming re-run at
    // an arbitrary prefix boundary: a break whose occurrence / first-due point
    // lies at or before the boundary must keep its already-decided prefix
    // contribution (arrival at its node, first-due clock, served bit).
    struct RouteBreakSeed
    {
        size_t occPos = std::numeric_limits<size_t>::max();      // route position of the break's node
        Duration arrivalAtOcc = 0;     // final-clock arrival at that node
        bool served = false;           // whether the break was served there
        int64_t firstDueVal = -1;      // final-clock first-due value (seconds)
        size_t firstDuePos = std::numeric_limits<size_t>::max(); // route boundary where it first fired
    };
    std::vector<RouteBreakSeed> breakSeed_;  // sized K when hasBreaks()
    bool breakSeedValid_ = false;

    // Tracks whether the route's cached statistics are in sync with its nodes
    // list. Statistics are only updated after calling ``update()``. If that
    // function has not yet been called after inserting, removing, or swapping
    // nodes, this flag is active. Debug assertions fail on statistics getters
    // when it is set; release getters that need live data (e.g. numClients)
    // fall back to scanning the nodes list.
    bool dirty = false;

public:
    /**
     * @return The client or depot node at the given ``idx``.
     */
    [[nodiscard]] inline Node *operator[](size_t idx);
    [[nodiscard]] inline Node const *operator[](size_t idx) const;

    [[nodiscard]] std::vector<Node *>::const_iterator begin() const;
    [[nodiscard]] std::vector<Node *>::const_iterator end() const;

    /**
     * Tests if this route is feasible.
     *
     * @return true if the route is feasible, false otherwise.
     */
    [[nodiscard]] inline bool isFeasible() const;

    /**
     * Determines whether this route is load-feasible.
     *
     * @return true if the route exceeds the capacity, false otherwise.
     */
    [[nodiscard]] inline bool hasExcessLoad() const;

    /**
     * Determines whether this route is distance-feasible.
     *
     * @return true if the route exceeds the maximum distance constraint, false
     *         otherwise.
     */
    [[nodiscard]] inline bool hasExcessDistance() const;

    /**
     * Determines whether this route is time-feasible.
     *
     * @return true if the route has time warp, false otherwise.
     */
    [[nodiscard]] inline bool hasTimeWarp() const;

    /**
     * Total loads on this route.
     */
    [[nodiscard]] inline std::vector<Load> const &load() const;

    /**
     * Pickup or delivery loads in excess of the vehicle's capacity.
     */
    [[nodiscard]] inline std::vector<Load> const &excessLoad() const;

    /**
     * Travel distance in excess of the assigned vehicle type's maximum
     * distance constraint.
     */
    [[nodiscard]] inline Distance excessDistance() const;

    /**
     * Capacity of the vehicle servicing this route.
     */
    [[nodiscard]] inline std::vector<Load> const &capacity() const;

    /**
     * @return The depot index of this route's starting depot.
     */
    [[nodiscard]] inline size_t startDepot() const;

    /**
     * @return The depot index of this route's ending depot.
     */
    [[nodiscard]] inline size_t endDepot() const;

    /**
     * @return The fixed cost of the vehicle servicing this route.
     */
    [[nodiscard]] inline Cost fixedVehicleCost() const;

    /**
     * @return Total distance travelled on this route.
     */
    [[nodiscard]] inline Distance distance() const;

    /**
     * @return Cost of the distance travelled on this route.
     */
    [[nodiscard]] inline Cost distanceCost() const;

    /**
     * @return Cost per unit of distance travelled on this route.
     */
    [[nodiscard]] inline Cost unitDistanceCost() const;

    /**
     * Returns true if this route has distance-related cost components, either
     * via the objective or via penalised constraints. False otherwise.
     */
    [[nodiscard]] inline bool hasDistanceCost() const;

    /**
     * @return The duration of this route.
     */
    [[nodiscard]] inline Duration duration() const;

    /**
     * @return Overtime of this route.
     */
    [[nodiscard]] inline Duration overtime() const;

    /**
     * @return Cost of this route's duration, including overtime.
     */
    [[nodiscard]] inline Cost durationCost() const;

    /**
     * @return Cost per unit of duration travelled on this route.
     */
    [[nodiscard]] inline Cost unitDurationCost() const;

    /**
     * @return Cost per unit of overtime on this route.
     */
    [[nodiscard]] inline Cost unitOvertimeCost() const;

    /**
     * Returns true if this route has duration-related cost components, either
     * via the objective or via penalised constraints. False otherwise.
     */
    [[nodiscard]] inline bool hasDurationCost() const;

    /**
     * @return The (soft) maximum shift duration that the vehicle servicing this
     *         route supports. This may optionally be extended with overtime.
     */
    [[nodiscard]] inline Duration shiftDuration() const;

    /**
     * @return The (hard) maximum route duration that the vehicle servicing
     *         this route supports.
     */
    [[nodiscard]] inline Duration maxDuration() const;

    /**
     * @return The maximum overtime that the vehicle servicing this route
     *         supports.
     */
    [[nodiscard]] inline Duration maxOvertime() const;

    /**
     * @return The maximum route distance that the vehicle servicing this route
     *         supports.
     */
    [[nodiscard]] inline Distance maxDistance() const;

    /**
     * @return Total time warp on this route.
     */
    [[nodiscard]] inline Duration timeWarp() const;

    /**
     * @return Whether the vehicle servicing this route has any custom break
     *         rules configured.
     */
    [[nodiscard]] inline bool hasBreaks() const;

    /**
     * @return Number of mandatory break violations (breakDue) on this route.
     *         Returns 0 when no breaks are configured.
     */
    [[nodiscard]] inline int64_t breakDue() const;

    /**
     * @return Bitmask of the mandatory break ids that are violated (due) on
     *         this route. Bit ``i`` corresponds to break id ``i`` (max 16 ids
     *         per vehicle). Returns 0 when no breaks are configured.
     */
    [[nodiscard]] inline uint16_t breakDueMask() const;

    /**
     * @return Total idle waiting time on this route (the portion of the
     *         duration that is neither travel, service, setup, nor breaks).
     *         Waiting absorbed into a rest extension (D5) is excluded.
     */
    [[nodiscard]] inline Duration waiting() const;

    /**
     * @return The effective (possibly extended) service duration of the
     *         CUSTOM_BREAK node at the given position, or 0 for non-break
     *         nodes.
     */
    [[nodiscard]] inline Duration breakServiceAt(size_t pos) const;

    /**
     * @return The ids of the custom breaks that were actually served on this
     *         route, i.e. the breaks whose bit is set in the
     *         ``breaksTakenMask_`` AT THE BREAK NODE'S OWN POSITION in the
     *         forward pass (NOT the final mask — the merge re-sets bits when
     *         a trigger fires at boundaries after an ineligible break node,
     *         which would over-report served breaks). Empty when no breaks
     *         are configured or ``driveBefore`` is not populated.
     *
     * .. note::
     *
     *    Pre-existing PyVRP semantics: a break's time window ``close`` is
     *    exclusive (``isValidStart`` requires ``arrival < close``), so a
     *    break arriving exactly at the window close is not served.
     */
    [[nodiscard]] inline std::vector<size_t> breaksServed() const;

    /**
     * @return The routing profile of the vehicle servicing this route.
     */
    [[nodiscard]] inline size_t profile() const;

    /**
     * True if this route has no client visits, false otherwise.
     */
    [[nodiscard]] inline bool empty() const;

    /**
     * Returns whether this route has a CUSTOM_BREAK activity at a position at
     * or after ``pos``. Answered from the cached ascending break positions, so
     * it costs one comparison instead of a scan over the tail.
     */
    /**
     * Whether any client on this route carries a non-zero release time.
     */
    [[nodiscard]] inline bool hasReleaseTimes() const
    {
        return hasReleaseTimes_;
    }

    /**
     * Arrival clock at position ``m`` given the clock at an earlier position
     * ``q``, in O(1). Sound only while NO time warp has accumulated (the
     * stream's clock is duration()+startEarly(), which does not subtract warp,
     * while DurationSegment::merge computes its own arrival as
     * duration_ - timeWarp_ + edge), and only when the caller's own clock at
     * ``q`` is at least ownClockAt(q) -- cumM_ is a prefix max from position 0,
     * so a caller arriving earlier than the route did would inherit window
     * clamps its own prefix never saw.
     */
    [[nodiscard]] inline int64_t clockAt(size_t q,
                                         int64_t clockQ,
                                         size_t m) const
    {
        assert(m < cumT_.size() && q < cumT_.size());
        return cumT_[m] + std::max(clockQ - cumT_[q], cumM_[m]);
    }

    /**
     * This route's OWN arrival clock at ``m``; the anchor bound clockAt()
     * requires.
     */
    [[nodiscard]] inline int64_t ownClockAt(size_t m) const
    {
        assert(m < cumT_.size());
        return cumT_[m] + cumM_[m];
    }

    /**
     * Whether the max-plus tables are populated (routes with breaks).
     */
    [[nodiscard]] inline bool hasClockTables() const
    {
        return !cumT_.empty();
    }

    [[nodiscard]] inline bool hasBreakAtOrAfter(size_t pos) const
    {
        assert(!dirty);
        return !breakPositions_.empty() && breakPositions_.back() >= pos;
    }

    /**
     * Number of activities on this route.
     */
    [[nodiscard]] inline size_t size() const;

    /**
     * Number of clients in this route.
     */
    [[nodiscard]] inline size_t numClients() const;

    /**
     * Number of shipments in this route.
     */
    [[nodiscard]] inline size_t numShipments() const;
    [[nodiscard]] inline size_t numPickups() const;  // same; for convenience

    /**
     * Returns the number of pickup nodes before (and including) end.
     */
    [[nodiscard]] inline size_t numPickups(size_t end) const;

    /**
     * Returns the number of delivery nodes before (and including) end.
     */
    [[nodiscard]] inline size_t numDeliveries(size_t end) const;

    /**
     * Returns the number of start, end, and reload depots in this route.
     */
    [[nodiscard]] inline size_t numDepots() const;

    /**
     * Returns the number of trips in this route.
     */
    [[nodiscard]] inline size_t numTrips() const;

    /**
     * Returns the maximum number of allowed trips for this route.
     */
    [[nodiscard]] inline size_t maxTrips() const;

    /**
     * Returns an object that can be queried for data associated with the node
     * at idx.
     */
    [[nodiscard]] inline SegmentBetween at(size_t idx) const;

    /**
     * Returns an object that can be queried for data associated with the
     * segment starting at start.
     */
    [[nodiscard]] inline SegmentAfter after(size_t start) const;

    /**
     * Returns an object that can be queried for data associated with the
     * segment ending at end.
     */
    [[nodiscard]] inline SegmentBefore before(size_t end) const;

    /**
     * Returns an object that can be queried for data associated with the
     * segment between [start, end].
     */
    [[nodiscard]] inline SegmentBetween between(size_t start, size_t end) const;

    /**
     * @return This route's vehicle type.
     */
    [[nodiscard]] size_t vehicleType() const;

    /**
     * Clears all clients on this route. After calling this method, ``empty()``
     * returns true.
     */
    void clear();

    /**
     * Reserves capacity for at least given ``size`` number of nodes (depots
     * and clients).
     */
    void reserve(size_t size);

    /**
     * Inserts the given node before index ``idx``. Assumes the given index is
     * valid. Depot nodes are copied into internal memory, but of client nodes
     * no ownership is taken.
     */
    void insert(size_t idx, Node *node);

    /**
     * Appends the given node pointer at the end of the route. Depot nodes are
     * copied into internal memory, but of client nodes no ownership is taken.
     */
    void push_back(Node *node);

    /**
     * Removes the node at ``idx`` from the route. Start and end depots cannot
     * be removed.
     */
    void remove(size_t idx);

    /**
     * Swaps the given nodes.
     */
    static void swap(Node *first, Node *second);

    /**
     * Updates this route. To be called after swapping nodes/changing the
     * solution.
     */
    void update();

    bool operator==(Route const &other) const;

    Route(ProblemData const &data, size_t vehicleType);
    ~Route();
};

/**
 * Convenience method accessing the node directly before the argument.
 */
inline Route::Node *p(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->pos() - 1];
}

inline Route::Node const *p(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->pos() - 1];
}

/**
 * Convenience method accessing the node directly after the argument.
 */
inline Route::Node *n(Route::Node *node)
{
    auto &route = *node->route();
    return route[node->pos() + 1];
}

inline Route::Node const *n(Route::Node const *node)
{
    auto const &route = *node->route();
    return route[node->pos() + 1];
}

SegmentProxy::SegmentProxy(Activity activity, size_t location)
    : activity_(activity), location_(location)
{
}

Activity SegmentProxy::activity() const { return activity_; }

size_t SegmentProxy::location() const { return location_; }

Activity Route::Node::activity() const { return activity_; }

size_t Route::Node::idx() const { return activity_.idx(); }

Activity::ActivityType Route::Node::type() const { return activity_.type(); }

size_t Route::Node::pos() const { return pos_; }

size_t Route::Node::trip() const { return trip_; }

Route *Route::Node::route() const { return route_; }

bool Route::Node::isClient() const { return activity_.isClient(); }

bool Route::Node::isDepot() const { return activity_.isDepot(); }

bool Route::Node::isCustomBreak() const { return activity_.isCustomBreak(); }

bool Route::Node::isStartDepot() const
{
    return route_ && this == &route_->depots_[0];
}

bool Route::Node::isEndDepot() const
{
    return route_ && this == &route_->depots_[1];
}

bool Route::Node::isReloadDepot() const
{
    return isDepot() && !isStartDepot() && !isEndDepot();
}

bool Route::Node::isShipment() const { return activity_.isShipment(); }

bool Route::Node::isPickup() const { return activity_.isPickup(); }

bool Route::Node::isDelivery() const { return activity_.isDelivery(); }

Route::SegmentAfter::SegmentAfter(Route const &route, size_t start)
    : route_(route), start(start)
{
    assert(start < route.size());
}

Route::SegmentBefore::SegmentBefore(Route const &route, size_t end)
    : route_(route), end(end)
{
    assert(end < route.size());
}

Route::SegmentBetween::SegmentBetween(Route const &route,
                                      size_t start,
                                      size_t end)
    : route_(route), start(start), end(end)
{
    assert(start <= end && end < route.size());

    // The segment must consist of a single trip only, possibly including the
    // depot that begins the next trip (and ends this one). So the difference
    // in trips is at most one.
    assert(route[end]->trip() - route[start]->trip() <= route[end]->isDepot());
}

size_t Route::SegmentAfter::startIdx() const { return start; }

size_t Route::SegmentAfter::endIdx() const { return route_.size() - 1; }

size_t Route::SegmentBefore::startIdx() const { return 0; }

size_t Route::SegmentBefore::endIdx() const { return end; }

size_t Route::SegmentBetween::startIdx() const { return start; }

size_t Route::SegmentBetween::endIdx() const { return end; }

Distance Route::SegmentAfter::distance([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return {route_.cumDist.back() - route_.cumDist[start]};
}

DurationSegment const &
Route::SegmentAfter::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durAfter[start];
}

LoadSegment const &Route::SegmentAfter::load(size_t dimension) const
{
    return route_.loadAfter[dimension][start];
}

Distance Route::SegmentBefore::distance([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.cumDist[end];
}

DurationSegment const &
Route::SegmentBefore::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durBefore[end];
}

LoadSegment const &Route::SegmentBefore::load(size_t dimension) const
{
    return route_.loadBefore[dimension][end];
}

// No caller on the cost path -- the search operators reach drive state through
// the proposal evaluator, not through segments. It stays because
// tests/cpp/test_segment_fold_parity.cpp uses it as the handle for checking
// that the cached drive fold reproduces the forward pass, which is worth
// keeping even though the shipped binary never calls it.
DriveSegment
Route::SegmentBefore::driveState([[maybe_unused]] size_t profile) const
{
    if (!route_.driveBefore.has_value())
        return {};
    return route_.driveBefore.value()[end];
}

Route const *Route::SegmentBefore::route() const { return &route_; }

SegmentProxy Route::SegmentBefore::front() const
{
    return {route_.nodes.front()->activity(), route_.locations.front()};
}

SegmentProxy Route::SegmentBefore::back() const
{
    return {route_.nodes[end]->activity(), route_.locations[end]};
}

size_t Route::SegmentBefore::size() const { return end + 1; }

size_t Route::SegmentBefore::numClients() const
{
    return route_.numClients_[end];
}

size_t Route::SegmentBefore::numPickups() const
{
    return route_.numPickups_[end];
}

bool Route::SegmentBefore::startsAtReloadDepot() const { return false; }

bool Route::SegmentBefore::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Route const *Route::SegmentAfter::route() const { return &route_; }

SegmentProxy Route::SegmentAfter::front() const
{
    return {route_.nodes[start]->activity(), route_.locations[start]};
}

SegmentProxy Route::SegmentAfter::back() const
{
    return {route_.nodes.back()->activity(), route_.locations.back()};
}

size_t Route::SegmentAfter::size() const { return route_.size() - start; }

size_t Route::SegmentAfter::numClients() const
{
    // fromStart is (start, end]. So we need to check if start itself is also
    // a client, and add 1 if it is.
    auto const fromStart = route_.numClients() - route_.numClients_[start];
    return fromStart + route_[start]->isClient();
}

size_t Route::SegmentAfter::numPickups() const
{
    // fromStart is (start, end]. So we need to check if start itself is also
    // a pickup, and add 1 if it is.
    auto const fromStart = route_.numPickups() - route_.numPickups_[start];
    return fromStart + route_[start]->isPickup();
}

bool Route::SegmentAfter::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}
bool Route::SegmentAfter::endsAtReloadDepot() const { return false; }

Route const *Route::SegmentBetween::route() const { return &route_; }

SegmentProxy Route::SegmentBetween::front() const
{
    return {route_.nodes[start]->activity(), route_.locations[start]};
}

SegmentProxy Route::SegmentBetween::back() const
{
    return {route_.nodes[end]->activity(), route_.locations[end]};
}

size_t Route::SegmentBetween::size() const { return end - start + 1; }

size_t Route::SegmentBetween::numClients() const
{
    // fromStart is (start, end]. So we need to check if start itself is also
    // a client, and add 1 if it is.
    auto const fromStart = route_.numClients_[end] - route_.numClients_[start];
    return fromStart + route_[start]->isClient();
}

size_t Route::SegmentBetween::numPickups() const
{
    // fromStart is (start, end]. So we need to check if start itself is also
    // a pickup, and add 1 if it is.
    auto const fromStart = route_.numPickups_[end] - route_.numPickups_[start];
    return fromStart + route_[start]->isPickup();
}

bool Route::SegmentBetween::startsAtReloadDepot() const
{
    return route_.nodes[start]->isReloadDepot();
}

bool Route::SegmentBetween::endsAtReloadDepot() const
{
    return route_.nodes[end]->isReloadDepot();
}

Distance Route::SegmentBetween::distance(size_t profile) const
{
    if (profile != route_.profile())  // then we have to compute the distance
    {                                 // segment from scratch.
        auto const &mat = route_.data.distanceMatrix(profile);
        Distance distance = 0;

        for (size_t step = start; step != end; ++step)
        {
            auto const from = route_.locations[step];
            auto const to = route_.locations[step + 1];
            distance += mat(from, to);
        }

        return distance;
    }

    auto const startDist = route_.cumDist[start];
    auto const endDist = route_.cumDist[end];

    assert(startDist <= endDist);
    return endDist - startDist;
}

DurationSegment Route::SegmentBetween::duration(size_t profile) const
{
    auto const &mat = route_.data.durationMatrix(profile);
    auto segment = route_.durAt[start];

    if (size() != 1 && route_[start]->isReloadDepot())  // first need to add the
    {                                                   // start depot's service
        auto const &depot = route_.data.depot(route_[start]->idx());
        segment = DurationSegment::merge(segment, {depot.serviceDuration});
    }

    for (size_t step = start; step != end; ++step)
    {
        auto const from = route_.locations[step];
        auto const to = route_.locations[step + 1];
        auto const &durAt = route_.durAt[step + 1];
        segment = DurationSegment::merge(mat(from, to), segment, durAt);
    }

    return segment;
}

LoadSegment Route::SegmentBetween::load(size_t dimension) const
{
    auto const &loads = route_.loadAt[dimension];

    auto loadSegment = loads[start];
    for (size_t step = start; step != end; ++step)
        loadSegment = LoadSegment::merge(loadSegment, loads[step + 1]);

    return loadSegment;
}

DriveSegment
Route::SegmentBetween::driveState(size_t profile) const
{
    if (!route_.driveAt.has_value())
        return {};

    auto const &mat = route_.data.durationMatrix(profile);
    auto durSeg = route_.durAt[start];
    auto drvSeg = route_.driveAt.value()[start];

    if (size() != 1 && route_[start]->isReloadDepot())
    {
        auto const &depot = route_.data.depot(route_[start]->idx());
        durSeg = DurationSegment::merge(durSeg, {depot.serviceDuration});
    }

    // Upcoming breaks: bitmask of breaks whose CUSTOM_BREAK node lies at a
    // route position strictly after each position. Mirrors Route::update()'s
    // forward pass: their triggers must not fire at earlier boundaries — the
    // break is still scheduled ahead (the gate at its own node decides
    // service; violations are only incurred at subsequent boundaries).
    // Only the step loop below reads this, and that loop does not run when
    // ``start == end`` -- which is the single-node case ShiftBreak asks for on
    // every candidate. Building it unconditionally meant a heap allocation and
    // an O(n) backward scan per candidate on the break-only unary path.
    detail::SmallBuf<uint16_t, 64> upcomingBreakMaskAt(
        start != end ? route_.size() : 0, 0);
    if (start != end)
    {
        uint16_t run = 0;
        for (size_t i = route_.size(); i-- > 0;)
        {
            upcomingBreakMaskAt[i] = run;
            if (route_[i]->isCustomBreak())
                run |= static_cast<uint16_t>(1u)
                       << (route_[i]->idx() & 0xF);
        }
    }

    for (size_t step = start; step != end; ++step)
    {
        auto const edgeDur = mat(route_.locations[step],
                                 route_.locations[step + 1]);
        auto const &nextDurAt = route_.durAt[step + 1];
        auto const &nextDriveAt = route_.driveAt.value()[step + 1];

        // atSecond: exact scheduled arrival at step+1, mirroring the
        // evaluateForwardPass clock exactly (D5): the duration fold carries
        // travel + service in duration() and the absorbed waiting as a
        // startEarly() offset, so the ABSOLUTE (midnight-anchored) clock is
        // duration() + startEarly() + edge (+ setup). timeWarp is a late-side
        // penalty and must NOT be subtracted here (subtracting it would make
        // the clock non-monotonic and collapse below earlier arrivals). The
        // value is clamped to the destination node's tw_early: clients and
        // depots clamp to their own twEarly; CUSTOM_BREAK clamps to its
        // window startEarly (mirroring the Proposal gate behaviour).
        Duration setup = 0;
        if (route_[step + 1]->isClient()
            && route_.locations[step + 1] != route_.locations[step])
            setup = route_.data.setupDuration(route_.locations[step + 1]);

        Duration const earlyArrival
            = durSeg.duration() + durSeg.startEarly() + edgeDur + setup;

        Duration nodeEarly = 0;
        auto const *node = route_[step + 1];
        if (node->isClient())
            nodeEarly = route_.data.client(node->idx()).twEarly;
        else if (node->isDepot())
            nodeEarly = route_.data.depot(node->idx()).twEarly;
        else if (node->isCustomBreak())
            nodeEarly = nextDurAt.startEarly();

        auto const atSecond = std::max(earlyArrival, nodeEarly);

        durSeg = DurationSegment::merge(edgeDur, durSeg, nextDurAt);
        drvSeg = DriveSegment::merge(edgeDur, drvSeg, nextDriveAt,
                                     route_.vehicleType_.custom_breaks,
                                     atSecond,
                                     upcomingBreakMaskAt[step + 1]);

        // If the node at step+1 is a CUSTOM_BREAK, check eligibility and
        // apply its reset.  The DriveSegment::merge above skipped the break
        // (mask already set in driveAt), but the reset must be applied at
        // this position to match Route::update() behaviour (see forward-pass
        // CUSTOM_BREAK handling in Route::update()).
        if (route_[step + 1]->isCustomBreak())
        {
            auto const breakId = route_[step + 1]->idx();
            for (auto const &brk : route_.vehicleType_.custom_breaks)
            {
                if (brk.id == static_cast<size_t>(breakId))
                {
                    if (isBreakEligible(drvSeg, brk))
                    {
                        // D5: extend the served rest to absorb waiting before
                        // the next client's (still-closed) window opens.
                        // Applies to any served DUTY_TIME break, not only
                        // overnight rests.
                        bool const extend
                            = brk.trigger == CustomBreakTrigger::DUTY_TIME
                              && step + 2 < route_.size()
                              && route_[step + 2]->isClient();
                        Duration travel = 0;
                        Duration nextOpen = 0;
                        if (extend)
                        {
                            travel = mat(route_.locations[step + 1],
                                         route_.locations[step + 2]);
                            // nextOpen is the next client's absolute
                            // (midnight-anchored) twEarly, matching the
                            // absolute atSecond clock — no anchor
                            // subtraction.
                            nextOpen
                                = route_.data.client(route_[step + 2]->idx())
                                      .twEarly;
                        }
                        auto const effSvc
                            = breakEffectiveService(brk.service,
                                                    atSecond,
                                                    extend,
                                                    travel,
                                                    nextOpen);

                        switch (brk.reset)
                        {
                        case CustomBreakReset::ALL_TIMERS:
                            drvSeg.driveTime_ = 0;
                            drvSeg.workTime_ = 0;
                            drvSeg.dutyTime_ = 0;
                            drvSeg.lastResetAt_ = atSecond.get()
                                                  + effSvc.get();
                            break;
                        case CustomBreakReset::DRIVE_AND_WORK:
                            drvSeg.driveTime_ = 0;
                            drvSeg.workTime_ = 0;
                            break;
                        case CustomBreakReset::DRIVE_TIMER:
                            drvSeg.driveTime_ = 0;
                            break;
                        case CustomBreakReset::WORK_TIMER:
                            drvSeg.workTime_ = 0;
                            break;
                        case CustomBreakReset::NONE:
                            break;
                        }
                        drvSeg.breaksTakenMask_
                            |= static_cast<uint16_t>(1u)
                               << (breakId & 0xF);
                    }
                    else
                    {
                        // Not eligible: no reset, drop the optimistic bit so
                        // the trigger can fire (and breakDue be accounted)
                        // at later boundaries, mirroring Route::update().
                        drvSeg.breaksTakenMask_
                            &= ~(static_cast<uint16_t>(1u)
                                 << (breakId & 0xF));
                    }
                    break;
                }
            }
        }
    }

    return drvSeg;
}

bool Route::isFeasible() const
{
    assert(!dirty);
    // A mandatory break violation makes the route infeasible unless the
    // violated break is marked relaxable (then it is penalised instead).
    auto const relaxableMask = vehicleType_.relaxableBreakMask();
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance()
           && (breakDueMask_ & ~relaxableMask) == 0;
}

bool Route::hasExcessLoad() const
{
    assert(!dirty);
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Route::hasExcessDistance() const
{
    assert(!dirty);
    return excessDistance() > 0;
}

bool Route::hasTimeWarp() const
{
    assert(!dirty);
    return timeWarp() > 0;
}

Route::Node *Route::operator[](size_t idx)
{
    assert(idx < nodes.size());
    return nodes[idx];
}

Route::Node const *Route::operator[](size_t idx) const
{
    assert(idx < nodes.size());
    return nodes[idx];
}

std::vector<Load> const &Route::load() const
{
    assert(!dirty);
    return load_;
}

std::vector<Load> const &Route::excessLoad() const
{
    assert(!dirty);
    return excessLoad_;
}

Distance Route::excessDistance() const
{
    assert(!dirty);
    return excessDistance_;
}

std::vector<Load> const &Route::capacity() const
{
    return vehicleType_.capacity;
}

size_t Route::startDepot() const { return vehicleType_.startDepot; }

size_t Route::endDepot() const { return vehicleType_.endDepot; }

Cost Route::fixedVehicleCost() const { return vehicleType_.fixedCost; }

Distance Route::distance() const
{
    assert(!dirty);
    return distance_;
}

Cost Route::distanceCost() const
{
    assert(!dirty);
    return distanceCost_;
}

Cost Route::unitDistanceCost() const { return vehicleType_.unitDistanceCost; }

bool Route::hasDistanceCost() const
{
    return unitDistanceCost() != 0
           || maxDistance() != std::numeric_limits<Distance>::max();
}

Duration Route::duration() const
{
    assert(!dirty);
    return duration_;
}

Duration Route::overtime() const
{
    assert(!dirty);
    return std::max<Duration>(duration() - shiftDuration(), 0);
}

Cost Route::durationCost() const
{
    assert(!dirty);
    return durationCost_;
}

Cost Route::unitDurationCost() const { return vehicleType_.unitDurationCost; }

Cost Route::unitOvertimeCost() const { return vehicleType_.unitOvertimeCost; }

bool Route::hasDurationCost() const
{
    // clang-format off
    return data.hasTimeWindows()
        || unitDurationCost() != 0
        || (unitOvertimeCost() != 0 && maxOvertime() != 0)
        || maxDuration() != std::numeric_limits<Duration>::max()
        || hasBreaks()       // breakDue penalty must be tracked
        || data.hasSetup();  // setup durations affect the route duration
    // clang-format on
}

Duration Route::shiftDuration() const { return vehicleType_.shiftDuration; }

Duration Route::maxDuration() const { return vehicleType_.maxDuration; }

Duration Route::maxOvertime() const { return vehicleType_.maxOvertime; }

Distance Route::maxDistance() const { return vehicleType_.maxDistance; }

Duration Route::timeWarp() const
{
    assert(!dirty);
    return timeWarp_;
}

bool Route::hasBreaks() const { return vehicleType_.hasBreaks(); }

int64_t Route::breakDue() const
{
    if (!hasBreaks())
        return 0;

    return breakDue_;
}

uint16_t Route::breakDueMask() const
{
    if (!hasBreaks())
        return 0;

    return breakDueMask_;
}

Duration Route::waiting() const
{
    assert(!dirty);
    return waiting_;
}

Duration Route::breakServiceAt(size_t pos) const
{
    if (pos >= breakServicesAt_.size())
        return 0;
    return breakServicesAt_[pos];
}

std::vector<size_t> Route::breaksServed() const
{
    if (!hasBreaks() || !driveBefore.has_value())
        return {};

    std::vector<size_t> served;

    // Read the mask AT each break node's own position, not the final mask.
    // driveBefore->at(pos) is the forward-pass state right after processing
    // the node at pos: for a CUSTOM_BREAK node, the only source of that
    // break's bit there is the eligibility check (Route::update() keeps the
    // optimistic bit when isBreakEligible() holds and drops it otherwise;
    // DriveSegment::merge() skips the trigger for breaks whose bit is
    // already taken). The FINAL mask (driveBefore->back()) would also
    // contain bits re-set when a trigger fires at boundaries AFTER an
    // ineligible break node — a break positioned too early is NOT served,
    // so it must not be reported as served.
    auto const &maskAt = driveBefore.value();
    for (size_t pos = 0; pos != nodes.size(); ++pos)
    {
        if (!nodes[pos]->isCustomBreak())
            continue;

        auto const id = nodes[pos]->idx();
        if (maskAt[pos].breaksTakenMask_ & (1u << (id & 0xF)))
            served.push_back(id);
    }

    return served;
}

size_t Route::profile() const { return vehicleType_.profile; }

bool Route::empty() const { return numClients() == 0 && numShipments() == 0; }

size_t Route::size() const { return nodes.size(); }

size_t Route::numClients() const
{
    // numClients_ is the maintained prefix counter, refreshed in update().
    // Its back() gives the total number of CLIENT nodes, excluding depots,
    // shipments, and CUSTOM_BREAK activities (isClient() only counts actual
    // client nodes) — the same semantics as counting the nodes list.
    // The size guard is a defensive fallback for callers that query a route
    // whose nodes list has changed but update() has not yet been called.
    if (numClients_.size() != nodes.size())
    {
        size_t count = 0;
        for (auto const *node : nodes)
            count += node->isClient();
        return count;
    }

    return numClients_.back();
}

size_t Route::numShipments() const
{
    assert(!dirty);
    return numPickups_.back();
}

size_t Route::numPickups() const
{
    assert(!dirty);
    return numPickups_.back();
}

size_t Route::numPickups(size_t end) const
{
    assert(!dirty);
    assert(end < size());
    return numPickups_[end];
}

size_t Route::numDeliveries(size_t end) const
{
    assert(!dirty);
    assert(end < size());
    return numDeliveries_[end];
}

size_t Route::numDepots() const { return depots_.size(); }

size_t Route::numTrips() const { return depots_.size() - 1; }

size_t Route::maxTrips() const { return vehicleType_.maxTrips(); }

Route::SegmentBetween Route::at(size_t idx) const
{
    assert(!dirty);
    return {*this, idx, idx};
}

Route::SegmentAfter Route::after(size_t start) const
{
    assert(!dirty);
    return {*this, start};
}

Route::SegmentBefore Route::before(size_t end) const
{
    assert(!dirty);
    return {*this, end};
}

Route::SegmentBetween Route::between(size_t start, size_t end) const
{
    assert(!dirty);
    return {*this, start, end};
}

template <Segment... Segments>
Route::Proposal<Segments...>::Proposal(Segments &&...segments)
    : segments_(std::forward<Segments>(segments)...)
{
    static_assert(sizeof...(Segments) > 0, "Proposal cannot be empty.");

    [[maybe_unused]] auto const numClients = std::apply(
        [](auto &&...args) { return (args.numClients() + ...); }, segments_);
    [[maybe_unused]] auto const numShipments = std::apply(
        [](auto &&...args) { return (args.numPickups() + ...); }, segments_);
    assert(numClients + numShipments != 0);  // proposal must not be empty

    [[maybe_unused]] auto &&first = std::get<0>(segments_);
    [[maybe_unused]] auto &&last = std::get<sizeof...(Segments) - 1>(segments_);
    assert(first.route() == last.route());  // must start and end at same route

    // Must start and end at the route start and end.
    [[maybe_unused]] auto const &route = *this->route();
    assert(first.front().activity() == route[0]->activity());
    assert(last.back().activity() == route[route.size() - 1]->activity());
}

template <Segment... Segments>
Route const *Route::Proposal<Segments...>::route() const
{
    return std::get<0>(segments_).route();
}

template <Segment... Segments>
Cost Route::Proposal<Segments...>::fixedVehicleCost() const
{
    return route()->fixedVehicleCost();
}

template <Segment... Segments>
bool Route::Proposal<Segments...>::empty() const
{
    auto const numClients = std::apply(
        [](auto &&...args) { return (args.numClients() + ...); }, segments_);
    auto const numShipments = std::apply(
        [](auto &&...args) { return (args.numPickups() + ...); }, segments_);
    return numClients == 0 && numShipments == 0;
}

template <Segment... Segments>
ForwardEvalResult Route::Proposal<Segments...>::runStreamForward() const
{
    // Streaming re-implementation of evaluateForwardPass() over the proposal's
    // flat sequence. It visits each activity exactly once per pass and keeps
    // only a constant amount of per-node state (the previous DurationSegment /
    // DriveSegment) plus small per-break-id arrays. The D5 rest-extension and
    // the due-ness window clearing that evaluateForwardPass applies to its
    // per-node ``durAt`` array are here re-derived from a tiny per-break
    // mutation store when the duration fold is re-run (second round).
    //
    // Round structure mirrors evaluateForwardPass exactly:
    //   round 1 (decide == true):  duration fold + drive fold on the base
    //       singletons; break eligibility / window-close / D5 decisions are
    //       computed and frozen into the per-break store.
    //   round 2 (decide == false): only when a served rest was extended or a
    //       non-due absolute window was cleared; the duration fold is re-run
    //       on the mutated singletons and the drive fold is re-run with the
    //       frozen decisions (final clock bookkeeping).
    auto const *r = route();
    auto const &data = r->data;
    auto const &vt = r->vehicleType_;
    auto const &durMatrix = data.durationMatrix(vt.profile);
    auto const &breaks = vt.custom_breaks;
    bool const resetAtReload = vt.reset_breaks_at_reload;
    Duration const MAX = std::numeric_limits<Duration>::max();

    // ---- flat-sequence descriptors over the proposal's segments ----
    // Route-based segments contribute a contiguous route-index range; any other
    // segment (e.g. BreakSegment) contributes its single front node.
    struct Desc
    {
        Route const *route;
        bool single;
        size_t a;    // route range start (single == true: unused)
        size_t b;    // route range end (inclusive; single == true: unused)
        Activity act;
        size_t loc;  // single-node location

        Desc()
            : route(nullptr),
              single(false),
              a(0),
              b(0),
              act(Activity::ActivityType::DEPOT, 0),
              loc(0)
        {
        }
    };
    constexpr size_t NSEGS = sizeof...(Segments);
    std::array<Desc, NSEGS> descs;
    std::array<size_t, NSEGS + 1> cum{};
    {
        size_t d = 0;
        auto const add = [&](auto const &segment)
        {
            using Seg = std::decay_t<decltype(segment)>;
            auto &desc = descs[d];
            size_t len = 1;
            if constexpr (std::is_same_v<Seg, SegmentBefore>)
            {
                desc.route = segment.route();
                desc.a = 0;
                desc.b = segment.endIdx();
                len = desc.b - desc.a + 1;
            }
            else if constexpr (std::is_same_v<Seg, SegmentAfter>)
            {
                desc.route = segment.route();
                desc.a = segment.startIdx();
                desc.b = desc.route->size() - 1;
                len = desc.b - desc.a + 1;
            }
            else if constexpr (std::is_same_v<Seg, SegmentBetween>)
            {
                desc.route = segment.route();
                desc.a = segment.startIdx();
                desc.b = segment.endIdx();
                len = desc.b - desc.a + 1;
            }
            else
            {
                auto const front = segment.front();
                desc.single = true;
                desc.act = front.activity();
                desc.loc = front.location();
            }
            cum[d + 1] = cum[d] + len;
            ++d;
        };
        std::apply([&](auto const &...segs) { (add(segs), ...); }, segments_);
    }
    size_t const n = cum[NSEGS];
    assert(n >= 2);

    // Fetches the RAW (uncorrected) activity/location at flat index ``idx``.
    // The passes walk ``idx`` forward, so the descriptor search resumes from
    // the last one used and only rewinds on the (rare) backward query.
    size_t descCursor = 0;
    auto const loadNode = [&](size_t idx, Activity &act, size_t &rawLoc)
    {
        size_t d = descCursor;
        if (idx < cum[d])
            d = 0;
        while (cum[d + 1] <= idx)
            ++d;
        descCursor = d;
        auto const off = idx - cum[d];
        if (descs[d].single)
        {
            act = descs[d].act;
            rawLoc = descs[d].loc;
        }
        else
        {
            auto const pos = descs[d].a + off;
            act = descs[d].route->activitiesAt_[pos];
            rawLoc = descs[d].route->locations[pos];
        }
    };

    // ---- start / end depot singletons (evaluateForwardPass Step 1) ----
    auto const &startDepot = data.depot(vt.startDepot);
    DurationSegment const vehStart(vt, vt.startLate);
    DurationSegment const depotStart(startDepot, startDepot.serviceDuration);
    DurationSegment const durAtStart = DurationSegment::merge(vehStart,
                                                              depotStart);

    auto const &endDepot = data.depot(vt.endDepot);
    DurationSegment const depotEnd(endDepot, Duration(0));
    DurationSegment const vehEnd(vt, vt.twLate);
    DurationSegment const durAtEnd = DurationSegment::merge(depotEnd, vehEnd);

    // ---- Alternativa D: intact route prefix (seeded fast-forward) ----
    // When the proposal's first segment is a contiguous range [0..k] of the
    // current route (the un-mutated prefix), the streaming pass can start at
    // flat index k+1 seeded with the route's cached state at position k,
    // skipping the O(k) prefix re-simulation that every candidate currently
    // pays. ``seeded`` is only set when the seed is available (the route was
    // updated after its last modification, so its cached states are live).
    size_t P = 0;  // leading flat nodes skipped (0 = full pass, as before)
    bool seeded = false;
    {
        bool const rangeFromStart
            = !descs[0].single && descs[0].a == 0;
        bool const seedReady = r->hasBreaks() && !r->dirty && r->breakSeedValid_
                               && r->fwdDrive_.has_value();
        if (rangeFromStart && seedReady)
        {
            size_t skip = cum[1];  // length of the first segment
            if (skip == n)
            {
                // The proposal equals the current route: reuse its cached
                // forward-pass totals directly (bit-identical to a full pass).
                PYVRP_STAT(calls, 1);
                PYVRP_STAT(shortCircuit, 1);
                return {r->duration_, r->timeWarp_, r->breakDue_,
                        r->breakDueMask_, r->waiting_};
            }
            // A CUSTOM_BREAK node sitting exactly at the prefix boundary has a
            // successor in the proposal that may differ from its route
            // successor (a removal / relocation right after the break). Its D5
            // rest extension absorbs waiting against that successor's window,
            // so the cached prefix fold at the boundary is stale. Pull the
            // boundary one position left and re-simulate the break node too.
            if (skip >= 2 && (*r)[skip - 1]->isCustomBreak())
            {
                if (skip >= 3)
                    --skip;
                else
                    skip = 0;  // prefix too short: fall back to a full pass
            }
            if (skip >= 2)  // seed requires a real prefix (boundary k >= 1)
            {
                P = skip;
                seeded = true;
            }
        }
    }

    // ---- per-break-id bookkeeping (sized by the vehicle's max break id) ----
    size_t maxBreakId = 0;
    for (auto const &brk : breaks)
        maxBreakId = std::max(maxBreakId, brk.id);
    size_t const K = maxBreakId + 1;

    // Flat, pre-computed rule table (VehicleType::breakRules) plus an
    // id -> index map, so neither the fold nor the decision block has to scan
    // ``breaks`` linearly at every break node.
    auto const &rules = vt.breakRules;
    detail::SmallBuf<int8_t, 16> ruleOf(K, -1);
    uint16_t mandatoryMask = 0;
    uint16_t collapseRequiredMask = 0;
    for (size_t i = 0; i != rules.size(); ++i)
    {
        if (rules[i].id < K)
            ruleOf[rules[i].id] = static_cast<int8_t>(i);
        if (rules[i].mandatory)
        {
            mandatoryMask |= rules[i].bit;
            collapseRequiredMask |= rules[i].bit;
        }
        // An ALL_TIMERS reset REPLACES takenMask rather than adding to it
        // (DriveSegment::merge), so an ALL_TIMERS break that has not fired yet
        // can clear a mandatory bit at a later boundary and let that mandatory
        // break become due again. Such a break must already be taken before
        // the tail can be treated as inert.
        if (rules[i].reset == CustomBreakReset::ALL_TIMERS)
            collapseRequiredMask |= rules[i].bit;
    }

    // The tail can only be collapsed onto the route's cached ``durAfter`` fold
    // when the proposal's last segment really is a contiguous suffix of one
    // route, that route has no reload depot (durAfter finalises the front
    // where the forward pass finalises the back), and the instance has no
    // setup durations (durAfter is built from bare edges).
    // ``presentMask`` aliases ids modulo 16 while ``occ``/``remaining`` are
    // indexed by the raw id, so ``remainingMask == 0`` is only a reliable
    // "no break node ahead" test while the id space stays below 16.
    bool const tailIsRouteSuffix
        = !descs[NSEGS - 1].single && descs[NSEGS - 1].route == r
          && descs[NSEGS - 1].b == r->size() - 1 && !r->dirty
          && maxBreakId < 16 && !data.hasSetup() && r->numTrips() == 1;

    // Gate for settling a cleared break window in a single round (see the
    // in-line clearing in the break branch below). The argument that clearing
    // leaves the pass clock invariant runs through DurationSegment::merge and
    // holds only for a single-trip sequence without release times:
    //   - a release time makes startEarly() clamp to releaseTime_, so the
    //     duration_/startEarly_ cancellation the argument relies on breaks;
    //   - a reload depot routes the fold through finaliseBack(), which reads
    //     startLate_/prevEndLate_ directly -- exactly the field clearing moves.
    // Either one falls back to the historical two-round scheme, which is
    // always correct, just slower. Same shape of gate as tailIsRouteSuffix.
    bool inlineClearOk = true;
    for (size_t d = 0; d != NSEGS; ++d)
    {
        auto const *dr = descs[d].route;
        if (descs[d].single || !dr)
            continue;
        if (dr->hasReleaseTimes() || dr->numTrips() != 1)
        {
            inlineClearOk = false;
            break;
        }
    }

    // Per-candidate bookkeeping. These used to be eight (ten, when seeded)
    // heap-allocated std::vectors per proposal evaluation; with millions of
    // proposals per solve the malloc/free traffic alone dominated the pass.
    // The taken/due masks cap the id space at 16, so the small buffers below
    // never touch the heap in practice.
    detail::SmallBuf<int64_t, 16> firstDue(K, -1);  // D2 first-due clock
#ifdef PYVRP_STREAM_STATS
    // Differential mode: the localiser records its answer, the walk runs on
    // anyway, and the two are compared after the loop. The distance gate and
    // test_stream_parity both passed on a version that changed a fifth of the
    // evaluations, so neither is sufficient evidence here.
    bool locAnswered = false;
    DurationSegment locDur;
    detail::SmallBuf<int64_t, 16> locFirst(K, -1);
    uint16_t locDue = 0;
    Duration locEnd = 0;
#else
    static constexpr bool locAnswered = false;
#endif
    detail::SmallBuf<int64_t, 16> occ(K, 0);        // occurrences per break id
    detail::SmallBuf<int64_t, 16> remaining(K, 0);  // occurrences still ahead
    detail::SmallBuf<int64_t, 16> atBreak(K, -1);   // final-clock arrival
    detail::SmallBuf<uint8_t, 16> eligible(K, 0);   // frozen eligibility
    detail::SmallBuf<uint8_t, 16> pastClose(K, 0);  // frozen window-close flag
    detail::SmallBuf<uint8_t, 16> cleared(K, 0);    // cleared absolute window
    detail::SmallBuf<int64_t, 16> extraSvc(K, 0);   // cumulative D5 extension
    uint16_t presentMask = 0;                // bits of present break ids

    // Alternativa D: prefix seeds (built when the route prefix is skipped).
    // ``seedFirstDue``/``seedAtBreak`` mirror the route's per-break-id D3
    // facts as of the boundary position k = P - 1 (value, or -1 when the
    // event lies after the boundary); ``seedDur``/``seedDrive`` are the fold
    // and drive states at k; ``seedAct``/``seedLoc``/``seedIdx`` describe the
    // node at k (the predecessor of the re-simulated span).
    detail::SmallBuf<int64_t, 16> seedFirstDue(seeded ? K : 0, -1);
    detail::SmallBuf<int64_t, 16> seedAtBreak(seeded ? K : 0, -1);
    uint16_t seedServed = 0;
    DurationSegment seedDur = durAtStart;
    DriveSegment seedDrive;
    Activity seedAct(Activity::ActivityType::DEPOT, 0);
    size_t seedIdx = 0;
    size_t seedLoc = 0;
    if (seeded)
    {
        size_t const k = P - 1;
        seedDur = r->durBefore[k];
        seedDrive = (*r->fwdDrive_)[k];
        loadNode(k, seedAct, seedLoc);
        seedIdx = k;
        size_t const nId = std::min(K, r->breakSeed_.size());
        for (size_t b = 0; b != nId; ++b)
        {
            auto const &info = r->breakSeed_[b];
            if (info.firstDuePos != std::numeric_limits<size_t>::max() && info.firstDuePos <= k)
                seedFirstDue[b] = info.firstDueVal;
            if (info.occPos != std::numeric_limits<size_t>::max() && info.occPos <= k)
            {
                seedAtBreak[b] = info.arrivalAtOcc.get();
                if (info.served)
                    seedServed |= static_cast<uint16_t>(1u) << (b & 0xF);
            }
        }
    }

    {
        // Pre-scan: count the break nodes present in the flat sequence (in a
        // seeded run, only those after the skipped prefix). The remaining-mask
        // (breaks whose node still lies ahead) is derived from these counts
        // during each round.
        PYVRP_STAT(prescanNodes, n - (seeded ? P : 0));
        size_t const from = seeded ? P : 0;
        auto const count = [&](Activity const &act)
        {
            if (act.isCustomBreak() && act.idx() < K)
            {
                occ[act.idx()]++;
                presentMask |= static_cast<uint16_t>(1u) << (act.idx() & 0xF);
            }
        };
        for (size_t d = 0; d != NSEGS; ++d)
        {
            if (cum[d + 1] <= from)  // wholly inside the skipped prefix
                continue;

            auto const &desc = descs[d];
            if (desc.single)
            {
                count(desc.act);
                continue;
            }

            // ``from`` can land inside this range; start there. Only the
            // break nodes matter, and the route knows where they are, so the
            // scan is O(#breaks) rather than O(range).
            size_t const skip = from > cum[d] ? from - cum[d] : 0;
            size_t const lo = desc.a + skip;
            for (auto const pos : desc.route->breakPositions_)
                if (pos >= lo && pos <= desc.b)
                    count(desc.route->activitiesAt_[pos]);
        }
    }

    PYVRP_STAT(calls, 1);
    PYVRP_STAT(flatNodes, n);
    PYVRP_STAT(seeded, seeded ? 1 : 0);

    uint16_t servedMask = 0;     // OR across rounds (decisions are frozen)
    uint16_t dueMask = 0;        // reset per round; final round wins
    Duration absorbedWaiting = 0;
    bool clearedWindows = false;

    // Running per-round state (declared outside the lambda so the final round
    // leaves the values used for the result).
    DurationSegment durBefore = durAtStart;  // durBefore at the last node
    Duration arrivalEnd = 0;                 // atSecond at the end depot
    DriveSegment driveBefore;                // driveBefore at the last node

    auto runRound = [&](bool decide)
    {
        firstDue.fill(-1);
        if (seeded)
            for (size_t b = 0; b != K; ++b)
                if (seedFirstDue[b] >= 0)
                    firstDue[b] = seedFirstDue[b];
        dueMask = 0;
        remaining.copyFrom(occ);
        uint16_t remainingMask = presentMask;

        durBefore = durAtStart;
        Duration arrival0 = 0;   // atSecond at the start depot (node 0)
        Duration arrivalCur = 0; // atSecond at the current node
        bool driveNode0Ready = false;

        Activity prevAct(Activity::ActivityType::DEPOT, 0);
        size_t prevIdx = 0;
        size_t prevLoc = 0;
        bool havePrev = false;

        if (seeded)
        {
            // Continue from the cached state after the intact prefix: fold and
            // drive states at the boundary, the prefix's served bits and its
            // per-break-id first-due / node-arrival facts. The re-simulated
            // span starts at flat index P (k + 1).
            durBefore = seedDur;
            driveBefore = seedDrive;
            driveNode0Ready = true;  // driveAt[0] init already covered
            servedMask |= seedServed;
            for (size_t b = 0; b != K; ++b)
                if (seedAtBreak[b] >= 0)
                    atBreak[b] = seedAtBreak[b];
            havePrev = true;
            prevAct = seedAct;
            prevIdx = seedIdx;
            prevLoc = seedLoc;
        }

        PYVRP_STAT(roundNodes, n - (seeded ? P : 0));
        bool scDone = false;
        bool locTriedOnce = false;
#ifdef PYVRP_STREAM_STATS
        // Which gate blocked the collapse, sampled at the last position where
        // firing was still possible.
        int scBlockedBy = 0;  // 1 struct, 2 remain, 3 served, 4 taken, 5 inert
#endif
        for (size_t idx = seeded ? P : 0; idx != n; ++idx)
        {
            Activity act(Activity::ActivityType::DEPOT, 0);
            size_t rawLoc;
            loadNode(idx, act, rawLoc);
            size_t loc = rawLoc;
            if (act.isCustomBreak() && havePrev)
                loc = prevLoc;  // break inherits previous node's location

            if (idx == 0)
            {
                arrival0 = durBefore.duration() - durBefore.timeWarp();
                havePrev = true;
                prevAct = act;
                prevIdx = idx;
                prevLoc = loc;
                continue;
            }

            // ---- duration fold step (mirrors runDurationPass at idx) ----
            bool const prevIsReload
                = prevAct.isDepot() && prevIdx > 0 && prevIdx < n - 1;

            // The reload case rebuilds the left operand; every other node
            // folds against ``durBefore`` itself. Binding by pointer keeps the
            // common path from copying the 80-byte segment at every node.
            DurationSegment const *before = &durBefore;
            DurationSegment reloadBefore;
            if (prevIsReload) [[unlikely]]
            {
                reloadBefore = durBefore.finaliseBack();
                auto const &depot = data.depot(prevAct.idx());
                reloadBefore = DurationSegment::merge(
                    reloadBefore,
                    DurationSegment(depot, depot.serviceDuration));
                before = &reloadBefore;
            }

            auto const edgeDur = durMatrix(prevLoc, loc);

            // ---- tail collapse ----
            // Once no break node remains ahead and no mandatory break can
            // still change its verdict, the rest of the route contributes
            // nothing but its duration fold — which the route already caches
            // in ``durAfter``. Folding it in one merge replaces the remaining
            // per-node simulation. ``tailIsRouteSuffix`` has already checked
            // the structural conditions (same route, clean, ends at the route
            // end, no setup, single trip, id space below the mask width).
#ifdef PYVRP_STREAM_STATS
            if (!scDone && idx > 0 && idx + 1 < n)
            {
                if (!tailIsRouteSuffix || descCursor != NSEGS - 1)
                    scBlockedBy = 1;
                else if (remainingMask != 0)
                    scBlockedBy = 2;
                else if ((driveBefore.breaksTakenMask_ & collapseRequiredMask)
                         != collapseRequiredMask)
                    scBlockedBy = 4;
                else
                    scBlockedBy = 5;  // only the D3 check can still fail
            }
#endif
            // ---- crossing localiser ----
            //
            // On a vehicle carrying mandatory ALL_TIMERS rules that never fire,
            // the mask gates below can never be satisfied, so the collapse
            // never fires on the routes that cost the most. The clock tables
            // give the alternative: atSecond at any position in O(1) from the
            // clock at any earlier one, and S = atSecond + service is
            // non-decreasing across clients, so a DUTY_TIME rule's first
            // crossing is a binary search. Locating it exactly yields the
            // first-due clock D3 needs -- the only thing the masks stood in for.
            //
            // The guard is an ENDPOINT CHECK rather than a ban on time warp.
            // The identity survives warp on 98.6% of nodes and the tables on
            // 96.7%, so refusing every warped fold (an earlier version) threw
            // away almost all the coverage to catch ~3%. Instead the merged
            // fold -- which is exact -- is asked to confirm the table
            // reproduced it: one comparison at the last position.
            if (tailIsRouteSuffix && !scDone && idx > 0 && idx + 1 < n
                && descCursor == NSEGS - 1 && remainingMask == 0
                && driveNode0Ready && r->hasClockTables() && !locTriedOnce
                && !((servedMask & mandatoryMask) == mandatoryMask
                     && (driveBefore.breaksTakenMask_ & collapseRequiredMask)
                            == collapseRequiredMask))
            {
                locTriedOnce = true;  // crossings are absolute; retrying from a
                                      // later node recomputes the same answer
                PYVRP_STAT(locTried, 1);
                auto const q0 = descs[NSEGS - 1].a + (idx - cum[NSEGS - 1]);
                auto const lastPos = r->size() - 1;
                auto const merged = DurationSegment::merge(
                    edgeDur, *before, r->durAfter[q0]);

                bool ok = q0 + 1 < lastPos && r->driveAt.has_value()
                          && r->driveAt.value().size() == r->size();
                if (ok)
                    for (auto const &rule : rules)
                        if (rule.trigger != CustomBreakTrigger::DUTY_TIME
                            || rule.id >= K)
                        {
                            ok = false;
                            break;
                        }

                if (ok)
                {
                    auto const earlyArr = durBefore.duration().get()
                                          + durBefore.startEarly().get()
                                          + edgeDur.get();
                    // cumM_ is a prefix max from position 0, so anchoring here
                    // is only sound when this proposal is not EARLIER at q0
                    // than the route itself; otherwise it inherits window
                    // clamps its own prefix never saw.
                    if (earlyArr < r->ownClockAt(q0))
                        ok = false;

                    if (ok)
                    {
                        auto const clockQ = earlyArr;
                        auto const endClk = r->clockAt(q0, clockQ, lastPos);

                        // Endpoint check: the merged fold is exact, so it
                        // settles whether the table reproduced this one.
                        if (endClk + r->durAt[lastPos].duration().get()
                            != merged.duration().get()
                                   + merged.startEarly().get())
                        {
                            ok = false;
                            PYVRP_STAT(locEndBad, 1);
                        }

                        if (ok)
                        {
                            auto const &drvAt = r->driveAt.value();
                            auto const base = driveBefore.lastResetAt_;

                            // S is monotone across clients; the end depot
                            // carries zero drive-side duty against a non-zero
                            // duration-side service, so it is excluded from the
                            // search and tested directly.
                            auto const sEnd
                                = endClk + drvAt[lastPos].dutyTime_;

                            uint16_t crossedMask = 0;
                            uint16_t firedDue = 0;
                            int64_t crossedAt[16] = {};
                            for (auto const &rule : rules)
                            {
                                bool const taken
                                    = (driveBefore.breaksTakenMask_ & rule.bit)
                                      != 0;
                                if (taken && firstDue[rule.id] >= 0)
                                    continue;  // settled

                                if (firstDue[rule.id] >= 0)
                                {
                                    // Due already and not taken: fires at the
                                    // very first tail boundary, reset and all.
                                    if (rule.reset != CustomBreakReset::NONE)
                                    {
                                        ok = false;
                                        break;
                                    }
                                    if (rule.mandatory)
                                        firedDue |= rule.bit;
                                    continue;
                                }

                                auto const need
                                    = base
                                      + std::max<int64_t>(
                                          rule.triggerValue + 1,
                                          rule.conditionMinRouteS);

                                // S is monotone, so one comparison against
                                // the last node rules out the whole search.
                                // Worth having: the two mandatory rules here
                                // trigger at 43 200 s, which no route reaches,
                                // so this skips 2 of every 5 searches.
                                if (sEnd < need)
                                    continue;

                                size_t lo = q0 + 1, hi = lastPos;  // clients
                                while (lo < hi)
                                {
                                    auto const mid = lo + (hi - lo) / 2;
                                    auto const sMid
                                        = r->clockAt(q0, clockQ, mid)
                                          + drvAt[mid].dutyTime_;
                                    if (sMid >= need)
                                        hi = mid;
                                    else
                                        lo = mid + 1;
                                }
                                size_t cross = lastPos + 1;
                                if (lo < lastPos)
                                    cross = lo;
                                else if (sEnd >= need)
                                    cross = lastPos;
                                if (cross > lastPos)
                                    continue;  // never reaches its trigger

                                crossedMask |= rule.bit;
                                crossedAt[rule.id]
                                    = r->clockAt(q0, clockQ, cross);

                                if (!taken)
                                {
                                    if (rule.reset != CustomBreakReset::NONE)
                                    {
                                        ok = false;
                                        break;
                                    }
                                    if (rule.mandatory)
                                        firedDue |= rule.bit;
                                }
                            }

                            if (ok)
                            {
                                PYVRP_STAT(locHits, 1);
#ifdef PYVRP_STREAM_STATS
                                locAnswered = true;
                                locDur = merged;
                                for (size_t b2 = 0; b2 != K; ++b2)
                                    locFirst[b2] = firstDue[b2];
                                for (auto const &rule : rules)
                                    if (crossedMask & rule.bit)
                                        locFirst[rule.id] = crossedAt[rule.id];
                                locDue = static_cast<uint16_t>(dueMask
                                                               | firedDue);
                                locEnd = Duration(endClk);
#else
                                // Mandatory only: the D3 tail skips every
                                // other rule, so a non-mandatory clock cannot
                                // reach the returned value. Restricting the
                                // write makes that an invariant of the code
                                // rather than a property to re-derive — the
                                // differential harness reports the residue on
                                // non-mandatory ids at 1.2% and on mandatory
                                // ids at 0 over 895 551 comparisons.
                                for (auto const &rule : rules)
                                    if ((crossedMask & rule.bit)
                                        && rule.mandatory)
                                        firstDue[rule.id] = crossedAt[rule.id];
                                dueMask |= firedDue;
                                arrivalEnd = Duration(endClk);
                                durBefore = merged;
                                scDone = true;
                                PYVRP_STAT(scSaved, n - idx);
                                break;
#endif
                            }
                        }
                    }
                }
            }

            if (tailIsRouteSuffix && !scDone && idx > 0 && idx + 1 < n
                && !locAnswered
                && descCursor == NSEGS - 1 && remainingMask == 0
                && (servedMask & mandatoryMask) == mandatoryMask
                && (driveBefore.breaksTakenMask_ & collapseRequiredMask)
                       == collapseRequiredMask)
            {
                // A mandatory break that is served but has not recorded a
                // first-due clock yet would record one somewhere in the tail.
                // atSecond is non-decreasing, so that clock would be at least
                // the arrival at the break's node and its D3 lateness term
                // would stay zero — the same value the skipped clock produces.
                // The check below anchors that argument on the prefix instead
                // of assuming it.
                bool inert = true;
                for (auto const &rule : rules)
                    if (rule.mandatory && rule.id < K && firstDue[rule.id] < 0
                        && atBreak[rule.id] > arrivalCur.get())
                        inert = false;

                if (inert)
                {
                    auto const q = descs[NSEGS - 1].a
                                   + (idx - cum[NSEGS - 1]);
                    durBefore = DurationSegment::merge(
                        edgeDur, *before, descs[NSEGS - 1].route->durAfter[q]);
                    scDone = true;
                    PYVRP_STAT(scHits, 1);
                    PYVRP_STAT(scSaved, n - idx);
                    // ``arrivalEnd`` stays unset: every mandatory break is
                    // served, so the end-clock branch of the D3 tail below is
                    // unreachable for this candidate.
                    break;
                }
            }

            Duration setup = 0;
            if (act.isClient() && loc != prevLoc)
                setup = data.setupDuration(loc);

            // atSecond is computed from the NON-finalised durBefore[prev].
            auto const earlyArrival = durBefore.duration()
                                      + durBefore.startEarly() + edgeDur
                                      + setup;

            // Singleton (raw ``durAt`` leaf) at this node, with mutations from
            // previous rounds (D5 extra service, cleared absolute window)
            // re-derived from the per-break store.
            DurationSegment second;
            Duration nodeEarly = 0;
            Duration brkSvcNow = 0;    // CUSTOM_BREAK only; see below.
            Duration brkEarlyNow = 0;
            if (act.isClient())
            {
                second = DurationSegment(data.client(act.idx()));
                nodeEarly = data.client(act.idx()).twEarly;
            }
            else if (act.isDepot())
            {
                if (idx == n - 1)
                    second = durAtEnd;
                else
                    second = DurationSegment(data.depot(act.idx()),
                                             Duration(0));
                nodeEarly = data.depot(act.idx()).twEarly;
            }
            else  // CUSTOM_BREAK
            {
                auto const breakId = act.idx();
                auto const ri = breakId < K ? ruleOf[breakId] : int8_t(-1);
                BreakRule const *brk = ri >= 0 ? &rules[ri] : nullptr;
                Duration svc = brk ? Duration(brk->service) : Duration(0);
                Duration early = 0;
                Duration late = MAX;
                if (brk && brk->hasWindow)
                {
                    // openAbs/closeAbs already carry the relative-window
                    // anchor, so there is nothing left to add here.
                    early = Duration(brk->openAbs);
                    late = Duration(brk->closeAbs);
                }
                if (breakId < K && extraSvc[breakId] != 0)
                    svc = svc + Duration(extraSvc[breakId]);
                if (breakId < K && cleared[breakId])
                    late = MAX;
                second = DurationSegment(svc, Duration(0), early, late);
                nodeEarly = second.startEarly();

                // Kept so the singleton can be rebuilt with the window cleared
                // once the eligibility decision is known, without re-deriving
                // the rule (see the in-line clearing below).
                brkSvcNow = svc;
                brkEarlyNow = early;
            }

            auto const atSecond = std::max(earlyArrival, nodeEarly);
            arrivalCur = atSecond;
            if (idx == n - 1)
                arrivalEnd = atSecond;

            // ---- drive fold step (mirrors runDrivePass at idx) ----
            // Deliberately ordered BEFORE the duration merge below. The drive
            // fold and the break decision need only atSecond, which is already
            // known, and nothing between here and the merge reads the
            // post-merge durBefore. Deciding first is what allows a break whose
            // window turns out not to apply to be folded with the window
            // already cleared, instead of folding it clamped and then re-running
            // the whole pass to undo the clamp (the historical round 2).

            if (!driveNode0Ready)  // idx == 1: (re)initialise driveAt[0]
            {
                DriveSegment driveAt0 = DriveSegment::fromDepot();
                auto const effStart = std::max(arrival0,
                                               atSecond - edgeDur);
                driveAt0.lastResetAt_ = effStart.get();
                driveBefore = driveAt0;
                driveNode0Ready = true;
            }

            DriveSegment driveSecond;
            if (act.isCustomBreak())
                driveSecond = DriveSegment(
                    0,
                    0,
                    0,
                    static_cast<uint16_t>(1u) << (act.idx() & 0xF),
                    0);
            else if (act.isDepot())
                driveSecond = DriveSegment::fromDepot();
            else
                driveSecond = DriveSegment::fromClient(
                    data.client(act.idx()).serviceDuration);

            uint16_t curBit = 0;
            if (act.isCustomBreak())
                curBit = static_cast<uint16_t>(1u) << (act.idx() & 0xF);
            auto const upcoming = static_cast<uint16_t>(remainingMask
                                                        & ~curBit);

            auto drs = DriveSegment::merge(edgeDur,
                                           driveBefore,
                                           driveSecond,
                                           rules,
                                           atSecond,
                                           upcoming,
                                           setup,
                                           &dueMask,
                                           firstDue.data());

            bool const curIsReloadDepot = act.isDepot() && idx > 0
                                          && idx < n - 1;
            if (resetAtReload && curIsReloadDepot)
                drs = DriveSegment(0,
                                   0,
                                   0,
                                   drs.breaksTakenMask_,
                                   drs.lastResetAt_);

            if (act.isCustomBreak())
            {
                auto const breakId = act.idx();
                auto const declIdx = breakId < K ? ruleOf[breakId]
                                                 : int8_t(-1);
                if (declIdx >= 0)
                {
                    auto const &brk = rules[declIdx];

                    bool elig;
                    bool pastWinClose;
                    if (decide)  // first drive pass: decide and freeze
                    {
                        elig = isBreakEligible(drs, brk);
                        pastWinClose = isBreakPastWindowClose(brk, atSecond);
                        if (breakId < K)
                        {
                            eligible[breakId] = elig ? 1 : 0;
                            pastClose[breakId] = pastWinClose ? 1 : 0;
                        }
                    }
                    else  // re-run: reuse the frozen decisions (D3/N1)
                    {
                        elig = breakId < K && eligible[breakId];
                        pastWinClose = breakId < K && pastClose[breakId];
                    }

                    auto const bit = static_cast<uint16_t>(1u)
                                     << (breakId & 0xF);
                    if (elig && !pastWinClose)
                    {
                        // Served: mark for the lateness formula (D3).
                        servedMask |= bit;

                        // D5: extend a served DUTY_TIME rest whose next
                        // activity is a client with a still-closed window so
                        // the waiting is absorbed into the rest. The extension
                        // is a function of the arrival at the rest, so it is
                        // recomputed on every round and accumulated into the
                        // store (matching evaluateForwardPass, which applies
                        // withService() again on each drive pass when no
                        // extendedBreakServices buffer is supplied).
                        bool extend = brk.trigger
                                          == CustomBreakTrigger::DUTY_TIME
                                      && idx + 1 < n;
                        Duration travel = 0;
                        Duration nextOpen = 0;
                        if (extend)
                        {
                            Activity nextAct(Activity::ActivityType::DEPOT, 0);
                            size_t nextRaw;
                            loadNode(idx + 1, nextAct, nextRaw);
                            if (nextAct.isClient())
                            {
                                travel = durMatrix(loc, nextRaw);
                                nextOpen = data.client(nextAct.idx()).twEarly;
                            }
                            else
                                extend = false;
                        }

                        Duration const minSvc = Duration(brk.service);
                        Duration effSvc = breakEffectiveService(minSvc,
                                                                atSecond,
                                                                extend,
                                                                travel,
                                                                nextOpen);
                        if (effSvc != minSvc)
                        {
                            absorbedWaiting += effSvc - minSvc;
                            if (breakId < K)
                                extraSvc[breakId] += (effSvc - minSvc).get();
                        }

                        switch (brk.reset)
                        {
                        case CustomBreakReset::ALL_TIMERS:
                            drs.driveTime_ = 0;
                            drs.workTime_ = 0;
                            drs.dutyTime_ = 0;
                            drs.lastResetAt_ = atSecond.get() + effSvc.get();
                            break;
                        case CustomBreakReset::DRIVE_AND_WORK:
                            drs.driveTime_ = 0;
                            drs.workTime_ = 0;
                            break;
                        case CustomBreakReset::DRIVE_TIMER:
                            drs.driveTime_ = 0;
                            break;
                        case CustomBreakReset::WORK_TIMER:
                            drs.workTime_ = 0;
                            break;
                        case CustomBreakReset::NONE:
                            break;
                        }
                        drs.breaksTakenMask_ |= bit;
                    }
                    else
                    {
                        // Not servable (not yet due, or due-but-past-close):
                        // drop the optimistic taken bit so the trigger can fire
                        // at later boundaries. A non-due break with an
                        // ABSOLUTE window close clears the close in the fold
                        // (due-ness gate) so it cannot warp the route.
                        drs.breaksTakenMask_
                            &= ~(static_cast<uint16_t>(1u) << (breakId & 0xF));

                        if (!elig && !brk.twsRelative && brk.hasWindow)
                        {
                            if (breakId < K)
                                cleared[breakId] = 1;

                            // The clamp this clearing undoes can only have
                            // produced time warp when arrival was already past
                            // the close: diffTw > 0 requires
                            // earlyArrival > close, and atSecond >= earlyArrival,
                            // so pastWinClose is a conservative superset of
                            // "the clamp warped".
                            //
                            // When it did NOT warp, clearing changes only this
                            // node's startLate_ (slack). Downstream, startLate_
                            // enters merge() solely through diffWait, which adds
                            // to duration_ and subtracts from startEarly_ --
                            // leaving duration() + startEarly(), the clock this
                            // pass runs on, invariant. Every downstream atSecond,
                            // and therefore every downstream drive state and
                            // decision, is unchanged. So folding the cleared
                            // singleton here yields exactly what the second
                            // round would have produced, and the second round is
                            // unnecessary.
                            //
                            // When it DID warp, the second round's clock differs
                            // from the first's, and the reference freezes the
                            // decisions taken on the clamped clock. Reproducing
                            // that requires the two rounds, so fall back.
                            if (pastWinClose || !inlineClearOk)
                            {
                                clearedWindows = true;
                                PYVRP_STAT(r2Warp, 1);
                            }
                            else
                            {
                                second = DurationSegment(brkSvcNow,
                                                         Duration(0),
                                                         brkEarlyNow,
                                                         MAX);
                                PYVRP_STAT(clrInline, 1);
                            }
                        }
                    }
                }
            }

            // The duration merge, deferred from above so the break decision
            // could rewrite `second` first.
            if (setup != 0)
                second = second.withService(setup);
            durBefore = DurationSegment::merge(edgeDur, *before, second);

            // Break countdown for the upcoming mask and the final-clock
            // arrival at each break node (D3 lateness).
            if (act.isCustomBreak())
            {
                auto const breakId = act.idx();
                if (breakId < K)
                {
                    atBreak[breakId] = arrivalCur.get();
                    if (remaining[breakId] > 0 && --remaining[breakId] == 0)
                        remainingMask &= ~curBit;
                }
            }

            driveBefore = drs;

            havePrev = true;
            prevAct = act;
            prevIdx = idx;
            prevLoc = loc;
        }
#ifdef PYVRP_STREAM_STATS
        if (!scDone)
            switch (scBlockedBy)
            {
            case 1: PYVRP_STAT(scNoStruct, 1); break;
            case 2: PYVRP_STAT(scNoRemain, 1); break;
            case 3: PYVRP_STAT(scNoServed, 1); break;
            case 4: PYVRP_STAT(scNoTaken, 1); break;
            case 5: PYVRP_STAT(scNoInert, 1); break;
            default: break;
            }
#endif
    };

    runRound(true);
    if (absorbedWaiting > 0 || clearedWindows)
    {
        PYVRP_STAT(round2, 1);
        if (absorbedWaiting > 0 && clearedWindows)
            PYVRP_STAT(r2Both, 1);
        else if (absorbedWaiting > 0)
            PYVRP_STAT(r2Absorb, 1);
        else
            PYVRP_STAT(r2Cleared, 1);
        runRound(false);
    }

#ifdef PYVRP_STREAM_STATS
    if (locAnswered)
    {
        PYVRP_STAT(locChecked, 1);
        if (locDur.duration() != durBefore.duration())
            PYVRP_STAT(locDiffDur, 1);
        if (locDur.timeWarp() != durBefore.timeWarp())
            PYVRP_STAT(locDiffWarp, 1);
        if (locDue != dueMask)
            PYVRP_STAT(locDiffDue, 1);
        if (locEnd != arrivalEnd)
            PYVRP_STAT(locDiffEnd, 1);
        // Only MANDATORY ids can reach the returned value: the D3 tail
        // skips every other rule. Both are counted so the harmless residue
        // stays visible instead of being quietly dropped.
        for (size_t b2 = 0; b2 != K; ++b2)
            if (locFirst[b2] != firstDue[b2])
            {
                PYVRP_STAT(locDiffFirst, 1);
                break;
            }
        for (size_t b2 = 0; b2 != K; ++b2)
        {
            auto const mbit = static_cast<uint16_t>(1u) << (b2 & 0xF);
            if ((mandatoryMask & mbit) && locFirst[b2] != firstDue[b2])
            {
                PYVRP_STAT(locDiffMand, 1);
                break;
            }
        }
    }
#endif

    // D3: per-mandatory-break lateness in SECONDS, all terms on the final
    // clock. Mirrors the tail of evaluateForwardPass().
    auto const endClock = arrivalEnd.get();
    int64_t breakDueSeconds = 0;
    for (auto const &brk : breaks)
    {
        if (!brk.mandatory)
            continue;
        auto const id = static_cast<size_t>(brk.id);
        if (id >= firstDue.size() || firstDue[id] < 0)
            continue;
        auto const bit = static_cast<uint16_t>(1u) << (brk.id & 0xF);
        if (servedMask & bit)
        {
            auto const nodeArr = id < atBreak.size() ? atBreak[id] : -1;
            if (nodeArr >= 0)
                breakDueSeconds
                    += std::max<int64_t>(0, nodeArr - firstDue[id]);
        }
        else
            breakDueSeconds += std::max<int64_t>(brk.service.get(),
                                                 endClock - firstDue[id]);
    }

    auto const waiting = durBefore.waiting();
    auto const duration = durBefore.duration();
    auto const timeWarp = durBefore.timeWarp(vt.maxDuration);

#ifndef NDEBUG
    assert(waiting.get() >= 0);
    assert(waiting.get() <= duration.get());
#endif

    return {duration, timeWarp, breakDueSeconds, dueMask, waiting};
}

template <Segment... Segments>
std::pair<Cost, Distance> Route::Proposal<Segments...>::distance() const
{
    if (empty())
        return std::make_pair(0, 0);

    PYVRP_PHASE(PH_DISTANCE);

    auto const &data = route()->data;
    auto const unitDistanceCost = route()->unitDistanceCost();
    auto const maxDistance = route()->maxDistance();
    auto const profile = route()->profile();
    auto const &matrix = data.distanceMatrix(profile);

    // ---- corrected path for break-configured routes ----
    // Cross-route moves (e.g. Exchange11) can leave segment front/back
    // locations stale because SegmentProxy uses route_.locations[idx],
    // which still reflects the original (pre-move) predecessor.
    // By collecting the flat forward sequence with break-location
    // correction (matching Route::update()), we compute distance by
    // construction — exactly the same way Route::update() computes
    // cumDist — so distance() and duration() can never diverge again.
    // NOT [[unlikely]]: this fork exists for break-configured routes, so in
    // its own workload every candidate takes this branch. Marking it unlikely
    // puts the whole break path in .text.unlikely and makes every evaluation
    // jump into the cold section and back.
    if (route()->hasBreaks())
    {
        // Prefix-sum corrected distance (H6): the distance over the flat
        // forward sequence is an ordinary sum of consecutive matrix edges
        // (distance is a pure monoid — breaks only turn their incoming edge
        // into a self-edge via Route::update()'s location inheritance, which
        // is already baked into the route's cumDist prefix). So each
        // contiguous [a..b] range of a route contributes cumDist[b] -
        // cumDist[a] in O(1); only the range's leading boundary needs a
        // correction: a CUSTOM_BREAK at the range start inherits the location
        // of the node preceding it *in the proposal* (not necessarily the
        // route's own predecessor), so each such leading break contributes a
        // self-edge at the current location and never advances it. This keeps
        // the result bit-identical to the previous per-node flat walk while
        // removing the O(n) matrix lookups every break-route proposal paid.
        Distance dist = 0;
        bool havePrev = false;
        size_t curLoc = 0;  // current corrected location (break-inherit aware)

        // Single proposal node with the given activity/location, mirroring the
        // flat-walk semantics exactly.
        auto const step = [&](bool isBreak, size_t loc)
        {
            if (isBreak)
            {
                // Inherits the current location: the edge into the break is
                // matrix(curLoc, curLoc) (a self-edge), matching Route::update().
                // A break with no predecessor yet (only possible at the very
                // start of the flat sequence) keeps its own given location.
                if (!havePrev)
                {
                    curLoc = loc;
                    havePrev = true;
                }
                else
                    dist += matrix(curLoc, curLoc);
                return;
            }

            if (havePrev)
                dist += matrix(curLoc, loc);
            curLoc = loc;
            havePrev = true;
        };

        // Contiguous [a..b] range of route ``r`` (inclusive). Falls back to a
        // per-node walk when the range's route uses a different routing
        // profile than the proposal (rare: then the route's cumDist was built
        // from another distance matrix and cannot be reused).
        auto const walkRange = [&](Route const *r, size_t a, size_t b)
        {
            if (r->profile() != profile) [[unlikely]]
            {
                for (size_t i = a; i <= b; ++i)
                    step((*r)[i]->isCustomBreak(), r->locations[i]);
                return;
            }

            // Leading CUSTOM_BREAK nodes never advance the current location;
            // each contributes one self-edge at the current location.
            size_t first = a;
            while (first <= b && (*r)[first]->isCustomBreak())
            {
                step(true, r->locations[first]);
                ++first;
            }

            if (first > b)  // the whole range consists of breaks only
                return;

            step(false, r->locations[first]);  // edge into first non-break
            dist += r->cumDist[b] - r->cumDist[first];  // edges first+1..b
            curLoc = r->locations[b];  // trailing breaks keep this location
        };

        auto const walk = [&](auto const &segment)
        {
            using Seg = std::decay_t<decltype(segment)>;

            if constexpr (std::is_same_v<Seg, SegmentBefore>)
                walkRange(segment.route(), 0, segment.endIdx());
            else if constexpr (std::is_same_v<Seg, SegmentAfter>)
                walkRange(segment.route(), segment.startIdx(),
                          segment.route()->size() - 1);
            else if constexpr (std::is_same_v<Seg, SegmentBetween>)
                walkRange(segment.route(), segment.startIdx(),
                          segment.endIdx());
            else
            {
                auto const front = segment.front();
                step(front.activity().isCustomBreak(), front.location());
            }
        };

        std::apply([&](auto const &... segs) { (walk(segs), ...); },
                   segments_);

        auto const excess = std::max<Distance>(dist - maxDistance, 0);
        auto const cost = unitDistanceCost * static_cast<Cost>(dist);
        return std::make_pair(cost, excess);
    }

    // ---- original segment-based path (no breaks — all locations are
    //      correct because there are no custom breaks to correct) ----
    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto distance = segment.distance(profile);
        auto lastLoc = segment.back().location();

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            auto const edgeDist = matrix(lastLoc, other.front().location());
            distance += edgeDist;
            distance += other.distance(profile);
            lastLoc = other.back().location();

            if constexpr (sizeof...(args) != 0)
                self(self, std::forward<decltype(args)>(args)...);
        };

        merge(merge, std::forward<decltype(args)>(args)...);

        auto const excess = std::max<Distance>(distance - maxDistance, 0);
        auto const cost = unitDistanceCost * static_cast<Cost>(distance);

        return std::make_pair(cost, excess);
    };

    return std::apply(fn, segments_);
}

template <Segment... Segments>
Duration Route::Proposal<Segments...>::durationLowerBound() const
{
    if (empty())
        return 0;

    // NOTE: deliberately does NOT gate on ``route()->hasBreaks() ||
    // data.hasSetup()`` the way duration()'s ``hasBrk`` does.
    // ``ProblemData::hasSetup()`` is an O(numLocations) std::any_of scan,
    // not a cached flag -- calling it here would re-scan it on every
    // candidate for every non-break route with duration cost (i.e. nearly
    // every candidate in the search), which is far more expensive than the
    // duration() fold this is meant to help avoid. Instead, each range below
    // independently checks its OWN route's cached prefix sums (O(1)
    // ``std::optional`` test) and contributes 0 if they are not populated --
    // exactly the routes this coarser gate would have skipped anyway, since
    // those prefix sums are populated under that same condition (see
    // Route::update()). Off the break/setup path, this reduces to a handful
    // of O(1) checks per proposal (duration() itself is already cheap
    // there), and 0 is always a trivially valid lower bound regardless: past
    // this point duration() only ever ADDS non-negative terms to the
    // objective (see CostEvaluator::deltaCost).

    // Mirrors distance()'s hasBreaks() corrected path: each contiguous
    // Route range in the proposal contributes its cached prefix-sum slice
    // directly, dropping the cross-segment/cross-route boundary edge
    // entering the range (always >= 0, so omitting it can only loosen the
    // bound, never invalidate it).
    Duration total = 0;

    auto const addRange = [&](Route const *r, size_t a, size_t b)
    {
        if (!r->cumDurEdge || !r->cumSvcLB)
            return;  // rare cross-route mix with a route that never
                      // populated these (no breaks there, and no global
                      // setup); skip -- still a safe underestimate.

        // Service is a pure per-node minimum with no location dependence,
        // so it is always safe to sum over the full range as stored.
        total += (*r->cumSvcLB)[b];
        if (a != 0)
            total -= (*r->cumSvcLB)[a - 1];

        // Edges are NOT always safe to reuse verbatim: cumDurEdge bakes in
        // route r's OWN ``locations[]``, where a CUSTOM_BREAK inherits r's
        // OWN predecessor's location (Route::update()). If this range's
        // real predecessor in the ASSEMBLED proposal is a different
        // segment/route (the same cross-route staleness distance() corrects
        // for -- see its ``walkRange``), a leading break's cached outgoing
        // edge can be arbitrarily wrong in EITHER direction, so it must not
        // be reused as a lower bound. Skip forward past any leading
        // CUSTOM_BREAK run and only sum edges from the first non-break
        // position onward -- an internal position, whose location is a
        // real client/depot/shipment location and thus context-independent
        // -- to keep this an underestimate.
        size_t first = a;
        while (first <= b && (*r)[first]->isCustomBreak())
            ++first;

        if (first <= b)
            total += (*r->cumDurEdge)[b] - (*r->cumDurEdge)[first];
    };

    auto const walk = [&](auto const &segment)
    {
        using Seg = std::decay_t<decltype(segment)>;

        if constexpr (std::is_same_v<Seg, SegmentBefore>)
            addRange(segment.route(), 0, segment.endIdx());
        else if constexpr (std::is_same_v<Seg, SegmentAfter>)
            addRange(segment.route(), segment.startIdx(),
                     segment.route()->size() - 1);
        else if constexpr (std::is_same_v<Seg, SegmentBetween>)
            addRange(segment.route(), segment.startIdx(), segment.endIdx());
        // else: a lone, not-yet-routed node (e.g. ClientSegment) has no
        // cached prefix sums and has no internal edge by construction (a
        // single node has none); its own minimum service is dropped here
        // too, which only loosens the bound.
    };

    std::apply([&](auto const &... segs) { (walk(segs), ...); }, segments_);

    return total;
}

template <Segment... Segments>
DurationSegment Route::Proposal<Segments...>::foldDuration() const
{
    auto const &data = route()->data;
    auto const profile = route()->profile();
    auto const &matrix = data.durationMatrix(profile);

    // Finalising is expensive with duration segments. However, finaliseFront is
    // significantly less expensive than finaliseBack. To use it, we iterate the
    // segments in reverse (right to left, rather than default left to right).
    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto ds = segment.duration(profile);
        auto firstLoc = segment.front().location();

        if (segment.startsAtReloadDepot())
            ds = ds.finaliseFront();

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            auto edgeDur = matrix(other.back().location(), firstLoc);
            auto otherDS = other.duration(profile);

            if (other.endsAtReloadDepot())
            {
                auto const &activity = other.back().activity();
                assert(activity.isDepot());

                auto const &depot = data.depot(activity.idx());
                DurationSegment depotDS = {depot, depot.serviceDuration};
                ds = DurationSegment::merge(edgeDur, depotDS, ds);
                ds = ds.finaliseFront();

                edgeDur = 0;
            }

            ds = DurationSegment::merge(edgeDur, otherDS, ds);

            firstLoc = other.front().location();

            if constexpr (sizeof...(args) != 0)
            {
                if (other.startsAtReloadDepot() && other.size() > 1)
                    ds = ds.finaliseFront();

                self(self, std::forward<decltype(args)>(args)...);
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...);
        return ds;
    };

    return std::apply(fn, detail::reverse(segments_));
}

template <Segment... Segments>
Duration Route::Proposal<Segments...>::durationLowerBoundFold() const
{
    // EXPERIMENTAL: see the docstring on the declaration. This reuses the
    // exact same DurationSegment composition as duration()'s no-break/
    // no-setup path -- it does not know about break decisions, so its
    // admissibility on break-configured routes is unverified here; that is
    // checked (not assumed) at the CostEvaluator::deltaCost call site.
    if (empty())
        return 0;

    auto const ds = foldDuration();
    return ds.duration() - ds.waiting();
}

template <Segment... Segments>
std::pair<Cost, Duration> Route::Proposal<Segments...>::duration() const
{
    if (empty())
        return std::make_pair(0, 0);

    PYVRP_PHASE(PH_DURATION);

    auto const &data = route()->data;
    auto const unitDurationCost = route()->unitDurationCost();
    auto const unitOvertimeCost = route()->unitOvertimeCost();
    auto const shiftDuration = route()->shiftDuration();
    auto const maxDuration = route()->maxDuration();

    // ---- breakDue: shared forward-pass evaluator (parity by construction) ----
    bool const hasBrk = route()->hasBreaks() || data.hasSetup();
    // See distance(): the break path is the hot path here, not the cold one.
    if (hasBrk)
    {
        // Streaming forward pass over the proposal's flat sequence (no
        // fwdActs_/fwdLocs_/atSecond vectors are materialised per candidate).
        // Uses exactly the scalar totals evaluateForwardPass() returns.
        auto const result = runStreamForward();
        breakDue_ = result.breakDue;
        waiting_ = result.waiting.get();

        // Parity: use the same forward-pass duration/timeWarp that
        // Route::update() produces internally, instead of the reverse
        // DurationSegment fold (which is not perfectly associative
        // with time windows and diverges on cross-route moves).
        auto const dur = result.duration;
        auto const overtime = std::max<Duration>(dur - shiftDuration, 0);
        // wait-cost-root-fix: duration cost excludes waiting (cached
        // above); the CostEvaluator charges it separately at its wait
        // rate. Overtime stays on the full duration.
        auto const dCost = unitDurationCost
                               * static_cast<Cost>(dur - result.waiting)
                           + unitOvertimeCost * static_cast<Cost>(overtime);
        return std::make_pair(dCost, result.timeWarp);
    }
    else
        breakDue_ = 0;

    auto const ds = foldDuration();

    auto const duration = ds.duration();
    auto const overtime = std::max<Duration>(duration - shiftDuration, 0);

    // wait-cost-root-fix: assign the ABSOLUTE cached waiting value BEFORE
    // computing the cost — the waiting() accessor falls back to duration()
    // and would recurse forever if called before this assignment (see
    // waiting() below). The duration cost excludes waiting; the
    // CostEvaluator charges waiting separately at its wait rate.
    auto const waiting = ds.waiting();
    waiting_ = waiting.get();

    auto const cost = unitDurationCost * static_cast<Cost>(duration - waiting)
                      + unitOvertimeCost * static_cast<Cost>(overtime);
    auto const timeWarp = ds.timeWarp(maxDuration);

    return std::make_pair(cost, timeWarp);
}

template <Segment... Segments>
int64_t Route::Proposal<Segments...>::breakDue() const
{
    // If the cached value is available (duration() was called first and
    // hasBreaks() is true), return it directly — zero cost.
    if (breakDue_ >= 0)
        return breakDue_;

    if (!route()->hasBreaks())
        return 0;

    // Defensive: compute breakDue_ as a side effect via the shared
    // forward-pass evaluator in duration().
    (void)duration();
    return breakDue_;
}

template <Segment... Segments>
Duration Route::Proposal<Segments...>::waiting() const
{
    // If the cached value is available (duration() was called first), return
    // it directly — zero cost.
    if (waiting_ >= 0)
        return Duration(waiting_);

    // Defensive: compute waiting_ as a side effect via duration().
    (void)duration();
    return waiting_ >= 0 ? Duration(waiting_) : Duration(0);
}

template <Segment... Segments>
Load Route::Proposal<Segments...>::excessLoad(size_t dimension) const
{
    if (empty())
        return 0;

    auto const &capacities = route()->capacity();
    auto const capacity = capacities[dimension];

    auto const fn = [&](auto &&segment, auto &&...args)
    {
        auto ls = segment.load(dimension);
        if (segment.endsAtReloadDepot())
            ls = ls.finalise(capacity);

        auto const merge = [&](auto const &self, auto &&other, auto &&...args)
        {
            if (other.startsAtReloadDepot())
                ls = ls.finalise(capacity);

            ls = LoadSegment::merge(ls, other.load(dimension));

            if constexpr (sizeof...(args) != 0)
            {
                if (other.endsAtReloadDepot() && other.size() > 1)
                    // Only when the segment contains more than just the depot.
                    // Checking for size speeds up the common case of a reload
                    // depot insertion.
                    ls = ls.finalise(capacity);

                self(self, std::forward<decltype(args)>(args)...);
            }
        };

        merge(merge, std::forward<decltype(args)>(args)...);
        return ls.excessLoad(capacity);
    };

    return std::apply(fn, segments_);
}
}  // namespace pyvrp::search

// Outputs a route into a given ostream in human-readable format
std::ostream &operator<<(std::ostream &out, pyvrp::search::Route const &route);

std::ostream &operator<<(std::ostream &out,  // for debugging
                         pyvrp::search::Route::Node const &node);

template <>  // specialisation for pyvrp::search::Route
inline pyvrp::Cost
pyvrp::CostEvaluator::penalisedCost(pyvrp::search::Route const &route) const
{
    if (route.empty())
        return 0;

    auto out = route.distanceCost()
               + route.durationCost()
               + route.fixedVehicleCost()
               + excessLoadPenalties(route.excessLoad())
               + twPenalty(route.timeWarp())
               + distPenalty(route.excessDistance(), 0);

    if (breakDuePenalty_ != 0)
        out += breakDuePenalty(route.breakDue());
    if (waitCostRate_ != 0)
        out += waitPenalty(route.waiting());

    return out;
}

#endif  // PYVRP_SEARCH_ROUTE_H
