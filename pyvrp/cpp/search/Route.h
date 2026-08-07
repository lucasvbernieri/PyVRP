#ifndef PYVRP_SEARCH_ROUTE_H
#define PYVRP_SEARCH_ROUTE_H

#include "Activity.h"
#include "CostEvaluator.h"
#include "DriveSegment.h"
#include "DurationSegment.h"
#include "LoadSegment.h"
#include "ProblemData.h"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <iosfwd>
#include <optional>
#include <utility>

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
    return std::make_tuple(std::get<sizeof...(Indices) - 1 - Indices>(
        std::forward<Tuple>(tuple))...);
}

template <class Tuple> auto constexpr reverse(Tuple &&tuple)
{
    auto constexpr size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
    auto constexpr indices = std::make_index_sequence<size>{};
    return reverse_impl(tuple, indices);
}
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
     * A simple class that tracks a proposed route structure. This new structure
     * can be efficiently evaluated by calling appropriate member functions,
     * detailing the newly proposed route's statistics.
     *
     * .. note::
     *
     *    The member functions may shortcut if they detect that a particular
     *    statistic has no impact on the newly proposed route's cost.
     */
    template <Segment... Segments> class Proposal
    {
        std::tuple<Segments...> segments_;

        // Cached breakDue value, computed during duration() when hasBreaks()
        // is true. -1 means not yet computed; breakDue() does its own fold
        // as a fallback.
        mutable int64_t breakDue_ = -1;

        // Helper: collects the forward-order (Activity, location) sequence
        // from segments_, with break locations corrected to inherit the
        // previous node's location (matching Route::update() behavior).
        // This guarantees distance() and duration() use consistent, corrected
        // locations and can never diverge again on cross-route moves.
        [[nodiscard]] std::pair<std::vector<Activity>, std::vector<size_t>>
        collectForwardSequence() const;

    public:
        Proposal(Segments &&...segments);

        /**
         * The proposal's route. This is the route associated with the first
         * and last segments, and determines the vehicle type and route profile
         * used when evaluating the proposal.
         */
        Route const *route() const;

        /**
         * Returns whether the proposed route is empty.
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
         * Returns the excess load of the proposed route.
         */
        Load excessLoad(size_t dimension) const;

        /**
         * Returns the number of mandatory break violations (breakDue) of the
         * proposed route. Returns 0 when no breaks are configured. If
         * ``duration()`` was called first, the cached value is used; otherwise
         * a dedicated fold over the segment chain is performed.
         */
        uint16_t breakDue() const;
    };

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
         * Assigns the node to the given route, at the given position, in the
         * given trip.
         */
        void assign(Route *route, size_t pos, size_t trip);

        /**
         * Removes the node from its assigned route, if any.
         */
        void unassign();
    };

private:
    using LoadSegments = std::vector<LoadSegment>;

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

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentAfter(Route const &route, size_t start);

        /** Index of the first node in this segment. */
        inline size_t startIdx() const;

        /** Index of the last node in this segment (end depot, always
         *  route.size() - 1). */
        inline size_t endIdx() const;

        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
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

        inline bool startsAtReloadDepot() const;
        inline bool endsAtReloadDepot() const;

        inline SegmentBefore(Route const &route, size_t end);

        /** Index of the first node in this segment (always 0, the start
         *  depot). */
        inline size_t startIdx() const;

        /** Index of the last node in this segment. */
        inline size_t endIdx() const;

        inline Distance distance(size_t profile) const;
        inline DurationSegment duration(size_t profile) const;
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
        Route const &route_;
        size_t const start;
        size_t const end;

    public:
        inline Route const *route() const;

        inline SegmentProxy front() const;  // at start
        inline SegmentProxy back() const;   // at end

        inline size_t size() const;
        inline size_t numClients() const;

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

    ProblemData const &data;

    VehicleType const &vehicleType_;

    Distance distance_;  // Separately cached cost components
    Cost distanceCost_;
    Distance excessDistance_;
    Duration duration_;
    Cost durationCost_;
    Duration timeWarp_;

    std::vector<Node> depots_;  // start, end, and reload depots (in that order)
    std::vector<Node> breaks_;  // CUSTOM_BREAK activities owned by this route

    std::vector<Node *> nodes;      // Nodes in this route
    std::vector<size_t> locations;  // Visited locations in this route

    std::vector<size_t> numClients_;  // Clients on start -> node (incl.)

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
    std::optional<std::vector<DriveSegment>> driveAfter;
    std::optional<std::vector<DriveSegment>> driveBefore;

#ifndef NDEBUG
    // When debug assertions are enabled, we use this flag to check whether
    // the statistics are still in sync with the route's nodes list. Statistics
    // are only updated after calling ``update()``. If that function has not
    // yet been called after inserting or removing nodes, this flag is active,
    // and asserts on statistics getters will fail.
    bool dirty = false;
#endif

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
    [[nodiscard]] inline uint16_t breakDue() const;

    /**
     * @return The routing profile of the vehicle servicing this route.
     */
    [[nodiscard]] inline size_t profile() const;

    /**
     * True if this route has no client visits, false otherwise.
     */
    [[nodiscard]] inline bool empty() const;

    /**
     * Number of activities on this route.
     */
    [[nodiscard]] inline size_t size() const;

    /**
     * Number of clients in this route.
     */
    [[nodiscard]] inline size_t numClients() const;

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

DurationSegment
Route::SegmentAfter::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durAfter[start];
}

LoadSegment const &Route::SegmentAfter::load(size_t dimension) const
{
    return route_.loadAfter[dimension][start];
}

DriveSegment
Route::SegmentAfter::driveState([[maybe_unused]] size_t profile) const
{
    if (!route_.driveAfter.has_value())
        return {};
    return route_.driveAfter.value()[start];
}

Distance Route::SegmentBefore::distance([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.cumDist[end];
}

DurationSegment
Route::SegmentBefore::duration([[maybe_unused]] size_t profile) const
{
    assert(profile == route_.profile());
    return route_.durBefore[end];
}

LoadSegment const &Route::SegmentBefore::load(size_t dimension) const
{
    return route_.loadBefore[dimension][end];
}

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

DurationSegment
Route::SegmentBetween::duration([[maybe_unused]] size_t profile) const
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
    std::vector<uint16_t> upcomingBreakMaskAt(route_.size(), 0);
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

        // atSecond: exact scheduled arrival at step+1, clamped to
        // the destination node's tw_early (via nextDurAt.startEarly()).
        auto const atSecond
            = std::max(durSeg.duration() - durSeg.timeWarp() + edgeDur,
                       nextDurAt.startEarly());

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
                        switch (brk.reset)
                        {
                        case CustomBreakReset::ALL_TIMERS:
                            drvSeg.driveTime_ = 0;
                            drvSeg.workTime_ = 0;
                            drvSeg.dutyTime_ = 0;
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
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance();
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
        || hasBreaks();  // breakDue penalty must be tracked
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

uint16_t Route::breakDue() const
{
    if (!hasBreaks() || !driveBefore.has_value())
        return 0;

    return driveBefore.value().back().breakDue_;
}

size_t Route::profile() const { return vehicleType_.profile; }

bool Route::empty() const { return numClients() == 0; }

size_t Route::size() const { return nodes.size(); }

size_t Route::numClients() const { return size() - numDepots(); }

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

    [[maybe_unused]] auto &&first = std::get<0>(segments_);
    [[maybe_unused]] auto &&last = std::get<sizeof...(Segments) - 1>(segments_);
    assert(first.route() == last.route());  // must start and end at same route

    // Must start and end at the route start and end.
    [[maybe_unused]] auto const &route = *this->route();
    assert(first.front().activity() == route[0]->activity());
    assert(last.back().activity() == route[route.size() - 1]->activity());
}

template <Segment... Segments> bool Route::Proposal<Segments...>::empty() const
{
    auto const numClients = std::apply(
        [](auto &&...args) { return (args.numClients() + ...); }, segments_);

    return numClients == 0;
}

template <Segment... Segments>
Route const *Route::Proposal<Segments...>::route() const
{
    return std::get<0>(segments_).route();
}

template <Segment... Segments>
std::pair<std::vector<Activity>, std::vector<size_t>>
Route::Proposal<Segments...>::collectForwardSequence() const
{
    std::vector<Activity> fwdActs;
    std::vector<size_t> fwdLocs;

    auto const pushNode = [&](Activity act, size_t loc)
    {
        if (act.isCustomBreak() && fwdLocs.size() > 0)
            loc = fwdLocs.back();  // break inherits previous location
        fwdActs.push_back(act);
        fwdLocs.push_back(loc);
    };

    auto const collect = [&](auto const &segment)
    {
        using Seg = std::decay_t<decltype(segment)>;

        if constexpr (std::is_same_v<Seg, SegmentBefore>)
        {
            auto const *r = segment.route();
            for (size_t i = 0; i <= segment.endIdx(); ++i)
                pushNode((*r)[i]->activity(), r->locations[i]);
        }
        else if constexpr (std::is_same_v<Seg, SegmentAfter>)
        {
            auto const *r = segment.route();
            auto const n = r->size();
            for (size_t i = segment.startIdx(); i < n; ++i)
                pushNode((*r)[i]->activity(), r->locations[i]);
        }
        else if constexpr (std::is_same_v<Seg, SegmentBetween>)
        {
            auto const *r = segment.route();
            for (size_t i = segment.startIdx(); i <= segment.endIdx(); ++i)
                pushNode((*r)[i]->activity(), r->locations[i]);
        }
        else
        {
            // BreakSegment, ClientSegment, or any other single-node
            // segment: push the front activity and its location.
            pushNode(segment.front().activity(),
                     segment.front().location());
        }
    };

    std::apply([&](auto const &... segs) { (collect(segs), ...); },
               segments_);

    return {std::move(fwdActs), std::move(fwdLocs)};
}

template <Segment... Segments>
std::pair<Cost, Distance> Route::Proposal<Segments...>::distance() const
{
    if (empty())
        return std::make_pair(0, 0);

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
    if (route()->hasBreaks()) [[unlikely]]
    {
        auto const [fwdActs, fwdLocs] = collectForwardSequence();

        Distance dist = 0;
        for (size_t i = 1; i < fwdLocs.size(); ++i)
            dist += matrix(fwdLocs[i - 1], fwdLocs[i]);

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
std::pair<Cost, Duration> Route::Proposal<Segments...>::duration() const
{
    if (empty())
        return std::make_pair(0, 0);

    auto const &data = route()->data;
    auto const unitDurationCost = route()->unitDurationCost();
    auto const unitOvertimeCost = route()->unitOvertimeCost();
    auto const shiftDuration = route()->shiftDuration();
    auto const maxDuration = route()->maxDuration();
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

        auto const duration = ds.duration();
        auto const overtime = std::max<Duration>(duration - shiftDuration, 0);
        auto const cost = unitDurationCost * static_cast<Cost>(duration)
                          + unitOvertimeCost * static_cast<Cost>(overtime);
        auto const timeWarp = ds.timeWarp(maxDuration);

        return std::make_pair(cost, timeWarp);
    };

    // ---- breakDue: shared forward-pass evaluator (parity by construction) ----
    bool const hasBrk = route()->hasBreaks();
    if (hasBrk) [[unlikely]]
    {
        auto const [fwdActs, fwdLocs] = collectForwardSequence();

        if (fwdActs.size() >= 2)
        {
            std::vector<Duration> atSecond(fwdActs.size());
            auto const result = evaluateForwardPass(fwdActs, fwdLocs, atSecond,
                                                    nullptr, data,
                                                    route()->vehicleType_);
            breakDue_ = result.breakDue;

            // Parity: use the same forward-pass duration/timeWarp that
            // Route::update() produces internally, instead of the reverse
            // DurationSegment fold (which is not perfectly associative
            // with time windows and diverges on cross-route moves).
            auto const dur = result.duration;
            auto const overtime = std::max<Duration>(dur - shiftDuration, 0);
            auto const dCost = unitDurationCost * static_cast<Cost>(dur)
                               + unitOvertimeCost * static_cast<Cost>(overtime);
            return std::make_pair(dCost, result.timeWarp);
        }
        else
            breakDue_ = 0;
    }
    else
        breakDue_ = 0;

    return std::apply(fn, detail::reverse(segments_));
}

template <Segment... Segments>
uint16_t Route::Proposal<Segments...>::breakDue() const
{
    // If the cached value is available (duration() was called first and
    // hasBreaks() is true), return it directly — zero cost.
    if (breakDue_ >= 0)
        return static_cast<uint16_t>(breakDue_);

    if (empty() || !route()->hasBreaks())
        return 0;

    // Defensive: compute breakDue_ as a side effect via the shared
    // forward-pass evaluator in duration().
    (void)duration();
    return static_cast<uint16_t>(breakDue_);
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
pyvrp::Cost
pyvrp::CostEvaluator::penalisedCost(pyvrp::search::Route const &route) const;

#endif  // PYVRP_SEARCH_ROUTE_H
