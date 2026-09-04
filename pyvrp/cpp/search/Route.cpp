#include "Route.h"

#include <ostream>
#include <utility>

using pyvrp::search::Route;

Route::Node::Node(Activity::ActivityType type, size_t idx)
    : Node(Activity{type, idx})
{
}

Route::Node::Node(Activity activity)
    : activity_(activity), pos_(0), trip_(0), route_(nullptr)
{
}

void Route::Node::assign(Route *route, size_t pos, size_t trip)
{
    pos_ = pos;
    trip_ = trip;
    route_ = route;
}

void Route::Node::unassign()
{
    pos_ = 0;
    trip_ = 0;
    route_ = nullptr;
}

Route::Route(ProblemData const &data, size_t vehicleType)
    : data(data),
      vehicleType_(data.vehicleType(vehicleType)),
      loadAt(data.numLoadDimensions()),
      loadAfter(data.numLoadDimensions()),
      loadBefore(data.numLoadDimensions()),
      load_(data.numLoadDimensions()),
      excessLoad_(data.numLoadDimensions())
{
    clear();
}

Route::~Route() { clear(); }

std::vector<Route::Node *>::const_iterator Route::begin() const
{
    return nodes.begin();
}

std::vector<Route::Node *>::const_iterator Route::end() const
{
    return nodes.end();
}

size_t Route::vehicleType() const
{
    auto const &vehicleTypes = data.vehicleTypes();
    return std::distance(&vehicleTypes[0], &vehicleType_);
}

void Route::clear()
{
    if (nodes.size() == 2)  // then the route is already empty and we have
        return;             // nothing to do.

    for (auto *node : nodes)        // only unassign if in route; node may not
        if (node->route() == this)  // be if it's been assigned to another route
            node->unassign();       // while loading a new solution into the LS

    nodes.clear();
    depots_.clear();
    breaks_.clear();

    depots_.emplace_back(Activity::ActivityType::DEPOT,
                         vehicleType_.startDepot);
    depots_.emplace_back(Activity::ActivityType::DEPOT, vehicleType_.endDepot);

    for (size_t idx : {0, 1})
    {
        nodes.push_back(&depots_[idx]);
        depots_[idx].assign(this, idx, idx);
    }

    update();
    assert(empty());
}

void Route::reserve(size_t size) { nodes.reserve(size); }

void Route::insert(size_t idx, Node *node)
{
    assert(0 < idx && idx < nodes.size());

    auto const isBreak = node->isCustomBreak();

    switch (node->type())
    {
    case Activity::ActivityType::DEPOT:  // insert copy into owned memory
    {
        if (depots_.size() == depots_.capacity())  // then we reallocate and
        {                                          // must update references
            depots_.reserve(depots_.size() + 1);
            for (auto &depot : depots_)
                nodes[depot.pos()] = &depot;
        }

        node = &depots_.emplace_back(node->activity());
        break;
    }

    case Activity::ActivityType::CUSTOM_BREAK:  // insert copy into owned memory
    {
        if (breaks_.size() == breaks_.capacity())  // reallocate and fix refs
        {
            breaks_.reserve(breaks_.size() + 1);
            for (auto &brk : breaks_)
                nodes[brk.pos()] = &brk;
        }

        node = &breaks_.emplace_back(node->activity());
        break;
    }

    default:
        break;
    }

    if (!isBreak && numTrips() > maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");

    nodes.insert(nodes.begin() + idx, node);
    node->assign(this, idx, nodes[idx - 1]->trip());

    for (size_t after = idx; after != nodes.size(); ++after)
    {
        nodes[after]->pos_ = after;
        if (node->isDepot())  // then we need to bump each following trip index
            nodes[after]->trip_++;
    }

    dirty = true;
}

void Route::push_back(Node *node) { insert(nodes.size() - 1, node); }

void Route::remove(size_t idx)
{
    assert(0 < idx && idx < nodes.size() - 1);  // is not start or end depot
    assert(nodes[idx]->route() == this);        // must be in this route

    auto const isDepot = nodes[idx]->isReloadDepot();
    auto const isBreak = nodes[idx]->isCustomBreak();

    if (isDepot)
    {
        // We own this node - it's in our depots vector. We erase it, and then
        // update reload depot references that were invalidated by the erasure.
        auto const depotIdx = std::distance(depots_.data(), nodes[idx]);
        auto it = depots_.erase(depots_.begin() + depotIdx);
        for (; it != depots_.end(); ++it)
            nodes[it->pos()] = &*it;
    }
    else if (isBreak)
    {
        // We own this node - it's in our breaks_ vector. Erase and fix
        // pointers for any remaining break nodes whose storage moved.
        auto const breakIdx = std::distance(breaks_.data(), nodes[idx]);
        auto it = breaks_.erase(breaks_.begin() + breakIdx);
        for (; it != breaks_.end(); ++it)
            nodes[it->pos()] = &*it;
    }
    else
        // We do not own this node, so we only unassign it.
        nodes[idx]->unassign();

    nodes.erase(nodes.begin() + idx);  // remove dangling pointer
    for (auto after = idx; after != nodes.size(); ++after)
    {
        nodes[after]->pos_ = after;
        if (isDepot)  // then we need to decrease each following trip index
            nodes[after]->trip_--;
    }

    dirty = true;
}

void Route::swap(Node *first, Node *second)
{
    assert(!first->isDepot() && !second->isDepot());

    // TODO specialise std::swap for Node
    if (first->route_)
        first->route_->nodes[first->pos_] = second;

    if (second->route_)
        second->route_->nodes[second->pos_] = first;

    std::swap(first->route_, second->route_);
    std::swap(first->pos_, second->pos_);
    std::swap(first->trip_, second->trip_);

    if (first->route_)
        first->route_->dirty = true;

    if (second->route_)
        second->route_->dirty = true;
}

#ifdef PYVRP_STREAM_STATS
namespace
{
unsigned long long identityChecked = 0, identityBad = 0, identityWarp = 0;
unsigned long long clockChecked = 0, clockBad = 0;
struct IdentityReport
{
    ~IdentityReport()
    {
        if (identityChecked)
            std::fprintf(stderr,
                         "[identity] checked=%llu warp=%llu(%.3f) "
                         "mismatch=%llu(%.6f)\n",
                         identityChecked,
                         identityWarp,
                         double(identityWarp) / double(identityChecked),
                         identityBad,
                         double(identityBad) / double(identityChecked));
        if (clockChecked)
            std::fprintf(stderr,
                         "[clock] checked=%llu mismatch=%llu(%.6f)\n",
                         clockChecked,
                         clockBad,
                         double(clockBad) / double(clockChecked));
    }
} identityReport;
}  // namespace
#endif

void Route::update()
{
    PYVRP_PHASE(PH_UPDATE);

    locations.clear();
    activitiesAt_.clear();
    activitiesAt_.reserve(nodes.size());
    breakPositions_.clear();
    hasReleaseTimes_ = false;
    for (auto const *node : nodes)
    {
        assert(node->isDepot() || node->isClient()
               || node->isCustomBreak() || node->isShipment());

        if (node->isCustomBreak())
            breakPositions_.push_back(
                static_cast<uint32_t>(activitiesAt_.size()));
        activitiesAt_.push_back(node->activity());

        switch (node->type())
        {
        case Activity::ActivityType::DEPOT:
            locations.emplace_back(data.depot(node->idx()).location);
            break;

        case Activity::ActivityType::CLIENT:
        {
            auto const &client = data.client(node->idx());
            locations.emplace_back(client.location);
            // Gates the single-round break evaluation; see the in-line window
            // clearing in runStreamForward(). Free here: this loop already
            // touches every client.
            hasReleaseTimes_ |= client.releaseTime > 0;
            break;
        }

        case Activity::ActivityType::PICKUP:
        {
            auto const &pickup = data.shipment(node->idx()).pickup;
            locations.emplace_back(pickup.location);
            break;
        }

        case Activity::ActivityType::DELIVERY:
        {
            auto const &delivery = data.shipment(node->idx()).delivery;
            locations.emplace_back(delivery.location);
            break;
        }

        case Activity::ActivityType::CUSTOM_BREAK:
            // CUSTOM_BREAK activities are location-less; they occur at the
            // vehicle's current stop. Use previous node's location as the
            // break's location (travel duration will be zero on the incoming
            // edge).
            locations.emplace_back(locations.empty() ? 0 : locations.back());
            break;
        }
    }

    // Client counter.
    numClients_.resize(nodes.size());
    numClients_[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        numClients_[idx] = numClients_[idx - 1] + nodes[idx]->isClient();

    // Pickup counter.
    numPickups_.resize(nodes.size());
    numPickups_[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        numPickups_[idx] = numPickups_[idx - 1] + nodes[idx]->isPickup();

    // Delivery counter.
    numDeliveries_.resize(nodes.size());
    numDeliveries_[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        numDeliveries_[idx]
            = numDeliveries_[idx - 1] + nodes[idx]->isDelivery();

    // Distance.
    auto const &distMat = data.distanceMatrix(profile());

    cumDist.resize(nodes.size());
    cumDist[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        cumDist[idx]
            = cumDist[idx - 1] + distMat(locations[idx - 1], locations[idx]);

    // Duration.
    durAt.resize(nodes.size());

    auto const &start = data.depot(startDepot());
    DurationSegment const vehStart(vehicleType_, vehicleType_.startLate);
    DurationSegment const depotStart(start, start.serviceDuration);
    durAt[0] = DurationSegment::merge(vehStart, depotStart);

    auto const &end = data.depot(endDepot());
    DurationSegment const depotEnd(end, 0);
    DurationSegment const vehEnd(vehicleType_, vehicleType_.twLate);
    durAt[nodes.size() - 1] = DurationSegment::merge(depotEnd, vehEnd);

    for (size_t idx = 1; idx != nodes.size() - 1; ++idx)
    {
        auto const *node = nodes[idx];
switch (node->type())
        {
        case Activity::ActivityType::DEPOT:
            durAt[idx] = {data.depot(node->idx()), 0};
            break;

        case Activity::ActivityType::CLIENT:
            durAt[idx] = {data.client(node->idx())};
            break;

        case Activity::ActivityType::PICKUP:
            durAt[idx] = {data.shipment(node->idx()).pickup};
            break;

        case Activity::ActivityType::DELIVERY:
            durAt[idx] = {data.shipment(node->idx()).delivery};
            break;

        case Activity::ActivityType::CUSTOM_BREAK:
        {
            // CUSTOM_BREAK: use the break's service duration. The break's
            // id is in node->idx(). The tws and reset are handled by the
            // DriveSegment and forward-pass logic in update(). Lookup by id
            // (break ids need not match vector positions), mirroring the
            // eligibility gate and setSchedule().
            auto const breakId = node->idx();
            Duration svc(0);
            Duration early = 0;
            Duration late = std::numeric_limits<Duration>::max();
            for (auto const &brk : vehicleType_.custom_breaks)
                if (brk.id == static_cast<size_t>(breakId))
                {
                    svc = brk.service;
                    if (!brk.tws.empty())
                    {
                        if (brk.twsRelative)
                        {
                            early = brk.tws.front().first
                                    + vehicleType_.twEarly;
                            late = brk.tws.back().second
                                   + vehicleType_.twEarly;
                        }
                        else
                        {
                            early = brk.tws.front().first;
                            late = brk.tws.back().second;
                        }
                    }
                    break;
                }
            durAt[idx] = DurationSegment(svc, Duration(0), early, late);
            break;
        }
        }
    }

    auto const &durations = data.durationMatrix(profile());

    durBefore.resize(nodes.size());
    durBefore[0] = durAt[0];

    // Pre-allocate atSecond vector for CLOCK_TIME evaluation. Needed when the
    // vehicle type has break rules configured, or when setup durations require
    // the location-aware forward pass (setup shifts the boundary times used by
    // the drive segment merges below).
    std::vector<Duration> atSecondVec;
    if (vehicleType_.hasBreaks() || data.hasSetup())
    {
        atSecondVec.resize(nodes.size());
        atSecondVec[0] = durBefore[0].duration() - durBefore[0].timeWarp();
        cumT_.assign(nodes.size(), 0);
        cumM_.assign(nodes.size(), 0);
        cumT_[0] = 0;
        cumM_[0] = atSecondVec[0].get();  // twEarly(0) - cumT_[0], by anchor
    }

    for (size_t idx = 1; idx != nodes.size(); ++idx)
    {
        auto const prev = idx - 1;
        auto before = nodes[prev]->isReloadDepot()
                          ? durBefore[prev].finaliseBack()
                          : durBefore[prev];

        if (nodes[prev]->isReloadDepot())
        {
            // Then we need to first account for depot service before we merge
            // with the idx segment.
            auto const &depot = data.depot(nodes[prev]->idx());
            before = DurationSegment::merge(before, {depot.serviceDuration});
        }

        auto const edgeDur = durations(locations[prev], locations[idx]);

        // Setup: charged when entering a client whose location differs from
        // the previous node's location (VROOM canonical rule). Added to the
        // client's service duration so it contributes to route duration
        // without shifting the time window.
        Duration setup = 0;
        if (nodes[idx]->isClient() && locations[idx] != locations[prev])
            setup = data.setupDuration(locations[idx]);

        auto second = setup == 0 ? durAt[idx] : durAt[idx].withService(setup);
        durBefore[idx] = DurationSegment::merge(edgeDur, before, second);

        // Compute atSecond — conservative arrival time at this node (the
        // merge boundary), used for CLOCK_TIME interval-crossing detection
        // in the DriveSegment forward/backward passes.
        if (vehicleType_.hasBreaks() || data.hasSetup())
        {
            // duration()+startEarly() is the real clock of this fold (waiting
            // lives in the startEarly offset; diffWait preserves the
            // invariant). timeWarp is NOT part of the clock here — mirroring
            // evaluateForwardPass (D5), subtracting it would collapse the
            // clock below earlier arrivals (non-monotonic) and poison the
            // first-due/lateness terms. The clock is ABSOLUTE (midnight-
            // anchored), so startEarly() is added directly — no anchor
            // subtraction (which would double-count the anchor for
            // twEarly > 0).
            Duration const earlyArrival = durBefore[prev].duration()
                                          + durBefore[prev].startEarly()
                                          + edgeDur + setup;

            Duration nodeEarly = 0;
            auto const *node = nodes[idx];
            if (node->isClient())
                nodeEarly = data.client(node->idx()).twEarly;
            else if (node->isDepot())
                nodeEarly = data.depot(node->idx()).twEarly;
            else if (node->isCustomBreak())
                // Clamp to the break's window startEarly, mirroring the
                // Proposal gate behaviour.  Without this, lastResetAt_ for
                // ALL_TIMERS breaks would be set from the un-clamped arrival,
                // creating a permanent offset for any absolute-window break
                // with startEarly > 0.
                nodeEarly = durAt[idx].startEarly();

            atSecondVec[idx] = std::max(earlyArrival, nodeEarly);

            // Max-plus tables (see Route.h). cumT_ accumulates the edge into
            // this node plus the service at the previous one; cumM_ is the
            // running max of twEarly - cumT_, which is the only prefix-
            // independent part of the unrolled recurrence.
            cumT_[idx] = cumT_[prev] + edgeDur.get() + setup.get()
                         + durAt[prev].duration().get();
            cumM_[idx] = std::max(cumM_[prev],
                                  nodeEarly.get() - cumT_[idx]);

#ifdef PYVRP_STREAM_STATS
            // The table must reproduce the walk exactly from any earlier
            // anchor. Checked here against the anchor that is always available
            // (the route start), on every node of every update.
            if (durBefore[idx].timeWarp().get() == 0)
            {
                auto const predicted
                    = cumT_[idx]
                      + std::max(atSecondVec[0].get() - cumT_[0], cumM_[idx]);
                ++clockChecked;
                if (predicted != atSecondVec[idx].get())
                {
                    ++clockBad;
                    if (clockBad <= 3)
                        std::fprintf(stderr,
                                     "[clock] idx=%zu got=%lld want=%lld\n",
                                     idx,
                                     (long long)predicted,
                                     (long long)atSecondVec[idx].get());
                }
            }
#endif

#ifdef PYVRP_STREAM_STATS
            // Validating the identity the max-plus unrolling rests on:
            //   S(i) == atSecond(i) + service(i),  with S = duration+startEarly
            // If it holds, atSecond over a stretch unrolls to
            //   atSecond(m) = max( S(q-1) + cum(q..m),
            //                      max_j<=m ( twEarly(j) + cum(j..m) ) )
            // whose second term is prefix-INDEPENDENT and so cacheable per
            // route -- which would make locating a trigger crossing O(1) per
            // probe instead of needing interior range folds.
            {
                auto const lhs = durBefore[idx].duration().get()
                                 + durBefore[idx].startEarly().get();
                auto const rhs = atSecondVec[idx].get()
                                 + second.duration().get();
                ++identityChecked;
                if (durBefore[idx].timeWarp().get() > 0)
                    ++identityWarp;
                else if (lhs != rhs)
                {
                    ++identityBad;
                    if (identityBad <= 3)
                        std::fprintf(stderr,
                                     "[identity] idx=%zu lhs=%lld rhs=%lld"
                                     " diff=%lld\n",
                                     idx,
                                     (long long)lhs,
                                     (long long)rhs,
                                     (long long)(lhs - rhs));
                }
            }
#endif
        }
    }

    // ----- Drive arrays (parallel arrays for break tracking) -----
    if (vehicleType_.hasBreaks() || data.hasSetup())
    {
        auto const &breaks = vehicleType_.custom_breaks;
        auto const resetAtReload = vehicleType_.reset_breaks_at_reload;
        auto const n = nodes.size();

        // --- driveAt: per-node drive segment ---
        // ``emplace()`` on an engaged optional destroys the vector -- freeing
        // its buffer -- before ``resize`` allocates a fresh one. Assigning
        // instead keeps the buffer (and its cache residency) across updates,
        // with the same all-default contents.
        if (!driveAt)
            driveAt.emplace();
        driveAt->assign(n, DriveSegment{});
        driveAt->at(0) = DriveSegment::fromDepot();    // start depot
        // Initialize lastResetAt_ to the effective route start time so
        // DUTY_TIME does not count midnight-to-departure waiting. Uses
        // the same formula as evaluateForwardPass for parity.
        {
            auto const edgeDepot = durations(locations[0], locations[1]);
            auto const effectiveStart = std::max(
                atSecondVec[0],
                atSecondVec[1] - edgeDepot);
            driveAt->at(0).lastResetAt_ = effectiveStart.get();
        }
        driveAt->at(n - 1) = DriveSegment::fromDepot();  // end depot
        for (size_t idx = 1; idx != n - 1; ++idx)
        {
            auto const *node = nodes[idx];
            if (node->isCustomBreak())
            {
                // CUSTOM_BREAK: mark this break as already taken in the mask
                // so the merge does not re-trigger it. The reset is applied
                // in the forward pass loop below.
                auto const breakId = node->idx();
                driveAt->at(idx)
                    = DriveSegment(0, 0, 0,
                                   static_cast<uint16_t>(1u) << (breakId & 0xF),
                                   0);
            }
            else if (node->isDepot())
                driveAt->at(idx) = DriveSegment::fromDepot();
            else
                driveAt->at(idx)
                    = DriveSegment::fromClient(data.client(node->idx()).serviceDuration);
        }

        // --- driveBefore: forward prefix-sum ---
        if (!driveBefore)
            driveBefore.emplace();
        driveBefore->assign(n, DriveSegment{});
        driveBefore->at(0) = driveAt->at(0);

        // upcomingBreakMaskAt[idx]: bitmask of breaks whose CUSTOM_BREAK node
        // lies at a position strictly AFTER idx. Their triggers must not fire
        // at this boundary — the break is still scheduled ahead in the route
        // (the gate at its own node decides service; violations are only
        // incurred at boundaries subsequent to the break's position).
        std::vector<uint16_t> upcomingBreakMaskAt(n, 0);
        {
            uint16_t run = 0;
            for (size_t idx = n; idx-- > 0;)
            {
                upcomingBreakMaskAt[idx] = run;
                if (nodes[idx]->isCustomBreak())
                    run |= static_cast<uint16_t>(1u)
                           << (nodes[idx]->idx() & 0xF);
            }
        }

        for (size_t idx = 1; idx != n; ++idx)
        {
            auto const prev = idx - 1;
            auto const edgeDur = durations(locations[prev], locations[idx]);

            // Setup is work, never drive: pass it as extraWork so it enters
            // the work/duty accumulators but not driveTime_.
            Duration setup = 0;
            if (nodes[idx]->isClient() && locations[idx] != locations[prev])
                setup = data.setupDuration(locations[idx]);

            // The due-mask output of the drive pass is intentionally not
            // requested here: update()'s authoritative breakDueMask_ comes
            // from the shared evaluateForwardPass below, and this local drive
            // pass only feeds the driveAt/driveBefore arrays used
            // by the search operators.
            auto drs = DriveSegment::merge(edgeDur,
                                           driveBefore->at(prev),
                                           driveAt->at(idx),
                                           breaks,
                                           atSecondVec[idx],
                                           upcomingBreakMaskAt[idx],
                                           setup,
                                           nullptr);

            // reset_breaks_at_reload: arrival at a reload depot resets all
            // accumulators (but preserves mask).
            if (resetAtReload && nodes[idx]->isReloadDepot())
                drs = {0,
                       0,
                       0,
                       drs.breaksTakenMask_,
                       drs.lastResetAt_};

            // CUSTOM_BREAK: the break activity is visited at idx. The break is
            // only served (reset applied, bit kept) when the cumulative metric
            // has reached its trigger value (isBreakEligible). Otherwise the
            // reset is skipped and the optimistic taken-bit is removed from the
            // mask so the trigger can fire (and breakDue be accounted) at
            // subsequent route boundaries.
            if (nodes[idx]->isCustomBreak())
            {
                auto const breakId = nodes[idx]->idx();
                for (auto const &brk : breaks)
                {
                    if (brk.id == static_cast<size_t>(breakId))
                    {
                        // Window-close gate: a DUE break whose arrival is past
                        // its window close is not servable — drop it (no reset,
                        // no taken bit). Mirrors evaluateForwardPass.
                        auto const pastClose
                            = isBreakPastWindowClose(brk,
                                                    atSecondVec[idx],
                                                    vehicleType_.twEarly);
                        if (isBreakEligible(drs, brk) && !pastClose)
                        {
                            // D5: extend the served rest to absorb waiting
                            // before the next client's (still-closed) window
                            // opens. Applies to any served DUTY_TIME break, not
                            // only overnight rests.
                            bool const extend
                                = brk.trigger == CustomBreakTrigger::DUTY_TIME
                                  && idx + 1 < n && nodes[idx + 1]->isClient();
                            Duration travel = 0;
                            Duration nextOpen = 0;
                            if (extend)
                            {
                                travel = durations(locations[idx],
                                                   locations[idx + 1]);
                                // nextOpen is the next client's absolute
                                // (midnight-anchored) twEarly, matching the
                                // absolute atSecondVec clock — no anchor
                                // subtraction.
                                nextOpen
                                    = data.client(nodes[idx + 1]->idx()).twEarly;
                            }
                            auto const effSvc
                                = breakEffectiveService(brk.service,
                                                        atSecondVec[idx],
                                                        extend,
                                                        travel,
                                                        nextOpen);

                            switch (brk.reset)
                            {
                            case CustomBreakReset::ALL_TIMERS:
                                drs.driveTime_ = 0;
                                drs.workTime_ = 0;
                                drs.dutyTime_ = 0;
                                drs.lastResetAt_ = atSecondVec[idx].get()
                                                    + effSvc.get();
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
                            drs.breaksTakenMask_
                                |= static_cast<uint16_t>(1u)
                                   << (breakId & 0xF);
                            // breakDue is NOT incremented — the break is serviced
                        }
                        else
                        {
                            // Not eligible: no reset, drop the optimistic bit so
                            // the trigger can fire at later boundaries. The mask
                            // is 16-bit (max 16 distinct break ids per vehicle,
                            // pre-existing limitation).
                            drs.breaksTakenMask_
                                &= ~(static_cast<uint16_t>(1u)
                                     << (breakId & 0xF));
                        }
                        break;
                    }
                }
            }

            driveBefore->at(idx) = drs;
        }

        // ``driveAfter`` used to be built here: a backward suffix fold that
        // cost an O(n^2) nest of DriveSegment::merge calls per update, on the
        // CustomBreak overload that chases the rule vectors. Its only reader
        // was SegmentAfter::driveState(), which nothing ever called, so the
        // whole thing was dead work. Removed together with that accessor; if
        // a suffix drive state is ever needed, rebuild it from breakSeed_ or
        // from the forward pass rather than reinstating this nest.

        // --- cumDurEdge / cumSvcLB: prefix sums for the cheap duration
        // lower bound used by Proposal::durationLowerBound() (see
        // CostEvaluator::deltaCost). Same pattern as ``cumDist`` above, but
        // for duration edges, plus a parallel prefix of per-node MINIMUM
        // service. Only built here (break/setup path) since that is the
        // only place duration() is expensive enough to be worth pruning.
        if (!cumDurEdge)
            cumDurEdge.emplace();
        cumDurEdge->resize(n);
        cumDurEdge->at(0) = 0;
        for (size_t idx = 1; idx != n; ++idx)
            cumDurEdge->at(idx) = cumDurEdge->at(idx - 1)
                + durations(locations[idx - 1], locations[idx]);

        if (!cumSvcLB)
            cumSvcLB.emplace();
        cumSvcLB->resize(n);
        for (size_t idx = 0; idx != n; ++idx)
        {
            // The end depot (always the last node) never contributes
            // service -- mirrors durAt[n - 1]'s depotEnd construction
            // above, which is built with serviceDuration 0.
            Duration svc = 0;
            if (idx != n - 1)
            {
                auto const *node = nodes[idx];
                switch (node->type())
                {
                case Activity::ActivityType::DEPOT:
                    svc = data.depot(node->idx()).serviceDuration;
                    break;

                case Activity::ActivityType::CLIENT:
                    svc = data.client(node->idx()).serviceDuration;
                    break;

                case Activity::ActivityType::PICKUP:
                    svc = data.shipment(node->idx()).pickup.serviceDuration;
                    break;

                case Activity::ActivityType::DELIVERY:
                    svc = data.shipment(node->idx()).delivery.serviceDuration;
                    break;

                case Activity::ActivityType::CUSTOM_BREAK:
                {
                    // Minimum (unextended) service for this break id. D5
                    // only ever EXTENDS a served break's service, so using
                    // the configured minimum here keeps the bound from ever
                    // overestimating.
                    auto const breakId = node->idx();
                    for (auto const &rule : vehicleType_.breakRules)
                        if (rule.id == static_cast<size_t>(breakId))
                        {
                            svc = Duration(rule.service);
                            break;
                        }
                    break;
                }
                }
            }

            cumSvcLB->at(idx)
                = (idx == 0 ? Duration(0) : cumSvcLB->at(idx - 1)) + svc;
        }
    }
    else
    {
        // No breaks configured: deallocate to save memory.
        driveAt.reset();
        driveBefore.reset();
        cumDurEdge.reset();
        cumSvcLB.reset();
    }

    // Load.
    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        auto const capacity = vehicleType_.capacity[dim];

        loadAt[dim].resize(nodes.size());
        loadAt[dim][0] = {vehicleType_, dim};  // initial load
        loadAt[dim][nodes.size() - 1] = {};

for (size_t pos = 1; pos != nodes.size() - 1; ++pos)
            switch (nodes[pos]->type())
            {
            case Activity::ActivityType::DEPOT:
                loadAt[dim][pos] = {};
                break;

            case Activity::ActivityType::CLIENT:
                loadAt[dim][pos] = {data.client(nodes[pos]->idx()), dim};
                break;

            case Activity::ActivityType::PICKUP:
                [[fallthrough]];
            case Activity::ActivityType::DELIVERY:
            {
                auto const &shipment = data.shipment(nodes[pos]->idx());
                loadAt[dim][pos] = {shipment, nodes[pos]->type(), dim};
                break;
            }

            case Activity::ActivityType::CUSTOM_BREAK:
                loadAt[dim][pos] = {};
                break;
            }

        loadBefore[dim].resize(nodes.size());
        loadBefore[dim][0] = loadAt[dim][0];
        for (size_t idx = 1; idx != nodes.size(); ++idx)
        {
            auto const prev = idx - 1;
            if (nodes[prev]->isReloadDepot())
                loadBefore[dim][idx] = LoadSegment::merge(
                    loadBefore[dim][prev].finalise(capacity), loadAt[dim][idx]);
            else
                loadBefore[dim][idx] = LoadSegment::merge(loadBefore[dim][prev],
                                                          loadAt[dim][idx]);
        }

        load_[dim] = 0;
        excessLoad_[dim]
            = loadBefore[dim][nodes.size() - 1].excessLoad(capacity);
        for (auto it = depots_.begin() + 1; it != depots_.end(); ++it)
            load_[dim] += loadBefore[dim][it->pos()].load();

        loadAfter[dim].resize(nodes.size());
        loadAfter[dim][nodes.size() - 1] = loadAt[dim][nodes.size() - 1];
        for (size_t idx = nodes.size() - 1; idx != 0; --idx)
        {
            auto const prev = idx - 1;
            if (nodes[idx]->isReloadDepot())
                loadAfter[dim][prev] = LoadSegment::merge(
                    loadAt[dim][prev], loadAfter[dim][idx].finalise(capacity));
            else
                loadAfter[dim][prev] = LoadSegment::merge(loadAt[dim][prev],
                                                          loadAfter[dim][idx]);
        }
    }

    // These cost components are separately cached as well because they are
    // requested *a lot*.
    distance_ = cumDist.back();
    excessDistance_ = std::max<Distance>(distance_ - maxDistance(), 0);
    distanceCost_ = unitDistanceCost() * static_cast<Cost>(distance_);

    // Setup-aware duration/timeWarp: reuse the shared forward-pass evaluator
    // so that setup is charged exactly once, in a single choke point, and
    // update() can never diverge from Proposal. This also covers break routes
    // (setup == 0 there), guaranteeing parity by construction rather than by
    // re-implementing the location-aware rule in the reverse fold.
    breakServicesAt_.assign(nodes.size(), 0);
    fwdDrive_.reset();
    breakSeedValid_ = false;
    breakSeed_.clear();
    if (vehicleType_.hasBreaks() || data.hasSetup())
    {
        std::vector<Activity> acts;
        acts.reserve(nodes.size());
        for (auto const *node : nodes)
            acts.push_back(node->activity());

        std::vector<Duration> at2(nodes.size());

        // Alternativa D: authoritative per-position seed outputs of the shared
        // forward pass (final round). ``drvRows`` captures the per-node final
        // drive state; ``firstDueVals``/``firstDuePoss``/``servedMask`` the
        // per-id D3 facts used to seed prefix fast-forwarding in proposals.
        bool const wantSeed = vehicleType_.hasBreaks();
        std::optional<std::vector<DriveSegment>> drvRows;
        std::vector<int64_t> firstDueVals;
        std::vector<size_t> firstDuePoss;
        uint16_t servedMask = 0;
        if (wantSeed)
        {
            size_t maxBreakId = 0;
            for (auto const &brk : vehicleType_.custom_breaks)
                maxBreakId = std::max(maxBreakId, static_cast<size_t>(brk.id));
            breakSeed_.assign(maxBreakId + 1, RouteBreakSeed{});
            firstDueVals.assign(maxBreakId + 1, -1);
            firstDuePoss.assign(maxBreakId + 1, std::numeric_limits<size_t>::max());
            if (!drvRows)
                drvRows.emplace();
            drvRows->assign(nodes.size(), DriveSegment{});
        }

        // Refresh the cached prefix fold (``durBefore``) from the evaluator's
        // FINAL duration pass: the evaluator may have mutated ``durAt`` (D5
        // rest extension, due-ness window clearing) and re-run its pass, and
        // ``durBefore`` was computed earlier from the pre-mutation ``durAt``.
        auto const result = evaluateForwardPass(
            acts, locations, at2, &durBefore, &breakServicesAt_, data,
            vehicleType_, &durAt, wantSeed ? &*drvRows : nullptr,
            wantSeed ? firstDueVals.data() : nullptr,
            wantSeed ? firstDuePoss.data() : nullptr,
            wantSeed ? &servedMask : nullptr);

        if (wantSeed)
        {
            fwdDrive_ = std::move(drvRows);
            for (size_t p = 1; p + 1 < nodes.size(); ++p)
            {
                auto const *node = nodes[p];
                if (!node->isCustomBreak())
                    continue;
                auto const b = node->idx();
                if (b < breakSeed_.size())
                {
                    breakSeed_[b].occPos = p;
                    breakSeed_[b].arrivalAtOcc = at2[p];
                }
            }
            for (size_t b = 0; b < breakSeed_.size(); ++b)
            {
                breakSeed_[b].firstDueVal = firstDueVals[b];
                breakSeed_[b].firstDuePos = firstDuePoss[b];
                auto const bit = static_cast<uint16_t>(1u) << (b & 0xF);
                breakSeed_[b].served = (servedMask & bit) != 0;
            }
            breakSeedValid_ = true;
        }

        duration_ = result.duration;
        timeWarp_ = result.timeWarp;
        waiting_ = result.waiting;
        breakDueMask_ = result.breakDueMask;
        breakDue_ = result.breakDue;
    }
    else
    {
        duration_ = durBefore.back().duration();
        timeWarp_ = durBefore.back().timeWarp(maxDuration());
        waiting_ = durBefore.back().waiting();
        breakDueMask_ = 0;
    }

    // ----- Duration suffix fold (durAfter) -----
    // Computed here, AFTER the shared forward pass, on the FINAL (possibly
    // D5-mutated) ``durAt``. Kept after the evaluator so the cached
    // SegmentAfter summaries reproduce the forward pass exactly — a segment
    // fold recomposition of the route must not report a D5-absorbed rest as
    // idle waiting (wrong waiting(), wrong atSecond chains downstream).
    durAfter.resize(nodes.size());
    durAfter[nodes.size() - 1] = durAt[nodes.size() - 1];
    for (size_t next = nodes.size() - 1; next != 0; --next)
    {
        auto const idx = next - 1;
        auto after = nodes[next]->isReloadDepot()
                         ? durAfter[next].finaliseFront()
                         : durAfter[next];

        if (nodes[idx]->isReloadDepot())
        {
            // This is not entirely correct logically, since we now do service
            // at idx after already travelling to next, but that's OK since
            // we're essentially using the trick of adding service to the
            // outgoing edge.
            auto const &depot = data.depot(nodes[idx]->idx());
            after = DurationSegment::merge({depot.serviceDuration}, after);
        }

        auto const edgeDur = durations(locations[idx], locations[next]);
        durAfter[idx] = DurationSegment::merge(edgeDur, durAt[idx], after);
    }

    auto const overtime = std::max<Duration>(duration_ - shiftDuration(), 0);
    // wait-cost-root-fix: duration cost excludes waiting (idle time is
    // charged separately by the CostEvaluator at its wait rate). Overtime
    // stays on the full duration (a driver held beyond the shift pays it).
    durationCost_ = unitDurationCost() * static_cast<Cost>(duration_ - waiting_)
                    + unitOvertimeCost() * static_cast<Cost>(overtime);

    dirty = false;
}

bool Route::operator==(Route const &other) const
{
    assert(!dirty && !other.dirty);

    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the nodes are all equal.
    // clang-format off
    return distance_ == other.distance_
        && duration_ == other.duration_
        && timeWarp_ == other.timeWarp_
        && vehicleType_ == other.vehicleType_
        && nodes == other.nodes;
    // clang-format on
}

std::ostream &operator<<(std::ostream &out, Route const &route)
{
    for (size_t idx = 1; idx != route.size() - 1; ++idx)
    {
        if (idx != 1)
            out << ' ';

        if (route[idx]->isReloadDepot())
            out << '|';
        else
            out << *route[idx];
    }

    return out;
}

std::ostream &operator<<(std::ostream &out, Route::Node const &node)
{
    return out << node.activity();
}
