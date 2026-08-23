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
    auto const isDepot = node->isDepot();
    auto const isBreak = node->isCustomBreak();

    if (isDepot)  // is depot, so we need to insert a copy into our own memory
    {
        if (depots_.size() == depots_.capacity())  // then we reallocate and
        {                                          // must update references
            depots_.reserve(depots_.size() + 1);
            for (auto &depot : depots_)
                nodes[depot.pos()] = &depot;
        }

        node = &depots_.emplace_back(node->activity());
    }
    else if (isBreak)  // is break, insert a copy into breaks_ owned storage
    {
        if (breaks_.size() == breaks_.capacity())  // reallocate and fix refs
        {
            breaks_.reserve(breaks_.size() + 1);
            for (auto &brk : breaks_)
                nodes[brk.pos()] = &brk;
        }

        node = &breaks_.emplace_back(node->activity());
    }

    if (!isBreak && numTrips() > maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");

    nodes.insert(nodes.begin() + idx, node);
    node->assign(this, idx, nodes[idx - 1]->trip());

    for (size_t after = idx; after != nodes.size(); ++after)
    {
        nodes[after]->pos_ = after;
        if (isDepot)  // then we need to bump each following trip index
            nodes[after]->trip_++;
    }
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

#ifndef NDEBUG
    dirty = true;
#endif
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

#ifndef NDEBUG
    if (first->route_)
        first->route_->dirty = true;

    if (second->route_)
        second->route_->dirty = true;
#endif
}

void Route::update()
{
    locations.clear();
    for (auto const *node : nodes)
    {
        assert(node->isDepot() || node->isClient()
               || node->isCustomBreak());

        if (node->isDepot())
            locations.emplace_back(data.depot(node->idx()).location);
        else if (node->isCustomBreak())
            // CUSTOM_BREAK activities are location-less; they occur at the
            // vehicle's current stop. Use previous node's location as the
            // break's location (travel duration will be zero on the incoming
            // edge). The Route lane (3.7) will refine this when full break
            // infrastructure (DriveSegment) is integrated.
            locations.emplace_back(locations.empty() ? 0 : locations.back());
        else
            locations.emplace_back(data.client(node->idx()).location);
    }

    // Client counter.
    numClients_.resize(nodes.size());
    numClients_[0] = 0;
    for (size_t idx = 1; idx != nodes.size(); ++idx)
        numClients_[idx] = numClients_[idx - 1] + nodes[idx]->isClient();

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

        if (node->isCustomBreak())
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
                            // Anchor the relative offsets to the search-clock
                            // baseline (vehicle.twEarly). The search clock
                            // tracks progress from vehicle.twEarly, and
                            // startTime_ - vehicle.twEarly is absorbed by the
                            // forward pass, so offsets relative to route start
                            // map to [twEarly+early, twEarly+late] in
                            // search-clock units.
                            early = brk.tws.front().first
                                    + vehicleType_.twEarly;
                            late = brk.tws.back().second
                                   + vehicleType_.twEarly;
                        }
                        else
                        {
                            // Absolute (clock-time) windows: enforced directly.
                            // Mirroring evaluateForwardPass
                            // (DriveSegment.cpp, Step 1) — without this the
                            // local atSecondVec clamp below reads startEarly=0
                            // and the local drive fold diverges from the
                            // shared forward pass (parity violations, D5).
                            early = brk.tws.front().first;
                            late = brk.tws.back().second;
                        }
                    }
                    break;
                }
            durAt[idx] = DurationSegment(svc, Duration(0), early, late);
        }
        else if (!node->isReloadDepot())
            durAt[idx] = {data.client(node->idx())};
        else
            durAt[idx] = {data.depot(node->idx()), 0};
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
        }
    }

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

    // ----- Drive arrays (parallel arrays for break tracking) -----
    if (vehicleType_.hasBreaks() || data.hasSetup())
    {
        auto const &breaks = vehicleType_.custom_breaks;
        auto const resetAtReload = vehicleType_.reset_breaks_at_reload;
        auto const n = nodes.size();

        // --- driveAt: per-node drive segment ---
        driveAt.emplace();
        driveAt->resize(n);
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
        driveBefore.emplace();
        driveBefore->resize(n);
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
            // pass only feeds the driveAt/driveBefore/driveAfter arrays used
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
                        if (isBreakEligible(drs, brk))
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

        // --- driveAfter: backward suffix-sum ---
        // Compute each suffix DriveSegment from scratch using a forward
        // merge (same direction as the forward pass).  A backward merge
        // would start the break evaluation with zero accumulators at the
        // left boundary, missing any drive/work/duty accumulated in the
        // prefix, which leads to wrong trigger decisions (Cause C).
        //
        // Complexity O(n^2) in route length, which is acceptable because:
        //  - typical routes have tens to low hundreds of nodes,
        //  - this array is only built once per route update(),
        //  - CLOCK_TIME triggers use atSecond (exact) so break semantics
        //    are correct independent of accumulator history.
        driveAfter.emplace();
        driveAfter->resize(n);
        driveAfter->at(n - 1) = driveAt->at(n - 1);

        for (size_t start = 0; start < n - 1; ++start)
        {
            DriveSegment suffix = driveAt->at(start);
            for (size_t idx = start + 1; idx < n; ++idx)
            {
                auto const edgeDur
                    = durations(locations[idx - 1], locations[idx]);

                // Setup is work, never drive (extraWork), mirroring the
                // forward pass.
                Duration setup = 0;
                if (nodes[idx]->isClient()
                    && locations[idx] != locations[idx - 1])
                    setup = data.setupDuration(locations[idx]);

                suffix = DriveSegment::merge(edgeDur,
                                             suffix,
                                             driveAt->at(idx),
                                             breaks,
                                             atSecondVec[idx],
                                             upcomingBreakMaskAt[idx],
                                             setup);
                // Apply break reset at CUSTOM_BREAK nodes, matching the
                // forward-pass behaviour in update(). The eligibility gate
                // mirrors the forward pass: cumulative-trigger breaks are
                // only served when the metric reaches the trigger value.
                if (nodes[idx]->isCustomBreak())
                {
                    auto const breakId = nodes[idx]->idx();
                    for (auto const &brk : breaks)
                    {
                        if (brk.id == static_cast<size_t>(breakId))
                        {
                            if (isBreakEligible(suffix, brk))
                            {
                                switch (brk.reset)
                                {
                                case CustomBreakReset::ALL_TIMERS:
                                    // NOTE: lastResetAt_ is intentionally NOT
                                    // set here. The backward suffix rebuilds
                                    // from driveAt[start] with lastResetAt_=0;
                                    // SwapTails/Exchange operators already
                                    // reject CUSTOM_BREAK nodes, so practical
                                    // impact is nil.
                                    suffix.driveTime_ = 0;
                                    suffix.workTime_ = 0;
                                    suffix.dutyTime_ = 0;
                                    break;
                                case CustomBreakReset::DRIVE_AND_WORK:
                                    suffix.driveTime_ = 0;
                                    suffix.workTime_ = 0;
                                    break;
                                case CustomBreakReset::DRIVE_TIMER:
                                    suffix.driveTime_ = 0;
                                    break;
                                case CustomBreakReset::WORK_TIMER:
                                    suffix.workTime_ = 0;
                                    break;
                                case CustomBreakReset::NONE:
                                    break;
                                }
                                suffix.breaksTakenMask_
                                    |= static_cast<uint16_t>(1u)
                                       << (breakId & 0xF);
                            }
                            else
                            {
                                // Not eligible: no reset, drop the optimistic bit so
                                // the trigger can fire at later boundaries. The mask
                                // is 16-bit (max 16 distinct break ids per vehicle,
                                // pre-existing limitation).
                                suffix.breaksTakenMask_
                                    &= ~(static_cast<uint16_t>(1u)
                                         << (breakId & 0xF));
                            }
                            break;
                        }
                    }
                }
            }
            driveAfter->at(start) = suffix;
        }
    }
    else
    {
        // No breaks configured: deallocate to save memory.
        driveAt.reset();
        driveBefore.reset();
        driveAfter.reset();
    }

    // Load.
    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        auto const capacity = vehicleType_.capacity[dim];

        loadAt[dim].resize(nodes.size());
        loadAt[dim][0] = {vehicleType_, dim};  // initial load
        loadAt[dim][nodes.size() - 1] = {};

        for (size_t idx = 1; idx != nodes.size() - 1; ++idx)
            loadAt[dim][idx]
                = (nodes[idx]->isReloadDepot() || nodes[idx]->isCustomBreak())
                      ? LoadSegment{}
                      : LoadSegment{data.client(nodes[idx]->idx()), dim};

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
    if (vehicleType_.hasBreaks() || data.hasSetup())
    {
        std::vector<Activity> acts;
        acts.reserve(nodes.size());
        for (auto const *node : nodes)
            acts.push_back(node->activity());

        std::vector<Duration> at2(nodes.size());
        auto const result = evaluateForwardPass(acts, locations, at2, nullptr,
                                                &breakServicesAt_, data,
                                                vehicleType_, &durAt);

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

    auto const overtime = std::max<Duration>(duration_ - shiftDuration(), 0);
    // wait-cost-root-fix: duration cost excludes waiting (idle time is
    // charged separately by the CostEvaluator at its wait rate). Overtime
    // stays on the full duration (a driver held beyond the shift pays it).
    durationCost_ = unitDurationCost() * static_cast<Cost>(duration_ - waiting_)
                    + unitOvertimeCost() * static_cast<Cost>(overtime);

#ifndef NDEBUG
    dirty = false;
#endif
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

template <>
pyvrp::Cost
pyvrp::CostEvaluator::penalisedCost(pyvrp::search::Route const &route) const
{
    if (route.empty())
        return 0;

    // clang-format off
    return route.distanceCost()
         + route.durationCost()
         + route.fixedVehicleCost()
         + excessLoadPenalties(route.excessLoad())
         + twPenalty(route.timeWarp())
         + distPenalty(route.excessDistance(), 0)
         + breakDuePenalty(route.breakDue())
         + waitPenalty(route.waiting());
    // clang-format on
}
