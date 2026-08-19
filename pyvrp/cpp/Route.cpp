#include "Route.h"
#include "DriveSegment.h"
#include "DurationSegment.h"
#include "LoadSegment.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <limits>

using pyvrp::Activity;
using pyvrp::Cost;
using pyvrp::Distance;
using pyvrp::Duration;
using pyvrp::Load;
using pyvrp::Route;

using Client = size_t;

Route::ScheduledActivity::ScheduledActivity(Activity activity,
                                            size_t trip,
                                            Duration startTime,
                                            Duration endTime,
                                            Duration waitDuration,
                                            Duration timeWarp)
    : Activity(activity),
      trip_(trip),
      startTime_(startTime),
      endTime_(endTime),
      waitDuration_(waitDuration),
      timeWarp_(timeWarp)
{
    assert(startTime_ <= endTime_);
}

size_t Route::ScheduledActivity::trip() const { return trip_; }

Duration Route::ScheduledActivity::startTime() const { return startTime_; }

Duration Route::ScheduledActivity::endTime() const { return endTime_; }

Duration Route::ScheduledActivity::duration() const
{
    return endTime_ - startTime_;
}

Duration Route::ScheduledActivity::waitDuration() const
{
    return waitDuration_;
}

Duration Route::ScheduledActivity::timeWarp() const { return timeWarp_; }

void Route::validate(ProblemData const &data,
                     Activities const &activities) const
{
    auto const &vehData = data.vehicleType(vehicleType_);

    size_t numTrips = 1;
    for (auto const &activity : activities)  // some quick checks up front
    {
        if (activity.isDepot())
        {
            numTrips++;  // this is a reload depot, so we start another trip

            if (activity.idx() >= data.numDepots())
            {
                std::ostringstream msg;
                msg << "Depot " << activity << " is not understood.";
                throw std::invalid_argument(msg.str());
            }
        }

        if (activity.isClient() && activity.idx() >= data.numClients())
        {
            std::ostringstream msg;
            msg << "Client " << activity << " is not understood.";
            throw std::invalid_argument(msg.str());
        }
    }

    if (numTrips > vehData.maxTrips())
        throw std::invalid_argument("Vehicle cannot perform this many trips.");
}

void Route::setSchedule(ProblemData const &data,
                        Activities const &activities,
                        std::vector<Duration> const &breakServices)
{
    schedule_.reserve(activities.size() + 2);  // incl. start and end depots

    auto const &vehData = data.vehicleType(vehicleType_);
    auto const &durations = data.durationMatrix(vehData.profile);
    auto const &start = data.depot(vehData.startDepot);

    // Compute the relaxable break mask for feasibility checks.
    relaxableMask_ = vehData.relaxableBreakMask();

    // Precompute setup per activity (forward order), applying the VROOM
    // canonical rule: a client pays setup iff its location differs from the
    // location of the immediately preceding node. The first client is
    // compared against the start depot. Breaks do not reset the block.
    std::vector<Duration> setupPerActivity(activities.size(), 0);
    {
        size_t prevLoc = start.location;
        for (size_t i = 0; i != activities.size(); ++i)
        {
            auto const &act = activities[i];
            if (act.isClient())
            {
                auto const loc = data.client(act.idx()).location;
                if (loc != prevLoc)
                    setupPerActivity[i] = data.setupDuration(loc);
                prevLoc = loc;
            }
            else if (act.isDepot())
                prevLoc = data.depot(act.idx()).location;
            // CUSTOM_BREAK: prevLoc unchanged (does not reset the block)
        }
    }

    std::vector<Duration> releaseTimes;  // per trip, for the schedule

    auto const &end = data.depot(vehData.endDepot);
    auto ds = DurationSegment::merge({end, 0}, {vehData, vehData.twLate});
    size_t nextLoc = end.location;
    size_t fwdIdx = activities.size();
    for (auto it = activities.rbegin(); it != activities.rend(); ++it)
    {
        --fwdIdx;
        if (it->isDepot())
        {
            auto const &depot = data.depot(it->idx());

            auto const depotService = depot.serviceDuration;
            service_ += depotService;

            auto const edgeDur = durations(depot.location, nextLoc);
            travel_ += edgeDur;

            ds = DurationSegment::merge(edgeDur, {depot, depotService}, ds);
            releaseTimes.insert(releaseTimes.begin(), ds.releaseTime());

            ds = ds.finaliseFront();
            nextLoc = depot.location;
        }
        else if (it->isCustomBreak())
        {
            // CUSTOM_BREAK: serviced at the current location (no travel).
            // The break config is looked up by id (break ids need not match
            // vector positions). This mirrors the forward-pass handling in
            // the schedule loop below; without it, breaks were treated as
            // clients here (data.client(breakId)), causing an out-of-bounds
            // read for break ids >= number of clients and a non-deterministic
            // segfault in Solution::unload().
            auto const &brks = vehData.custom_breaks;
            auto brkIt = std::find_if(
                brks.begin(), brks.end(),
                [&](auto const &brk)
                { return brk.id == static_cast<size_t>(it->idx()); });

            Duration service = 0;
            if (!breakServices.empty() && breakServices.size() == activities.size())
                service = breakServices[fwdIdx];
            else if (brkIt != brks.end())
                service = brkIt->service;

            service_ += service;

            ds = DurationSegment::merge({service, Duration(0)}, ds);
            // nextLoc unchanged — the vehicle does not move during a break
        }
        else
        {
            auto const &clientData = data.client(it->idx());
            auto const setup = setupPerActivity[fwdIdx];
            service_ += clientData.serviceDuration;
            setup_ += setup;

            auto const edgeDur = durations(clientData.location, nextLoc);
            travel_ += edgeDur;

            auto clientDS = DurationSegment(clientData);
            if (setup != 0)
                clientDS = clientDS.withService(setup);
            ds = DurationSegment::merge(edgeDur, clientDS, ds);
            nextLoc = clientData.location;
        }
    }

    auto const edgeDur = durations(start.location, nextLoc);

    service_ += start.serviceDuration;
    travel_ += edgeDur;

    ds = DurationSegment::merge(edgeDur, {start, start.serviceDuration}, ds);
    ds = DurationSegment::merge({vehData, vehData.startLate}, ds);

    releaseTimes.insert(releaseTimes.begin(), ds.releaseTime());

    duration_ = ds.duration();
    overtime_ = std::max<Duration>(duration_ - vehData.shiftDuration, 0);
    durationCost_ = vehData.unitDurationCost * static_cast<Cost>(duration_)
                    + vehData.unitOvertimeCost * static_cast<Cost>(overtime_);
    startTime_ = ds.startEarly();
    releaseTime_ = ds.releaseTime();
    slack_ = ds.slack();
    timeWarp_ = ds.timeWarp(vehData.maxDuration);

    auto now = startTime_;
    auto const handle = [&](Activity activity,
                            size_t trip,
                            Duration early,
                            Duration late,
                            Duration service)
    {
        auto const wait = std::max<Duration>(early - now, 0);
        auto const tw = std::max<Duration>(now - late, 0);

        now += wait;
        now -= tw;

        schedule_.emplace_back(activity, trip, now, now + service, wait, tw);

        now += service;
    };

    handle({Activity::ActivityType::DEPOT, vehData.startDepot},
           0,
           std::max(start.twEarly, std::min(releaseTime_, start.twLate)),
           std::min(start.twLate, vehData.startLate),
           start.serviceDuration);

    size_t prevLoc = start.location;
    size_t actIdx = 0;
    for (size_t tripIdx = 0; auto const &activity : activities)
    {
        if (activity.isDepot())
        {
            auto const releaseTime = releaseTimes[++tripIdx];

            auto const &depot = data.depot(activity.idx());
            now += durations(prevLoc, depot.location);

            handle(activity,
                   tripIdx,
                   std::max(depot.twEarly, std::min(releaseTime, depot.twLate)),
                   depot.twLate,
                   depot.serviceDuration);

            prevLoc = depot.location;
        }
        else if (activity.isCustomBreak())
        {
            // CUSTOM_BREAK: serviced at the current location (no travel).
            // The break config is looked up by id (break ids need not match
            // vector positions). Time-window gating semantics live in the
            // search layer (DriveSegment); here we only schedule the break.
            auto const &brks = vehData.custom_breaks;
            auto brkIt = std::find_if(
                brks.begin(), brks.end(),
                [&](auto const &brk)
                { return brk.id == static_cast<size_t>(activity.idx()); });

            Duration early = 0;
            Duration late = std::numeric_limits<Duration>::max();
            Duration service = 0;
            if (!breakServices.empty()
                && breakServices.size() == activities.size())
                service = breakServices[actIdx];
            else if (brkIt != brks.end())
                service = brkIt->service;

            if (brkIt != brks.end() && !brkIt->tws.empty())
            {
                early = brkIt->tws.front().first;
                late = brkIt->tws.back().second;
            }

            if (brkIt != brks.end() && brkIt->twsRelative)
            {
                early += startTime_;
                late += startTime_;
            }

            handle(activity, tripIdx, early, late, service);
            // prevLoc unchanged — the vehicle does not move during a break
        }
        else
        {
            auto const &clientData = data.client(activity.idx());
            now += durations(prevLoc, clientData.location);

            Duration setup = 0;
            if (clientData.location != prevLoc)
                setup = data.setupDuration(clientData.location);

            handle(activity,
                   tripIdx,
                   clientData.twEarly,
                   clientData.twLate,
                   setup + clientData.serviceDuration);

            prevLoc = clientData.location;
        }
        ++actIdx;
    }

    now += durations(prevLoc, end.location);
    handle({Activity::ActivityType::DEPOT, vehData.endDepot},
           releaseTimes.size(),
           end.twEarly,
           end.twLate,
           0);
}

void Route::setDistance(ProblemData const &data)
{
    auto const &vehData = data.vehicleType(vehicleType_);
    auto const &distances = data.distanceMatrix(vehData.profile);

    size_t frmLoc = data.depot(startDepot()).location;
    for (size_t idx = 1; idx != schedule_.size(); ++idx)
    {
        auto const &activity = schedule_[idx];

        size_t toLoc = frmLoc;  // CUSTOM_BREAK: no travel
        if (activity.isDepot())
            toLoc = data.depot(activity.idx()).location;
        else if (activity.isClient())
            toLoc = data.client(activity.idx()).location;

        distance_ += distances(frmLoc, toLoc);
        frmLoc = toLoc;
    }

    distanceCost_ = vehData.unitDistanceCost * static_cast<Cost>(distance_);
    excessDistance_ = std::max<Distance>(distance_ - vehData.maxDistance, 0);
}

void Route::setLoad(ProblemData const &data)
{
    auto const &vehData = data.vehicleType(vehicleType_);

    for (size_t dim = 0; dim != data.numLoadDimensions(); ++dim)
    {
        LoadSegment ls;

        if (vehData.initialLoad[dim] > 0)  // start with initial vehicle load
            ls = {vehData, dim};

        for (size_t idx = 1; idx != schedule_.size(); ++idx)
        {
            auto const &activity = schedule_[idx];

            if (activity.isDepot())
            {
                delivery_[dim] += ls.delivery();
                pickup_[dim] += ls.pickup();
                ls = ls.finalise(vehData.capacity[dim]);
            }

            if (activity.isClient())
                ls = LoadSegment::merge(ls, {data.client(activity.idx()), dim});
        }

        excessLoad_[dim] = ls.excessLoad(vehData.capacity[dim]);
    }
}

void Route::setOtherStatistics(ProblemData const &data)
{
    auto const &vehData = data.vehicleType(vehicleType_);
    fixedVehicleCost_ = vehData.fixedCost;

    for (auto const &activity : schedule_)
        if (activity.isClient())
            prizes_ += data.client(activity.idx()).prize;
}

Route::Route(ProblemData const &data,
             std::vector<Client> const &visits,
             VehicleType vehicleType)
{
    std::vector<Activity> activities;
    activities.reserve(visits.size());

    for (auto const client : visits)
        activities.emplace_back(Activity::ActivityType::CLIENT, client);

    *this = Route(data, activities, vehicleType);
}

Route::Route(ProblemData const &data,
             Activities const &activities,
             size_t vehType,
             std::vector<Duration> breakServices)
    : delivery_(data.numLoadDimensions(), 0),
      pickup_(data.numLoadDimensions(), 0),
      excessLoad_(data.numLoadDimensions(), 0),
      breakServices_(std::move(breakServices)),
      vehicleType_(vehType)
{
    validate(data, activities);
    setSchedule(data, activities, breakServices_);  // duration statistics and
                                                    // route schedule
    setDistance(data);                              // distance statistics
    setLoad(data);                                  // load statistics
    setOtherStatistics(data);                       // e.g. prizes, fixed cost

    // D6: price mandatory-break lateness for Python-constructed routes
    // (warm starts, tests) with the SAME shared forward pass the search
    // uses. A breakless warm start must not carry breakDue=0 — otherwise the
    // ILS "never worse than the manual order" guarantee is priced against an
    // underpriced baseline and the solver would keep the breakless order
    // even though skipping the mandatory rests is expensive (seconds × rate).
    auto const &vehData = data.vehicleType(vehicleType_);
    if (vehData.hasBreaks())
    {
        std::vector<Activity> acts;
        std::vector<size_t> locations;
        acts.reserve(schedule_.size());
        locations.reserve(schedule_.size());
        size_t prevLoc = 0;
        for (auto const &sa : schedule_)
        {
            acts.push_back(sa);  // ScheduledActivity extends Activity
            size_t loc = prevLoc;  // CUSTOM_BREAK: no travel
            if (sa.isClient())
                loc = data.client(sa.idx()).location;
            else if (sa.isDepot())
                loc = data.depot(sa.idx()).location;
            locations.push_back(loc);
            prevLoc = loc;
        }
        std::vector<Duration> atSecond(schedule_.size());
        auto const result = pyvrp::search::evaluateForwardPass(
            acts, locations, atSecond, nullptr, nullptr, data, vehData);
        breakDue_ = result.breakDue;
    }
}

Route::Route(Schedule schedule,
             Distance distance,
             Cost distanceCost,
             Distance excessDistance,
             std::vector<Load> delivery,
             std::vector<Load> pickup,
             std::vector<Load> excessLoad,
             Duration duration,
             Duration overtime,
             Cost durationCost,
             Duration timeWarp,
             Duration travel,
             Duration service,
             Duration setup,
             Duration startTime,
             Duration releaseTime,
             Duration slack,
             Cost prizes,
             size_t vehicleType,
             int64_t breakDue,
             std::vector<size_t> breaksServed,
             uint16_t breakDueMask,
             uint16_t relaxableMask)
    : schedule_(std::move(schedule)),
      distance_(distance),
      distanceCost_(distanceCost),
      excessDistance_(excessDistance),
      delivery_(std::move(delivery)),
      pickup_(std::move(pickup)),
      excessLoad_(std::move(excessLoad)),
      duration_(duration),
      overtime_(overtime),
      durationCost_(durationCost),
      timeWarp_(timeWarp),
      travel_(travel),
      service_(service),
      setup_(setup),
      startTime_(startTime),
      releaseTime_(releaseTime),
      slack_(slack),
      prizes_(prizes),
      breakDue_(breakDue),
      breakDueMask_(breakDueMask),
      relaxableMask_(relaxableMask),
      breaksServed_(std::move(breaksServed)),
      vehicleType_(vehicleType)
{
}

bool Route::empty() const { return numClients() == 0; }

size_t Route::size() const { return schedule_.size(); }

size_t Route::numClients() const
{
    size_t count = 0;
    for (auto const &sa : schedule_)
        if (sa.isClient())
            ++count;
    return count;
}

size_t Route::numDepots() const { return numTrips() + 1; }

size_t Route::numTrips() const { return schedule_.back().trip(); }

Route::ScheduledActivity const &Route::operator[](size_t idx) const
{
    if (idx >= size())
        throw std::out_of_range("Index out of range.");

    return schedule_[idx];
}

Route::Schedule::const_iterator Route::begin() const
{
    return schedule_.begin();
}

Route::Schedule::const_iterator Route::end() const { return schedule_.end(); }

Route::Schedule const &Route::schedule() const { return schedule_; }

Cost Route::fixedVehicleCost() const { return fixedVehicleCost_; }

Distance Route::distance() const { return distance_; }

Cost Route::distanceCost() const { return distanceCost_; }

Distance Route::excessDistance() const { return excessDistance_; }

std::vector<Load> const &Route::delivery() const { return delivery_; }

std::vector<Load> const &Route::pickup() const { return pickup_; }

std::vector<Load> const &Route::excessLoad() const { return excessLoad_; }

Duration Route::duration() const { return duration_; }

Duration Route::overtime() const { return overtime_; }

Cost Route::durationCost() const { return durationCost_; }

Duration Route::serviceDuration() const { return service_; }

Duration Route::setupDuration() const { return setup_; }

Duration Route::timeWarp() const { return timeWarp_; }

Duration Route::waitDuration() const
{
    return duration_ - travel_ - service_ - setup_;
}

Duration Route::travelDuration() const { return travel_; }

Duration Route::startTime() const { return startTime_; }

Duration Route::endTime() const { return startTime_ + duration_ - timeWarp_; }

Duration Route::slack() const { return slack_; }

Duration Route::releaseTime() const { return releaseTime_; }

Cost Route::prizes() const { return prizes_; }

int64_t Route::breakDue() const { return breakDue_; }

uint16_t Route::breakDueMask() const { return breakDueMask_; }

bool Route::hasHardBreakDue() const
{
    return (breakDueMask_ & ~relaxableMask_) != 0;
}

std::vector<Duration> const &Route::breakServices() const
{
    return breakServices_;
}

std::vector<size_t> const &Route::breaksServed() const
{
    return breaksServed_;
}

size_t Route::vehicleType() const { return vehicleType_; }

size_t Route::startDepot() const
{
    auto const &activity = schedule_.front();

    assert(activity.isDepot());
    return activity.idx();
}

size_t Route::endDepot() const
{
    auto const &activity = schedule_.back();

    assert(activity.isDepot());
    return activity.idx();
}

bool Route::isFeasible() const
{
    return !hasExcessLoad() && !hasTimeWarp() && !hasExcessDistance()
           && !hasHardBreakDue();
}

bool Route::hasExcessLoad() const
{
    return std::any_of(excessLoad_.begin(),
                       excessLoad_.end(),
                       [](auto const excess) { return excess > 0; });
}

bool Route::hasExcessDistance() const { return excessDistance_ > 0; }

bool Route::hasTimeWarp() const { return timeWarp_ > 0; }

bool Route::operator==(Route const &other) const
{
    // First compare simple attributes, since that's a quick and cheap check.
    // Only when these are the same we test if the activities are all equal.
    // clang-format off
    return distance_ == other.distance_
        && duration_ == other.duration_
        && timeWarp_ == other.timeWarp_
        && vehicleType_ == other.vehicleType_
        && schedule_ == other.schedule_;
    // clang-format on
}

template <> Cost pyvrp::CostEvaluator::penalisedCost(Route const &route) const
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
         + waitPenalty(route.waitDuration());
    // clang-format on
}

std::ostream &operator<<(std::ostream &out, Route const &route)
{
    for (size_t idx = 1; idx != route.size() - 1; ++idx)
    {
        auto const &activity = route[idx];
        if (activity.isDepot())
            out << '|';
        else
            out << activity;

        if (idx < route.size() - 2)  // then we'll insert more after this
            out << ' ';
    }

    return out;
}
