import sys

root = sys.argv[1]


def edit(rel, pairs):
    p = root + "/" + rel
    s = open(p, encoding="utf-8", newline="").read()
    nl = "\r\n" if "\r\n" in s else "\n"
    for old, new, count in pairs:
        old = old.replace("\n", nl)
        new = new.replace("\n", nl)
        assert s.count(old) == count, (rel, old[:70], s.count(old))
        s = s.replace(old, new)
    open(p, "w", encoding="utf-8", newline="").write(s)
    print("ok", rel)


# ---- search/Route.cpp -------------------------------------------------------
edit("pyvrp/cpp/search/Route.cpp", [
    ("""        duration_ = result.duration;
        timeWarp_ = result.timeWarp;
        waiting_ = result.waiting;
        breakDueMask_ = result.breakDueMask;
        breakDue_ = result.breakDue;
    }
    else
    {
        fwdDrive_.reset();
        duration_ = durBefore.back().duration();
        timeWarp_ = durBefore.back().timeWarp(maxDuration());
        waiting_ = durBefore.back().waiting();
""", """        duration_ = result.duration;
        timeWarp_ = result.timeWarp;
        waiting_ = result.waiting;
        breakService_ = result.breakService;
        breakDueMask_ = result.breakDueMask;
        breakDue_ = result.breakDue;
    }
    else
    {
        fwdDrive_.reset();
        duration_ = durBefore.back().duration();
        timeWarp_ = durBefore.back().timeWarp(maxDuration());
        waiting_ = durBefore.back().waiting();
        breakService_ = 0;
""", 1),
    ("""    durationCost_ = unitDurationCost() * static_cast<Cost>(duration_ - waiting_)
                    + unitOvertimeCost() * static_cast<Cost>(overtime);
""", """    // break-free-duration: the served breaks' service is excluded the same
    // way (Proposal::duration() applies the identical formula to the same
    // evaluator totals, so deltas and update() agree by construction).
    auto active = duration_ - waiting_;
    if (breakFreeDuration)
        active -= breakService_;
    durationCost_ = unitDurationCost() * static_cast<Cost>(active)
                    + unitOvertimeCost() * static_cast<Cost>(overtime);
""", 1),
    ("""                    auto const breakId = node->idx();
                    if (inertBreaksFor(vehicleType_))
                        break;  // lane 16: a non-served break carries none
""", """                    auto const breakId = node->idx();
                    if (breakFreeDuration || inertBreaksFor(vehicleType_))
                        break;  // lane 16: a non-served break carries none;
                                // break-free: a served one is not priced
""", 1),
])

# ---- Route.cpp (solution-level) ---------------------------------------------
edit("pyvrp/cpp/Route.cpp", [
    ("""        std::vector<Duration> atSecond(schedule_.size());
        auto const result = pyvrp::search::evaluateForwardPass(
            acts, locations, atSecond, nullptr, nullptr, data, vehData);
        breakDue_ = result.breakDue;
    }
}
""", """        std::vector<Duration> atSecond(schedule_.size());
        uint16_t servedMask = 0;
        auto const result = pyvrp::search::evaluateForwardPass(
            acts, locations, atSecond, nullptr, nullptr, data, vehData,
            nullptr, nullptr, nullptr, nullptr, &servedMask);
        breakDue_ = result.breakDue;

        if (pyvrp::search::breakFreeDuration)
        {
            // break-free-duration: the served breaks' service leaves the
            // duration cost, as in the search (Route::update). With the
            // search's per-activity services (Solution::unload) a non-zero
            // entry IS a served break and is exactly what setSchedule()
            // folded; a Python-constructed route (no services) uses the
            // pass's served mask with the configured minimum service, which
            // is what setSchedule() folded for it.
            Duration served = 0;
            bool const haveSvc = !breakServices_.empty()
                                 && breakServices_.size() == activities.size();
            for (size_t i = 0; i != activities.size(); ++i)
            {
                auto const &act = activities[i];
                if (!act.isCustomBreak())
                    continue;
                if (haveSvc)
                    served += breakServices_[i];
                else if ((servedMask >> (act.idx() & 0xF)) & 1u)
                    for (auto const &brk : vehData.custom_breaks)
                        if (brk.id == static_cast<size_t>(act.idx()))
                        {
                            served += brk.service;
                            break;
                        }
            }
            durationCost_
                -= vehData.unitDurationCost * static_cast<Cost>(served);
        }
    }
}
""", 1),
])

# ---- search bindings --------------------------------------------------------
edit("pyvrp/cpp/search/bindings.cpp", [
    ("""    m.attr("PROXIMITY_WAIT_COST") = pyvrp::search::proximityWaitCost;
""", """    m.attr("PROXIMITY_WAIT_COST") = pyvrp::search::proximityWaitCost;
    m.attr("BREAK_FREE_DURATION") = pyvrp::search::breakFreeDuration;
""", 1),
    ("""        .def("waiting", &Route::waiting)
""", """        .def("waiting", &Route::waiting)
        .def("break_service", &Route::breakService)
""", 1),
])

edit("pyvrp/search/_search.pyi", [
    ("""    def waiting(self) -> int: ...
""", """    def waiting(self) -> int: ...
    def break_service(self) -> int: ...
""", 1),
])
