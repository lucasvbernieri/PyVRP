import sys

p = sys.argv[1]
s = open(p, encoding="utf-8", newline="").read()
nl = "\r\n" if "\r\n" in s else "\n"


def rep(old, new, count=1):
    global s
    old = old.replace("\n", nl)
    new = new.replace("\n", nl)
    assert s.count(old) == count, (old[:70], s.count(old))
    s = s.replace(old, new)


# 1. member
rep("""    Duration waiting_;
    uint16_t breakDueMask_ = 0;  // bitmask of violated (due) break ids
""", """    Duration waiting_;
    Duration breakService_;  // service folded for served breaks; see
                             // breakFreeDuration (DriveSegment.h)
    uint16_t breakDueMask_ = 0;  // bitmask of violated (due) break ids
""")

# 2. accessor decl
rep("""    [[nodiscard]] inline Duration waiting() const;

    /**
     * @return The effective (possibly extended) service duration of the
     *         CUSTOM_BREAK node at the given position, or 0 for non-break
     *         nodes.
     */
""", """    [[nodiscard]] inline Duration waiting() const;

    /**
     * @return Total service the duration fold charged for the breaks that
     *         are actually served on this route (D5 extension included).
     *         Excluded from the duration cost under ``breakFreeDuration``.
     */
    [[nodiscard]] inline Duration breakService() const;

    /**
     * @return The effective (possibly extended) service duration of the
     *         CUSTOM_BREAK node at the given position, or 0 for non-break
     *         nodes.
     */
""")

# 3. accessor def
rep("""Duration Route::waiting() const
{
    assert(!dirty);
    return waiting_;
}
""", """Duration Route::waiting() const
{
    assert(!dirty);
    return waiting_;
}

Duration Route::breakService() const
{
    assert(!dirty);
    return breakService_;
}
""")

# 4. Proposal::duration
rep("""        auto const dur = result.duration;
        auto const overtime = std::max<Duration>(dur - shiftDuration, 0);
        // wait-cost-root-fix: duration cost excludes waiting (cached
        // above); the CostEvaluator charges it separately at its wait
        // rate. Overtime stays on the full duration.
        auto const dCost = unitDurationCost
                               * static_cast<Cost>(dur - result.waiting)
                           + unitOvertimeCost * static_cast<Cost>(overtime);
        return std::make_pair(dCost, result.timeWarp);
""", """        auto const dur = result.duration;
        auto const overtime = std::max<Duration>(dur - shiftDuration, 0);
        // wait-cost-root-fix: duration cost excludes waiting (cached
        // above); the CostEvaluator charges it separately at its wait
        // rate. Overtime stays on the full duration.
        // break-free-duration: the served breaks' service is excluded the
        // same way (same formula as Route::update(), same evaluator totals).
        auto active = dur - result.waiting;
        if (breakFreeDuration)
            active -= result.breakService;
        auto const dCost = unitDurationCost * static_cast<Cost>(active)
                           + unitOvertimeCost * static_cast<Cost>(overtime);
        return std::make_pair(dCost, result.timeWarp);
""")

# 5. runComposed NSEGS==1
rep("""                out = {r->duration_, r->timeWarp_, r->breakDue_,
                       r->breakDueMask_, r->waiting_};
""", """                out = {r->duration_, r->timeWarp_, r->breakDue_,
                       r->breakDueMask_, r->waiting_, r->breakService_};
""")

# 6. runComposed seed break service
rep("""        int64_t const endEarly = r->endEarly_;
        int64_t endArr = -1;
        bool needEnd = false;
""", """        // break-free-duration: service folded for a served break inside the
        // seeded prefix (``breakServicesAt_`` is 0 for an unserved break); the
        // BREAK piece below adds its own when served.
        Duration breakSvc = 0;
        for (auto const pos : r->breakPositions_)
            if (pos <= k)
                breakSvc += r->breakServicesAt_[pos];

        int64_t const endEarly = r->endEarly_;
        int64_t endArr = -1;
        bool needEnd = false;
""")

# 7. BREAK piece served
rep("""                    served = true;
                    taken = true;
                    atBreak = arr;
                    L = arr + effSvc;
                    PYVRP_STAT(compServed, 1);
""", """                    served = true;
                    taken = true;
                    atBreak = arr;
                    L = arr + effSvc;
                    breakSvc += Duration(effSvc);
                    PYVRP_STAT(compServed, 1);
""")

# 8. composed out
rep("""        out = {acc.duration(), acc.timeWarp(vt.maxDuration), breakDue, dueMask,
               acc.waiting()};
""", """        out = {acc.duration(), acc.timeWarp(vt.maxDuration), breakDue, dueMask,
               acc.waiting(), breakSvc};
""")

# 9. stream accumulators
rep("""    uint16_t servedMask = 0;     // OR across rounds (decisions are frozen)
    uint16_t dueMask = 0;        // reset per round; final round wins
    Duration absorbedWaiting = 0;
    bool clearedWindows = false;
""", """    uint16_t servedMask = 0;     // OR across rounds (decisions are frozen)
    uint16_t dueMask = 0;        // reset per round; final round wins
    Duration absorbedWaiting = 0;
    bool clearedWindows = false;

    // break-free-duration: service the fold charges for the served breaks,
    // reset per round (final round wins). The seeded prefix contributes its
    // cached per-position services (``breakServicesAt_`` is 0 for an
    // unserved break); the walk adds each break singleton it folds as served.
    Duration breakSvc = 0;
    Duration seedBreakSvc = 0;
    if (seeded)
        for (auto const pos : r->breakPositions_)
            if (pos + 1 <= P)  // pos <= k = P - 1
                seedBreakSvc += r->breakServicesAt_[pos];
""")

# 10. ResumeState field + capture + restore
rep("""        bool jumpDone = false;
        size_t jumpShortDesc = 0;
        bool locTriedOnce = false;
    };
    ResumeState resume;
""", """        bool jumpDone = false;
        size_t jumpShortDesc = 0;
        bool locTriedOnce = false;
        Duration breakSvc = 0;
    };
    ResumeState resume;
""")
rep("""            rs.locTriedOnce = locTriedOnce;
            resumeValid = true;
""", """            rs.locTriedOnce = locTriedOnce;
            rs.breakSvc = breakSvc;
            resumeValid = true;
""")
rep("""            locTriedOnce = rs.locTriedOnce;
            startIdx = rs.idx;
""", """            locTriedOnce = rs.locTriedOnce;
            breakSvc = rs.breakSvc;
            startIdx = rs.idx;
""")

# 11. runRound start
rep("""        durBefore = durAtStart;
        Duration arrival0 = 0;   // atSecond at the start depot (node 0)
""", """        durBefore = durAtStart;
        breakSvc = seeded ? seedBreakSvc : Duration(0);
        Duration arrival0 = 0;   // atSecond at the start depot (node 0)
""")

# 12. served sites in the stream
rep("""                            second = DurationSegment(effSvc, Duration(0),
                                                     brkEarlyNow, brkLateNow);
                        else if (effSvc != minSvc)
                        {
                            absorbedWaiting += effSvc - minSvc;
                            if (breakId < K)
                                extraSvc[breakId] += (effSvc - minSvc).get();
                            if (decide && !resumeRecorded)
                                recordResume(idx);
                        }

""", """                            second = DurationSegment(effSvc, Duration(0),
                                                     brkEarlyNow, brkLateNow);
                        else if (effSvc != minSvc)
                        {
                            absorbedWaiting += effSvc - minSvc;
                            if (breakId < K)
                                extraSvc[breakId] += (effSvc - minSvc).get();
                            if (decide && !resumeRecorded)
                                recordResume(idx);
                        }

                        // break-free-duration: the service THIS round's fold
                        // charges for the served break -- ``second`` as
                        // merged below (on a re-run, the store's extension is
                        // already in ``brkSvcNow``). Added after
                        // recordResume(), so a round 2 restarting at the top
                        // of this node adds it again from the captured value.
                        breakSvc += inertBreaksFor(vt) ? effSvc : brkSvcNow;

""")

# 13. stream return
rep("""    return {duration, timeWarp, breakDueSeconds, dueMask, waiting};
}
""", """    return {duration, timeWarp, breakDueSeconds, dueMask, waiting, breakSvc};
}
""")

# 14. LB lambda
rep("""        if (inertBreaksFor(route()->vehicleType_))
            return 0;  // lane 16: a non-served break carries no service
""", """        if (breakFreeDuration || inertBreaksFor(route()->vehicleType_))
            return 0;  // lane 16: a non-served break carries no service;
                       // break-free: a served one is not priced either
""")

# 15. ceiling
rep("""                if (inertBreaksFor(route()->vehicleType_))
                    return;  // lane 16: may carry no service at all
""", """                if (breakFreeDuration
                    || inertBreaksFor(route()->vehicleType_))
                    return;  // lane 16 / break-free: may carry no service
""")

# 16. stats-only differential checks: compare breakService too
rep("""                auto const dWait = absll((long long)comp.waiting.get() - (long long)result.waiting.get());
""", """                auto const dWait = absll((long long)comp.waiting.get() - (long long)result.waiting.get());
                auto const dBrk = absll((long long)comp.breakService.get() - (long long)result.breakService.get());
                if (dBrk) { ++st.compDiffWait; bad = true; }
""")
rep("""            if (ref.waiting != result.waiting) { ++st.chkDiffWait; bad = true; }
""", """            if (ref.waiting != result.waiting) { ++st.chkDiffWait; bad = true; }
            if (ref.breakService != result.breakService) { ++st.chkDiffWait; bad = true; }
""")

open(p, "w", encoding="utf-8", newline="").write(s)
print("ok", p)
