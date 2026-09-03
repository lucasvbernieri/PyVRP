#ifndef PYVRP_SEARCH_PHASEPROFILE_H
#define PYVRP_SEARCH_PHASEPROFILE_H

// Cycle-counting phase profiler for the local search. Entirely compiled out
// unless PYVRP_STREAM_STATS is defined, so the shipped binary is unaffected.
//
// Phases nest (duration/distance run inside the operator phases), so the
// report prints raw shares and leaves the nesting to the reader.

#ifdef PYVRP_STREAM_STATS

#include <cstdint>
#include <cstdio>

#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

namespace pyvrp::search::detail
{
enum Phase
{
    PH_TOTAL = 0,
    PH_PERTURB,
    PH_PREINSERT,
    PH_SEARCH,
    PH_UNARY,
    PH_BINARY,
    PH_BREAKSCAN,
    PH_UPDATE,
    PH_DURATION,
    PH_DISTANCE,
    PH_INSERTREQ,
    PH_COUNT,
};

inline char const *phaseName(int p)
{
    static char const *names[PH_COUNT] = {
        "operator()",    "perturb",  "preInsert", "search",
        "unaryOps",      "binaryOps", "breakScan", "Route::update",
        "duration",      "distance",  "insertRequired"};
    return names[p];
}

struct PhaseProfile
{
    unsigned long long cycles[PH_COUNT] = {};
    unsigned long long calls[PH_COUNT] = {};

    ~PhaseProfile()
    {
        if (!cycles[PH_TOTAL])
            return;

        auto const total = double(cycles[PH_TOTAL]);
        std::fprintf(stderr, "[phase-profile] total=%.3f Gcycles\n",
                     total / 1e9);
        for (int p = 0; p != PH_COUNT; ++p)
            std::fprintf(stderr,
                         "  %-16s %7.2f%%  calls=%-12llu cyc/call=%.1f\n",
                         phaseName(p),
                         100.0 * double(cycles[p]) / total,
                         calls[p],
                         calls[p] ? double(cycles[p]) / double(calls[p]) : 0.0);
    }
};

inline PhaseProfile phaseProfile{};

struct PhaseScope
{
    int phase;
    unsigned long long start;

    explicit PhaseScope(int p) : phase(p), start(__rdtsc())
    {
        phaseProfile.calls[p]++;
    }

    ~PhaseScope() { phaseProfile.cycles[phase] += __rdtsc() - start; }
};
}  // namespace pyvrp::search::detail

#define PYVRP_PHASE(p)                                                        \
    ::pyvrp::search::detail::PhaseScope pyvrpPhaseScope_##p(                  \
        ::pyvrp::search::detail::p)
#else
#define PYVRP_PHASE(p) ((void)0)
#endif

#endif  // PYVRP_SEARCH_PHASEPROFILE_H
