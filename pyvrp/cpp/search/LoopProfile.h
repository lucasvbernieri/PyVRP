#ifndef PYVRP_SEARCH_LOOPPROFILE_H
#define PYVRP_SEARCH_LOOPPROFILE_H

// Lightweight, always-compiled instrumentation for the break-vs-nobreak loop.
// All state is inline C++17 so a single definition is shared across the
// translation units of the _search module. Exposed to Python via
// search/bindings.cpp (loop_profile_*). Overhead is two steady_clock reads
// per guarded block; guarded blocks are coarse (function-level) so the
// relative overhead is negligible.

#include <chrono>
#include <cstdint>

namespace pyvrp::search::loopprofile
{
struct Profile
{
    // --- call counts ---
    std::uint64_t searchSteps = 0;        // search() step-loop iterations
    std::uint64_t clientLoopBodies = 0;   // client-loop U iterations
    std::uint64_t breakScanEvals = 0;     // applyUnaryOps called on break nodes
    std::uint64_t updates = 0;            // Route::update() calls
    std::uint64_t fwdPassCalls = 0;       // evaluateForwardPass() calls
    std::uint64_t shiftBreakEvals = 0;
    std::uint64_t shiftBreakApps = 0;
    std::uint64_t proposalDurBreak = 0;   // Proposal::duration() hasBreaks path
    std::uint64_t proposalDurSetup = 0;   // Proposal::duration() hasSetup-only
    std::uint64_t proposalDurSeg = 0;     // Proposal::duration() segment fold

    // --- wall-time accumulators (ns) ---
    std::uint64_t nsClientLoop = 0;
    std::uint64_t nsBreakScan = 0;
    std::uint64_t nsUpdate = 0;
    std::uint64_t nsFwdPass = 0;
    std::uint64_t nsShiftBreakEval = 0;
    std::uint64_t nsProposalDur = 0;  // whole duration() body
    std::uint64_t nsSearch = 0;       // whole search() step loop incl. scans
};

inline Profile profile;

inline void reset() { profile = Profile{}; }

struct Timer
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::uint64_t &acc;

    explicit Timer(std::uint64_t &acc) : acc(acc) {}

    ~Timer()
    {
        auto const ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - start)
                            .count();
        acc += static_cast<std::uint64_t>(ns > 0 ? ns : 0);
    }
};
}  // namespace pyvrp::search::loopprofile

#endif  // PYVRP_SEARCH_LOOPPROFILE_H
