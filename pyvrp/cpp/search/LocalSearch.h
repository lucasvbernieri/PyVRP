#ifndef PYVRP_SEARCH_LOCALSEARCH_H
#define PYVRP_SEARCH_LOCALSEARCH_H

#include "CostEvaluator.h"
#include "LocalSearchOperator.h"
#include "PerturbationManager.h"
#include "ProblemData.h"
#include "RandomNumberGenerator.h"
#include "Route.h"
#include "SearchSpace.h"
#include "Solution.h"  // pyvrp::search::Solution

#include <chrono>
#include <functional>
#include <stdexcept>
#include <vector>

namespace pyvrp::search
{
class LocalSearch
{
    ProblemData const &data;

    // Stores the node-based solution representation used during LS.
    Solution solution_;

    // Manages the granular neighbourhood, promising nodes, and the order in
    // which nodes and routes are searched.
    SearchSpace searchSpace_;

    // Perturbation manager that determines the size of the perturbation during
    // each LS invocation.
    PerturbationManager &perturbationManager_;

    std::vector<UnaryOperator *> unaryOps_;
    std::vector<BinaryOperator *> binaryOps_;

    std::vector<int> lastTest_;    // tracks last client and pickup evaluations
    std::vector<int> lastUpdate_;  // tracks when routes were last modified

    size_t numUpdates_ = 0;         // modification counter
    bool searchCompleted_ = false;  // No further improving move found?

    // Safety valve (D5): hard limits on a single search() invocation so the
    // local search can never oscillate forever, even when the stop criterion
    // is only checked between ILS iterations (which is the case for every
    // stop criterion — MaxRuntime is checked by the Python ILS loop, never
    // inside the C++ search). The move cap has a finite default so the
    // guarantee holds for ALL callers without any change on their side; the
    // wall-clock deadline is an additional defence for pathologically slow
    // moves and is set by callers that know their runtime budget.
    size_t maxUpdates_ = 1'000'000;
    std::chrono::steady_clock::time_point deadline_
        = std::chrono::steady_clock::time_point::max();
    bool valveTriggered_ = false;

    // Counts accepted moves whose post-apply cost differs from the evaluated
    // delta (parity divergence between the proposal evaluation and the
    // recomputed solution). Such divergences can make the step loop oscillate
    // (moves accepted on stale deltas keep restarting the search), which the
    // safety valve above bounds. Zero in a healthy search; used by the parity
    // diagnostics (D5).
    size_t parityViolations_ = 0;

    // Tests the node U.
    bool applyUnaryOps(Route::Node *U, CostEvaluator const &costEvaluator);

    // Tests the node pair (U, V).
    bool applyBinaryOps(Route::Node *U,
                        Route::Node *V,
                        CostEvaluator const &costEvaluator);

    // Tests moves involving empty routes.
    void applyEmptyRouteMoves(Route::Node *U,
                              CostEvaluator const &costEvaluator);

    // Inserts U if it is an unplanned required client or shipment, or if it
    // belongs to a required client group that is currently missing.
    void insertRequired(Route::Node *U, CostEvaluator const &costEvaluator);

    // Updates solution state after an improving local search move.
    void update(Route *U, Route *V);

    // Performs search on the currently loaded solution.
    void search(CostEvaluator const &costEvaluator);

public:
    /**
     * Simple data structure that tracks statistics about the number of local
     * search moves applied to the most recently improved solution.
     *
     * Attributes
     * ----------
     * num_moves
     *     Number of evaluated operator moves.
     * num_improving
     *     Number of evaluated moves that led to an objective improvement.
     * num_updates
     *     Total number of changes to the solution. This always includes the
     *     number of evaluated improving moves, but also e.g. insertion of
     *     required but missing clients and shipments.
     */
    struct Statistics
    {
        // Number of evaluated operator moves.
        size_t const numMoves;

        // Number of evaluated moves that led to an objective improvement.
        size_t const numImproving;

        // Number of times the solution has been modified in some way.
        size_t const numUpdates;
    };

    /**
     * Adds a local search operator that works on client and pickup nodes U.
     */
    void addOperator(UnaryOperator &op);

    /**
     * Adds a local search operator that works on node pairs U and V.
     */
    void addOperator(BinaryOperator &op);

    /**
     * Returns the unary operators in use. Note that there is no defined
     * ordering.
     */
    std::vector<UnaryOperator *> const &unaryOperators() const;

    /**
     * Returns the binary operators in use. Note that there is no defined
     * ordering.
     */
    std::vector<BinaryOperator *> const &binaryOperators() const;

    /**
     * Set neighbourhood structure to use by the local search. For each client
     * and pickup activity, the neighbourhood structure is a vector of nearby
     * clients, pickups, and deliveries.
     */
    void setNeighbours(SearchSpace::Neighbours neighbours);

    /**
     * Returns the current neighbourhood structure.
     */
    SearchSpace::Neighbours const &neighbours() const;

    /**
     * Returns search statistics for the currently loaded solution.
     */
    Statistics statistics() const;

    /**
     * Sets the maximum number of solution updates a single search()
     * invocation may apply before it terminates gracefully (safety valve).
     * The returned solution is the current best at that point.
     */
    void setMaxUpdates(size_t maxUpdates);

    /**
     * Sets a wall-clock deadline (relative budget, in seconds) for the next
     * search() invocation(s). A non-positive value clears the deadline.
     */
    void setTimeBudget(double seconds);

    /**
     * True when the most recent search() invocation was terminated by the
     * safety valve (move cap or deadline) instead of converging.
     */
    bool valveTriggered() const;

    /**
     * Number of accepted moves whose post-apply cost differed from the
     * evaluated delta (parity divergence) since the last operator() call.
     */
    size_t parityViolations() const;

    /**
     * Performs a local search around the given solution, and returns a new,
     * hopefully improved solution.
     */
    pyvrp::Solution operator()(pyvrp::Solution const &solution,
                               CostEvaluator const &costEvaluator,
                               bool exhaustive = false);

    /**
     * Shuffles the order in which the node and route pairs are evaluated, and
     * the order in which operators are applied.
     */
    void shuffle(RandomNumberGenerator &rng);

    LocalSearch(ProblemData const &data,
                SearchSpace::Neighbours neighbours,
                PerturbationManager &perturbationManager);
};
}  // namespace pyvrp::search

#endif  // PYVRP_SEARCH_LOCALSEARCH_H
