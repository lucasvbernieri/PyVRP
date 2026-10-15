from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from pyvrp._pyvrp import ActivityType
from pyvrp.ProgressPrinter import ProgressPrinter
from pyvrp.Result import Result
from pyvrp.RingBuffer import RingBuffer
from pyvrp.Statistics import Statistics

if TYPE_CHECKING:
    from pyvrp.PenaltyManager import PenaltyManager
    from pyvrp._pyvrp import (
        CostEvaluator,
        ProblemData,
        Solution,
    )
    from pyvrp.search.SearchMethod import SearchMethod
    from pyvrp.stop.StoppingCriterion import StoppingCriterion


def _with_seeded_rests(data: ProblemData, solution: Solution) -> Solution:
    """
    Returns ``solution`` passed through the search's solution conversion
    (load and unload, no move) when one of its routes is of a vehicle type
    that uses the duty model with rest rules and carries no rest node: the
    conversion seeds the rest nodes such a route lacks exactly as the search
    does, so the initial solution is priced like every solution the search
    produces. Returns ``solution`` itself otherwise (nothing to seed).
    """
    veh_types = data.vehicle_types()
    if not any(
        veh_types[route.vehicle_type()].has_duty_model()
        and veh_types[route.vehicle_type()].has_breaks()
        and not any(
            act.type == ActivityType.CUSTOM_BREAK for act in route
        )
        for route in solution.routes()
    ):
        return solution

    from pyvrp.search._search import Solution as SearchSolution

    seeded = SearchSolution(data)
    seeded.load(solution)
    return seeded.unload()


class IteratedLocalSearchCallbacks:
    """
    Interface and no-op implementation of callbacks used inside the iterated
    local search algorithm. These callbacks allow user-space code to observe
    search progress.
    """

    def on_start(self, ils: IteratedLocalSearch):
        """
        Called at search start with the initialised ILS object.
        """
        pass

    def on_end(self, result: Result):
        """
        Called at search end with the final result.
        """
        pass

    def on_iteration(
        self,
        current: Solution,
        candidate: Solution,
        best: Solution,
        cost_evaluator: CostEvaluator,
    ):
        """
        Called at the end of each iteration, with the current, candidate, and
        best-found solutions so far. Includes the cost evaluator currently in
        use.
        """
        pass

    def on_restart(self, best: Solution):
        """
        Called when the search restarts, with the best solution found so far.
        """
        pass

    def on_best(self, best: Solution):
        """
        Called when the search finds a new best solution.
        """
        pass


@dataclass
class IteratedLocalSearchParams:
    """
    Parameters for the iterated local search algorithm.

    Parameters
    ----------
    num_iters_no_improvement
        Number of iterations without improvement before a restart occurs.
    history_length
        History length for the late acceptance hill-climbing criterion used by
        the algorithm. Must be positive.
    exhaustive_on_best
        Whether to perform a more expensive, exhaustive search for newly found
        best solutions.
    callbacks
        Optional callbacks to be called during the search.
    """

    num_iters_no_improvement: int = 150_000
    history_length: int = 300
    exhaustive_on_best: bool = True
    callbacks: IteratedLocalSearchCallbacks = field(  # type: ignore[assignment]
        default=None,
        compare=False,  # this doesn't influence parameter configurations
    )

    def __post_init__(self):
        if self.callbacks is None:
            self.callbacks = IteratedLocalSearchCallbacks()

        if self.num_iters_no_improvement < 0:
            raise ValueError("num_iters_no_improvement < 0 not understood.")

        if self.history_length <= 0:
            raise ValueError("history_length must be positive.")


class IteratedLocalSearch:
    """
    Creates an IteratedLocalSearch instance.

    Parameters
    ----------
    data
        The problem data instance.
    penalty_manager
        Penalty manager to use.
    search_method
        Search method to use.
    initial_solution
        Initial solution to start the search with.
    params
        Iterated local search parameters to use. If not provided, a default
        will be used.
    penalised_best
        Whether to drive the search by the best *penalised* solution as well
        (see :attr:`~pyvrp.Result.Result.best_penalised`): the stopping
        criterion receives its penalised cost, restarts begin from it, and an
        improvement of it resets the no-improvement counter and triggers the
        exhaustive search on it (when ``exhaustive_on_best`` is set). The
        best feasible solution is tracked exactly as without this rule. Also
        on, whatever this argument says, when a vehicle type of the instance
        uses the duty model (``duty_limit >= 0``). Default ``False``.

    Notes
    -----
    When a vehicle type of the instance uses the duty model with rest rules
    (generated rests), the search starts from the initial solution as the
    search sees it: passed through the search's own solution conversion,
    which seeds the missing rest nodes (parked after the last stop). A warm
    start built from clients only would otherwise be priced without its
    rests, paying just the duty excess, while every solution of the search
    carries them; it then looks cheaper than anything the search produces,
    and stays the current, best and best penalised solution forever. Without
    such vehicle types the initial solution is used as given.
    """

    def __init__(
        self,
        data: ProblemData,
        penalty_manager: PenaltyManager,
        search_method: SearchMethod,
        initial_solution: Solution,
        params: IteratedLocalSearchParams = IteratedLocalSearchParams(),
        penalised_best: bool = False,
    ):
        self._data = data
        self._pm = penalty_manager
        self._search = search_method
        self._init = initial_solution
        self._start = _with_seeded_rests(data, initial_solution)
        self._params = params
        self._penalised_best = penalised_best or any(
            veh_type.duty_limit >= 0 for veh_type in data.vehicle_types()
        )

    @property
    def penalty_manager(self) -> PenaltyManager:
        """
        Returns the penalty manager.
        """
        return self._pm

    @property
    def search(self) -> SearchMethod:
        """
        Returns the search method.
        """
        return self._search

    @property
    def initial_solution(self) -> Solution:
        """
        Returns the initial solution.
        """
        return self._init

    @property
    def params(self) -> IteratedLocalSearchParams:
        """
        Returns the algorithm's parameter configuration.
        """
        return self._params

    @property
    def penalised_best(self) -> bool:
        """
        Returns whether the search is driven by the best penalised solution
        as well (see the ``penalised_best`` argument).
        """
        return self._penalised_best

    def run(
        self,
        stop: StoppingCriterion,
        collect_stats: bool = True,
        display: bool = False,
        display_interval: float = 5.0,
    ) -> Result:
        """
        Runs the iterated local search algorithm with the provided stopping
        criterion. The algorithm uses late acceptance hill-climbing as the
        acceptance criterion; see [1]_ for details.

        Parameters
        ----------
        stop
            Stopping criterion to use. The algorithm runs until the first time
            the stopping criterion returns ``True``.
        collect_stats
            Whether to collect statistics about the solver's progress. Default
            ``True``.
        display
            Whether to display information about the solver progress. Default
            ``False``. Progress information is only available when
            ``collect_stats`` is also set.
        display_interval
            Time (in seconds) between iteration logs. Defaults to 5s.

        Returns
        -------
        Result
            A Result object, containing statistics (if collected) and the best
            found solution.

        References
        ----------
        .. [1] Burke, E.K., and Y. Bykov (2017). The Late Acceptance
               Hill-Climbing Heuristic. *European Journal of Operational
               Research*, 258(1): 70 - 78.
               https://doi.org/10.1016/j.ejor.2016.07.012.
        """
        callbacks = self._params.callbacks

        print_progress = ProgressPrinter(display, display_interval)
        print_progress.start(self._data)

        history: RingBuffer[Solution] = RingBuffer(self._params.history_length)
        stats = Statistics(collect_stats=collect_stats)

        start = time.perf_counter()
        iters = iters_no_improvement = 0
        best = curr = self._start
        best_penalised = self._start  # lowest penalised cost seen

        callbacks.on_start(self)

        cost_eval = self._pm.cost_evaluator()
        while not stop(
            cost_eval.penalised_cost(best_penalised)
            if self._penalised_best
            else cost_eval.cost(best)
        ):
            iters += 1

            if iters_no_improvement == self._params.num_iters_no_improvement:
                print_progress.restart()
                history.clear()

                curr = best_penalised if self._penalised_best else best
                iters_no_improvement = 0

                callbacks.on_restart(best)

            cost_eval = self._pm.cost_evaluator()
            cand = self._search(curr, cost_eval, exhaustive=False)
            self._pm.register(cand)

            iters_no_improvement += 1
            if self._penalised_best:
                # One rule for both bests: an improvement of either resets the
                # counter and gets the exhaustive search. Its result becomes
                # the best penalised solution if its penalised cost is lower,
                # and the best solution only if it is feasible and better.
                improved = False
                if cost_eval.cost(cand) < cost_eval.cost(best):
                    best = cand
                    improved = True

                cand_pen = cost_eval.penalised_cost(cand)
                if cand_pen < cost_eval.penalised_cost(best_penalised):
                    best_penalised = cand
                    improved = True

                if improved:
                    iters_no_improvement = 0

                    if self._params.exhaustive_on_best:
                        cand = self._search(cand, cost_eval, exhaustive=True)
                        if cand.is_feasible() and cost_eval.cost(
                            cand
                        ) < cost_eval.cost(best):
                            best = cand
                        if cost_eval.penalised_cost(
                            cand
                        ) < cost_eval.penalised_cost(best_penalised):
                            best_penalised = cand

                    callbacks.on_best(best)
            else:
                # Observed only: the best penalised solution is recorded for
                # the result without steering anything.
                if cost_eval.penalised_cost(cand) < cost_eval.penalised_cost(
                    best_penalised
                ):
                    best_penalised = cand

                if cost_eval.cost(cand) < cost_eval.cost(best):
                    best = cand
                    iters_no_improvement = 0

                    if self._params.exhaustive_on_best:
                        # Candidate is already a new (global) best, but let's
                        # see if we can improve it via an exhaustive search.
                        # That new candidate solution might be infeasible, so
                        # we need to check before updating best.
                        cand = self._search(cand, cost_eval, exhaustive=True)
                        if cand.is_feasible():
                            best = cand

                        if cost_eval.penalised_cost(
                            cand
                        ) < cost_eval.penalised_cost(best_penalised):
                            best_penalised = cand

                    callbacks.on_best(best)

            cand_cost = cost_eval.penalised_cost(cand)
            curr_cost = cost_eval.penalised_cost(curr)

            # We use either the initial cost or the current cost value from
            # some iterations ago to determine whether to accept the candidate
            # solution, if available.
            late_cost = cost_eval.penalised_cost(self._start)
            if (late := history.peek()) is not None:
                late_cost = cost_eval.penalised_cost(late)

            # Late-acceptance hill climbing of Burke and Bykov (2017). We use
            # both enhancements of section 4.2:
            # 1. We accept also when the candidate improves over the current
            #    solution;
            if cand_cost < late_cost or cand_cost < curr_cost:
                curr = cand
                curr_cost = cand_cost

            # 2. We update the history only when the current solution is better
            #    than the one already in the history.
            if curr_cost < late_cost or late is None:
                history.append(curr)
            else:
                history.skip()

            stats.collect(curr, cand, best, cost_eval)
            print_progress.iteration(stats)
            callbacks.on_iteration(curr, cand, best, cost_eval)

        # The best solution when no infeasible one is cheaper once penalised.
        if cost_eval.penalised_cost(best) <= cost_eval.penalised_cost(
            best_penalised
        ):
            best_penalised = best

        runtime = time.perf_counter() - start
        res = Result(
            best,
            stats,
            iters,
            runtime,
            wait_cost_rate=self._pm.wait_cost_rate,
            best_penalised=best_penalised,
        )

        print_progress.end(res)
        callbacks.on_end(res)

        return res
