from pyvrp._pyvrp import (
    Activity,
    CostEvaluator,
    ProblemData,
    RandomNumberGenerator,
    Solution,
)
from pyvrp.search._search import (
    BinaryOperator,
    LocalSearchStatistics,
    PerturbationManager,
    UnaryOperator,
)
from pyvrp.search._search import LocalSearch as _LocalSearch


class LocalSearch:
    """
    Local search method. This search method explores a granular neighbourhood
    in a very efficient manner using user-provided operators. This quickly
    results in much improved solutions.

    Parameters
    ----------
    data
        Data object describing the problem to be solved.
    rng
        Random number generator.
    neighbours
        Mapping from each client or pickup activity to the activities in its
        granular neighbourhood.
    perturbation_manager
        Perturbation manager that handles perturbation during each invocation.
        Uses a default perturbation manager if not provided.
    """

    def __init__(
        self,
        data: ProblemData,
        rng: RandomNumberGenerator,
        neighbours: dict[Activity, list[Activity]],
        perturbation_manager: PerturbationManager | None = None,
    ):
        if perturbation_manager is None:
            perturbation_manager = PerturbationManager(data)

        self._ls = _LocalSearch(data, neighbours, perturbation_manager)
        self._rng = rng

    def add_operator(self, op: UnaryOperator | BinaryOperator):
        """
        Adds an operator to this local search object. The operator will be used
        to improve a solution.

        Parameters
        ----------
        op
            The operator to add to this local search object.
        """
        self._ls.add_operator(op)

    def set_max_updates(self, max_updates: int):
        """
        Sets the maximum number of solution updates a single search() call may
        apply before terminating gracefully (safety valve).

        Parameters
        ----------
        max_updates
            Maximum number of solution updates per local-search invocation.
        """
        self._ls.set_max_updates(max_updates)

    def set_time_budget(self, seconds: float):
        """
        Sets a wall-clock deadline (relative budget, in seconds) for the next
        local-search invocation(s). A non-positive value clears the deadline.

        Parameters
        ----------
        seconds
            Relative wall-clock budget in seconds.
        """
        self._ls.set_time_budget(seconds)

    def valve_triggered(self) -> bool:
        """
        True when the most recent local-search invocation was terminated by the
        safety valve (move cap or deadline) instead of converging.
        """
        return self._ls.valve_triggered()

    def parity_violations(self) -> int:
        """
        Number of accepted moves whose post-apply cost differed from the
        evaluated delta (parity divergence) since the last invocation. Zero in
        a healthy search; used by the parity diagnostics (D5).
        """
        return self._ls.parity_violations()

    @property
    def neighbours(self) -> dict[Activity, list[Activity]]:
        """
        Returns the granular neighbourhood currently used by the local search.
        """
        return self._ls.neighbours

    @neighbours.setter
    def neighbours(self, neighbours: dict[Activity, list[Activity]]):
        """
        Convenience method to replace the current granular neighbourhood used
        by the local search object.
        """
        self._ls.neighbours = neighbours

    @property
    def unary_operators(self) -> list[UnaryOperator]:
        """
        Returns the unary operators in use.
        """
        return self._ls.unary_operators

    @property
    def binary_operators(self) -> list[BinaryOperator]:
        """
        Returns the binary operators in use.
        """
        return self._ls.binary_operators

    @property
    def statistics(self) -> LocalSearchStatistics:
        """
        Returns search statistics about the most recently improved solution.
        """
        return self._ls.statistics

    def __call__(
        self,
        solution: Solution,
        cost_evaluator: CostEvaluator,
        exhaustive: bool = False,
    ) -> Solution:
        """
        This method improves the given solution through a (default
        non-exhaustive) local search.

        Parameters
        ----------
        solution
            The solution to improve through local search.
        cost_evaluator
            Cost evaluator to use.
        exhaustive
            Performs an exhaustive, complete search if set. Otherwise does
            only a limited search over perturbed clients (default).

        Returns
        -------
        Solution
            The improved solution. This is not the same object as the
            solution that was passed in.
        """
        self._ls.shuffle(self._rng)
        return self._ls(solution, cost_evaluator, exhaustive)
