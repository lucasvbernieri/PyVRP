import math
from dataclasses import dataclass

from pyvrp.Statistics import Statistics
from pyvrp._pyvrp import CostEvaluator, Solution


@dataclass
class Result:
    """
    Stores the outcomes of a single run. An instance of this class is returned
    once the IteratedLocalSearch completes.

    Parameters
    ----------
    best
        The best observed solution.
    stats
        A Statistics object containing runtime statistics.
    num_iterations
        Number of iterations performed by the iterated local search algorithm.
    runtime
        Total runtime of the main iterated local search loop.
    wait_cost_rate
        Total per-second cost charged for idle waiting in the solve that
        produced this result. Used by :meth:`~Result.cost` to report the
        wait-inclusive objective. Default 0.

    Raises
    ------
    ValueError
        When the number of iterations or runtime are negative.
    """

    best: Solution
    stats: Statistics
    num_iterations: int
    runtime: float
    wait_cost_rate: float = 0.0

    def __post_init__(self):
        if self.num_iterations < 0:
            raise ValueError("Negative number of iterations not understood.")

        if self.runtime < 0:
            raise ValueError("Negative runtime not understood.")

    def cost(self) -> float:
        """
        Returns the cost (objective) value of the best solution. Returns inf
        if the best solution is infeasible.

        The wait-cost term is charged using the ``wait_cost_rate`` the solve
        was calibrated with, so the reported objective matches the business
        cost of the solve. As is the convention for ``Result.cost()``, all
        solve-time penalties (load, duration, distance and breakDue) are zero,
        so a feasible-but-late-break solution still reports a cost that
        excludes the breakDue penalty charged during the search.
        """
        if not self.best.is_feasible():
            return math.inf

        num_load_dims = len(self.best.excess_load())
        return CostEvaluator(
            [0] * num_load_dims, 0, 0, 0, self.wait_cost_rate
        ).cost(self.best)

    def is_feasible(self) -> bool:
        """
        Returns whether the best solution is feasible.
        """
        return self.best.is_feasible()

    def summary(self) -> str:
        """
        Returns a nicely formatted result summary.
        """
        obj_str = f"{self.cost()}" if self.is_feasible() else "INFEASIBLE"
        summary = [
            "Solution results",
            "================",
            f"    # routes: {self.best.num_routes()}",
            f"     # trips: {self.best.num_trips()}",
            f"   # clients: {self.best.num_clients()}",
            f" # shipments: {self.best.num_shipments()}",
            f"   objective: {obj_str}",
            f"    distance: {self.best.distance()}",
            f"    duration: {self.best.duration()}",
            f"# iterations: {self.num_iterations}",
            f"    run-time: {self.runtime:.2f} seconds",
        ]

        return "\n".join(summary)

    def __str__(self) -> str:
        content = [
            self.summary(),
            "",
            "Routes",
            "------",
            str(self.best),
        ]

        return "\n".join(content)
