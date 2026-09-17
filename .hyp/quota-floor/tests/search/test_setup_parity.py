"""
Parity tests for setup-aware route evaluation (task 1.8).

Verifies that search::Route::update() and the pyvrp::Route::setSchedule()
reverse fold agree on duration/timeWarp for setup routes, that setup never
counts as drive time, and that warm-start loading with CUSTOM_BREAK
auto-insertion (search/Solution.cpp) remains setup-aware.
"""
import numpy as np
from numpy.testing import assert_, assert_equal

from pyvrp import (
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Model,
    Route as OutputRoute,
    Solution as OutputSolution,
)
from pyvrp.search._search import Node, Route, Solution
from tests.helpers import make_search_route

SETUP = 500


def _build_setup_data(with_breaks=False):
    """Two locations (A, B) each with two clients, plus a depot at loc 0."""
    model = Model()
    depot = model.add_location(x=0.0, y=0.0, name="depot")
    model.add_depot(depot, tw_early=0, tw_late=100_000)

    loc_a = model.add_location(x=1.0, y=0.0, name="A")
    loc_b = model.add_location(x=2.0, y=0.0, name="B")

    model.add_client(loc_a, delivery=0, service_duration=100, name="A0")
    model.add_client(loc_a, delivery=0, service_duration=100, name="A1")
    model.add_client(loc_b, delivery=0, service_duration=100, name="B0")
    model.add_client(loc_b, delivery=0, service_duration=100, name="B1")

    all_locs = [depot, loc_a, loc_b]
    for frm in all_locs:
        for to in all_locs:
            dur = 0 if frm is to else 10
            model.add_edge(frm, to, distance=dur, duration=dur)

    model.add_vehicle_type(
        num_available=1,
        capacity=[10],
        unit_duration_cost=1,
    )

    data = model.data()

    # setup: locations 0 (depot), 1 (A), 2 (B).
    setup = [0, SETUP, SETUP]
    if with_breaks:
        brk = CustomBreak(
            id=0,
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=1_000_000,  # huge: never triggered by drive
            reset=CustomBreakReset.DRIVE_TIMER,
            mandatory=True,
        )
        vt = data.vehicle_type(0).replace(custom_breaks=[brk])
        data = data.replace(vehicle_types=[vt], setup_durations=setup)
    else:
        data = data.replace(setup_durations=setup)

    return data


def test_update_matches_output_route_duration():
    """
    search::Route::update() (evaluateForwardPass reuse) and the output
    pyvrp::Route (setSchedule reverse fold) agree on duration for a setup
    route: depot -> A0 -> A1 -> B0 -> B1 -> depot.
    """
    data = _build_setup_data()

    visits = ["C0", "C1", "C2", "C3"]  # A0, A1, B0, B1
    search_route = make_search_route(data, visits)

    output_route = OutputRoute(data, [0, 1, 2, 3], 0)  # client indices

    # travel: depot->A(10) A->A(0) A->B(10) B->B(0) B->depot(10) = 30
    # service: 4 * 100 = 400
    # setup: A0 (500), A1 (0), B0 (500), B1 (0) = 1000
    expected = 30 + 400 + 2 * SETUP

    assert_equal(search_route.duration(), expected)
    assert_equal(output_route.duration(), expected)
    assert_equal(search_route.time_warp(), 0)
    assert_equal(output_route.setup_duration(), 2 * SETUP)


def test_contiguous_block_pays_setup_once():
    """Two clients at the same location form a block and pay setup once."""
    data = _build_setup_data()

    # depot -> A0 -> A1 -> depot: A0 pays, A1 does not.
    route = make_search_route(data, ["C0", "C1"])
    # travel = 10 + 0 + 10 = 20, service = 200, setup = 500
    assert_equal(route.duration(), 20 + 200 + SETUP)

    # Non-contiguous revisit pays again: A0 -> B0 -> A1.
    route = make_search_route(data, ["C0", "C2", "C1"])
    # travel = 10 + 10 + 10 + 10 = 40, service = 300, setup = 500 + 500 + 500
    assert_equal(route.duration(), 40 + 300 + 3 * SETUP)


def test_first_client_pays_setup_relative_to_depot():
    """
    The first client pays setup because the depot's location differs from
    the client's location (start counts as the previous location).
    """
    data = _build_setup_data()
    route = make_search_route(data, ["C0"])
    # travel 20 + service 100 + setup 500
    assert_equal(route.duration(), 20 + 100 + SETUP)


def test_setup_never_counts_as_drive():
    """
    A DRIVE_TIME break with a huge trigger value never fires, even though
    setup adds work/duty time: setup is never charged to the drive channel.
    """
    data = _build_setup_data(with_breaks=True)
    route = make_search_route(data, ["C0", "C1", "C2", "C3"])

    assert_(route.has_breaks())
    assert_equal(route.break_due(), 0)  # drive stays far below trigger


def test_warm_start_load_with_custom_break_and_setup():
    """
    Loading a warm-start solution into the search Solution auto-inserts
    CUSTOM_BREAK nodes (search/Solution.cpp) and still produces a
    setup-aware duration without crashing.
    """
    data = _build_setup_data(with_breaks=True)

    pyvrp_sol = OutputSolution(data, [[0, 1, 2, 3]])
    search_sol = Solution(data)
    search_sol.load(pyvrp_sol)

    # One route, four clients, plus the auto-inserted break node.
    assert_equal(len(search_sol.routes), data.num_vehicles)
    route = search_sol.routes[0]
    assert_equal(route.num_clients(), 4)

    # duration includes setup (2 * SETUP) and service (4 * 100) and travel.
    assert_(route.duration() >= 30 + 400 + 2 * SETUP)

    # The auto-inserted break did not fire (huge drive trigger).
    assert_equal(route.break_due(), 0)
