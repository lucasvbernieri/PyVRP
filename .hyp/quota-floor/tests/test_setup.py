"""
Tests for the native ``setup`` primitive on ``ProblemData`` (task 1.10).

Covers the low-level ProblemData API (``setup_durations`` constructor
argument, ``has_setup()``, ``setup_duration(loc)``, validation) and the
exposed statistics of solved routes (duration and cost include setup).
"""
import pickle

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal, assert_raises

from pyvrp import Client, Depot, Location, ProblemData, Route, VehicleType
from pyvrp.stop import MaxIterations
from pyvrp import solve


def make_setup_data(setup_durations=None, service=50, unit_duration_cost=1):
    """Small instance: depot (loc 0) + one client (loc 1)."""
    locs = [Location(0, 0), Location(1, 1)]
    clients = [Client(location=1, delivery=[0], service_duration=service)]
    depots = [Depot(location=0)]
    vts = [VehicleType(capacity=[10], unit_duration_cost=unit_duration_cost)]

    mat = np.zeros((2, 2), dtype=int)
    mat[0, 1] = mat[1, 0] = 10

    return ProblemData(
        locs,
        clients,
        depots,
        vts,
        [mat],
        [mat],
        setup_durations=setup_durations or [],
    )


def test_setup_durations_constructor_and_accessors():
    """
    ``setup_durations`` is accepted by the constructor and exposed read-only
    via ``setup_durations()``, ``setup_duration(loc)`` and ``has_setup()``.
    """
    data = make_setup_data(setup_durations=[0, 100])

    assert_(data.has_setup())
    assert_equal(data.setup_durations(), [0, 100])
    assert_equal(data.setup_duration(0), 0)
    assert_equal(data.setup_duration(1), 100)

    # Without setup the instance must report has_setup() == False.
    plain = make_setup_data()
    assert_(not plain.has_setup())
    assert_equal(plain.setup_duration(1), 0)


def test_setup_duration_out_of_bounds():
    """``setup_duration`` raises IndexError for out-of-range locations."""
    data = make_setup_data(setup_durations=[0, 100])
    with assert_raises(IndexError):
        data.setup_duration(2)


def test_setup_validation_rejects_negative():
    """Negative setup durations are rejected with a ValueError."""
    with assert_raises(ValueError):
        make_setup_data(setup_durations=[0, -1])


def test_setup_validation_rejects_wrong_size():
    """Setup durations must match the number of locations (or be empty)."""
    with assert_raises(ValueError):
        make_setup_data(setup_durations=[100])  # only one entry, 2 locations


def test_setup_validation_rejects_non_integer():
    """Non-integer setup durations are rejected (integral Duration type)."""
    with assert_raises(TypeError):
        make_setup_data(setup_durations=[0, 1.5])


def test_setup_pickle_roundtrip():
    """ProblemData pickling preserves setup durations."""
    data = make_setup_data(setup_durations=[0, 100])
    restored = pickle.loads(pickle.dumps(data))
    assert_equal(restored, data)
    assert_(restored.has_setup())
    assert_equal(restored.setup_durations(), [0, 100])


def test_route_setup_duration():
    """
    A route visiting the single client pays setup once; the route duration
    includes travel + service + setup, and ``setup_duration()`` exposes it.
    """
    data = make_setup_data(setup_durations=[0, 100], service=50)
    route = Route(data, [0], 0)  # visit client 0

    assert_equal(route.setup_duration(), 100)
    assert_equal(route.service_duration(), 50)
    assert_equal(route.travel_duration(), 20)
    assert_equal(route.duration(), 20 + 50 + 100)
    assert_equal(route.wait_duration(), 0)


def test_contiguous_block_pays_setup_once():
    """
    Two clients at the same location form a contiguous block and pay setup
    exactly once (Python-level check of the canonical rule).
    """
    locs = [Location(0, 0), Location(1, 1), Location(2, 2)]
    clients = [
        Client(location=1, delivery=[0], service_duration=10),
        Client(location=1, delivery=[0], service_duration=10),
        Client(location=2, delivery=[0], service_duration=10),
    ]
    depots = [Depot(location=0)]
    vts = [VehicleType(capacity=[10])]
    mat = np.zeros((3, 3), dtype=int)
    for i in range(3):
        for j in range(3):
            if i != j:
                mat[i, j] = 5

    data = ProblemData(
        locs, clients, depots, vts, [mat], [mat],
        setup_durations=[0, 100, 100],
    )

    # depot -> client0 (loc1, setup 100) -> client1 (loc1, no setup)
    #      -> client2 (loc2, setup 100) -> depot
    route = Route(data, [0, 1, 2], 0)
    assert_equal(route.setup_duration(), 200)
    # travel = 5 + 0 + 5 + 5 (c0->c1 is same-location, zero travel)
    assert_equal(route.duration(), 15 + 3 * 10 + 200)


def test_solved_result_exposes_correct_duration_and_cost():
    """
    Solving a single-client instance with setup produces a Result whose
    duration and cost include the setup duration.
    """
    data = make_setup_data(
        setup_durations=[0, 100], service=50, unit_duration_cost=1
    )
    result = solve(data, stop=MaxIterations(5), seed=42)

    assert_(result.is_feasible())
    # duration = travel (20) + service (50) + setup (100)
    assert_equal(result.best.duration(), 170)
    # unit_distance_cost = 1 (default) and unit_duration_cost = 1, so cost
    # = distance (20) + duration (170).
    assert_equal(result.best.distance(), 20)
    assert_equal(result.cost(), 20 + 170)

    route = result.best.routes()[0]
    assert_equal(route.setup_duration(), 100)
