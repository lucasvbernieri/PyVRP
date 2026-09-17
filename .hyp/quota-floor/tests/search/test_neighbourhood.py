import numpy as np
from numpy.testing import assert_, assert_equal, assert_raises
from pytest import mark

from pyvrp import (
    Activity,
    Client,
    Depot,
    Location,
    ProblemData,
    Shipment,
    VehicleType,
)
from pyvrp.search import NeighbourhoodParams, compute_neighbours


def test_neighbourhood_params_raises_for_empty_neighbourhoods():
    """
    Test that ``NeighbourhoodParams`` raises for empty neighbourhoods.
    """
    with assert_raises(ValueError):
        NeighbourhoodParams(num_neighbours=0)


# fmt: off
@mark.parametrize(
    (
        "weight_wait_time",
        "num_neighbours",
        "symmetric_proximity",
        "idx_check",
        "expected_neighbours_check",
    ),
    [
        (20, 10, True, 1,
         {0, 2, 3, 4, 5, 6, 7, 44, 45, 99}),
        # From original C++ implementation
        (18, 34, True, 0,
         {1, 2, 3, 4, 5, 6, 7, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 52,
          53, 54, 59, 60, 67, 68, 69, 72, 77, 78, 80, 87, 89, 97, 99}),
        (18, 34, True, 98,
         {8, 9, 10, 11, 12, 13, 14, 15, 19, 21, 23, 46, 51, 52, 54, 55, 56, 57,
          58, 59, 63, 64, 65, 68, 73, 79, 81, 82, 85, 86, 87, 89, 90, 97}),
    ],
)
# fmt: on
def test_compute_neighbours(
    rc208,
    weight_wait_time: int,
    num_neighbours: int,
    symmetric_proximity: bool,
    idx_check: int,
    expected_neighbours_check: set[int],
):
    """
    Tests ``compute_neighbours`` on several well-understood cases.
    """
    params = NeighbourhoodParams(
        weight_wait_time,
        num_neighbours,
        symmetric_proximity,
    )
    neighbours = compute_neighbours(rc208, params)

    assert_equal(len(neighbours), rc208.num_clients)

    # We compare sets because the expected data (from the old C++
    # implementation) sorts by client ID (ascending), not proximity.
    activity = Activity(f"C{idx_check}")
    assert_equal(
        set(act.idx for act in neighbours[activity]),
        expected_neighbours_check,
    )

    for neighb in neighbours.values():
        assert_equal(len(neighb), num_neighbours)


def test_neighbours_are_sorted_by_proximity(small_cvrp):
    """
    Tests that the neighbourhood lists sort by their proximity: closest first.
    """
    params = NeighbourhoodParams(0, small_cvrp.num_clients)
    neighbours = compute_neighbours(small_cvrp, params)
    clients = range(small_cvrp.num_clients)

    # Only consider client location distances; depots do not factor into the
    # neighbourhoods.
    distances = small_cvrp.distance_matrix(profile=0)
    distances = distances[small_cvrp.num_depots:, small_cvrp.num_depots:]

    for client in clients:
        # Proximity is completely based on distance. We break ties by index
        # (using stable sort). The test below checks that this is the same as
        # what comes out of the granular neighbourhood calculation.
        valid = np.array([other for other in clients if other != client])
        dists = distances[client, valid]
        by_proximity = valid[np.argsort(dists, kind="stable")]
        as_activities = [Activity(f"C{idx}") for idx in by_proximity]
        assert_equal(as_activities, neighbours[Activity(f"C{client}")])


def test_more_neighbours_than_instance_size(rc208):
    """
    Tests that a value for ``num_neighbours`` larger than the number of clients
    in the instance results in neighbourhoods of maximum size.
    """
    params = NeighbourhoodParams(num_neighbours=rc208.num_clients)
    neighbours = compute_neighbours(rc208, params)

    for neighb in neighbours.values():
        assert_equal(len(neighb), rc208.num_clients - 1)


def test_proximity_with_prizes(prize_collecting):
    """
    Tests that prizes factor into the neighbourhood structure, and offset
    travel costs somewhat.
    """
    params = NeighbourhoodParams(0, num_neighbours=10)
    neighbours = compute_neighbours(prize_collecting, params)

    # We compare the number of times clients C19 and C35 are in other clients'
    # neighbourhoods. Client 19 is at location (57, 58), client 35 at (63, 69).
    # They're fairly close to each other, in one corner of the plane. The
    # biggest difference is in prizes: client 19 has a prize of 33, whereas
    # client 35 only yields 8. As a consequence, 35 should be in many fewer
    # neighbourhoods than 19.
    count_19 = sum(Activity("C19") in n for n in neighbours.values())
    count_35 = sum(Activity("C35") in n for n in neighbours.values())
    assert_(count_19 > count_35)


def test_proximity_with_mutually_exclusive_groups(
    ok_small_mutually_exclusive_groups,
):
    """
    Tests that clients that are a member of a mutually exclusive client group
    are not in each other's neighbourhood.
    """
    params = NeighbourhoodParams(0, num_neighbours=1)
    neighbours = compute_neighbours(ok_small_mutually_exclusive_groups, params)

    group = ok_small_mutually_exclusive_groups.group(0)
    members = group.clients
    for client in members:
        act = Activity(f"C{client}")
        assert_(
            all(Activity(f"C{other}") not in neighbours[act]
            for other in members)
        )


def test_different_routing_costs(ok_small):
    """
    High-level smoke test that checks that the granular neighbourhood takes
    into account the unit distance and duration costs of different vehicle
    types.
    """
    rng = np.random.default_rng(seed=42)
    new_dur = rng.integers(0, 1_000, size=(5, 5))
    np.fill_diagonal(new_dur, 0)

    # Because the OkSmall instance has the same distance and duration matrices,
    # it is hard to make this test work. So we first introduce a new duration
    # matrix.
    new_data = ok_small.replace(duration_matrices=[new_dur])
    new_neighbours = compute_neighbours(new_data)

    # Now we also add vehicle types with different cost profiles: the first
    # vehicle type is like the original and cares only about distance, whereas
    # the second type aims to minimise route duration. This should result in a
    # different neighbourhood structure.
    orig_type = ok_small.vehicle_type(0)
    different_cost_data = new_data.replace(
        vehicle_types=[
            orig_type.replace(unit_distance_cost=1, unit_duration_cost=0),
            orig_type.replace(unit_distance_cost=0, unit_duration_cost=1),
        ],
    )
    different_cost_neighbours = compute_neighbours(different_cost_data)
    assert_(different_cost_neighbours != new_neighbours)


def test_multiple_routing_profiles(ok_small):
    """
    Tests the granular neighbourhood selects the right profiles in the
    proximity calculation when working with multiple routing profiles.
    """
    huge_mat = np.where(np.eye(ok_small.num_locations), 0, 10_000)
    data = ok_small.replace(
        vehicle_types=[
            VehicleType(1, capacity=[10], profile=1),
            *ok_small.vehicle_types(),
        ],
        distance_matrices=[huge_mat, *ok_small.distance_matrices()],
        duration_matrices=[huge_mat, *ok_small.duration_matrices()],
    )

    # The new routing profile has huge distances and durations, but the
    # original profile and vehicles are still available. The neighbourhood
    # computation should recognise this and use the best profile for
    # neighbourhood computations, resulting in the same neighbourhood as with
    # the original (unchanged) data.
    assert_equal(compute_neighbours(data), compute_neighbours(ok_small))


def test_zero_clients_or_shipments():
    """
    Tests that the neighbourhood for an instance with zero clients and
    shipments is empty.
    """
    data = ProblemData(
        locations=[Location(0, 0)],
        clients=[],
        depots=[Depot(0)],
        vehicle_types=[VehicleType()],
        distance_matrices=[np.zeros((1, 1), dtype=int)],
        duration_matrices=[np.zeros((1, 1), dtype=int)],
        shipments=[],
    )

    assert_equal(compute_neighbours(data), {})


def test_shipments_exclude_either_activity_in_neighbourhood(small_shipments):
    """
    Tests that shipments exclude their own activities from the neighbourhood.
    """
    neighbours = compute_neighbours(small_shipments)

    for shipment in range(small_shipments.num_shipments):
        delivery = Activity(f"U{shipment}")
        assert_(delivery not in neighbours)  # only includes pickup activities

        pickup = Activity(f"L{shipment}")
        neighbourhood = neighbours[pickup]
        assert_(pickup not in neighbourhood)  # cannot be in own neighbourhood
        assert_(delivery not in neighbourhood)  # nor can delivery

        # The neighbourhood contains pickup and delivery activities for each
        # shipment, but excludes its own.
        assert_equal(len(neighbourhood), 2 * small_shipments.num_shipments - 2)


def test_shipment_proximity_uses_pickup_and_delivery():
    """
    Tests that shipment proximity considers both pickup and delivery.
    """
    distances = np.full((5, 5), 100, dtype=int)
    np.fill_diagonal(distances, 0)
    distances[3, 1] = distances[1, 3] = 10  # pickup closest to C0
    distances[4, 2] = distances[2, 4] = 1  # delivery closest to C1

    data = ProblemData(
        locations=[Location(idx, 0) for idx in range(5)],
        clients=[Client(1), Client(2)],
        depots=[Depot(0)],
        vehicle_types=[VehicleType()],
        distance_matrices=[distances],
        duration_matrices=[distances],
        shipments=[Shipment(3, 4)],
    )

    # Shipment 0's pickup is closest to C0, but its delivery is even closer to
    # C1. With one neighbour, the neighbour is therefore C1 rather than C0.
    params = NeighbourhoodParams(0, num_neighbours=1)
    neighbours = compute_neighbours(data, params)

    assert_equal(neighbours[Activity("L0")], [Activity("C1")])


def test_mixed_client_shipments(small_shipments):
    """
    Tests the neighbourhood of a small instance with mixed clients and
    shipments.
    """
    # Create a mixed instance, with clients and shipments.
    clients = [Client(2, delivery=[0]), Client(5, delivery=[1])]
    data = small_shipments.replace(clients=clients)
    assert_equal(data.num_clients, 2)
    assert_equal(data.num_shipments, 4)

    neighbours = compute_neighbours(data)
    assert_equal(len(neighbours), 6)  # for each client and shipment pickup

    # We should have neighbours for both clients and shipments.
    assert_(Activity("C0") in neighbours)
    assert_(Activity("L0") in neighbours)

    # The shipment is at location 2, which is also where the first shipment's
    # pickup happens. So, L0 should be the first entry in C0's neighbourhood,
    # and vice versa.
    assert_equal(neighbours[Activity("C0")][0], Activity("L0"))
    assert_equal(neighbours[Activity("L0")][0], Activity("C0"))

    # The neighbourhood can contain all other shipments and clients. In this
    # case, there is always one other client, and three other shipments. Thus,
    # a total of seven activities can be in the neighbourhood.
    assert_equal(len(neighbours[Activity("C0")]), 7)


# ---------------------------------------------------------------------------
# Cross-day quota (fork): same-day candidates first, then a bounded number of
# cross-day ones. Day = tw_early // 86_400 on the absolute clock.
# ---------------------------------------------------------------------------
_DAY = 86_400


def _two_day_instance(num_clients: int = 12, num_days: int = 2) -> ProblemData:
    """
    Clients on a line; client ``i`` is on day ``i % num_days``. Distances grow
    with the index gap and waiting is not priced (``weight_wait_time = 0`` in
    the tests, ``unit_duration_cost = 0``), so proximity is pure distance and
    the ranking interleaves the days: every client has same-day and cross-day
    candidates at every rank.
    """
    n = num_clients + 1
    coords = np.arange(n)
    distances = np.abs(coords[:, None] - coords[None, :]) * 10
    return ProblemData(
        locations=[Location(int(x), 0) for x in coords],
        clients=[
            Client(
                idx + 1,
                tw_early=(idx % num_days) * _DAY,
                tw_late=(idx % num_days) * _DAY + 36_000,
            )
            for idx in range(num_clients)
        ],
        depots=[Depot(0)],
        vehicle_types=[VehicleType(tw_late=num_days * _DAY)],
        distance_matrices=[distances],
        duration_matrices=[distances],
    )


def _day_of(data: ProblemData, act: Activity) -> int:
    return int(data.client(act.idx).tw_early) // _DAY


def test_cross_day_quota_default_is_module_constant():
    """
    Tests that the default quota is the environment-derived module constant,
    and that it is zero here (the test process does not set the variable).
    """
    from pyvrp.search import _search

    assert_equal(NeighbourhoodParams().cross_day_quota, _search.CROSS_DAY_QUOTA)
    assert_equal(_search.CROSS_DAY_QUOTA, 0)


@mark.parametrize("num_neighbours", [1, 4, 11, 50])
def test_cross_day_quota_zero_is_identical(num_neighbours: int):
    """
    Tests that ``cross_day_quota=0`` (the default) yields exactly the plain
    closest-``num_neighbours`` lists, day ignored: the same list, same order.
    """
    data = _two_day_instance()
    plain = compute_neighbours(data, NeighbourhoodParams(0, num_neighbours))
    quota0 = compute_neighbours(
        data, NeighbourhoodParams(0, num_neighbours, cross_day_quota=0)
    )
    assert_equal(plain, quota0)

    # And the plain list indeed mixes days when it can (else the test is moot).
    if num_neighbours > 1:
        act = Activity("C5")
        assert_(len({_day_of(data, o) for o in plain[act]}) == 2)


def test_cross_day_quota_zero_matches_default_on_real_instance(rc208):
    """
    Same no-op check on a benchmark instance: ``cross_day_quota=0`` and the
    default parameters give the same neighbourhoods.
    """
    neighbours = compute_neighbours(rc208, NeighbourhoodParams(20, 10, True, 0))
    assert_equal(neighbours, compute_neighbours(rc208, NeighbourhoodParams(20, 10)))


@mark.parametrize(("num_neighbours", "quota"), [(3, 1), (3, 2), (5, 5), (11, 3)])
def test_cross_day_quota_same_day_first_then_cross_day(
    num_neighbours: int, quota: int
):
    """
    Tests the shape of a quota list: at most ``num_neighbours`` same-day
    candidates first (closest first), followed by the cross-day block
    (closest first among those). The cross-day block holds ``quota``
    candidates, or as many as the plain closest-``num_neighbours`` list had if
    that is more: the quota only adds, every plain neighbour stays.
    """
    data = _two_day_instance()
    params = NeighbourhoodParams(0, num_neighbours, cross_day_quota=quota)
    neighbours = compute_neighbours(data, params)
    plain = compute_neighbours(data, NeighbourhoodParams(0, num_neighbours))
    full = compute_neighbours(data, NeighbourhoodParams(0, data.num_clients - 1))

    for client in range(data.num_clients):
        act = Activity(f"C{client}")
        day = _day_of(data, act)
        lst = neighbours[act]
        days = [_day_of(data, o) == day for o in lst]

        n_same = sum(days)
        n_cross = len(lst) - n_same
        plain_cross = sum(_day_of(data, o) != day for o in plain[act])
        assert_(n_same <= num_neighbours)
        assert_equal(n_cross, max(quota, plain_cross))
        # Same-day block first, then the cross-day block: no interleaving.
        assert_equal(days, [True] * n_same + [False] * n_cross)

        # Each block is the ranking restricted to that day, cut at its size.
        ranked_same = [o for o in full[act] if _day_of(data, o) == day]
        ranked_cross = [o for o in full[act] if _day_of(data, o) != day]
        assert_equal(lst[:n_same], ranked_same[:num_neighbours])
        assert_equal(lst[n_same:], ranked_cross[:n_cross])
        assert_(act not in lst)

        # Superset of the plain list: the quota never removes a neighbour.
        assert_(set(plain[act]) <= set(lst))
        assert_(len(lst) >= len(plain[act]))


def test_cross_day_quota_single_day_is_inert(rc208):
    """
    Tests that on a single-day instance (every ``tw_early`` on day 0) the
    quota changes nothing: there is no cross-day candidate to append.
    """
    assert_(all(c.tw_early < _DAY for c in rc208.clients()))
    plain = compute_neighbours(rc208, NeighbourhoodParams(20, 10))
    with_quota = compute_neighbours(rc208, NeighbourhoodParams(20, 10, cross_day_quota=20))
    assert_equal(plain, with_quota)


def test_cross_day_quota_respects_mutually_exclusive_groups(
    ok_small_mutually_exclusive_groups,
):
    """
    Tests that group members still do not neighbour each other with a quota
    in force (proximity max, so they rank last in every day block).
    """
    data = ok_small_mutually_exclusive_groups
    params = NeighbourhoodParams(0, num_neighbours=1, cross_day_quota=1)
    neighbours = compute_neighbours(data, params)

    members = data.group(0).clients
    for client in members:
        act = Activity(f"C{client}")
        assert_(all(Activity(f"C{o}") not in neighbours[act] for o in members))


def test_cross_day_quota_params_equality():
    """
    Tests that ``cross_day_quota`` takes part in parameter equality.
    """
    assert_equal(NeighbourhoodParams(0.2, 50, True, 0), NeighbourhoodParams())
    assert_(NeighbourhoodParams(0.2, 50, True, 5) != NeighbourhoodParams())
