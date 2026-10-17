"""
Characterization: on legacy rules (``eligible_from = -1``,
``duty_limit = -1``) a multi-day route built in Python with an overnight rest
followed by a wait for the next morning's window keeps, field by field, the
schedule, costs, duration, time warp, rest lateness and feasibility the fork
computed before the duty model (690fee3), in every PYVRP_* mode of the test
matrix. The hows-router compares its legacy manual candidate with the
solver's route by that penalised cost, so a change here changes which route
it emits.

Only ``breaks_served`` is new (690fee3 left it empty): it reports the rests
the shared forward pass serves, as the search's export does.

The goldens were produced by ``tests/legacy_overnight_characterization.py``
against the 690fee3 build, once per mode; a mode without goldens is skipped.
"""

import json
import pathlib

import pytest
from numpy.testing import assert_, assert_equal

from pyvrp import Route, Solution
from pyvrp.search._search import Node
from pyvrp.search._search import Solution as SearchSolution
from tests.duty_legacy_parity import mode_key as parity_mode_key
from tests.legacy_overnight_characterization import (
    H,
    SEQUENCES,
    activities,
    cost_evaluator,
    instance,
    measure,
    mode_key,
    route_fields,
)

GOLDENS = (
    pathlib.Path(__file__).parent / "data" / "legacy_overnight_goldens.json"
)


def _exported(data, acts):
    """The route the search exports for ``acts`` (see
    test_route_from_activities_breaks._exported)."""
    ssol = SearchSolution(data)
    other = activities(list(range(data.num_clients))[::-1])
    ssol.load(Solution(data, [Route(data, other, 0)]))
    sroute = ssol.routes[0]
    sroute.clear()
    for act in acts:
        node = ssol[act]
        sroute.append(node if node is not None else Node(act))
    sroute.update()
    return ssol.unload().routes()[0]


def test_mode_key_matches_parity_goldens():
    assert_equal(mode_key(), parity_mode_key())


def test_legacy_overnight_route_identical_to_previous_fork():
    goldens = json.loads(GOLDENS.read_text())
    key = mode_key()
    if key not in goldens:
        pytest.skip(f"no goldens for mode {key}")

    expected = goldens[key]
    actual = json.loads(json.dumps(measure()))  # same normalisation
    assert_equal(sorted(actual), sorted(expected))
    for name in expected:
        for field in expected[name]:
            assert_equal(
                actual[name][field], expected[name][field], f"{name} {field}"
            )


@pytest.mark.parametrize("start", [6 * H, 0])
@pytest.mark.parametrize("name", sorted(SEQUENCES))
def test_legacy_overnight_route_reports_served_rests(start, name):
    data = instance(start)
    acts = activities(SEQUENCES[name])
    built = Route(data, acts, 0)
    exported = _exported(data, acts)
    assert_equal(list(built.breaks_served()), list(exported.breaks_served()))
    # Reporting the served rests does not touch anything else: no
    # per-activity services, the previous fork's fields (goldens above).
    assert_equal(list(built.break_services()), [])


def test_overnight_then_morning_wait_serves_the_rest():
    # The review's case: the first night's rest is served, and the
    # Python-built route (previous fork) and the search's export price it
    # differently; the legacy route keeps the previous fork's price.
    data = instance(6 * H)
    acts = activities(SEQUENCES["overnight_then_morning_wait"])
    built = Route(data, acts, 0)
    exported = _exported(data, acts)
    assert_equal(list(built.breaks_served()), [0])
    ce = cost_evaluator()
    assert_(
        route_fields(data, ce, built)["penalised_cost"]
        != route_fields(data, ce, exported)["penalised_cost"]
    )
