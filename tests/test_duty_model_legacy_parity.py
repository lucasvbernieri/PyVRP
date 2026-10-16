"""
Legacy rules and vehicles (``eligible_from = -1``, ``duty_limit = -1``) stay
bit for bit what the fork computed before the duty model (690fee3), in every
PYVRP_* mode of the test matrix: fixed-route statistics, penalised costs, and
seeded solves (same routes, same costs, same number of iterations). Routes
built in Python WITH break nodes are the exception: they match the search's
export instead (see test_route_from_activities_breaks).

The goldens were produced by ``tests/duty_legacy_parity.py`` against the
690fee3 build, once per mode; a mode without goldens is skipped.
"""

import json
import pathlib

import pytest
from numpy.testing import assert_equal

from tests.duty_legacy_parity import measure, mode_key

GOLDENS = pathlib.Path(__file__).parent / "data" / "duty_legacy_goldens.json"


def test_legacy_rules_identical_to_previous_fork():
    goldens = json.loads(GOLDENS.read_text())
    key = mode_key()
    if key not in goldens:
        pytest.skip(f"no goldens for mode {key}")

    expected = goldens[key]
    actual = json.loads(json.dumps(measure()))  # same normalisation
    assert_equal(sorted(actual), sorted(expected))
    for name in expected:
        got, want = actual[name]["routes"], expected[name]["routes"]
        assert_equal(len(got), len(want), name)
        if name != "no_rules":
            # Routes alternate: clients only, then the same order with break
            # nodes. A Python-built route WITH break nodes no longer follows
            # the previous fork: it now matches the search's export of the
            # same route (a break the pass does not serve carries no service;
            # test_route_from_activities_breaks). Routes without break nodes
            # and every seeded solve stay bit for bit.
            got, want = got[::2], want[::2]
        assert_equal(got, want, name)
        assert_equal(actual[name]["solves"], expected[name]["solves"], name)
