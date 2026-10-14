"""
Legacy rules and vehicles (``eligible_from = -1``, ``duty_limit = -1``) stay
bit for bit what the fork computed before the duty model (690fee3), in every
PYVRP_* mode of the test matrix: fixed-route statistics, penalised costs, and
seeded solves (same routes, same costs, same number of iterations).

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
        assert_equal(actual[name]["routes"], expected[name]["routes"], name)
        assert_equal(actual[name]["solves"], expected[name]["solves"], name)
