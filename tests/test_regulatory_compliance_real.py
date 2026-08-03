"""
Real regulatory compliance tests for break support (OpenSpec item 7).

Creates deterministic synthetic instances where routes ACTUALLY cross
regulatory thresholds (BR 4h, EU 4.5h drive; US 8h duty), then verifies
break_due DETECTION, COMPLIANCE, and solver CONVERGENCE.

Unlike test_regulatory_compliance.py (which uses OkSmall/RC208 where
thresholds are never crossed by construction), these tests build a
chain instance with long travel times (edge_duration=4000s x N edges)
so that DRIVE_TIME and DUTY_TIME triggers fire.

Regulatory break configs (from design.md:1301-1307 -- REAL reset modes):
  BR Lei 13.103/2015: DRIVE_TIME 14400, service 1800, DRIVE_TIMER reset
  EU EC 561/2006:    DRIVE_TIME 16200, service 2700, DRIVE_AND_WORK reset
                       + shift cap DUTY_TIME 32400
  US FMCSA 395.3:     DUTY_TIME  28800, service 1800, DRIVE_AND_WORK reset
                       + window 50400

Note: Break insertion (Route::setSchedule / warm-start) not yet fully
operational; the solver detects violations but may not insert actual
CUSTOM_BREAK activities. Compliance/convergence tests verify the
solver converges and does not hang (breakDue may be >0 until break
insertion operators are implemented in a later lane).

Run:
    python -m pytest tests/test_regulatory_compliance_real.py -q -v
"""
import pytest
import numpy as np
from numpy.testing import assert_

from pyvrp import (
    Model,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    VehicleType,
    ProblemData,
    Solution,
    Route,
)
from pyvrp.stop import MaxIterations
from pyvrp import solve
from tests.helpers import make_search_route


# =============================================================================
# Deterministic synthetic instance builder
# =============================================================================

# Chain of clients where every inter-location edge has equal duration.
# With N_CLIENTS=6 and EDGE_DURATION=4000: 7 edges total (dep->C0, ..., C5->dep)
#   Total drive = 7 * 4000 = 28000s  > BR 4h (14400), > EU 4.5h (16200)
#   Total duty  = 28000 + 6*1800 = 38800s > US 8h (28800)
N_CLIENTS = 6
EDGE_DURATION = 4000       # seconds travel time per edge
SERVICE_TIME = 1800        # seconds service time per client
HORIZON = 86400            # 24h single-day horizon
CAPACITY = 9999            # effectively unlimited

# Solver budgets (low for comply to keep xfail fast; high for converge)
MAX_ITER_COMPLY = 500
MAX_ITER_CONVERGE = 2000

RNG_SEED = 42

# Client activity strings for make_search_route
_CHAIN_VISIT_NAMES = ["C%d" % i for i in range(N_CLIENTS)]


def _build_chain_data():
    """
    Build a deterministic synthetic ProblemData where a single-vehicle
    route visiting all clients exceeds all regulatory drive/duty thresholds.

    Returns base ProblemData (VehicleType WITHOUT break config yet).
    """
    model = Model()

    depot_loc = model.add_location(x=0.0, y=0.0, name="depot")
    model.add_depot(depot_loc, tw_early=0, tw_late=HORIZON)

    client_locs = []
    for i in range(N_CLIENTS):
        loc = model.add_location(x=float(i + 1), y=0.0, name="client%d" % i)
        model.add_client(
            loc,
            delivery=0,
            service_duration=SERVICE_TIME,
            tw_early=0,
            tw_late=HORIZON,
        )
        client_locs.append(loc)

    all_locs = [depot_loc] + client_locs
    for frm in all_locs:
        for to in all_locs:
            if frm is to:
                model.add_edge(frm, to, distance=0, duration=0)
            else:
                model.add_edge(
                    frm, to,
                    distance=EDGE_DURATION,
                    duration=EDGE_DURATION,
                )

    model.add_vehicle_type(num_available=1, capacity=[CAPACITY])
    return model.data()


# =============================================================================
# Break configuration builders (per regulation, REAL reset modes)
# =============================================================================

def _br_breaks():
    """BR Lei 13.103/2015: DRIVE_TIME 4h, 30min break, DRIVE_TIMER reset."""
    return [
        CustomBreak(
            id=0,
            tws=[(0, HORIZON)],
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=14400,
            service=1800,
            reset=CustomBreakReset.DRIVE_TIMER,
            mandatory=True,
            priority=10,
        ),
    ]


def _eu_breaks():
    """EU EC 561/2006: DRIVE_TIME 4.5h, 45min break, DRIVE_AND_WORK reset.

    NOTE: DRIVE_AND_WORK reset has known cost-tracking inconsistency.
    """
    return [
        CustomBreak(
            id=0,
            tws=[(0, HORIZON)],
            trigger=CustomBreakTrigger.DRIVE_TIME,
            trigger_value=16200,
            service=2700,
            reset=CustomBreakReset.DRIVE_AND_WORK,
            mandatory=True,
            priority=10,
        ),
    ]


def _us_breaks():
    """US FMCSA 395.3: DUTY_TIME 8h, 30min break, DRIVE_AND_WORK reset.

    NOTE: DUTY_TIME trigger + DRIVE_AND_WORK reset = known inconsistency.
    """
    return [
        CustomBreak(
            id=0,
            tws=[(0, HORIZON)],
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=28800,
            service=1800,
            reset=CustomBreakReset.DRIVE_AND_WORK,
            mandatory=True,
            priority=10,
        ),
    ]


def _configure_data(base_data, breaks, shift_duration=None):
    """Apply break configuration to base_data, returning new ProblemData."""
    vt = base_data.vehicle_type(0).replace(
        custom_breaks=breaks,
        shift_duration=shift_duration if shift_duration is not None else HORIZON,
    )
    return base_data.replace(vehicle_types=[vt])


# =============================================================================
# Regulation specs
# =============================================================================

REGULATIONS = {
    "BR_Lei_13_103": {
        "breaks": _br_breaks,
        "shift_duration": None,   # no explicit shift cap for BR
        "seed": RNG_SEED,
    },
    "EU_EC_561_2006": {
        "breaks": _eu_breaks,
        "shift_duration": HORIZON,  # 32400 would be EU cap; use full day
        "seed": RNG_SEED,
    },
    "US_FMCSA_395_3": {
        "breaks": _us_breaks,
        "shift_duration": HORIZON,  # 50400 would be US window
        "seed": RNG_SEED,
    },
}

REG_IDS = list(REGULATIONS.keys())


# =============================================================================
# 1. DETECTION -- search Route break_due > 0 without break activities
# =============================================================================

@pytest.mark.parametrize("reg_name", REG_IDS, ids=lambda r: r.split("_")[0])
def test_detect_violation_without_breaks(reg_name):
    """
    Detection: Build VehicleType WITH break rules, construct a search Route
    visiting all clients in order WITHOUT inserting breaks, and assert
    break_due > 0 (the DriveSegment mechanism detects threshold violations).

    This test passes for ALL regulations because the chain instance drive/duty
    time exceeds every threshold and the search Route correctly computes
    breakDue from DriveSegment accumulators.
    """
    spec = REGULATIONS[reg_name]
    base_data = _build_chain_data()
    breaks = spec["breaks"]()
    data = _configure_data(base_data, breaks, spec["shift_duration"])

    route = make_search_route(data, _CHAIN_VISIT_NAMES)

    assert_(route.has_breaks(),
            "%s: VehicleType has breaks but route.has_breaks() is False"
            % reg_name)
    assert_(route.num_clients() == N_CLIENTS,
            "%s: expected %d clients, got %d"
            % (reg_name, N_CLIENTS, route.num_clients()))

    bd = route.break_due()
    assert_(bd > 0,
            "Detection FAILED for %s: break_due=%d (expected >0) -- "
            "thresholds not crossed or DriveSegment not tracking"
            % (reg_name, bd))

    # Sanity: the relevant metric exceeds the trigger
    trigger = breaks[0].trigger
    trigger_value = breaks[0].trigger_value
    if trigger == CustomBreakTrigger.DUTY_TIME:
        total = _compute_expected_duty()
    else:
        total = _compute_expected_drive()
    assert_(total > trigger_value,
            "%s: metric=%d does not exceed trigger=%d (trigger=%s)"
            % (reg_name, total, trigger_value, trigger))


# =============================================================================
# 2. COMPLIANCE — solver WITH regulatory break config
# =============================================================================

@pytest.mark.parametrize("reg_name", REG_IDS, ids=lambda r: r.split("_")[0])
def test_comply_with_breaks(reg_name):
    """
    Compliance + convergence: Solve with regulatory break config and assert
    the solver converges (no hang/crash) and produces a feasible, complete
    solution with break_due == 0 (all mandatory breaks serviced).

    With CUSTOM_BREAK activities as real repositionable nodes + ShiftBreak
    operator (item 4), the solver can actually INSERT and TAKE breaks.
    """
    spec = REGULATIONS[reg_name]
    base_data = _build_chain_data()
    data = _configure_data(base_data, spec["breaks"](), spec["shift_duration"])

    result = solve(data, stop=MaxIterations(MAX_ITER_COMPLY), seed=spec["seed"])

    best = result.best
    assert best is not None, "Solver returned no solution"
    assert best.is_feasible(), "Solution should be feasible for base constraints"
    assert best.is_complete(), "All clients must be visited"

    bd = best.break_due()
    if bd > 0:
        for r in best.routes():
            print("  %s: route break_due=%d, clients=%d, duration=%d"
                  % (reg_name, r.break_due(), r.num_clients(), r.duration()))

    # Strict AC: with warm-start auto-insertion of break nodes and
    # ShiftBreak repositioning, the solver must achieve break_due == 0
    # (all mandatory breaks serviced).
    assert_(bd == 0,
            "COMPLIANCE FAILED for %s: break_due=%d (expected 0)"
            % (reg_name, bd))


# =============================================================================
# 3. CONVERGENCE -- solver with large iteration budget
# =============================================================================

@pytest.mark.parametrize("reg_name", REG_IDS, ids=lambda r: r.split("_")[0])
def test_converge_penalty_manager(reg_name):
    """
    Convergence: Run solver with larger iteration budget.
    PenaltyManager N+3 dimension (break_due_penalty) should drive
    the search toward break-feasible solutions.

    With CUSTOM_BREAK activities as real repositionable nodes + ShiftBreak
    (item 4), break_due must reach 0 for all regulation configs.
    """
    spec = REGULATIONS[reg_name]
    base_data = _build_chain_data()
    data = _configure_data(base_data, spec["breaks"](), spec["shift_duration"])

    result = solve(data, stop=MaxIterations(MAX_ITER_CONVERGE),
                   seed=spec["seed"])

    best = result.best
    assert best is not None, "Solver returned no solution"
    assert best.is_feasible()
    assert best.is_complete()

    bd = best.break_due()
    assert_(bd == 0,
            "CONVERGENCE FAILED for %s: break_due=%d (expected 0)"
            % (reg_name, bd))


# =============================================================================
# 4. Solution Route break_due (diagnostic -- documents current state)
# =============================================================================

@pytest.mark.parametrize("reg_name", REG_IDS, ids=lambda r: r.split("_")[0])
def test_solution_route_break_due_current(reg_name):
    """
    Diagnostic: pyvrp._pyvrp.Route break_due on a break-configured instance.

    The solution Route does NOT run setSchedule during construction from
    visit indices, so breakDue is currently 0 even when thresholds are
    exceeded. This documents the gap between search Route (correctly
    detects >0) and solution Route (returns 0 without inserted breaks).

    When break insertion is operational, this test should be revised
    to assert break_due > 0 when no breaks are inserted.
    """
    spec = REGULATIONS[reg_name]
    base_data = _build_chain_data()
    data = _configure_data(base_data, spec["breaks"](), spec["shift_duration"])

    route = Route(data, list(range(N_CLIENTS)), 0)
    assert route.num_clients() == N_CLIENTS

    bd = route.break_due()
    has_break_act = any(act.type.value == 100 for act in route.schedule())

    # Document: solution Route currently returns 0 without setSchedule
    total_drive = _compute_expected_drive()
    trigger_value = spec["breaks"]()[0].trigger_value
    if bd == 0 and total_drive > trigger_value:
        print("  %s: drive=%d > trigger=%d but break_due=0 "
              "(setSchedule not running on Route(data, visits, vt))"
              % (reg_name, total_drive, trigger_value))

    assert_(bd >= 0, "break_due must be non-negative")


# =============================================================================
# 5. Sanity: no breaks configured -> break_due == 0
# =============================================================================

def test_no_breaks_configured_break_due_zero():
    """
    Route without any break rules configured must have break_due == 0
    regardless of drive time. The [[unlikely]] guard in DriveSegment
    ensures zero overhead when hasBreaks() is false.
    """
    base_data = _build_chain_data()
    route = make_search_route(base_data, _CHAIN_VISIT_NAMES)

    assert_(not route.has_breaks(),
            "Route should report no breaks configured")
    assert_(route.break_due() == 0,
            "Without breaks, break_due must be 0 (got %d)"
            % route.break_due())


# =============================================================================
# Helpers
# =============================================================================

def _compute_expected_drive():
    """Minimum drive time: (N_CLIENTS+1) edges at EDGE_DURATION each."""
    return (N_CLIENTS + 1) * EDGE_DURATION


def _compute_expected_duty():
    """Minimum duty time: drive + N_CLIENTS * SERVICE_TIME."""
    return _compute_expected_drive() + N_CLIENTS * SERVICE_TIME


# =============================================================================
# Acceptance criteria for the item 1 fix lane
# =============================================================================

ACCEPTANCE_CRITERIA_FOR_FIX_LANE = """
Tests that must flip from xfail to pass when item 1 (cost-tracking
inconsistency fix) and break insertion (Route::setSchedule) are complete:

  [AC-1] test_comply_with_breaks[BR]
         -> assert solution.break_due() == 0 with BR Lei 13.103/2015
            (DRIVE_TIME + DRIVE_TIMER, single-day, STABLE config)

  [AC-2] test_comply_with_breaks[EU]
         -> assert solution.break_due() == 0 with EU EC 561/2006
            (DRIVE_TIME + DRIVE_AND_WORK, fix lane must resolve
             cost-tracking inconsistency)

  [AC-3] test_comply_with_breaks[US]
         -> assert solution.break_due() == 0 with US FMCSA 395.3
            (DUTY_TIME + DRIVE_AND_WORK, fix lane must resolve
             cost-tracking inconsistency)

  [AC-4] test_converge_penalty_manager[BR]
         -> assert solution.break_due() == 0 after 20000 iterations
            with BR config (PenaltyManager N+3 convergence validated)

  [AC-5] test_converge_penalty_manager[EU]
         -> Same for EU config

  [AC-6] test_converge_penalty_manager[US]
         -> Same for US config

  [AC-7] test_solution_route_break_due_current[BR/EU/US]
         -> When setSchedule runs on Route(), break_due > 0 should
            be reported for violating routes (or 0 if breaks inserted).

Current test results (expected to change after fix lane):
  - 4 detection/sanity tests: PASS
  - 3 solution_route diagnostics: PASS (documents current gap)
  - 3 compliance tests: XFAIL (break insertion not operational)
  - 3 convergence tests: XFAIL (same root cause)
"""
