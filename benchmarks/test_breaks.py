"""
Break-support benchmark tests (OpenSpec task 9.2).

Mirrors benchmarks/test_solve.py style. Each test loads a synthetic multi-day
instance, configures a VehicleType with CustomBreak(s) appropriate for the
scenario, solves, and benchmarks.

Run standalone:
    python -m pytest benchmarks/test_breaks.py -q
"""
import pathlib

import pytest

from pyvrp import CustomBreak, CustomBreakReset, CustomBreakTrigger, VehicleType
from pyvrp.read import read as _read
from pyvrp.stop import MaxIterations
from pyvrp import solve

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

INSTANCES_DIR = pathlib.Path(__file__).parent / "instances"

INSTANCE_FILES = sorted(INSTANCES_DIR.glob("multi_day_*.vrp"))


# ---------------------------------------------------------------------------
# Break-configuration helpers
# ---------------------------------------------------------------------------

def build_clock_lunch_break(departure: int = 0, id_: int = 0) -> CustomBreak:
    """
    Lunch break anchored at time-of-day (CLOCK_TIME), e.g. 4.5h after
    shift start, 30min service.

    NOTE: id_ must be 0-15 (16-bit breakTakenMask).
    """
    return CustomBreak(
        id=id_,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(departure + 16200, departure + 18900)],
        service=2700,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        priority=10,
    )


def build_drive_time_break(
    trigger_value: int = 14400,
    service: int = 1800,
    reset: CustomBreakReset = CustomBreakReset.ALL_TIMERS,
    id_: int = 0,
) -> CustomBreak:
    """
    DRIVE_TIME triggered break (e.g. BR Lei 13.103/2015 4h drive).
    Uses ALL_TIMERS reset by default (fully correct with exact atSecond).
    """
    return CustomBreak(
        id=id_,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=trigger_value,
        reset=reset,
        mandatory=True,
        service=service,
    )


def build_overnight_rest(departure: int = 0, day_index: int = 1, id_: int = 10) -> CustomBreak:
    """
    Overnight rest break anchored at departure + day*86400 + shift_cap.
    For EU 561: 9h shift, so rest at departure + 86400 + 32400.
    Service = 43200 (12h rest).

    NOTE: id_ must be 0-15 (16-bit breakTakenMask).
    """
    rest_start = departure + day_index * 86400 + 32400
    return CustomBreak(
        id=id_,
        trigger=CustomBreakTrigger.CLOCK_TIME,
        tws=[(rest_start, rest_start + 43200)],
        service=43200,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=1,
    )


def build_duty_break(
    trigger_value: int = 28800,
    service: int = 1800,
    reset: CustomBreakReset = CustomBreakReset.ALL_TIMERS,
    id_: int = 0,
) -> CustomBreak:
    """
    DUTY_TIME triggered break (e.g. US FMCSA 395.3 8h on-duty).
    ALL_TIMERS reset by default; all reset modes correct with exact atSecond.
    """
    return CustomBreak(
        id=id_,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=trigger_value,
        reset=reset,
        mandatory=True,
        service=service,
    )


# ---------------------------------------------------------------------------
# Per-instance VehicleType builders
# ---------------------------------------------------------------------------

def _vehicle_for_lunch(data):
    """
    2-day: two DRIVE_TIME breaks (lunch equivalents at 4.5h drive each day).
    DRIVE_TIME used for simplicity; CLOCK_TIME is also correct with exact atSecond.
    """
    vt = data.vehicle_type(0)
    vt = vt.replace(
        custom_breaks=[
            build_drive_time_break(trigger_value=16200, service=2700,
                                   id_=0),  # day 1 lunch
            build_drive_time_break(trigger_value=16200, service=2700,
                                   id_=1),  # day 2 lunch
        ],
        shift_duration=172800,
    )
    return vt


def _vehicle_for_overnight(data):
    """
    2-day: lunch + overnight + lunch. All as DRIVE_TIME/DUTY_TIME equivalents.
    Overnight rest uses ALL_TIMERS reset to properly restart the day.
    AtSecond clamping is exact for all trigger types.
    """
    vt = data.vehicle_type(0)
    vt = vt.replace(
        custom_breaks=[
            build_drive_time_break(trigger_value=16200, service=2700,
                                   id_=0),  # day 1 lunch
            build_duty_break(trigger_value=32400, service=43200,
                             reset=CustomBreakReset.ALL_TIMERS,
                             id_=2),         # overnight rest (9h duty)
            build_drive_time_break(trigger_value=16200, service=2700,
                                   id_=1),  # day 2 lunch
        ],
        shift_duration=172800,
    )
    return vt


def _vehicle_for_multi_break(data):
    """Multi-break: drive lunch + duty overnight + drive lunch + duty break."""
    vt = data.vehicle_type(0)
    vt = vt.replace(
        custom_breaks=[
            build_drive_time_break(trigger_value=16200, service=2700, id_=0),
            build_duty_break(trigger_value=32400, service=43200,
                             reset=CustomBreakReset.ALL_TIMERS, id_=1),
            build_drive_time_break(trigger_value=16200, service=2700, id_=2),
            build_duty_break(trigger_value=28800, id_=3),
        ],
        shift_duration=172800,
    )
    return vt


def _vehicle_for_shift_cap(data):
    """
    3-day EU EC 561 shift caps.
    Each day: DRIVE_TIME break at 16200 (4.5h) + duty break at 21600 (6h).
    Overnight rest at 32400 duty (9h). All as DRIVE_TIME/DUTY_TIME.
    AtSecond clamping is exact.
    """
    vt = data.vehicle_type(0)
    breaks = []
    for day in range(3):
        id_base = day * 3
        breaks.append(
            CustomBreak(
                id=id_base + 1,
                trigger=CustomBreakTrigger.DRIVE_TIME,
                trigger_value=16200,
                reset=CustomBreakReset.ALL_TIMERS,
                mandatory=True,
                service=2700,  # 45min
            )
        )
        # Lunch-equivalent at 6h duty
        breaks.append(
            CustomBreak(
                id=id_base + 2,
                trigger=CustomBreakTrigger.DUTY_TIME,
                trigger_value=21600,
                service=2700,
                reset=CustomBreakReset.ALL_TIMERS,
                mandatory=True,
            )
        )
        # overnight rest at 9h duty (ALL_TIMERS to properly reset)
        breaks.append(
            CustomBreak(
                id=id_base + 3,
                trigger=CustomBreakTrigger.DUTY_TIME,
                trigger_value=32400,  # 9h shift
                service=39600,  # 11h rest
                reset=CustomBreakReset.ALL_TIMERS,
                mandatory=True,
                priority=1,
            )
        )
    vt = vt.replace(custom_breaks=breaks, shift_duration=259200)
    return vt


def _vehicle_for_heterogeneous(data):
    """Half of vehicles get breaks, half don't. Build 3+3 vehicle types."""
    vt_base = data.vehicle_type(0)
    vt_breaks = vt_base.replace(
        custom_breaks=[
            build_drive_time_break(trigger_value=16200, service=2700, id_=0),
            build_drive_time_break(trigger_value=16200, service=2700, id_=1),
        ],
        shift_duration=172800,
    )
    vt_no_breaks = vt_base.replace(shift_duration=172800)
    # 3 with breaks, 3 without
    vt_no_breaks2 = vt_no_breaks.replace()
    vt_no_breaks3 = vt_no_breaks.replace()
    return [vt_breaks, vt_breaks, vt_breaks, vt_no_breaks, vt_no_breaks2, vt_no_breaks3]


VEHICLE_BUILDERS = {
    "multi_day_2day_lunch":         _vehicle_for_lunch,
    "multi_day_2day_overnight":     _vehicle_for_overnight,
    "multi_day_2day_multi_break":   _vehicle_for_multi_break,
    "multi_day_3day_shift_cap":     _vehicle_for_shift_cap,
    "multi_day_heterogeneous":      _vehicle_for_heterogeneous,
}


# ---------------------------------------------------------------------------
# Parametrized benchmark test
# ---------------------------------------------------------------------------

def _instance_key(filepath: pathlib.Path) -> str:
    """Extract instance key from filename, e.g. 'multi_day_2day_lunch'."""
    return filepath.stem  # removes .vrp


@pytest.mark.parametrize(
    "inst_path",
    sorted(INSTANCE_FILES),
    ids=lambda p: _instance_key(p),
)
def test_breaks_benchmark(inst_path, benchmark):
    """
    Load a synthetic multi-day instance, configure breaks, solve, and
    assert the solution completes without errors.
    """
    data = _read(inst_path, round_func="dimacs")
    key = _instance_key(inst_path)
    builder = VEHICLE_BUILDERS[key]

    # Build replacement data with break-configured vehicle types
    vtypes = builder(data)
    if not isinstance(vtypes, list):
        vtypes = [vtypes]
    data = data.replace(vehicle_types=vtypes)

    # Solve and benchmark
    result = benchmark(solve, data, stop=MaxIterations(100), seed=0)
    assert result is not None
    assert result.best is not None


# ---------------------------------------------------------------------------
# Sanity check (runs without benchmark fixture when invoked directly)
# ---------------------------------------------------------------------------

def _load_data(inst_path: pathlib.Path):
    """Load data for standalone validation (no benchmark)."""
    data = _read(inst_path, round_func="dimacs")
    key = _instance_key(inst_path)
    builder = VEHICLE_BUILDERS[key]
    vtypes = builder(data)
    if not isinstance(vtypes, list):
        vtypes = [vtypes]
    return data.replace(vehicle_types=vtypes)


@pytest.mark.parametrize(
    "inst_path",
    sorted(INSTANCE_FILES),
    ids=lambda p: _instance_key(p),
)
def test_breaks_load_only(inst_path):
    """
    Verify that each instance can be loaded and break-configured vehicles
    are constructed without errors. (No solver execution.)
    """
    data = _load_data(inst_path)
    assert data.num_vehicles > 0
    for vt_idx in range(data.num_vehicle_types):
        vt = data.vehicle_type(vt_idx)
        assert isinstance(vt, VehicleType)

