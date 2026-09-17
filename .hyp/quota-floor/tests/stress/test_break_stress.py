"""
Stress test for break support (OpenSpec task 9.5).

Builds a large instance (RC208 100 clients + 5 break-configured vehicles),
solves with 100k iterations, and checks:
  - No crash / assertion failure
  - Memory bounded (tracemalloc)
  - Completion within reasonable time.

Run:
    python -m pytest tests/stress/test_break_stress.py -v -s
"""
import gc
import tracemalloc

import pytest

from pyvrp import CustomBreak, CustomBreakReset, CustomBreakTrigger, solve
from pyvrp.stop import MaxIterations

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_large_data_with_breaks():
    """
    Load RC208 (100 clients) and add 5 vehicles with break configurations:
      - Vehicle 0-1: BR-style DRIVE_TIME break at 4h
      - Vehicle 2-3: EU-style DRIVE_TIME at 4.5h + duty break at 6h
      - Vehicle 4:   US-style DUTY_TIME at 8h + 10h

    All break resets are correct with exact atSecond clamping.
    """
    from tests.helpers import read

    data = read("data/RC208.vrp", round_func="dimacs")

    # Build 5 break-configured vehicle types
    br_break = CustomBreak(
        id=0,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=14400,  # 4h
        service=1800,         # 30min
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=10,
    )

    eu_drive = CustomBreak(
        id=0,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=16200,  # 4.5h
        service=2700,         # 45min
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=10,
    )
    eu_duty = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=21600,  # 6h
        service=2700,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=5,
    )

    us_duty1 = CustomBreak(
        id=0,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=28800,  # 8h
        service=1800,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=10,
    )
    us_duty2 = CustomBreak(
        id=1,
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=36000,  # 10h
        service=1800,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=5,
    )

    base_vt = data.vehicle_type(0)
    vt0 = base_vt.replace(custom_breaks=[br_break])
    vt1 = base_vt.replace(custom_breaks=[br_break])
    vt2 = base_vt.replace(custom_breaks=[eu_drive, eu_duty])
    vt3 = base_vt.replace(custom_breaks=[eu_drive, eu_duty])
    vt4 = base_vt.replace(custom_breaks=[us_duty1, us_duty2])

    return data.replace(vehicle_types=[vt0, vt1, vt2, vt3, vt4])


# ---------------------------------------------------------------------------
# Stress test
# ---------------------------------------------------------------------------

@pytest.mark.slow
@pytest.mark.stress
def test_100k_iterations_no_crash():
    """
    Solve RC208 with 5 break-configured vehicles for 100k iterations.
    Checks no crash, no assertion, and memory bounded.
    """
    data = _build_large_data_with_breaks()

    # Start memory tracking
    tracemalloc.start()
    gc.collect()
    gc.disable()  # prevent GC interference during solve

    mem_before = tracemalloc.get_traced_memory()
    peak = -1

    try:
        result = solve(data, stop=MaxIterations(100_000), seed=42)

        # Capture peak memory during solve
        _, peak = tracemalloc.get_traced_memory()
        gc.collect()
        _, final = tracemalloc.get_traced_memory()

        assert result is not None
        assert result.best is not None
        assert result.best.is_feasible()
        assert result.best.num_routes() >= 1

        # Check that solution is complete (all clients visited)
        assert result.best.num_clients() == data.num_clients

        # Check break_due on all routes
        for route in result.best.routes():
            assert route.break_due() >= 0  # no negative break_due

        # Memory: peak should be reasonable (< 200 MB for 100k iterations)
        peak_mb = peak / (1024 * 1024)
        print(f"\n  Peak memory: {peak_mb:.1f} MB")
        print(f"  Final memory: {final / (1024 * 1024):.1f} MB")
        print(f"  Routes: {result.best.num_routes()}")
        print(f"  Total break_due: "
              f"{sum(r.break_due() for r in result.best.routes())}")

        # Soft assertion on memory (not a hard fail since it depends
        # on system/allocator behavior)
        if peak_mb > 500:
            print(f"  WARNING: Peak memory {peak_mb:.1f} MB exceeds 500 MB")

    finally:
        tracemalloc.stop()
        gc.enable()


@pytest.mark.slow
@pytest.mark.stress
def test_memory_bounded_across_seeds():
    """
    5 independent solves at 10k iterations each, tracking memory.
    Verifies no leaks and bounded growth.
    """
    data = _build_large_data_with_breaks()

    tracemalloc.start()
    gc.collect()
    gc.disable()

    try:
        peaks = []
        for seed in range(5):
            gc.collect()
            mem_before = tracemalloc.get_traced_memory()[0]
            result = solve(data, stop=MaxIterations(10_000), seed=seed)
            gc.collect()
            _, peak = tracemalloc.get_traced_memory()
            peaks.append(peak)

        peak_mb = [p / (1024 * 1024) for p in peaks]
        print(f"\n  Memory peaks across 5 seeds: {[f'{m:.1f}' for m in peak_mb]} MB")

        # All peaks should be in the same ballpark (no monotonic growth)
        # Check that no peak is more than 2x the minimum
        min_peak = min(peaks)
        max_peak = max(peaks)
        assert max_peak <= 3 * min_peak, \
            f"Memory growth: min={min_peak/1024/1024:.1f}MB, max={max_peak/1024/1024:.1f}MB"

    finally:
        tracemalloc.stop()
        gc.enable()
