"""
Verification test for tws_relative break positioning in the search layer.

Scenario: vehicle without time windows, 9 clients each taking ~2h service,
with a mandatory DUTY_TIME break at 14h duty, tws_relative=[(43200,50400)]
(12h-14h offset from route start). The break MUST be positioned mid-route
(after ~7th client) rather than at the end, because placing it at the end
would incur massive time-warp penalty from the DurationSegment merge.

WITHOUT the search-layer fix (DurationSegment with early/late for the break),
the solver would place the break at the end (time-warp ~0 because late=MAX).
WITH the fix, mid-route placement should result in low/zero time-warp.

This test fails if the break is the last non-depot activity.
"""
import numpy as np
from pyvrp import (
    ActivityType,
    Client,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Depot,
    Location,
    ProblemData,
    VehicleType,
    solve,
)
from pyvrp.stop import MaxIterations


def build_scenario():
    """Build a problem where a DUTY_TIME break with tws_relative must go mid-route."""
    # --- Locations ---
    # Depot at (0,0) + 9 clients in a line
    locs = [Location(x=i * 1000, y=0) for i in range(10)]  # 0=depot, 1-9=clients

    # --- Depot ---
    depots = [Depot(
        location=0,
        tw_early=0,
        tw_late=300_000,
        service_duration=0,
        name="depot",
    )]

    # --- Clients: 9 clients, each with ~2h service ---
    clients = []
    for i in range(9):
        c = Client(
            location=i + 1,
            delivery=[1],
            pickup=[0],
            service_duration=7200,  # 2 hours each
            tw_early=0,
            tw_late=300_000,
            name=f"c{i}",
        )
        clients.append(c)

    # --- Matrices ---
    n = len(locs)
    coords = np.array([(loc.x, loc.y) for loc in locs], dtype=float)
    dist = np.zeros((n, n), dtype=np.int64)
    dur = np.zeros((n, n), dtype=np.int64)
    for i in range(n):
        for j in range(n):
            dx = coords[i][0] - coords[j][0]
            dy = coords[i][1] - coords[j][1]
            val = int(np.sqrt(dx * dx + dy * dy))
            dist[i, j] = val
            dur[i, j] = val

    # --- Vehicle type ---
    vt = VehicleType(
        num_available=1,
        capacity=[20],
        start_depot=0,
        end_depot=0,
        name="test_vehicle",
        tw_early=0,
        tw_late=300_000,
        shift_duration=300_000,
        max_distance=1_000_000,
        unit_distance_cost=0,
        unit_duration_cost=1,
    )

    # --- CustomBreak: DUTY_TIME at 14h, tws_relative offset 12-14h ---
    brk = CustomBreak(
        id=0,
        tws=[(43200, 50400)],  # offset: 12h to 14h after route start
        service=1800,  # 30min break
        trigger=CustomBreakTrigger.DUTY_TIME,
        trigger_value=50400,  # 14h
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        priority=100,
        tws_relative=True,
    )

    vt = vt.replace(custom_breaks=[brk])

    # --- ProblemData ---
    data = ProblemData(
        locations=locs,
        clients=clients,
        depots=depots,
        vehicle_types=[vt],
        distance_matrices=[dist],
        duration_matrices=[dur],
    )

    return data


def verify_break_position(result):
    """Check that the break is mid-route, not at the end, and time_warp is low."""
    assert result is not None
    assert result.best is not None
    assert result.best.is_feasible()
    assert result.best.num_routes() >= 1

    found_break = False
    any_route_ok = False

    for route in result.best.routes():
        sched = route.schedule()

        # Find break activities using type field
        break_idxs = [i for i, a in enumerate(sched)
                      if a.type == ActivityType.CUSTOM_BREAK]
        if not break_idxs:
            continue

        found_break = True

        # The break should NOT be the last non-depot activity
        non_depot_idxs = [i for i, a in enumerate(sched)
                          if not a.is_depot()]
        if non_depot_idxs:
            last_non_depot = max(non_depot_idxs)

            for bi in break_idxs:
                assert bi != last_non_depot, (
                    f"FAIL: Break at position {bi} is the last non-depot activity! "
                    f"Schedule types: {[a.type.name for a in sched]}"
                )

                # There should be clients after the break
                after = sched[bi + 1:]
                num_clients_after = sum(1 for a in after
                                        if a.type == ActivityType.CLIENT)
                assert num_clients_after > 0, (
                    f"FAIL: Break at pos {bi} has no clients after it! "
                    f"After break: {[a.type.name for a in after]}"
                )

        any_route_ok = True

        # time_warp should be zero or very small (< 1h = 3600s)
        tw = route.time_warp()
        print(f"  Route time_warp: {tw}")
        print(f"  Schedule types: {[a.type.name for a in sched]}")
        print(f"  Break positions: {break_idxs} of {len(sched)} acts")

        assert tw < 3600, (
            f"FAIL: Time warp too large: {tw}. Expected < 3600."
        )

    assert found_break, "FAIL: No route had a break!"
    assert any_route_ok, "FAIL: No route passed verification!"

    print("\n=== VERIFICATION PASSED: break is mid-route with low time_warp. ===")


if __name__ == "__main__":
    data = build_scenario()
    print(f"Problem: {data.num_clients} clients, {data.num_vehicle_types} vehicle type(s)")

    result = solve(data, stop=MaxIterations(10_000), seed=42)

    best = result.best
    print(f"Solution: {best.num_routes()} routes, "
          f"feasible={best.is_feasible()}")
    print(f"Duration: {best.duration()}, TimeWarp: {best.time_warp()}, "
          f"Distance: {best.distance()}")

    verify_break_position(result)
