"""
Generate synthetic multi-day VRPLIB instances for break-support benchmarks.

Idempotent: running this script repeatedly produces identical output.
Self-contained: writes VRPLIB text directly; no pyvrp import required.

Outputs .vrp files into benchmarks/instances/.
"""
import os
import random
import sys


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_instance(
    name: str,
    comment: str,
    num_clients: int,
    num_vehicles: int,
    capacity: int,
    service_time: int,
    coords: list[tuple[int, int]],       # index 0 = depot
    demands: list[int],                  # index 0 = 0 (depot)
    time_windows: list[tuple[int, int]], # index 0 = depot TW
) -> str:
    """Build a VRPLIB CVRPTW string (EUC_2D)."""
    dim = 1 + num_clients  # depot + clients
    lines = [
        f"NAME : {name}",
        f"COMMENT : {comment}",
        "TYPE : CVRPTW",
        f"DIMENSION : {dim}",
        f"VEHICLES : {num_vehicles}",
        f"CAPACITY : {capacity}",
        f"SERVICE_TIME : {service_time}",
        "EDGE_WEIGHT_TYPE : EUC_2D",
        "NODE_COORD_SECTION",
    ]
    for i, (x, y) in enumerate(coords):
        lines.append(f"{i + 1} {x} {y}")
    lines.append("DEMAND_SECTION")
    for i, d in enumerate(demands):
        lines.append(f"{i + 1} {d}")
    lines.append("TIME_WINDOW_SECTION")
    for i, (lo, hi) in enumerate(time_windows):
        lines.append(f"{i + 1} {lo} {hi}")
    lines.append("DEPOT_SECTION")
    lines.append("1")
    lines.append("-1")
    lines.append("EOF")
    return "\n".join(lines) + "\n"


def _seeded_rng(seed: int) -> random.Random:
    return random.Random(seed)


# ---------------------------------------------------------------------------
# Coordinates helper
# ---------------------------------------------------------------------------

def _generate_coords(
    rng: random.Random,
    x_range: tuple[int, int],
    y_range: tuple[int, int],
    num_clients: int,
    depot: tuple[int, int] = (50, 50),
) -> list[tuple[int, int]]:
    """Return [depot, client1, ..., clientN] coordinates."""
    pts = [depot]
    for _ in range(num_clients):
        pts.append((rng.randint(*x_range), rng.randint(*y_range)))
    return pts


# ---------------------------------------------------------------------------
# Instance generators
# ---------------------------------------------------------------------------

def _make_2day_lunch(seed: int = 42) -> str:
    """
    20 clients, 2-day horizon (0..172800), lunch-window anchor.
    Clients split ~evenly across day 1 (0..57600) and day 2 (86400..144000).
    """
    rng = _seeded_rng(seed)
    n = 20
    coords = _generate_coords(rng, (0, 100), (0, 100), n)
    # Demand: 10-30 each
    demands = [0] + [rng.randint(10, 30) for _ in range(n)]
    # Depot TW: 0 .. 172800 (2 days)
    # Clients: 10 in day 1 [0..57600], 10 in day 2 [86400..144000]
    # Each client has a 4h (14400s) window within their day
    tws = [(0, 172800)]  # depot
    for i in range(n):
        if i < 10:
            day_start = 0
        else:
            day_start = 86400
        lo = rng.randint(day_start, day_start + 43200)
        tws.append((lo, lo + 14400))
    return _make_instance(
        name="multi-day-2day-lunch",
        comment="2-day horizon with lunch-break anchor points",
        num_clients=n,
        num_vehicles=5,
        capacity=200,
        service_time=600,
        coords=coords,
        demands=demands,
        time_windows=tws,
    )


def _make_2day_overnight(seed: int = 43) -> str:
    """
    20 clients, overnight rest (12h / 43200s) between day 1 and day 2.
    Day 1: 0..57600, Day 2: 100800..158400. The 43200s gap simulates an
    overnight rest period.
    """
    rng = _seeded_rng(seed)
    n = 20
    coords = _generate_coords(rng, (0, 100), (0, 100), n)
    demands = [0] + [rng.randint(10, 30) for _ in range(n)]
    # Depot: 0..172800
    tws = [(0, 172800)]
    for i in range(n):
        if i < 10:
            lo = rng.randint(0, 43200)
        else:
            lo = rng.randint(100800, 144000)
        tws.append((lo, lo + 14400))
    return _make_instance(
        name="multi-day-2day-overnight",
        comment="2-day horizon with 12h overnight rest gap",
        num_clients=n,
        num_vehicles=5,
        capacity=200,
        service_time=600,
        coords=coords,
        demands=demands,
        time_windows=tws,
    )


def _make_2day_multi_break(seed: int = 44) -> str:
    """
    20 clients over 2 days. Each day contains early and late windows,
    making it natural to insert both a lunch break and an overnight rest.
    Day 1 early: 0..28800, late: 28800..57600.
    Day 2 early: 86400..115200, late: 115200..144000.
    """
    rng = _seeded_rng(seed)
    n = 20
    coords = _generate_coords(rng, (0, 100), (0, 100), n)
    demands = [0] + [rng.randint(10, 30) for _ in range(n)]
    tws = [(0, 172800)]
    # 5 per day-part
    parts = [
        (0, 14400),          # day 1 early: start 0..14400
        (14400, 28800),      # day 1 late
        (86400, 100800),     # day 2 early
        (100800, 115200),    # day 2 late
    ]
    idx = 0
    for lo_base, hi_base in parts:
        for _ in range(5):
            lo = rng.randint(lo_base, hi_base)
            tws.append((lo, lo + 10800))  # 3h window
    return _make_instance(
        name="multi-day-2day-multi-break",
        comment="2-day with lunch + overnight + duty-time break windows",
        num_clients=n,
        num_vehicles=5,
        capacity=200,
        service_time=600,
        coords=coords,
        demands=demands,
        time_windows=tws,
    )


def _make_3day_shift_cap(seed: int = 45) -> str:
    """
    15 clients over 3 days (259200s). EU EC 561 shift caps (9h shifts).
    5 clients per day in a 9h window each day.
    """
    rng = _seeded_rng(seed)
    n = 15
    coords = _generate_coords(rng, (0, 100), (0, 100), n)
    demands = [0] + [rng.randint(10, 30) for _ in range(n)]
    tws = [(0, 259200)]  # 3-day depot
    for day in range(3):
        day_start = day * 86400
        for _ in range(5):
            lo = rng.randint(day_start, day_start + 28800)  # within first 8h
            tws.append((lo, lo + 14400))
    return _make_instance(
        name="multi-day-3day-shift-cap",
        comment="3-day horizon (259200s) with EU EC 561 9h shift caps",
        num_clients=n,
        num_vehicles=3,
        capacity=200,
        service_time=600,
        coords=coords,
        demands=demands,
        time_windows=tws,
    )


def _make_heterogeneous(seed: int = 46) -> str:
    """
    20 clients across 2 days. Designed for mixed fleet (half with breaks,
    half without). Clients spread across the full 172800s horizon.
    """
    rng = _seeded_rng(seed)
    n = 20
    coords = _generate_coords(rng, (0, 100), (0, 100), n)
    demands = [0] + [rng.randint(10, 30) for _ in range(n)]
    tws = [(0, 172800)]
    for i in range(n):
        lo = rng.randint(0, 158400)
        tws.append((lo, lo + 14400))
    return _make_instance(
        name="multi-day-heterogeneous",
        comment="2-day mixed fleet (half vehicles with breaks, half without)",
        num_clients=n,
        num_vehicles=6,  # 3 with breaks, 3 without (configured in code)
        capacity=200,
        service_time=600,
        coords=coords,
        demands=demands,
        time_windows=tws,
    )


# ---------------------------------------------------------------------------
# Registry & main
# ---------------------------------------------------------------------------

INSTANCES = {
    "multi_day_2day_lunch.vrp":         _make_2day_lunch,
    "multi_day_2day_overnight.vrp":     _make_2day_overnight,
    "multi_day_2day_multi_break.vrp":   _make_2day_multi_break,
    "multi_day_3day_shift_cap.vrp":     _make_3day_shift_cap,
    "multi_day_heterogeneous.vrp":      _make_heterogeneous,
}


def main(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    written = 0
    for fname, gen_func in INSTANCES.items():
        path = os.path.join(out_dir, fname)
        content = gen_func()
        with open(path, "w") as fh:
            fh.write(content)
        print(f"  wrote {path}  ({len(content)} bytes)")
        written += 1
    print(f"\nGenerated {written} instances in {out_dir}")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), "instances"
    )
    main(out)
