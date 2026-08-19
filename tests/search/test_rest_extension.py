"""
Overnight rest extension tests (OpenSpec task 2.3 / design D5).

Verifies:
  - An overnight (DUTY_TIME) rest extends to the next client's window open
    (11h -> 12h), with zero warp, zero idle, and the extended service exported.
  - When the next window is already open, the rest keeps the minimum service.
  - Lunch (CLOCK_TIME), DRIVE_TIME and WORK_TIME breaks are NOT extended.
  - A break followed by another break or a reload depot (not a client) is NOT
    extended.
  - Multiple overnights each extend.
"""
import numpy as np
import pytest
from numpy.testing import assert_, assert_equal

from pyvrp import (
    Activity,
    ActivityType,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Model,
)
from pyvrp.search._search import Node, Solution as SearchSolution

SVC = 600
TRAVEL = 1_000
REST = 39_600  # 11h
EXTENDED = 43_200  # 12h
OFFSET = 9_089  # vehicle.twEarly departure offset (clock normalisation)


def _overnight_chain(tw_early_c1, trigger=CustomBreakTrigger.DUTY_TIME):
    """
    depot -> C0 -> (break) -> C1 -> depot. The break arrives at 1600; the next
    client (C1) has its window open at ``tw_early_c1``. Fixed departure
    (start_late=0) makes any would-be waiting real (unavoidable).
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=tw_early_c1,
                 tw_late=300_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk = CustomBreak(
        id=1,
        trigger=trigger,
        trigger_value=1,
        reset=CustomBreakReset.ALL_TIMERS,
        mandatory=True,
        service=REST,
    )
    vt = base.vehicle_type(0).replace(custom_breaks=[brk], start_late=0)
    return base.replace(vehicle_types=[vt])


def _route_with_break(data):
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()
    return route


def _unloaded_break_services(data):
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()
    return sol.unload().routes()[0].break_services()


def test_rest_extends_to_next_window():
    """
    11h rest ends before the next client's window opens (06:00-style gap):
    the rest extends to land exactly on the window open (12h), with zero warp
    and zero idle waiting.
    """
    # Arrival at break = 1600; extended service = tw_early_c1 - travel - arrival.
    # For a 12h (43200) rest: tw_early_c1 = 1600 + 43200 + 1000 = 45800.
    data = _overnight_chain(tw_early_c1=45_800)

    route = _route_with_break(data)
    assert_equal(route.duration(), 1_600 + 43_200 + 1_000 + SVC + 1_000)
    assert_equal(route.waiting(), 0)  # absorbed into the rest
    assert_equal(route.time_warp(), 0)
    # Servido 600s após o first-due (trigger=1 cruzado no primeiro boundary)
    # — latência em SEGUNDOS (D3), não violação.
    assert_equal(route.break_due(), 600)
    assert_(route.is_feasible())

    services = _unloaded_break_services(data)
    assert_equal(services[1], EXTENDED)  # break service at activity index 1


def test_rest_minimum_when_window_open():
    """
    When the next client's window is already open at rest end, the rest keeps
    its minimum duration.
    """
    # Window open before rest end + travel (1600 + 39600 + 1000 = 42200).
    data = _overnight_chain(tw_early_c1=30_000)

    services = _unloaded_break_services(data)
    assert_equal(services[1], REST)  # minimum service, not extended


def test_lunch_break_not_extended():
    """
    A CLOCK_TIME (lunch) break is never extended, even when the next client's
    window opens after the lunch service + travel.
    """
    data = _overnight_chain(tw_early_c1=300_000,
                            trigger=CustomBreakTrigger.CLOCK_TIME)

    services = _unloaded_break_services(data)
    assert_equal(services[1], REST)


@pytest.mark.parametrize(
    "trigger",
    [CustomBreakTrigger.DRIVE_TIME, CustomBreakTrigger.WORK_TIME],
)
def test_drive_and_work_time_breaks_not_extended(trigger):
    """
    DRIVE_TIME and WORK_TIME breaks are served but never extended, even when
    the next client's window opens after the minimum rest + travel. Only
    DUTY_TIME breaks extend (D5).
    """
    # tw_early_c1 is chosen so the DUTY_TIME variant would extend (see
    # test_rest_extends_to_next_window); for DRIVE_TIME/WORK_TIME the minimum
    # rest must be preserved regardless.
    data = _overnight_chain(tw_early_c1=45_800, trigger=trigger)

    services = _unloaded_break_services(data)
    assert_equal(services[1], REST)


def test_next_activity_break_not_extended():
    """
    A served DUTY_TIME break followed by another break (not a client) is not
    extended: the window-open anchor only exists when the next activity is a
    client.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    for f in [depot, c0]:
        for t in [depot, c0]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk1 = CustomBreak(id=1, trigger=CustomBreakTrigger.DUTY_TIME,
                       trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                       mandatory=True, service=REST)
    brk2 = CustomBreak(id=2, trigger=CustomBreakTrigger.DUTY_TIME,
                       trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                       mandatory=True, service=REST)
    vt = base.vehicle_type(0).replace(custom_breaks=[brk1, brk2],
                                      start_late=0)
    data = base.replace(vehicle_types=[vt])

    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node(ActivityType.CUSTOM_BREAK, 2))
    route.update()

    services = sol.unload().routes()[0].break_services()
    # Break 1 is served but followed by break 2 (not a client): no extension.
    assert_equal(services[1], REST)
    # Break 2 is not due at its node (break 1 reset the ALL_TIMERS duty), so it
    # is not served and exports no service.
    assert_equal(services[2], 0)


def test_next_activity_reload_depot_not_extended():
    """
    A served DUTY_TIME break followed by a reload depot (not a client) is not
    extended.
    """
    m = Model()
    depot_loc = m.add_location(x=0, y=0, name="depot")
    depot = m.add_depot(depot_loc, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=45_800,
                 tw_late=300_000, required=True)
    for f in [depot_loc, c0, c1]:
        for t in [depot_loc, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999],
                       reload_depots=[depot], max_reloads=1)
    base = m.data()

    brk = CustomBreak(id=1, trigger=CustomBreakTrigger.DUTY_TIME,
                      trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                      mandatory=True, service=REST)
    vt = base.vehicle_type(0).replace(custom_breaks=[brk], start_late=0)
    data = base.replace(vehicle_types=[vt])

    # depot -> C0 -> break -> reload depot -> C1 -> depot. The break is
    # followed by the reload depot, so it must not extend even though C1's
    # window would otherwise allow an extension.
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node(Activity("D0")))
    route.append(Node("C1"))
    route.update()

    services = sol.unload().routes()[0].break_services()
    assert_equal(services[1], REST)


def test_multiple_overnights_extend():
    """
    Two overnight breaks in a multi-day route each extend to their next
    client's window open.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=45_800,
                 tw_late=300_000, required=True)
    c2 = m.add_location(x=3, y=0, name="C2")
    m.add_client(c2, delivery=0, service_duration=SVC, tw_early=90_600,
                 tw_late=300_000, required=True)
    locs = [depot, c0, c1, c2]
    for f in locs:
        for t in locs:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk1 = CustomBreak(id=1, trigger=CustomBreakTrigger.DUTY_TIME,
                       trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                       mandatory=True, service=REST)
    brk2 = CustomBreak(id=2, trigger=CustomBreakTrigger.DUTY_TIME,
                       trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                       mandatory=True, service=REST)
    vt = base.vehicle_type(0).replace(custom_breaks=[brk1, brk2], start_late=0)
    data = base.replace(vehicle_types=[vt])

    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 2))
    route.append(Node("C2"))
    route.update()

    assert_equal(route.waiting(), 0)
    assert_equal(route.time_warp(), 0)
    # Dois overnights servidos. firstDue[2] = 1000 (gravado no boundary do C0:
    # trigger=1 dispara no primeiro boundary, antes de qualquer break).
    # 46000 = 600 (service do C0) + 45400 (latência até o serviço do break 2),
    # sem relação com o reset do break 1. Latência em SEGUNDOS (D3).
    assert_equal(route.break_due(), 46_000)

    services = sol.unload().routes()[0].break_services()
    assert_equal(services[1], EXTENDED)  # first overnight
    assert_equal(services[3], EXTENDED)  # second overnight


def test_waiting_parity_multi_trip():
    """
    Parity: the search fold's ``waiting()`` must match the exported schedule's
    ``wait_duration()`` for a multi-trip route with waiting. The search layer
    uses ``waiting()`` for delta-cost evaluation; the exported ``pyvrp.Route``
    reports ``wait_duration()``. Both must agree.
    """
    m = Model()
    depot_loc = m.add_location(x=0, y=0, name="depot")
    depot = m.add_depot(depot_loc, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=SVC, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=10_000,
                 tw_late=300_000, required=True)
    for f in [depot_loc, c0, c1]:
        for t in [depot_loc, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=TRAVEL, duration=TRAVEL)
    m.add_vehicle_type(num_available=1, capacity=[9999],
                       reload_depots=[depot], max_reloads=1,
                       start_late=0)
    data = m.data()

    # depot -> C0 -> reload depot -> C1 -> depot. C1 opens at 10_000 while the
    # vehicle arrives at 3_600, forcing 6_400 units of waiting.
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(Activity("D0")))
    route.append(Node("C1"))
    route.update()

    exported = sol.unload().routes()[0]

    assert_equal(route.waiting(), 6_400)
    assert_equal(route.waiting(), exported.wait_duration())


def _offset_chain(tw_early_c1, offset=OFFSET, travel=TRAVEL, c0_svc=SVC):
    """
    depot -> C0 -> (break) -> C1 -> depot, with the vehicle departing at
    ``twEarly=offset`` (fixed ``start_late=offset``). The break (DUTY_TIME,
    min REST) is served at C0's location; C1's window open is ``tw_early_c1``
    in the ABSOLUTE clock. The forward pass uses the search clock anchored at
    ``vehicle.twEarly``, so the D5 extension must normalise C1's absolute
    ``twEarly`` by subtracting the departure offset before comparing.
    """
    m = Model()
    depot = m.add_location(x=0, y=0, name="depot")
    m.add_depot(depot, tw_early=0, tw_late=300_000)
    c0 = m.add_location(x=1, y=0, name="C0")
    m.add_client(c0, delivery=0, service_duration=c0_svc, tw_early=0,
                 tw_late=300_000, required=True)
    c1 = m.add_location(x=2, y=0, name="C1")
    m.add_client(c1, delivery=0, service_duration=SVC, tw_early=tw_early_c1,
                 tw_late=300_000, required=True)
    for f in [depot, c0, c1]:
        for t in [depot, c0, c1]:
            if f is t:
                m.add_edge(f, t, distance=0, duration=0)
            else:
                m.add_edge(f, t, distance=travel, duration=travel)
    m.add_vehicle_type(num_available=1, capacity=[9999])
    base = m.data()

    brk = CustomBreak(id=1, trigger=CustomBreakTrigger.DUTY_TIME,
                      trigger_value=1, reset=CustomBreakReset.ALL_TIMERS,
                      mandatory=True, service=REST)
    vt = base.vehicle_type(0).replace(custom_breaks=[brk], tw_early=offset,
                                      start_late=offset)
    return base.replace(vehicle_types=[vt])


def test_rest_extension_normalises_departure_offset():
    """
    With a non-zero departure offset (vehicle.twEarly > 0), a window that opens
    after rest end + travel (in the ABSOLUTE clock) extends exactly to the
    window open — not offset-slipped. Break arrival (absolute) = 9089 + 1000 +
    600 = 10689; restEnd (absolute) = 50289; + travel = 51289. A window opening
    at 60000 extends the rest to 60000 - 1000 - 10689 = 48311.
    """
    arrival_abs = OFFSET + TRAVEL + SVC  # 10689
    expected = 60_000 - TRAVEL - arrival_abs  # 48311

    data = _offset_chain(tw_early_c1=60_000)
    services = _unloaded_break_services(data)
    assert_equal(services[1], expected)

    # The break must land C1 exactly on its window open (absolute clock).
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()
    assert_equal(route.time_warp(), 0)
    assert_equal(route.waiting(), 0)

    exported = sol.unload().routes()[0]
    schedule = exported.schedule()
    # schedule order: DEPOT, C0, BREAK, C1, DEPOT
    assert_equal(schedule[3].start_time, 60_000)  # C1 arrives at window open


def test_rest_not_overextended_when_window_open_before_rest_end():
    """
    Regression (O5-1): when the next client's window opens BEFORE the minimum
    rest ends + travel (absolute clock) but AFTER the search-clock rest end
    (because the departure offset underestimates restEnd), the rest must NOT be
    extended. Without the clock normalisation this over-extended by the offset.
    """
    # Absolute break arrival = 10689; restEnd_abs + travel = 51289.
    # restEnd_rel + travel = 42200. A window at 45000 sits in the buggy gap
    # (between the two), so no extension may fire.
    data = _offset_chain(tw_early_c1=45_000)

    services = _unloaded_break_services(data)
    assert_equal(services[1], REST)  # minimum service, not over-extended


def test_offset_extension_parity_with_schedule():
    """
    Parity: for an offset (twEarly > 0) extension, the search fold's time warp
    and waiting match the exported schedule's time_warp and wait_duration, and
    the exported break service equals the fold's extended service.
    """
    arrival_abs = OFFSET + TRAVEL + SVC
    expected = 60_000 - TRAVEL - arrival_abs

    data = _offset_chain(tw_early_c1=60_000)
    sol = SearchSolution(data)
    route = sol.routes[0]
    route.append(Node("C0"))
    route.append(Node(ActivityType.CUSTOM_BREAK, 1))
    route.append(Node("C1"))
    route.update()

    exported = sol.unload().routes()[0]

    assert_equal(route.time_warp(), 0)
    assert_equal(route.time_warp(), exported.time_warp())
    assert_equal(route.waiting(), 0)
    assert_equal(route.waiting(), exported.wait_duration())
    assert_equal(list(exported.break_services())[1], expected)
