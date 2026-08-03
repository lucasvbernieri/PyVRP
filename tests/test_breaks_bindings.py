import pickle

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal, assert_raises

from pyvrp import (
    ActivityType,
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    VehicleType,
)

_INT_MAX = np.iinfo(np.int64).max


# =============================================================================
# CustomBreak tests
# =============================================================================


def test_custom_break_constructor_defaults():
    """
    Tests that a CustomBreak can be constructed with just an id, and all
    other fields have their expected defaults.
    """
    cb = CustomBreak(id=1)

    assert_equal(cb.id, 1)
    assert_equal(cb.tws, [])
    assert_equal(cb.service, 0)
    assert_equal(cb.trigger, CustomBreakTrigger.CLOCK_TIME)
    assert_equal(cb.trigger_value, 0)
    assert_equal(cb.reset, CustomBreakReset.NONE)
    assert_equal(cb.mandatory, False)
    assert_equal(cb.condition_min_route_s, 0)
    assert_equal(cb.priority, 0)
    assert_equal(cb.supersedes, [])


def test_custom_break_all_fields():
    """
    Tests that all CustomBreak fields are set correctly when provided.
    """
    cb = CustomBreak(
        id=2,
        tws=[(100, 200), (300, 400)],
        service=15,
        trigger=CustomBreakTrigger.DRIVE_TIME,
        trigger_value=240,
        reset=CustomBreakReset.DRIVE_TIMER,
        mandatory=True,
        condition_min_route_s=3600,
        priority=5,
        supersedes=[1, 2],
    )

    assert_equal(cb.id, 2)
    assert_equal(cb.tws, [(100, 200), (300, 400)])
    assert_equal(cb.service, 15)
    assert_equal(cb.trigger, CustomBreakTrigger.DRIVE_TIME)
    assert_equal(cb.trigger_value, 240)
    assert_equal(cb.reset, CustomBreakReset.DRIVE_TIMER)
    assert_equal(cb.mandatory, True)
    assert_equal(cb.condition_min_route_s, 3600)
    assert_equal(cb.priority, 5)
    assert_equal(cb.supersedes, [1, 2])


def test_custom_break_is_valid_start():
    """
    Tests the isValidStart method of CustomBreak.
    """
    cb = CustomBreak(id=3, tws=[(100, 200), (300, 400)])

    # Inside first window
    assert_(cb.is_valid_start(100))
    assert_(cb.is_valid_start(150))
    assert_(cb.is_valid_start(199))

    # Inside second window
    assert_(cb.is_valid_start(300))
    assert_(cb.is_valid_start(350))

    # Outside all windows
    assert_(not cb.is_valid_start(0))
    assert_(not cb.is_valid_start(200))
    assert_(not cb.is_valid_start(250))
    assert_(not cb.is_valid_start(400))
    assert_(not cb.is_valid_start(500))

    # Empty tws: nothing is valid
    cb_empty = CustomBreak(id=4)
    assert_(not cb_empty.is_valid_start(100))


def test_custom_break_trigger_enum():
    """
    Tests the CustomBreakTrigger enum values.
    """
    assert_equal(int(CustomBreakTrigger.CLOCK_TIME), 0)
    assert_equal(int(CustomBreakTrigger.DRIVE_TIME), 1)
    assert_equal(int(CustomBreakTrigger.WORK_TIME), 2)
    assert_equal(int(CustomBreakTrigger.DUTY_TIME), 3)


def test_custom_break_reset_enum():
    """
    Tests the CustomBreakReset enum values.
    """
    assert_equal(int(CustomBreakReset.NONE), 0)
    assert_equal(int(CustomBreakReset.DRIVE_TIMER), 1)
    assert_equal(int(CustomBreakReset.WORK_TIMER), 2)
    assert_equal(int(CustomBreakReset.DRIVE_AND_WORK), 3)
    assert_equal(int(CustomBreakReset.ALL_TIMERS), 4)


# =============================================================================
# VehicleType breaks tests
# =============================================================================


def test_vehicle_type_no_breaks_by_default():
    """
    Tests that a VehicleType without breaks has has_breaks() == False
    and empty custom_breaks list.
    """
    vt = VehicleType()
    assert_(not vt.has_breaks())
    assert_equal(vt.custom_breaks, [])
    assert_equal(vt.reset_breaks_at_reload, False)


def test_vehicle_type_with_breaks():
    """
    Tests that a VehicleType with breaks reports them correctly.
    Breaks are sorted by descending priority in C++.
    """
    b1 = CustomBreak(id=1, priority=10)
    b2 = CustomBreak(id=2, priority=5)
    b3 = CustomBreak(id=3, priority=5)  # same priority as b2

    vt = VehicleType(
        custom_breaks=[b1, b2, b3],
        reset_breaks_at_reload=True,
    )

    assert_(vt.has_breaks())
    assert_equal(vt.reset_breaks_at_reload, True)

    # Breaks are sorted by descending priority; ties preserve original order
    breaks = vt.custom_breaks
    assert_equal(len(breaks), 3)
    assert_equal(breaks[0].id, 1)  # priority 10, highest first
    # b2 and b3 have equal priority, order preserved
    assert_equal(breaks[1].id, 2)
    assert_equal(breaks[2].id, 3)


def test_vehicle_type_replace_breaks():
    """
    Tests that replace() can preserve and override breaks.
    """
    b1 = CustomBreak(id=1, priority=10)
    vt = VehicleType(custom_breaks=[b1], reset_breaks_at_reload=False)

    # replace() with nothing should keep existing breaks
    vt2 = vt.replace()
    assert_(vt2.has_breaks())
    assert_equal(len(vt2.custom_breaks), 1)
    assert_equal(vt2.custom_breaks[0].id, 1)
    assert_equal(vt2.reset_breaks_at_reload, False)

    # replace() with new breaks should override
    # NOTE: break.id must be < 16 (16-bit breaksTakenMask_).
    b2 = CustomBreak(id=14, priority=1)
    vt3 = vt.replace(custom_breaks=[b2], reset_breaks_at_reload=True)
    assert_(vt3.has_breaks())
    assert_equal(len(vt3.custom_breaks), 1)
    assert_equal(vt3.custom_breaks[0].id, 14)
    assert_equal(vt3.reset_breaks_at_reload, True)

    # replace() with empty list should clear breaks (since empty != None)
    vt4 = vt.replace(custom_breaks=[])
    assert_(not vt4.has_breaks())
    assert_equal(vt4.custom_breaks, [])


# =============================================================================
# Backward compatibility tests
# =============================================================================


def test_backward_compat_default_vehicle_type():
    """
    Tests that a VehicleType constructed with only original args behaves
    identically to before (no break fields, has_breaks() == False).
    """
    vt = VehicleType(
        num_available=7,
        start_depot=29,
        end_depot=43,
        capacity=[13],
        fixed_cost=3,
        tw_early=17,
        tw_late=19,
        shift_duration=23,
        max_distance=31,
        unit_distance_cost=37,
        unit_duration_cost=41,
        start_late=18,
        max_overtime=43,
        name="vehicle_type name",
    )

    assert_equal(vt.num_available, 7)
    assert_equal(vt.start_depot, 29)
    assert_equal(vt.end_depot, 43)
    assert_equal(vt.capacity, [13])
    assert_equal(vt.fixed_cost, 3)
    assert_equal(vt.tw_early, 17)
    assert_equal(vt.tw_late, 19)
    assert_equal(vt.shift_duration, 23)
    assert_equal(vt.max_distance, 31)
    assert_equal(vt.unit_distance_cost, 37)
    assert_equal(vt.unit_duration_cost, 41)
    assert_equal(vt.start_late, 18)
    assert_equal(vt.max_overtime, 43)
    assert_equal(vt.name, "vehicle_type name")

    # Break fields are at defaults
    assert_equal(vt.custom_breaks, [])
    assert_equal(vt.reset_breaks_at_reload, False)
    assert_(not vt.has_breaks())


def test_backward_compat_pickle():
    """
    Tests that a VehicleType with breaks can be pickled and unpickled.
    """
    b1 = CustomBreak(id=10, priority=10, trigger_value=120)
    before = VehicleType(
        num_available=5,
        capacity=[3],
        name="pickle-test",
        custom_breaks=[b1],
        reset_breaks_at_reload=True,
    )

    data = pickle.dumps(before)
    after = pickle.loads(data)

    assert_equal(after.num_available, 5)
    assert_equal(after.capacity, [3])
    assert_equal(after.name, "pickle-test")
    assert_(after.has_breaks())
    assert_equal(len(after.custom_breaks), 1)
    assert_equal(after.custom_breaks[0].id, 10)
    assert_equal(after.custom_breaks[0].trigger_value, 120)
    assert_equal(after.reset_breaks_at_reload, True)


# =============================================================================
# ActivityType tests
# =============================================================================


def test_activity_type_custom_break():
    """
    Tests that ActivityType.CUSTOM_BREAK exists and equals 100.
    """
    assert_equal(int(ActivityType.CUSTOM_BREAK), 100)
    assert_(ActivityType.CUSTOM_BREAK != ActivityType.CLIENT)
    assert_(ActivityType.CUSTOM_BREAK != ActivityType.DEPOT)


def test_activity_custom_break():
    """
    Tests that an Activity can be created with CUSTOM_BREAK type.
    """
    from pyvrp import Activity

    act = Activity(ActivityType.CUSTOM_BREAK, 0)
    assert_equal(act.type, ActivityType.CUSTOM_BREAK)
    assert_equal(act.idx, 0)
    assert_(not act.is_client())
    assert_(not act.is_depot())
    # is_custom_break() is not yet bound in this task set, skip
