from rover_autonomy.safety_controller import apply_safety_override


def test_safety_override_stops_forward_motion_when_front_is_too_close():
    linear_x, angular_z, active = apply_safety_override(
        requested_linear_x=0.3,
        requested_angular_z=0.0,
        front_min_m=0.15,
        left_min_m=0.6,
        right_min_m=0.4,
        hard_stop_distance_m=0.2,
    )
    assert linear_x == 0.0
    assert angular_z > 0.0  # turns toward more-open left side
    assert active is True


def test_safety_override_passes_through_when_front_clear():
    linear_x, angular_z, active = apply_safety_override(
        requested_linear_x=0.2,
        requested_angular_z=0.1,
        front_min_m=1.5,
        left_min_m=0.8,
        right_min_m=0.6,
        hard_stop_distance_m=0.2,
    )
    assert linear_x == 0.2
    assert angular_z == 0.1
    assert active is False


def test_safety_override_turns_right_when_right_more_open():
    linear_x, angular_z, active = apply_safety_override(
        requested_linear_x=0.3,
        requested_angular_z=0.0,
        front_min_m=0.15,
        left_min_m=0.3,
        right_min_m=0.8,
        hard_stop_distance_m=0.2,
    )
    assert linear_x == 0.0
    assert angular_z < 0.0  # turns toward more-open right side
    assert active is True


def test_safety_override_ignores_zero_front():
    """front_min_m=0.0 should NOT trigger override (sensor artifact)."""
    linear_x, angular_z, active = apply_safety_override(
        requested_linear_x=0.3,
        requested_angular_z=0.0,
        front_min_m=0.0,
        left_min_m=0.5,
        right_min_m=0.5,
        hard_stop_distance_m=0.2,
    )
    assert linear_x == 0.3
    assert active is False


def test_safety_override_at_exact_threshold():
    """front_min_m == hard_stop_distance_m should NOT trigger (only <)."""
    linear_x, angular_z, active = apply_safety_override(
        requested_linear_x=0.3,
        requested_angular_z=0.0,
        front_min_m=0.2,
        left_min_m=0.5,
        right_min_m=0.5,
        hard_stop_distance_m=0.2,
    )
    assert linear_x == 0.3
    assert active is False
