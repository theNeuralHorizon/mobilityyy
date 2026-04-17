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
    assert angular_z > 0.0
    assert active is True
