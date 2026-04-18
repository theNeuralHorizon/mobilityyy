from rover_autonomy.safety_controller import apply_safety_override, should_keep_reverse_escape_latched
from rover_autonomy.hsv_utils import detect_stop_zone

import numpy as np


def test_stops_turning_into_left_wall_with_reverse_escape():
    linear_x, angular_z, active, reason = apply_safety_override(
        requested_linear_x=0.12,
        requested_angular_z=0.40,
        front_min_m=0.35,
        front_left_min_m=0.16,
        front_right_min_m=0.40,
        left_min_m=0.20,
        right_min_m=0.60,
        hard_stop_distance_m=0.22,
    )

    assert active is True
    assert linear_x < 0.0
    assert reason == "reverse_escape"
    assert angular_z < 0.0


def test_detect_stop_zone_requires_red_and_orange_presence():
    red_mask = np.zeros((80, 80), dtype=np.uint8)
    orange_mask = np.zeros((80, 80), dtype=np.uint8)
    red_mask[:, :] = 255
    orange_mask[10:50, 10:50] = 255

    assert detect_stop_zone(red_mask, orange_mask) is True

    orange_mask[:, :] = 0
    assert detect_stop_zone(red_mask, orange_mask) is False


def test_reverse_escape_latch_holds_until_timeout():
    assert should_keep_reverse_escape_latched(now=11.0, reverse_escape_until=11.4) is True
    assert should_keep_reverse_escape_latched(now=11.5, reverse_escape_until=11.4) is False
