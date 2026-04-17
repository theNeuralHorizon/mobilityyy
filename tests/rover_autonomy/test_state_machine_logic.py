import math

from rover_autonomy.state_machine_controller import (
    MissionPhase,
    advance_phase_on_tag,
    compute_follow_command,
    compute_wall_follow_command,
    _normalize_angle,
)


# --- advance_phase_on_tag: full happy-path sequence ---

def test_full_happy_path_sequence():
    """START -> TAG2 -> TAG1 -> RETURN_TAG2 -> TAG3 -> TAG4 -> TAG5"""
    phase = MissionPhase.STATE_START
    phase = advance_phase_on_tag(phase, mission_label=2, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_TAG2_FOUND

    phase = advance_phase_on_tag(phase, mission_label=1, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_TAG1_DEAD_END

    phase = advance_phase_on_tag(phase, mission_label=2, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_RETURN_TAG2

    phase = advance_phase_on_tag(phase, mission_label=3, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_TAG3_GREEN_PATH

    phase = advance_phase_on_tag(phase, mission_label=4, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_TAG4_UTURN

    phase = advance_phase_on_tag(phase, mission_label=5, skip_tag1_return=False)
    assert phase is MissionPhase.STATE_TAG5_ORANGE_PATH


def test_state_machine_uses_legal_return_path_when_skip_disabled():
    phase = MissionPhase.STATE_TAG2_FOUND
    next_phase = advance_phase_on_tag(phase, mission_label=1, skip_tag1_return=False)
    assert next_phase is MissionPhase.STATE_TAG1_DEAD_END


def test_state_machine_skips_return_when_override_enabled():
    phase = MissionPhase.STATE_TAG1_DEAD_END
    next_phase = advance_phase_on_tag(phase, mission_label=3, skip_tag1_return=True)
    assert next_phase is MissionPhase.STATE_TAG3_GREEN_PATH


# --- Invalid tag in wrong state ---

def test_invalid_tag_in_wrong_state_is_noop():
    """Seeing Tag 5 during STATE_START should not change state."""
    phase = MissionPhase.STATE_START
    result = advance_phase_on_tag(phase, mission_label=5, skip_tag1_return=False)
    assert result is MissionPhase.STATE_START


def test_seeing_tag1_during_start_is_noop():
    phase = MissionPhase.STATE_START
    result = advance_phase_on_tag(phase, mission_label=1, skip_tag1_return=False)
    assert result is MissionPhase.STATE_START


def test_seeing_tag4_during_tag2_found_is_noop():
    phase = MissionPhase.STATE_TAG2_FOUND
    result = advance_phase_on_tag(phase, mission_label=4, skip_tag1_return=False)
    assert result is MissionPhase.STATE_TAG2_FOUND


# --- Skip override with non-tag3 ---

def test_skip_override_with_non_tag3_stays_in_dead_end():
    phase = MissionPhase.STATE_TAG1_DEAD_END
    result = advance_phase_on_tag(phase, mission_label=2, skip_tag1_return=True)
    assert result is MissionPhase.STATE_TAG1_DEAD_END


# --- compute_follow_command ---

def test_follow_command_spins_when_no_target():
    linear, angular = compute_follow_command(target_detected=False, yaw_error=0.0, linear_speed=0.14)
    assert linear == 0.0
    assert angular == 0.35


def test_follow_command_drives_forward_when_target_centered():
    linear, angular = compute_follow_command(target_detected=True, yaw_error=0.0, linear_speed=0.14)
    assert linear == 0.14
    assert angular == 0.0


def test_follow_command_steers_for_yaw_error():
    linear, angular = compute_follow_command(target_detected=True, yaw_error=0.5, linear_speed=0.14)
    assert linear == 0.14
    assert angular < 0.0  # steers opposite to error


# --- compute_wall_follow_command ---

def test_wall_follow_turns_left_when_front_blocked():
    linear, angular = compute_wall_follow_command(
        front_min=0.2, left_min=1.0, right_min=1.0,
        linear_speed=0.18, wall_follow_distance=0.35,
    )
    assert linear < 0.18
    assert angular > 0.0  # turn left


def test_wall_follow_goes_straight_with_right_wall():
    linear, angular = compute_wall_follow_command(
        front_min=2.0, left_min=2.0, right_min=0.35,
        linear_speed=0.18, wall_follow_distance=0.35,
    )
    assert linear == 0.18
    assert angular == 0.0


def test_wall_follow_turns_right_when_right_open():
    linear, angular = compute_wall_follow_command(
        front_min=2.0, left_min=2.0, right_min=2.0,
        linear_speed=0.18, wall_follow_distance=0.35,
    )
    assert angular < 0.0  # turn right to seek wall


def test_wall_follow_veers_left_when_right_too_close():
    linear, angular = compute_wall_follow_command(
        front_min=2.0, left_min=2.0, right_min=0.15,
        linear_speed=0.18, wall_follow_distance=0.35,
    )
    assert angular > 0.0  # veer left


# --- _normalize_angle ---

def test_normalize_angle_positive():
    assert abs(_normalize_angle(3 * math.pi) - math.pi) < 0.01


def test_normalize_angle_negative():
    assert abs(_normalize_angle(-3 * math.pi) - (-math.pi)) < 0.01


def test_normalize_angle_within_range():
    assert _normalize_angle(1.0) == 1.0
