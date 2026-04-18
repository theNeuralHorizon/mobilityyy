from rover_autonomy.state_machine_controller import (
    MissionPhase,
    phase_entry_maneuver_steps,
    should_start_scan,
)


def test_phase_entry_steps_include_rotation_then_progress():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_TAG2_FOUND)

    assert steps[0][0] > 0.0
    assert steps[0][1] == 0.0
    assert steps[0][2] < 0.0
    assert any(step[1] > 0.0 for step in steps)


def test_tag3_phase_starts_with_u_turn():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_TAG3_GREEN_PATH)

    assert steps[0][0] > 0.0
    assert steps[0][1] == 0.0
    assert steps[0][2] > 0.0
    assert steps[1][1] > 0.0


def test_return_tag2_phase_uses_forward_arc_to_reach_tag3_branch():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_RETURN_TAG2)

    assert steps[0][0] > 0.0
    assert steps[0][1] > 0.0
    assert steps[0][2] > 0.0


def test_tag4_phase_pushes_forward_after_u_turn():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_TAG4_UTURN)

    assert steps[0][2] > 0.0
    assert steps[1][1] >= 0.15


def test_scan_only_starts_in_start_or_return_phases():
    assert should_start_scan(
        MissionPhase.STATE_START,
        {"front": 0.9, "right": 1.3, "fright": 1.1, "left": 0.4, "fleft": 0.4},
        seconds_since_last_tag=7.0,
        seconds_since_last_scan=7.0,
    ) is True
    assert should_start_scan(
        MissionPhase.STATE_TAG2_FOUND,
        {"front": 0.9, "right": 1.3, "fright": 1.1, "left": 0.4, "fleft": 0.4},
        seconds_since_last_tag=7.0,
        seconds_since_last_scan=7.0,
    ) is False
