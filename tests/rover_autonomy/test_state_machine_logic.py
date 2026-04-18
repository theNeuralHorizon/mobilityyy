from rover_autonomy.state_machine_controller import (
    ControlState,
    final_goal_commit_command,
    MissionPhase,
    net_progress_distance,
    route_turn_steps,
    should_force_return_tag2_timed_assist,
    should_force_tag1_timed_assist,
    should_trigger_stuck_recovery,
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
    assert steps[0][1] > 0.0
    assert steps[0][2] == 0.0


def test_return_tag2_phase_uses_forward_arc_to_reach_tag3_branch():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_RETURN_TAG2)

    assert steps[0][0] > 0.0
    assert steps[0][1] > 0.0
    assert steps[0][2] > 0.0


def test_tag4_phase_pushes_forward_after_u_turn():
    steps = phase_entry_maneuver_steps(MissionPhase.STATE_TAG4_UTURN)

    assert steps[0][2] > 0.0
    assert len(steps) == 1


def test_final_goal_has_commit_step_into_end_zone():
    steps = route_turn_steps(MissionPhase.STATE_FINAL_GOAL, 1)

    assert len(steps) == 1
    assert steps[0][1] > 0.0
    assert steps[0][2] == 0.0


def test_timed_assists_keep_early_route_progressing():
    assert should_force_tag1_timed_assist(MissionPhase.STATE_TAG2_FOUND, 1, 8.1) is True
    assert should_force_tag1_timed_assist(MissionPhase.STATE_TAG2_FOUND, 0, 8.1) is False
    assert should_force_return_tag2_timed_assist(MissionPhase.STATE_TAG1_DEAD_END, 7.1) is True
    assert should_force_return_tag2_timed_assist(MissionPhase.STATE_RETURN_TAG2, 7.1) is False


def test_final_goal_commit_stops_only_at_finish_wall():
    linear_x, angular_z, should_stop = final_goal_commit_command(front=0.45, left=0.60, right=0.30)

    assert should_stop is False
    assert linear_x > 0.0
    assert angular_z > 0.0

    _, _, should_stop = final_goal_commit_command(front=0.18, left=0.50, right=0.30)
    assert should_stop is True


def test_scan_only_starts_in_start_or_return_phases():
    assert should_start_scan(
        MissionPhase.STATE_START,
        {"front": 0.9, "right": 1.3, "fright": 1.1, "left": 0.4, "fleft": 0.4},
        seconds_since_last_tag=7.0,
        seconds_since_last_scan=7.0,
    ) is True
    assert should_start_scan(
        MissionPhase.STATE_RETURN_TAG2,
        {"front": 0.9, "right": 1.3, "fright": 1.1, "left": 0.4, "fleft": 0.4},
        seconds_since_last_tag=7.0,
        seconds_since_last_scan=7.0,
    ) is False


def test_stuck_recovery_triggers_after_15_seconds_without_progress():
    state = ControlState(phase=MissionPhase.STATE_TAG4_UTURN)

    assert should_trigger_stuck_recovery(
        state,
        seconds_since_progress=5.1,
        currently_recovering=False,
        timeout_s=4.9,
    ) is True
    assert should_trigger_stuck_recovery(
        state,
        seconds_since_progress=4.0,
        currently_recovering=False,
        timeout_s=4.9,
    ) is False


def test_net_progress_distance_ignores_wiggle_near_anchor():
    assert net_progress_distance(0.0, 0.0, 0.03, -0.02) < 0.05
    assert net_progress_distance(0.0, 0.0, -0.04, 0.01) < 0.05


def test_net_progress_distance_counts_real_escape_motion():
    assert net_progress_distance(0.0, 0.0, -0.14, 0.03) > 0.12
