from rover_autonomy.state_machine_controller import MissionPhase, advance_phase_on_tag


def test_state_machine_uses_legal_return_path_when_skip_disabled():
    phase = MissionPhase.STATE_TAG2_FOUND

    next_phase = advance_phase_on_tag(phase, mission_label=1, skip_tag1_return=False)

    assert next_phase is MissionPhase.STATE_TAG1_DEAD_END


def test_state_machine_skips_return_when_override_enabled():
    phase = MissionPhase.STATE_TAG1_DEAD_END

    next_phase = advance_phase_on_tag(phase, mission_label=3, skip_tag1_return=True)

    assert next_phase is MissionPhase.STATE_TAG3_GREEN_PATH
