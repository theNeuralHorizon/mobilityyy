from rover_autonomy.tag_mapper import TagMapper


def test_tag_mapper_maps_ids_to_labels_and_decisions():
    mapper = TagMapper({12: 2, 4: 1, 18: 3, 7: 4, 22: 5})

    event = mapper.build_event(tag_id=18, distance_m=0.9, bearing_rad=0.1)

    assert event is not None
    assert event["mission_label"] == 3
    assert event["decision_taken"] == "TRACK_GREEN"
    assert event["tag_id"] == 18


def test_tag_mapper_returns_none_for_unknown_id():
    mapper = TagMapper({1: 1, 2: 2})

    assert mapper.mission_label_for(99) is None
    assert mapper.build_event(tag_id=99, distance_m=1.0, bearing_rad=0.0) is None


def test_tag_mapper_returns_none_for_unmapped_label():
    mapper = TagMapper({10: 99})  # label 99 has no decision

    assert mapper.decision_for(99) is None
    assert mapper.build_event(tag_id=10, distance_m=0.5, bearing_rad=0.1) is None


def test_tag_mapper_all_five_labels():
    mapper = TagMapper({0: 1, 1: 2, 2: 3, 3: 4, 4: 5})

    decisions = [mapper.decision_for(label) for label in range(1, 6)]
    assert decisions == [
        "U_TURN_TO_TAG2",
        "TURN_TO_TAG1",
        "TRACK_GREEN",
        "U_TURN_TO_TAG5",
        "TRACK_ORANGE",
    ]


def test_tag_mapper_build_event_fields():
    mapper = TagMapper({5: 4})

    event = mapper.build_event(tag_id=5, distance_m=1.23, bearing_rad=-0.5)

    assert event is not None
    assert event["tag_id"] == 5
    assert event["mission_label"] == 4
    assert event["decision_taken"] == "U_TURN_TO_TAG5"
    assert abs(event["distance_m"] - 1.23) < 1e-6
    assert abs(event["bearing_rad"] - (-0.5)) < 1e-6
