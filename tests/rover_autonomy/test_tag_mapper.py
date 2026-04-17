from rover_autonomy.tag_mapper import TagMapper


def test_tag_mapper_maps_ids_to_labels_and_decisions():
    mapper = TagMapper({12: 2, 4: 1, 18: 3, 7: 4, 22: 5})

    event = mapper.build_event(tag_id=18, distance_m=0.9, bearing_rad=0.1)

    assert event["mission_label"] == 3
    assert event["decision_taken"] == "TRACK_GREEN"
    assert event["tag_id"] == 18
