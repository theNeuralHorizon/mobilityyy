from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


DECISION_BY_LABEL = {
    1: "TURN_LEFT_BACKTRACK",
    2: "TURN_RIGHT_TO_TAG1",
    3: "START_GREEN",
    4: "U_TURN",
    5: "START_ORANGE",
}


@dataclass(frozen=True)
class TagMapper:
    tag_id_to_label: Dict[int, int]

    def mission_label_for(self, tag_id: int) -> int:
        return int(self.tag_id_to_label[tag_id])

    def decision_for(self, mission_label: int) -> str:
        return DECISION_BY_LABEL[int(mission_label)]

    def build_event(self, tag_id: int, distance_m: float, bearing_rad: float) -> dict:
        mission_label = self.mission_label_for(tag_id)
        return {
            "tag_id": int(tag_id),
            "mission_label": mission_label,
            "decision_taken": self.decision_for(mission_label),
            "distance_m": float(distance_m),
            "bearing_rad": float(bearing_rad),
        }
