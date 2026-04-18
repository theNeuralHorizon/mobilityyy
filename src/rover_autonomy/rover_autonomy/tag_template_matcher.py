from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def _normalize_patch(image: np.ndarray, size: int = 128) -> np.ndarray:
    resized = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
    if len(resized.shape) == 3:
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    normalized = resized.astype(np.float32) / 255.0
    return normalized


def load_tag_templates(template_dir: Path) -> dict[int, np.ndarray]:
    templates: dict[int, np.ndarray] = {}
    for tag_id in range(5):
        path = template_dir / f"tag_{tag_id}_padded.png"
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        templates[tag_id] = _normalize_patch(image)
    return templates


def match_tag_patch(patch: np.ndarray, templates: dict[int, np.ndarray]) -> tuple[int | None, float]:
    if patch is None or len(templates) == 0:
        return None, float("inf")
    normalized_patch = _normalize_patch(patch)
    best_tag_id: int | None = None
    best_score = float("inf")
    for tag_id, template in templates.items():
        for rotation in range(4):
            rotated = np.rot90(template, rotation)
            score = float(np.mean(np.abs(normalized_patch - rotated)))
            if score < best_score:
                best_score = score
                best_tag_id = int(tag_id)
    return best_tag_id, best_score
