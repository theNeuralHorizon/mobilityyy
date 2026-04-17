from __future__ import annotations

import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def _as_points(contour: np.ndarray) -> np.ndarray:
    points = contour.reshape(-1, 2).astype(np.float32)
    if len(points) < 3:
        raise ValueError("Contour must contain at least 3 points")
    return points


def estimate_arrow_yaw_error(contour: np.ndarray, image_width: int) -> float:
    points = _as_points(contour)
    centroid = points.mean(axis=0)
    distances = np.linalg.norm(points - centroid, axis=1)
    tip = points[np.argmax(distances)]
    image_center_x = image_width / 2.0
    normalized = float((tip[0] - image_center_x) / max(image_width / 2.0, 1.0))
    normalized = max(-1.0, min(1.0, normalized))
    return normalized * (math.pi / 4.0)


def find_largest_arrow_contour(mask: np.ndarray) -> np.ndarray | None:
    if cv2 is None:
        raise RuntimeError("OpenCV is required for contour extraction")
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)
