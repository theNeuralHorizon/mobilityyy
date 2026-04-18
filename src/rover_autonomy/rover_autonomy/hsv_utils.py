from __future__ import annotations

from typing import Iterable, Tuple

import cv2
import numpy as np


def make_mask(hsv_image: np.ndarray, low: Iterable[int], high: Iterable[int]) -> np.ndarray:
    mask = cv2.inRange(
        hsv_image,
        np.array(list(low), dtype=np.uint8),
        np.array(list(high), dtype=np.uint8),
    )
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def lower_roi(image: np.ndarray, top_fraction: float = 0.35) -> Tuple[np.ndarray, int]:
    start_row = int(image.shape[0] * top_fraction)
    return image[start_row:, :], start_row
