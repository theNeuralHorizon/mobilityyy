import math

import numpy as np

from rover_autonomy.arrow_geometry import estimate_mask_yaw_error


def test_mask_yaw_error_is_positive_for_right_biased_mask():
    mask = np.zeros((80, 160), dtype=np.uint8)
    mask[:, 100:140] = 255

    yaw_error = estimate_mask_yaw_error(mask, image_width=160)

    assert 0.0 < yaw_error < math.pi / 2
