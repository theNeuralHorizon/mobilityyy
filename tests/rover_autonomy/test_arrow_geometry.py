import math

import numpy as np

from rover_autonomy.arrow_geometry import estimate_arrow_yaw_error


def test_arrow_geometry_returns_positive_yaw_for_rightward_tip():
    contour = np.array(
        [[[20, 50]], [[80, 20]], [[140, 50]], [[95, 50]], [[95, 90]], [[45, 90]], [[45, 50]]],
        dtype=np.int32,
    )

    yaw_error = estimate_arrow_yaw_error(contour, image_width=160)

    assert yaw_error > 0.0
    assert yaw_error < math.pi / 2
