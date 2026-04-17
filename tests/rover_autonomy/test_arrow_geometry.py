import math

import numpy as np
import pytest

from rover_autonomy.arrow_geometry import estimate_arrow_yaw_error, find_largest_arrow_contour


def test_arrow_geometry_returns_positive_yaw_for_rightward_tip():
    contour = np.array(
        [[[20, 50]], [[80, 20]], [[140, 50]], [[95, 50]], [[95, 90]], [[45, 90]], [[45, 50]]],
        dtype=np.int32,
    )

    yaw_error = estimate_arrow_yaw_error(contour, image_width=160)

    assert yaw_error > 0.0
    assert yaw_error < math.pi / 2


def test_arrow_geometry_returns_negative_yaw_for_leftward_tip():
    # Mirror the rightward contour
    contour = np.array(
        [[[140, 50]], [[80, 20]], [[20, 50]], [[65, 50]], [[65, 90]], [[115, 90]], [[115, 50]]],
        dtype=np.int32,
    )

    yaw_error = estimate_arrow_yaw_error(contour, image_width=160)

    assert yaw_error < 0.0
    assert yaw_error > -math.pi / 2


def test_arrow_geometry_near_zero_for_centered_tip():
    # Symmetric arrow with tip at center
    contour = np.array(
        [[[60, 80]], [[80, 10]], [[100, 80]], [[90, 80]], [[90, 100]], [[70, 100]], [[70, 80]]],
        dtype=np.int32,
    )

    yaw_error = estimate_arrow_yaw_error(contour, image_width=160)

    assert abs(yaw_error) < math.pi / 8  # near zero


def test_arrow_geometry_returns_clamped_value():
    """Even extreme contours should stay within [-pi/4, pi/4]."""
    contour = np.array(
        [[[0, 50]], [[1, 50]], [[155, 10]], [[155, 50]]],
        dtype=np.int32,
    )

    yaw_error = estimate_arrow_yaw_error(contour, image_width=160)

    assert -math.pi / 4 <= yaw_error <= math.pi / 4


def test_find_largest_arrow_contour_returns_none_for_empty_mask():
    pytest.importorskip('cv2')
    empty_mask = np.zeros((100, 160), dtype=np.uint8)

    result = find_largest_arrow_contour(empty_mask)

    assert result is None


def test_find_largest_arrow_contour_returns_largest():
    cv2 = pytest.importorskip('cv2')
    mask = np.zeros((100, 200), dtype=np.uint8)
    # Draw two blobs, one larger than the other
    mask[10:30, 10:30] = 255  # small blob (20x20 = 400 px)
    mask[50:90, 50:150] = 255  # large blob (40x100 = 4000 px)

    result = find_largest_arrow_contour(mask)

    assert result is not None
    # The largest contour should be near the large blob area
    assert cv2.contourArea(result) > 2000
