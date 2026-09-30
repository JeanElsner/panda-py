"""The success check of move_to_pose, without a robot.

It used Eigen's relative isApprox: 1% of the goal's distance from the base for
the position, and 1% of the raw quaternion coefficients for the orientation,
which also made q and -q, the same orientation, compare unequal. It now
compares an absolute distance and the rotation angle between the two.
"""

import numpy as np
import pytest

from panda_py import _core

GOAL_POSITION = np.array([0.3, 0.0, 0.48])
# The start pose's end effector points down: 180 degrees about x, scalar last.
GOAL_ORIENTATION = np.array([1.0, 0.0, 0.0, 0.0])


def about_z(angle):
    """Scalar-last quaternion of GOAL_ORIENTATION rotated by angle about z."""
    half = angle / 2
    turn = np.array([0.0, 0.0, np.sin(half), np.cos(half)])
    x1, y1, z1, w1 = turn
    x2, y2, z2, w2 = GOAL_ORIENTATION
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def test_distance_is_absolute():
    offset = np.array([0.003, -0.004, 0.0])  # 5 mm
    distance, angle = _core._pose_error(
        GOAL_POSITION, GOAL_ORIENTATION, GOAL_POSITION + offset, GOAL_ORIENTATION
    )
    assert distance == pytest.approx(0.005)
    assert angle == pytest.approx(0.0, abs=1e-9)


def test_distance_does_not_depend_on_the_distance_from_the_base():
    offset = np.array([0.0, 0.0, 0.005])
    near = _core._pose_error(np.zeros(3), GOAL_ORIENTATION, offset, GOAL_ORIENTATION)[0]
    far = _core._pose_error(
        GOAL_POSITION * 2,
        GOAL_ORIENTATION,
        GOAL_POSITION * 2 + offset,
        GOAL_ORIENTATION,
    )[0]
    assert near == pytest.approx(far) == pytest.approx(0.005)


@pytest.mark.parametrize("degrees", [0.5, 3.0, 10.0])
def test_angle_is_the_rotation_between_the_orientations(degrees):
    _, angle = _core._pose_error(
        GOAL_POSITION, GOAL_ORIENTATION, GOAL_POSITION, about_z(np.radians(degrees))
    )
    assert np.degrees(angle) == pytest.approx(degrees)


def test_opposite_quaternion_signs_are_the_same_orientation():
    orientation = about_z(np.radians(2.0))
    same = _core._pose_error(
        GOAL_POSITION, GOAL_ORIENTATION, GOAL_POSITION, orientation
    )
    flipped = _core._pose_error(
        GOAL_POSITION, -GOAL_ORIENTATION, GOAL_POSITION, -orientation
    )
    mixed = _core._pose_error(
        GOAL_POSITION, GOAL_ORIENTATION, GOAL_POSITION, -orientation
    )
    assert same[1] == pytest.approx(flipped[1]) == pytest.approx(mixed[1])
    assert np.degrees(mixed[1]) == pytest.approx(2.0)


def test_unnormalised_quaternions_are_accepted():
    _, angle = _core._pose_error(
        GOAL_POSITION,
        3 * GOAL_ORIENTATION,
        GOAL_POSITION,
        0.5 * about_z(np.radians(1.0)),
    )
    assert np.degrees(angle) == pytest.approx(1.0)


# Worst final errors of move_to_pose on an FER at the default impedance: four
# 5 cm moves from the start pose, measured on 2026-09-30.
MEASURED_AT_DEFAULT_IMPEDANCE = [
    (0.0077, 3.01),
    (0.0059, 1.20),
    (0.0041, 3.01),
    (0.0042, 1.56),
]


@pytest.mark.parametrize("distance, degrees", MEASURED_AT_DEFAULT_IMPEDANCE)
def test_defaults_accept_what_the_default_impedance_achieves(distance, degrees):
    """The FER stopped within 7.7 mm and 3 degrees; that is not a failure."""
    assert distance <= _core._MOVE_TO_POSE_POSITION_THRESHOLD
    assert np.radians(degrees) <= _core._MOVE_TO_POSE_ORIENTATION_THRESHOLD
