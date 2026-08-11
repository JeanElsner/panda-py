"""The joint limit sets and the relationships between them.

The virtual walls throw when the robot starts outside their range, so applying
the wrong set to a robot takes out the control loop. These pin the values and
the subset relationship the selection logic relies on.
"""

import numpy as np
import pytest

from panda_py import constants

LIMIT_PAIRS = [
    ("JOINT_LIMITS_LOWER", "JOINT_LIMITS_UPPER"),
    ("JOINT_LIMITS_LOWER_FR3", "JOINT_LIMITS_UPPER_FR3"),
    ("JOINT_LIMITS_LOWER_FR3_5_9", "JOINT_LIMITS_UPPER_FR3_5_9"),
]


@pytest.mark.parametrize("lower_name,upper_name", LIMIT_PAIRS)
def test_limits_are_seven_joints_and_ordered(lower_name, upper_name):
    lower = np.asarray(getattr(constants, lower_name))
    upper = np.asarray(getattr(constants, upper_name))
    assert lower.shape == (7,)
    assert upper.shape == (7,)
    assert np.all(lower < upper)


def test_start_position_is_within_every_envelope():
    start = np.asarray(constants.JOINT_POSITION_START)
    assert start.shape == (7,)
    for lower_name, upper_name in LIMIT_PAIRS:
        lower = np.asarray(getattr(constants, lower_name))
        upper = np.asarray(getattr(constants, upper_name))
        assert np.all(start >= lower), lower_name
        assert np.all(start <= upper), upper_name


def test_older_fr3_envelope_is_contained_in_the_newer_one():
    """Robot system 5.9.0 widened the limits to the datasheet values.

    The selection logic assumes the earlier set is the conservative one, so that
    using it on a newer robot only costs reach rather than being wrong.
    """
    old_lower = np.asarray(constants.JOINT_LIMITS_LOWER_FR3)
    old_upper = np.asarray(constants.JOINT_LIMITS_UPPER_FR3)
    new_lower = np.asarray(constants.JOINT_LIMITS_LOWER_FR3_5_9)
    new_upper = np.asarray(constants.JOINT_LIMITS_UPPER_FR3_5_9)
    assert np.all(new_lower <= old_lower)
    assert np.all(old_upper <= new_upper)


def test_fr3_joint_six_differs_from_the_fer():
    """Joint 6 is why the envelopes cannot be shared.

    The FER covers -0.0175 to 3.7525 while the FR3 runs roughly 0.44 to 4.62, so
    a large part of the FR3's travel is outside the FER envelope entirely.
    """
    fer_lower = np.asarray(constants.JOINT_LIMITS_LOWER)[5]
    fer_upper = np.asarray(constants.JOINT_LIMITS_UPPER)[5]
    fr3_lower = np.asarray(constants.JOINT_LIMITS_LOWER_FR3)[5]
    fr3_upper = np.asarray(constants.JOINT_LIMITS_UPPER_FR3)[5]
    assert fr3_lower > fer_lower
    assert fr3_upper > fer_upper
    # Travel the FER envelope would wrongly reject on an FR3.
    assert fr3_upper - fer_upper > 0.5
