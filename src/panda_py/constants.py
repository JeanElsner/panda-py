"""
Commonly used constants.
"""

# pylint: disable=no-name-in-module
from ._core import (
    _JOINT_LIMITS_LOWER,
    _JOINT_LIMITS_LOWER_FR3,
    _JOINT_LIMITS_LOWER_FR3_5_9,
    _JOINT_LIMITS_UPPER,
    _JOINT_LIMITS_UPPER_FR3,
    _JOINT_LIMITS_UPPER_FR3_5_9,
    _JOINT_POSITION_START,
)

__all__ = [
    "JOINT_POSITION_START",
    "JOINT_LIMITS_LOWER",
    "JOINT_LIMITS_UPPER",
    "JOINT_LIMITS_LOWER_FR3",
    "JOINT_LIMITS_UPPER_FR3",
    "JOINT_LIMITS_LOWER_FR3_5_9",
    "JOINT_LIMITS_UPPER_FR3_5_9",
]

JOINT_POSITION_START = _JOINT_POSITION_START
"""
Common start pose of the robot. State-space around this pose has
high manipulability, reachability, distance to joint limits etc.
"""

JOINT_LIMITS_LOWER = _JOINT_LIMITS_LOWER
"""
Lower joint position limits of the Franka Emika Robot (FER) in radian.
"""

JOINT_LIMITS_UPPER = _JOINT_LIMITS_UPPER
"""
Upper joint position limits of the Franka Emika Robot (FER) in radian.
"""

JOINT_LIMITS_LOWER_FR3 = _JOINT_LIMITS_LOWER_FR3
"""
Lower joint position limits of the Franka Research 3 (FR3) with robot
system version below 5.9.0, in radian.
"""

JOINT_LIMITS_UPPER_FR3 = _JOINT_LIMITS_UPPER_FR3
"""
Upper joint position limits of the Franka Research 3 (FR3) with robot
system version below 5.9.0, in radian.
"""

JOINT_LIMITS_LOWER_FR3_5_9 = _JOINT_LIMITS_LOWER_FR3_5_9
"""
Lower joint position limits of the Franka Research 3 (FR3) with robot
system version 5.9.0 or later, in radian. Robot system 5.9.0 widened the
limits to the values given in the datasheet.
"""

JOINT_LIMITS_UPPER_FR3_5_9 = _JOINT_LIMITS_UPPER_FR3_5_9
"""
Upper joint position limits of the Franka Research 3 (FR3) with robot
system version 5.9.0 or later, in radian.
"""
