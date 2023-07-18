"""
Commonly used constants.
"""

# pylint: disable=no-name-in-module
from ._core import _JOINT_POSITION_START, _JOINT_LIMITS_LOWER, _JOINT_LIMITS_UPPER

__all__ = ['JOINT_POSITION_START', 'JOINT_LIMITS_LOWER', 'JOINT_LIMITS_UPPER']

JOINT_POSITION_START = _JOINT_POSITION_START
"""
Common start pose of the robot. State-space around this pose has
high manipulability, reachability, distance to joint limits etc.
"""

JOINT_LIMITS_LOWER = _JOINT_LIMITS_LOWER
"""
Lower joint position limits in radian.
"""

JOINT_LIMITS_UPPER = _JOINT_LIMITS_UPPER
"""
Upper joint position limits in radian.
"""
