"""
Motion generation for the Panda robot. These are also directly
integrated as convenience methods of the :py:class:`panda_py.Panda` class.
"""

# pylint: disable=no-name-in-module
from ._core import JointTrajectory, CartesianTrajectory

__all__ = ['JointTrajectory', 'CartesianTrajectory']
