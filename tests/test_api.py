"""Smoke tests for the public API.

These exist mostly so that CI imports the wheels it builds. Every matrix row
produced 48 wheels without ever loading one, so a broken module initialisation
would have shipped unnoticed.
"""

import panda_py
from panda_py import constants, controllers, libfranka, motion


def test_version_is_exposed():
    assert panda_py.__version__


def test_top_level_symbols():
    for name in ["Panda", "PandaContext", "Desk", "fk", "ik", "ik_full"]:
        assert hasattr(panda_py, name), name


def test_controllers_are_exposed():
    for name in controllers.__all__:
        assert hasattr(controllers, name), name


def test_motion_generators_are_exposed():
    assert hasattr(motion, "JointTrajectory")
    assert hasattr(motion, "CartesianTrajectory")


def test_libfranka_core_types():
    for name in ["Robot", "Model", "RobotState", "Gripper", "Torques", "Errors"]:
        assert hasattr(libfranka, name), name


def test_libfranka_control_overloads():
    """Every overload of Robot::control is bound, not just the torque one.

    A callback returning JointPositions used to be cast to Torques and fail with
    an unhelpful conversion error.
    """
    for name in [
        "control",
        "control_joint_position",
        "control_joint_velocity",
        "control_cartesian_pose",
        "control_cartesian_velocity",
        "control_torque_joint_position",
        "control_torque_joint_velocity",
        "control_torque_cartesian_pose",
        "control_torque_cartesian_velocity",
    ]:
        assert hasattr(libfranka.Robot, name), name


def test_constants_are_exposed():
    for name in constants.__all__:
        assert hasattr(constants, name), name
