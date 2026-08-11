"""Time-optimal trajectory generation.

Also pure computation, so it runs without a robot.
"""

import numpy as np
import pytest

import panda_py
from panda_py import constants, motion

START = np.asarray(constants.JOINT_POSITION_START)


def test_joint_trajectory_starts_and_ends_at_the_waypoints():
    goal = START + 0.3
    traj = motion.JointTrajectory([START, goal], speed_factor=0.2)
    assert traj.get_duration() > 0
    np.testing.assert_allclose(traj.get_joint_positions(0.0), START, atol=1e-9)
    np.testing.assert_allclose(
        traj.get_joint_positions(traj.get_duration()), goal, atol=1e-6
    )


def test_joint_trajectory_starts_and_ends_at_rest():
    """Velocity is negligible at both ends.

    Not exactly zero: the time-optimal parameterisation is discretised at 1e-3,
    so the first and last samples carry one step of velocity.
    """
    traj = motion.JointTrajectory([START, START + 0.3], speed_factor=0.2)
    peak = np.max(np.abs(traj.get_joint_velocities(traj.get_duration() / 2)))
    for time in (0.0, traj.get_duration()):
        velocity = np.abs(traj.get_joint_velocities(time))
        assert np.all(velocity < 0.01), velocity
        assert np.all(velocity < 0.05 * peak), velocity


def test_a_slower_speed_factor_takes_longer():
    waypoints = [START, START + 0.3]
    fast = motion.JointTrajectory(waypoints, speed_factor=0.4).get_duration()
    slow = motion.JointTrajectory(waypoints, speed_factor=0.1).get_duration()
    assert slow > fast


def test_joint_trajectory_accepts_intermediate_waypoints():
    waypoints = [START, START + 0.2, START + 0.1, START + 0.3]
    traj = motion.JointTrajectory(waypoints, speed_factor=0.2)
    assert traj.get_duration() > 0
    np.testing.assert_allclose(
        traj.get_joint_positions(traj.get_duration()), waypoints[-1], atol=1e-6
    )


def test_blended_waypoints_produce_a_usable_trajectory():
    """max_deviation rounds the corners at intermediate waypoints.

    The cumulative section lengths used to omit half of every blend, which made
    the trajectory resolve positions to the wrong waypoint.
    """
    waypoints = [START, START + 0.2, START + 0.1, START + 0.3]
    blended = motion.JointTrajectory(waypoints, speed_factor=0.2, max_deviation=0.05)
    sharp = motion.JointTrajectory(waypoints, speed_factor=0.2, max_deviation=0.0)
    assert blended.get_duration() > 0
    # Cutting corners cannot take longer than going through them.
    assert blended.get_duration() <= sharp.get_duration()


IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])  # scalar last, as panda-py uses


def _pose(x, y, z):
    """Homogeneous transform with identity rotation at the given position."""
    pose = np.eye(4)
    pose[:3, 3] = (x, y, z)
    return pose


def test_cartesian_trajectory_from_positions_and_orientations():
    positions = [np.array([0.3, 0.0, 0.5]), np.array([0.35, 0.0, 0.5])]
    traj = motion.CartesianTrajectory(
        positions=positions,
        orientations=[IDENTITY_QUAT, IDENTITY_QUAT],
        speed_factor=0.2,
    )
    assert traj.get_duration() > 0
    np.testing.assert_allclose(traj.get_position(0.0), positions[0], atol=1e-9)
    np.testing.assert_allclose(
        traj.get_position(traj.get_duration()), positions[1], atol=1e-6
    )


def test_cartesian_trajectory_from_poses():
    """Regression: the pose-list constructor left the trajectory unset.

    It built a temporary instead of initialising the object, so the resulting
    object was unusable.
    """
    poses = [_pose(0.3, 0.0, 0.5), _pose(0.35, 0.0, 0.5)]
    traj = motion.CartesianTrajectory(poses=poses, speed_factor=0.2)
    assert traj.get_duration() > 0
    pose = traj.get_pose(0.0)
    assert pose.shape == (4, 4)
    np.testing.assert_allclose(pose[:3, 3], poses[0][:3, 3], atol=1e-9)


def test_cartesian_trajectory_orientation_is_a_unit_quaternion():
    poses = [_pose(0.3, 0.0, 0.5), _pose(0.35, 0.0, 0.5)]
    traj = motion.CartesianTrajectory(poses=poses, speed_factor=0.2)
    quaternion = traj.get_orientation(traj.get_duration() / 2)
    assert quaternion.shape == (4,)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0, abs=1e-9)


def test_non_finite_input_raises_instead_of_crashing():
    """Garbage in must not take the interpreter down.

    A NaN quaternion used to segfault inside the trajectory generation.
    """
    positions = [np.array([0.3, 0.0, 0.5]), np.array([0.35, 0.0, 0.5])]
    nan_quaternion = np.full(4, np.nan)
    with pytest.raises((ValueError, RuntimeError)):
        motion.CartesianTrajectory(
            positions=positions,
            orientations=[nan_quaternion, nan_quaternion],
            speed_factor=0.2,
        )
