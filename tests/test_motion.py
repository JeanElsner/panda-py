"""Time-optimal trajectory generation.

Also pure computation, so it runs without a robot.
"""

import subprocess
import sys

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


CONSTRUCT_WITHOUT_THE_GIL = """
import numpy as np
from panda_py import constants, motion

start = np.asarray(constants.JOINT_POSITION_START)
motion.JointTrajectory([start, start + 0.1], speed_factor=0.2)
positions = [np.array([0.3, 0.0, 0.5]), np.array([0.35, 0.0, 0.5])]
orientation = np.array([1.0, 0.0, 0.0, 0.0])
motion.CartesianTrajectory(positions, [orientation, orientation], speed_factor=0.2)
pose = np.eye(4)
pose[:3, 3] = positions[0]
end = pose.copy()
end[:3, 3] = positions[1]
motion.CartesianTrajectory([pose, end], speed_factor=0.2)
"""


def test_trajectories_can_be_built_without_the_gil():
    """The move_to_* methods build their trajectory with the GIL released.

    In 1.0.0 the constructors released it once more on their own, which
    segfaults on a thread that no longer holds it, so every move_to_* call
    crashed on a real robot. The constructors are bound with the GIL released
    too, so constructing one here takes the same path. It runs in a subprocess
    so that a regression fails this test instead of killing the session.
    """
    result = subprocess.run(
        [sys.executable, "-c", CONSTRUCT_WITHOUT_THE_GIL],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"exit {result.returncode}: {result.stderr[-2000:]}"


POSITIONS = [np.zeros(3), np.full(3, 0.1)]
ORIENTATIONS = [np.array([1.0, 0.0, 0.0, 0.0])] * 2


@pytest.mark.parametrize("speed_factor", [0.0, 1e-4, -0.2, np.nan, np.inf])
@pytest.mark.parametrize("kind", ["joint", "cartesian"])
def test_unusable_speed_factor_raises(kind, speed_factor):
    """A zero speed factor never finished and ate memory until killed.

    Tiny ones did the same for as many steps as the trajectory has
    milliseconds. The error is raised under the constructors' GIL release,
    which segfaults on Python 3.9 through 3.11 unless the GIL is back by the
    time the exception unwinds, so this also guards that path.
    """
    with pytest.raises(ValueError, match="speed_factor must be at least"):
        if kind == "joint":
            motion.JointTrajectory([START, START + 0.3], speed_factor=speed_factor)
        else:
            motion.CartesianTrajectory(
                POSITIONS, ORIENTATIONS, speed_factor=speed_factor
            )


def test_the_minimum_speed_factor_computes_quickly():
    """At the minimum, even a move across the full joint range stays cheap."""
    limits = panda_py.conservative_limits()
    lower = np.asarray(limits.q_lower) + 0.1
    upper = np.asarray(limits.q_upper) - 0.1
    trajectory = motion.JointTrajectory([lower, upper], speed_factor=1e-3)
    assert trajectory.get_duration() > 1000
    motion.CartesianTrajectory(POSITIONS, ORIENTATIONS, speed_factor=1e-3)
