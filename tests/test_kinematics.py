"""Analytical forward and inverse kinematics.

No robot required: these are pure functions of the joint configuration.
"""

import numpy as np
import pytest

import panda_py
from panda_py import constants

START = np.asarray(constants.JOINT_POSITION_START)

CONFIGURATIONS = [
    START,
    START + np.array([0.2, -0.1, 0.15, 0.2, -0.3, 0.1, 0.25]),
    START + np.array([-0.4, 0.2, -0.2, 0.3, 0.4, -0.2, -0.3]),
]


@pytest.mark.parametrize("q", CONFIGURATIONS)
def test_forward_kinematics_returns_a_valid_transform(q):
    pose = panda_py.fk(q)
    assert pose.shape == (4, 4)
    np.testing.assert_allclose(pose[3], [0, 0, 0, 1], atol=1e-12)
    rotation = pose[:3, :3]
    # A rotation matrix is orthonormal with determinant +1.
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)


@pytest.mark.parametrize("q", CONFIGURATIONS)
def test_inverse_kinematics_inverts_forward_kinematics(q):
    pose = panda_py.fk(q)
    recovered = panda_py.ik(pose, q_init=q, q_7=q[6])
    assert not np.any(np.isnan(recovered)), "no solution found for a reachable pose"
    # The analytical solution is not exact to machine precision.
    np.testing.assert_allclose(recovered, q, atol=5e-3)


@pytest.mark.parametrize("q", CONFIGURATIONS)
def test_inverse_kinematics_reproduces_the_pose(q):
    pose = panda_py.fk(q)
    recovered = panda_py.ik(pose, q_init=q, q_7=q[6])
    np.testing.assert_allclose(panda_py.fk(recovered), pose, atol=5e-3)


def test_inverse_kinematics_reports_unreachable_poses_as_nan():
    """Far outside the workspace there is no solution, signalled by NaN."""
    pose = panda_py.fk(START)
    pose[0, 3] += 10.0
    assert np.any(np.isnan(panda_py.ik(pose, q_init=START, q_7=START[6])))


def test_ik_full_returns_all_four_branches():
    pose = panda_py.fk(START)
    solutions = panda_py.ik_full(pose, q_init=START, q_7=START[6])
    assert solutions.shape == (4, 7)
