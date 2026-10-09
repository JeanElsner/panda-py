"""Forward and inverse kinematics. No robot required."""

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


RNG = np.random.default_rng(3)
LIMITS = panda_py.conservative_limits()


def random_q(n):
    return RNG.uniform(LIMITS.q_lower + 0.05, LIMITS.q_upper - 0.05, size=(n, 7))


# (q, O_T_EE) from panda-py 1.x's symbolic kinematics, whose Franka Hand
# was 0.103 m long; the pose is 0.0004 m short along its own z.
REFERENCE = [
    ( [0.6739393167859831, 1.3606955966013776, 1.569919731670279, -2.3637043777692517, -1.1016832712844142, 3.309504108291582, -2.817316196620878] ,
      [[0.01444191550939206, -0.16680525471060956, -0.9858840895751125, -0.38124749734602187], [0.14275344856346828, -0.9755426356507488, 0.16714669889299988, 0.44428656055099736], [-0.9896529108731287, -0.14315227217378904, 0.009723321028569404, 0.4392142423012123], [0.0, 0.0, 0.0, 1.0]] ),
    ( [1.7305859811953153, 1.0176410351330094, -0.1825976175361479, -2.1465486194460697, -1.221539600488437, 1.3866346784290193, -0.31276846852068063] ,
      [[-0.10124741985446233, 0.4178745554597926, 0.9028453997623818, 0.2380723718524131], [-0.8799270504560095, 0.38582315955413504, -0.2772523677580627, 0.326972010511829], [-0.46419537465231375, -0.8225091765147337, 0.32863552562139875, 0.03538569321840601], [0.0, 0.0, 0.0, 1.0]] ),
    ( [0.024503290310077475, 0.18326052926638137, 2.821675914045493, -0.7803354468178649, 0.67357409190913, 3.668188138995298, -1.6212030870275582] ,
      [[-0.5287995424969119, 0.8467865355424673, -0.057650733551639244, -0.08636653355276872], [-0.7894461274409532, -0.46577349155734554, 0.399787276474579, 0.1966085186293009], [0.3116822993426781, 0.2569194772423158, 0.9147931604958556, 1.2068903050073823], [0.0, 0.0, 0.0, 1.0]] ),
    ( [-1.8305736887942485, 0.38551566839769413, -2.5970678414631063, -2.892541318138035, 0.08208206615606439, 2.0434683267109985, 2.3756036012240167] ,
      [[0.9738442604956542, 0.0689165249126369, 0.21651297627079558, -0.03517007558594759], [0.1622439153328059, -0.8780274105585644, -0.45027189368791837, 0.22658661971169758], [0.15907315372857075, 0.4736226123210215, -0.8662432411632763, 0.15865605703510016], [0.0, 0.0, 0.0, 1.0]] ),
    ( [0.6961935234448697, 0.04836141019129481, -0.017804534808150585, -2.301459113067139, -2.6914795371841658, 1.1924858635063456, 1.0935461155737216] ,
      [[0.6610143850502745, -0.5537987320732267, 0.5063269172290701, 0.5104111453716871], [0.6865455869305972, 0.7186829701594458, -0.11022679106695994, 0.2844648746382692], [-0.302845075612481, 0.42047800508098343, 0.8552678571186848, 0.5765483747075174], [0.0, 0.0, 0.0, 1.0]] ),
]


@pytest.mark.parametrize("q, pose", REFERENCE)
def test_fk_matches_panda_py_1(q, pose):
    pose = np.asarray(pose)
    new = panda_py.fk(q)
    np.testing.assert_allclose(new[:3, :3], pose[:3, :3], atol=1e-12)
    np.testing.assert_allclose(new[:3, 3], pose[:3, 3] + 0.0004 * pose[:3, 2], atol=1e-12)


def test_fk_takes_an_end_effector():
    q = constants.JOINT_POSITION_START
    flange = panda_py.fk(q, F_T_EE=np.eye(4))
    shifted = np.eye(4)
    shifted[2, 3] = 0.2
    np.testing.assert_allclose(panda_py.fk(q, F_T_EE=shifted)[:3, 3],
                               flange[:3, 3] + 0.2 * flange[:3, 2], atol=1e-12)


@pytest.mark.parametrize("q", random_q(10))
def test_the_jacobian_is_the_derivative_of_fk(q):
    J = panda_py.jacobian(q)
    eps = 1e-7
    for j in range(7):
        dq = np.zeros(7)
        dq[j] = eps
        a, b = panda_py.fk(q - dq), panda_py.fk(q + dq)
        np.testing.assert_allclose(J[:3, j], (b[:3, 3] - a[:3, 3]) / (2 * eps), atol=1e-6)
        dR = (b[:3, :3] - a[:3, :3]) / (2 * eps) @ panda_py.fk(q)[:3, :3].T
        omega = np.array([dR[2, 1], dR[0, 2], dR[1, 0]])
        np.testing.assert_allclose(J[3:, j], omega, atol=1e-6)


@pytest.mark.parametrize("q_goal", random_q(100))
def test_ik_reaches_poses_from_random_configurations(q_goal):
    pose = panda_py.fk(q_goal)
    q = panda_py.ik(pose)
    np.testing.assert_allclose(panda_py.fk(q)[:3, 3], pose[:3, 3], atol=2e-5)
    assert np.all(q >= LIMITS.q_lower) and np.all(q <= LIMITS.q_upper)


def test_ik_stays_near_q_init():
    """From a nearby start the solution is the nearby one, not another branch."""
    for q_goal in random_q(30):
        q = panda_py.ik(panda_py.fk(q_goal), q_init=q_goal + 0.05)
        assert np.abs(q - q_goal).max() < 0.3


def quaternion(R):
    """Scalar-last quaternion of a rotation matrix (Shepperd's method)."""
    w, x, y, z = (1 + np.trace(R)) / 4, (1 + 2 * R[0, 0] - np.trace(R)) / 4, \
        (1 + 2 * R[1, 1] - np.trace(R)) / 4, (1 + 2 * R[2, 2] - np.trace(R)) / 4
    i = int(np.argmax([w, x, y, z]))
    s = np.sqrt([w, x, y, z][i])
    if i == 0:
        q = [s, (R[2, 1] - R[1, 2]) / (4 * s), (R[0, 2] - R[2, 0]) / (4 * s), (R[1, 0] - R[0, 1]) / (4 * s)]
    elif i == 1:
        q = [(R[2, 1] - R[1, 2]) / (4 * s), s, (R[0, 1] + R[1, 0]) / (4 * s), (R[0, 2] + R[2, 0]) / (4 * s)]
    elif i == 2:
        q = [(R[0, 2] - R[2, 0]) / (4 * s), (R[0, 1] + R[1, 0]) / (4 * s), s, (R[1, 2] + R[2, 1]) / (4 * s)]
    else:
        q = [(R[1, 0] - R[0, 1]) / (4 * s), (R[0, 2] + R[2, 0]) / (4 * s), (R[1, 2] + R[2, 1]) / (4 * s), s]
    return np.array(q[1:] + q[:1])


def test_ik_takes_position_and_orientation():
    for q_goal in [np.asarray(constants.JOINT_POSITION_START), *random_q(5)]:
        pose = panda_py.fk(q_goal)
        q = panda_py.ik((pose[:3, 3], quaternion(pose[:3, :3])), q_init=q_goal)
        np.testing.assert_allclose(panda_py.fk(q), pose, atol=1e-4)


def test_ik_respects_the_limits_it_is_given():
    fr3 = panda_py.limits(10)
    q_goal = np.clip(random_q(1)[0], fr3.q_lower + 0.1, fr3.q_upper - 0.1)
    q = panda_py.ik(panda_py.fk(q_goal), limits=fr3)
    assert np.all(q >= fr3.q_lower) and np.all(q <= fr3.q_upper)


def test_an_unreachable_pose_raises_with_the_best_found():
    pose = np.eye(4)
    pose[:3, 3] = [2.0, 0.0, 0.5]  # well out of reach
    with pytest.raises(panda_py.IKError) as info:
        panda_py.ik(pose, restarts=2)
    assert not info.value.result.success
    assert info.value.result.position_error > 0.5
    assert isinstance(info.value, ValueError)
