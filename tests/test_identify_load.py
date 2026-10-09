import numpy as np
import pytest

import panda_py
from panda_py import constants, identify_load


def poses():
    start = np.asarray(constants.JOINT_POSITION_START)
    return start + identify_load.POSE_OFFSETS


@pytest.mark.parametrize("sign", [1, -1])
def test_fit_recovers_a_load(sign):
    rng = np.random.default_rng(0)
    m, c = 0.35, np.array([0.02, -0.01, 0.05])
    b = rng.normal(0, 0.3, 7)
    qs = poses()
    taus = [sign * identify_load.regressor(q) @ np.hstack([m, m * c]) + b +
                    rng.normal(0, 0.01, 7) for q in qs]
    result = identify_load.fit(qs, taus)
    assert result["mass"] == pytest.approx(m, abs=0.01)
    np.testing.assert_allclose(result["com"], c, atol=3e-3)
    np.testing.assert_allclose(result["offsets"], b, atol=0.05)
    assert result["residual_rms"] < 0.03
    assert result["mass_std"] < 0.01


def test_regressor_matches_a_static_wrench():
    # A point mass at c in the flange frame: the external torques are
    # J^T [f; (p_c - p_F) x f] with f = m g, the Jacobian taken at the flange.
    q = poses()[7]
    m, c = 0.5, np.array([0.01, 0.03, 0.08])
    T = panda_py.fk(q, F_T_EE=np.eye(4))
    f = m * identify_load.G
    wrench = np.hstack([f, np.cross(T[:3, :3] @ c, f)])
    J = panda_py.jacobian(q, F_T_EE=np.eye(4))
    np.testing.assert_allclose(identify_load.regressor(q) @ np.hstack([m, m * c]),
                                                         J.T @ wrench, atol=1e-12)


def test_poses_are_within_the_limits():
    for limits in (panda_py.limits(5), panda_py.limits(10)):
        for q in poses():
            assert np.all(q > limits.q_lower + 0.05) and np.all(q < limits.q_upper - 0.05)


def test_no_load():
    qs = poses()
    result = identify_load.fit(qs, [np.full(7, 0.1) for _ in qs])
    assert result["mass"] < 1e-9
    assert result["com"] == [0, 0, 0]
