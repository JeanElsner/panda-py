"""The TaskImpedance law, without a robot.

The controller's law is compared with panda_py.reference, an independent
transcription of the simulator's controller, on random states, and checked for
the properties the law is meant to have.
"""

import numpy as np
import pytest

from panda_py import reference
from panda_py.controllers import TaskImpedance

STIFFNESS = np.array([400.0, 400.0, 400.0, 30.0, 30.0, 30.0])
# The above-hole posture of the insertion task.
Q0 = np.array([0.0, 0.2987, 0.0, -2.5492, 0.0, 2.8479, -0.7853])


def random_rotation(rng):
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    return q if np.linalg.det(q) > 0 else -q


def axis_angle_matrix(v):
    angle = np.linalg.norm(v)
    if angle == 0:
        return np.eye(3)
    k = v / angle
    skew = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def xyzw(r):
    """Scalar-last quaternion of a rotation matrix."""
    w, x, y, z = reference._matrix_to_quat(r)  # pylint: disable=protected-access
    return np.array([x, y, z, w])


def random_state(rng, rotation=0.4, translation=0.025):
    """A state and a reference near it, within the simulator's leashes."""
    pose = np.eye(4)
    pose[:3, :3] = random_rotation(rng)
    pose[:3, 3] = rng.uniform(-0.6, 0.6, 3)
    axis = rng.normal(size=3)
    turn = axis / np.linalg.norm(axis) * rng.uniform(0, rotation)
    a = rng.normal(size=(7, 7))
    return {
        "q": rng.uniform(-2, 2, 7),
        "dq": rng.normal(scale=0.3, size=7),
        "pose": pose,
        "jacobian": rng.normal(scale=0.4, size=(6, 7)),
        "mass": a @ a.T * 0.1 + np.eye(7) * 0.05,
        "position_ref": pose[:3, 3] + rng.uniform(-translation, translation, 3),
        "orientation_ref": xyzw(axis_angle_matrix(turn) @ pose[:3, :3]),
        "rotation": turn,
    }


def law(state, stiffness=STIFFNESS, nullspace="dynamic", k_ns=10.0, alpha=1.0, zeta=1.0, **kw):
    args = {k: v for k, v in state.items() if k != "rotation"}
    return TaskImpedance.compute(
        **args,
        stiffness=stiffness,
        damping=TaskImpedance.critical_damping(stiffness, zeta),
        q_nullspace=Q0,
        nullspace_stiffness=k_ns,
        nullspace=nullspace,
        alpha=alpha,
        **kw,
    )


def ref(state, stiffness=STIFFNESS, nullspace="dynamic", k_ns=10.0, alpha=1.0, zeta=1.0, **kw):
    args = {k: v for k, v in state.items() if k != "rotation"}
    return reference.task_impedance(
        **args,
        stiffness=stiffness,
        q_nullspace=Q0,
        nullspace_stiffness=k_ns,
        nullspace=nullspace,
        alpha=alpha,
        zeta=zeta,
        **kw,
    )


@pytest.mark.parametrize("nullspace", ["dynamic", "kinematic", "none"])
@pytest.mark.parametrize("alpha", [1.0, 0.3, 0.0])
def test_matches_the_reference(nullspace, alpha):
    rng = np.random.default_rng(0)
    for _ in range(200):
        state = random_state(rng)
        stiffness = np.exp(rng.uniform(np.log([50] * 3 + [5] * 3), np.log([800] * 3 + [150] * 3)))
        ours = law(state, stiffness, nullspace, alpha=alpha)
        theirs = ref(state, stiffness, nullspace, alpha=alpha)
        for key in ("wrench_active", "wrench_passive", "tau_task", "tau_nullspace", "tau"):
            np.testing.assert_allclose(ours[key], theirs[key], rtol=1e-9, atol=1e-9, err_msg=key)


def test_matches_the_reference_with_nullspace_damping():
    rng = np.random.default_rng(1)
    for nullspace in ("dynamic", "kinematic"):
        state = random_state(rng)
        ours = law(state, nullspace=nullspace, nullspace_damping=1e-3)
        theirs = ref(state, nullspace=nullspace, nullspace_damping=1e-3)
        np.testing.assert_allclose(ours["tau"], theirs["tau"], rtol=1e-9, atol=1e-9)


def test_dynamic_nullspace_does_not_leak_into_the_task():
    """J M^-1 tau_ns, the task acceleration the posture term causes, vanishes."""
    rng = np.random.default_rng(2)
    for _ in range(100):
        state = random_state(rng)
        tau_ns = law(state, nullspace="dynamic")["tau_nullspace"]
        leak = state["jacobian"] @ np.linalg.solve(state["mass"], tau_ns)
        assert np.linalg.norm(leak) < 1e-9 * np.linalg.norm(tau_ns)


def test_kinematic_nullspace_does_not_move_the_task_kinematically():
    """The kinematic projection gives no task velocity: J tau_ns = 0."""
    rng = np.random.default_rng(3)
    state = random_state(rng)
    tau_ns = law(state, nullspace="kinematic")["tau_nullspace"]
    assert np.linalg.norm(state["jacobian"] @ tau_ns) < 1e-9 * np.linalg.norm(tau_ns)


def test_zero_nullspace_stiffness_drops_the_posture_term():
    state = random_state(np.random.default_rng(4))
    for nullspace in ("dynamic", "kinematic"):
        np.testing.assert_array_equal(law(state, nullspace=nullspace, k_ns=0.0)["tau_nullspace"], 0)


def test_alpha_scales_the_spring_but_never_the_damping():
    state = random_state(np.random.default_rng(5))
    full, gated = law(state, nullspace="none"), law(state, nullspace="none", alpha=0.0)
    np.testing.assert_allclose(gated["wrench_active"], full["wrench_active"])
    np.testing.assert_allclose(
        gated["tau_task"], state["jacobian"].T @ full["wrench_passive"], atol=1e-12
    )


def test_damping_is_critical():
    np.testing.assert_allclose(
        TaskImpedance.critical_damping(STIFFNESS), 2 * np.sqrt(STIFFNESS)
    )
    np.testing.assert_allclose(
        TaskImpedance.critical_damping(STIFFNESS, 0.5), np.sqrt(STIFFNESS)
    )


def test_the_orientation_error_is_the_full_rotation():
    """Not the quaternion's vector part, which is about half the angle."""
    rng = np.random.default_rng(6)
    for angle in (1e-6, 1e-3, 0.3, 1.5, 3.0):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        start = random_rotation(rng)
        goal = axis_angle_matrix(axis * angle) @ start
        q_start, q_goal = xyzw(start), xyzw(goal)
        np.testing.assert_allclose(
            TaskImpedance.orientation_error(q_goal, q_start), axis * angle, atol=1e-9
        )
        # q and -q are the same orientation.
        np.testing.assert_allclose(
            TaskImpedance.orientation_error(-q_goal, q_start), axis * angle, atol=1e-9
        )


def test_shift_jacobian_gives_the_velocity_of_the_offset_point():
    rng = np.random.default_rng(7)
    jacobian, dq, offset = rng.normal(size=(6, 7)), rng.normal(size=7), rng.normal(size=3)
    v, w = (jacobian @ dq)[:3], (jacobian @ dq)[3:]
    shifted = TaskImpedance.shift_jacobian(jacobian, offset) @ dq
    np.testing.assert_allclose(shifted[:3], v + np.cross(w, offset))
    np.testing.assert_allclose(shifted[3:], w)


def test_holds_with_zero_active_wrench_at_the_reference():
    state = random_state(np.random.default_rng(8))
    state["position_ref"] = state["pose"][:3, 3]
    state["orientation_ref"] = xyzw(state["pose"][:3, :3])
    np.testing.assert_allclose(law(state)["wrench_active"], 0, atol=1e-9)


def test_constructor():
    ctrl = TaskImpedance()
    assert ctrl.frame == "end_effector"
    np.testing.assert_allclose(ctrl.get_damping(), 2 * np.sqrt(ctrl.get_stiffness()))
    offset = np.eye(4)
    offset[2, 3] = 0.088
    ctrl = TaskImpedance(STIFFNESS, frame="flange", frame_transform=offset, nullspace="kinematic")
    assert ctrl.frame == "flange"
    np.testing.assert_allclose(ctrl.frame_transform, offset)
    ctrl.set_stiffness(STIFFNESS * 2)
    np.testing.assert_allclose(ctrl.get_damping(), 2 * np.sqrt(STIFFNESS * 2))
    ctrl.set_damping_ratio(0.5)
    np.testing.assert_allclose(ctrl.get_damping(), np.sqrt(STIFFNESS * 2))
    with pytest.raises(ValueError):
        TaskImpedance(nullspace="pseudo")
    with pytest.raises(ValueError):
        TaskImpedance(frame="hand")
    with pytest.raises(ValueError):
        TaskImpedance(frame_transform=np.ones((4, 4)))


# -- parity with the simulator's reference implementation --------------------
#
# vic_reference.py (contact/hardware in the phd repository) is the NumPy port
# of the simulator's controller that the deployment request specifies for the
# parity checks. It is not part of panda-py: these tests load it from a phd
# checkout, $VIC_REFERENCE or ~/dev/phd/contact/hardware/vic_reference.py, and
# are skipped without one.


def _load_vic_reference():
    import importlib.util  # pylint: disable=import-outside-toplevel
    import os  # pylint: disable=import-outside-toplevel
    import pathlib  # pylint: disable=import-outside-toplevel
    import sys  # pylint: disable=import-outside-toplevel

    path = pathlib.Path(
        os.environ.get(
            "VIC_REFERENCE",
            pathlib.Path.home() / "dev" / "phd" / "contact" / "hardware" / "vic_reference.py",
        )
    )
    if not path.is_file():
        pytest.skip(f"no vic_reference.py at {path}")
    spec = importlib.util.spec_from_file_location("vic_reference", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["vic_reference"] = module  # its dataclass looks itself up there
    spec.loader.exec_module(module)
    return module


@pytest.fixture(name="vic", scope="module")
def fixture_vic():
    return _load_vic_reference()


def wxyz(r):
    return reference._matrix_to_quat(r)  # pylint: disable=protected-access


VARIANTS = {
    "fixed": {},
    "fixed-kinematic": {"nullspace_projection": "kinematic"},
    "variable": {"variable_stiffness": True},
    "power-tank": {"tank_E0": 0.05, "tank_mode": "power"},
    "impulse-tank": {"tank_E0": 0.05, "tank_mode": "impulse"},
}


@pytest.mark.parametrize("variant", VARIANTS)
def test_matches_vic_reference_over_a_trial(vic, variant):
    """Policy steps at 50 Hz, torques at 1 kHz, as on the arm.

    The reference integrates and leashes the reference pose, maps the
    stiffness and runs the tank; the controller law is fed what the reference
    holds at each tick (x_ref, q_ref, K and alpha), which is what the 50 Hz
    side and the tank will hand it, and must give the same torques. Its torque
    clip is off: panda-py saturates torques after the law, in the Panda class.
    """
    rng = np.random.default_rng(10)
    ref_ = vic.VicReference(clip_torque=False, **VARIANTS[variant])
    action_size = 12 if ref_.variable_stiffness else 6
    state = random_state(rng)
    pose = state["pose"]
    ref_.reset(pose[:3, 3], wxyz(pose[:3, :3]))
    worst = 0.0
    for step in range(20):
        # A new state each step stands in for the arm having moved.
        state = random_state(rng)
        pose = state["pose"]
        x, quat = pose[:3, 3], wxyz(pose[:3, :3])
        if step:
            ref_.process_action(rng.uniform(-1.2, 1.2, action_size), x, quat)
        for _ in range(20):
            state["dq"] = rng.normal(scale=0.3, size=7)
            v6 = state["jacobian"] @ state["dq"]
            tau, tel = ref_.torque(
                x, quat, v6[:3], v6[3:], state["q"], state["dq"],
                state["jacobian"], state["mass"], 1e-3,
            )
            w, *xyz = tel["q_ref"]
            ours = TaskImpedance.compute(
                q=state["q"], dq=state["dq"], pose=pose, jacobian=state["jacobian"],
                mass=state["mass"], position_ref=tel["x_ref"], orientation_ref=[*xyz, w],
                stiffness=tel["K"], damping=TaskImpedance.critical_damping(tel["K"]),
                q_nullspace=ref_.q0, nullspace_stiffness=ref_.nullspace_kp,
                nullspace=ref_.nullspace_projection, alpha=tel["alpha"],
            )
            np.testing.assert_allclose(ours["wrench_active"], tel["w_active"], atol=1e-9)
            np.testing.assert_allclose(ours["wrench_passive"], tel["w_passive"], atol=1e-9)
            np.testing.assert_allclose(ours["tau_nullspace"], tel["tau_ns"], atol=1e-9)
            worst = max(worst, np.linalg.norm(ours["tau"] - tau) / np.linalg.norm(tau))
    # The request's acceptance is 1 % of the torque norm.
    assert worst < 1e-9


def test_reference_transcription_matches_vic_reference(vic):
    """panda_py.reference, which CI runs without the phd repository, agrees."""
    rng = np.random.default_rng(11)
    for projection in ("dynamic", "kinematic"):
        ref_ = vic.VicReference(clip_torque=False, nullspace_projection=projection)
        for _ in range(50):
            state = random_state(rng)
            pose = state["pose"]
            x, quat = pose[:3, 3], wxyz(pose[:3, :3])
            ref_.reset(x, quat)
            ref_.process_action(rng.uniform(-1, 1, 6), x, quat)
            v6 = state["jacobian"] @ state["dq"]
            tau, tel = ref_.torque(
                x, quat, v6[:3], v6[3:], state["q"], state["dq"],
                state["jacobian"], state["mass"], 1e-3,
            )
            w, *xyz = tel["q_ref"]
            ours = reference.task_impedance(
                q=state["q"], dq=state["dq"], pose=pose, jacobian=state["jacobian"],
                mass=state["mass"], position_ref=tel["x_ref"], orientation_ref=[*xyz, w],
                stiffness=tel["K"], q_nullspace=ref_.q0,
                nullspace_stiffness=ref_.nullspace_kp, nullspace=projection,
            )
            np.testing.assert_allclose(ours["tau"], tau, rtol=1e-9, atol=1e-9)
