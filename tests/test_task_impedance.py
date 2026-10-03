"""The TaskImpedance law, without a robot.

The controller's law and its reference update are compared with
vic_reference.py, the NumPy port of the insertion simulator's controller that
the deployment request specifies for parity checks (vendored from the phd
repository, contact/hardware, at fd83ea0), and checked for the properties the
law is meant to have.
"""

import numpy as np
import pytest
import vic_reference as vic

from panda_py.controllers import TaskImpedance

STIFFNESS = np.array([400.0, 400.0, 400.0, 30.0, 30.0, 30.0])
Q0 = vic.POSTURE_ABOVE_HOLE


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


def wxyz(r):
    """Scalar-first unit quaternion of a rotation matrix, vic_reference's order."""
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = np.empty(4)
        q[0] = (r[k, j] - r[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (r[j, i] + r[i, j]) / s
        q[1 + k] = (r[k, i] + r[i, k]) / s
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def xyzw(q):
    """panda-py's scalar-last order, from a scalar-first quaternion or a matrix."""
    q = wxyz(q) if np.shape(q) == (3, 3) else np.asarray(q)
    return np.array([q[1], q[2], q[3], q[0]])


def random_state(rng):
    pose = np.eye(4)
    pose[:3, :3] = random_rotation(rng)
    pose[:3, 3] = rng.uniform(-0.6, 0.6, 3)
    a = rng.normal(size=(7, 7))
    return {
        "q": rng.uniform(-2, 2, 7),
        "dq": rng.normal(scale=0.3, size=7),
        "pose": pose,
        "jacobian": rng.normal(scale=0.4, size=(6, 7)),
        "mass": a @ a.T * 0.1 + np.eye(7) * 0.05,
    }


def law(state, position_ref, orientation_ref, stiffness=STIFFNESS, nullspace="dynamic",
        k_ns=10.0, alpha=1.0, **kw):
    return TaskImpedance.compute(
        **state,
        position_ref=position_ref,
        orientation_ref=orientation_ref,
        stiffness=stiffness,
        damping=TaskImpedance.critical_damping(stiffness),
        q_nullspace=Q0,
        nullspace_stiffness=k_ns,
        nullspace=nullspace,
        alpha=alpha,
        **kw,
    )


def near_reference(rng, state, rotation=0.4, translation=0.025):
    """A reference within the simulator's leashes of the state's pose."""
    pose = state["pose"]
    axis = rng.normal(size=3)
    turn = axis / np.linalg.norm(axis) * rng.uniform(0, rotation)
    return (
        pose[:3, 3] + rng.uniform(-translation, translation, 3),
        xyzw(axis_angle_matrix(turn) @ pose[:3, :3]),
    )


def reference_torque(ref_, state):
    """vic_reference's torque for a state; v and omega are J dq."""
    pose = state["pose"]
    v6 = state["jacobian"] @ state["dq"]
    return ref_.torque(
        pose[:3, 3], wxyz(pose[:3, :3]), v6[:3], v6[3:], state["q"], state["dq"],
        state["jacobian"], state["mass"], 1e-3,
    )


# -- parity with vic_reference -------------------------------------------------

VARIANTS = {
    "fixed": {},
    "fixed-kinematic": {"nullspace_projection": "kinematic"},
    "variable": {"variable_stiffness": True},
    "power-tank": {"tank_E0": 0.05, "tank_mode": "power"},
    "impulse-tank": {"tank_E0": 0.05, "tank_mode": "impulse"},
}


@pytest.mark.parametrize("variant", VARIANTS)
def test_matches_vic_reference_over_a_trial(variant):
    """Policy steps at 50 Hz, torques at 1 kHz, as on the arm.

    The reference integrates and leashes the reference pose, maps the
    stiffness and runs the tank. The controller's reference update is fed the
    same action and pose, and its law what it then holds, plus the
    reference's K and alpha; both must agree with the reference. Its torque
    clip is off: panda-py saturates torques after the law, in the Panda class.
    """
    rng = np.random.default_rng(10)
    ref_ = vic.VicReference(clip_torque=False, **VARIANTS[variant])
    action_size = 12 if ref_.variable_stiffness else 6
    state = random_state(rng)
    pose = state["pose"]
    ref_.reset(pose[:3, 3], wxyz(pose[:3, :3]))
    position_ref, orientation_ref = pose[:3, 3].copy(), xyzw(pose[:3, :3])
    worst = 0.0
    for step in range(20):
        # A new state each step stands in for the arm having moved.
        state = random_state(rng)
        pose = state["pose"]
        if step:
            a = rng.uniform(-1.2, 1.2, action_size)
            ref_.process_action(a, pose[:3, 3], wxyz(pose[:3, :3]))
            clipped = np.clip(a, -1, 1)
            position_ref, orientation_ref = TaskImpedance.step_reference_update(
                position_ref, orientation_ref, clipped[:3] * vic.POS_STEP,
                clipped[3:6] * vic.ROT_STEP, pose[:3, 3], xyzw(pose[:3, :3]),
                vic.LEASH_POS, vic.LEASH_ROT,
            )
            np.testing.assert_allclose(position_ref, ref_.x_ref, atol=1e-12)
            # q and -q are the same orientation.
            assert abs(np.dot(orientation_ref, xyzw(ref_.q_ref))) == pytest.approx(1, abs=1e-12)
        for _ in range(20):
            state["dq"] = rng.normal(scale=0.3, size=7)
            tau, tel = reference_torque(ref_, state)
            ours = law(
                state, position_ref, orientation_ref, stiffness=tel["K"],
                nullspace=ref_.nullspace_projection, k_ns=ref_.nullspace_kp, alpha=tel["alpha"],
            )
            np.testing.assert_allclose(ours["wrench_active"], tel["w_active"], atol=1e-9)
            np.testing.assert_allclose(ours["wrench_passive"], tel["w_passive"], atol=1e-9)
            np.testing.assert_allclose(ours["tau_nullspace"], tel["tau_ns"], atol=1e-9)
            worst = max(worst, np.linalg.norm(ours["tau"] - tau) / np.linalg.norm(tau))
    # The request's acceptance is 1 % of the torque norm.
    assert worst < 1e-9


@pytest.mark.parametrize("nullspace", ["dynamic", "kinematic"])
@pytest.mark.parametrize("alpha", [1.0, 0.3, 0.0])
def test_law_matches_vic_reference_on_random_states(nullspace, alpha):
    rng = np.random.default_rng(0)
    ref_ = vic.VicReference(clip_torque=False, nullspace_projection=nullspace)
    for _ in range(200):
        state = random_state(rng)
        position_ref, orientation_ref = near_reference(rng, state)
        ref_.K = np.exp(rng.uniform(np.log([50] * 3 + [5] * 3), np.log([800] * 3 + [150] * 3)))
        ref_.x_ref, ref_.q_ref = position_ref, np.roll(orientation_ref, 1)
        tau, tel = reference_torque(ref_, state)
        ours = law(state, position_ref, orientation_ref, stiffness=ref_.K, nullspace=nullspace,
                   alpha=alpha)
        wrench = alpha * tel["w_active"] + tel["w_passive"]
        np.testing.assert_allclose(ours["tau"], state["jacobian"].T @ wrench + tel["tau_ns"],
                                   rtol=1e-9, atol=1e-9)
        if alpha == 1.0:
            np.testing.assert_allclose(ours["tau"], tau, rtol=1e-9, atol=1e-9)


def test_leash_matches_vic_reference():
    rng = np.random.default_rng(1)
    for _ in range(200):
        state = random_state(rng)
        pose = state["pose"]
        ref_ = vic.VicReference()
        start_x, start_q = near_reference(rng, state, rotation=1.5, translation=0.1)
        ref_.reset(start_x, np.roll(start_q, 1))
        a = rng.uniform(-1, 1, 6)
        ref_.process_action(a, pose[:3, 3], wxyz(pose[:3, :3]))
        position_ref, orientation_ref = TaskImpedance.step_reference_update(
            start_x, start_q, a[:3] * vic.POS_STEP, a[3:6] * vic.ROT_STEP,
            pose[:3, 3], xyzw(pose[:3, :3]), vic.LEASH_POS, vic.LEASH_ROT,
        )
        np.testing.assert_allclose(position_ref, ref_.x_ref, atol=1e-12)
        assert abs(np.dot(orientation_ref, xyzw(ref_.q_ref))) == pytest.approx(1, abs=1e-12)


def test_reference_update_without_a_leash_is_the_plain_step():
    position, orientation = np.zeros(3), xyzw(np.eye(3))
    position_ref, orientation_ref = TaskImpedance.step_reference_update(
        position, orientation, [0.5, 0, 0], [0, 0, np.pi / 2], position, orientation
    )
    np.testing.assert_allclose(position_ref, [0.5, 0, 0])
    np.testing.assert_allclose(orientation_ref, [0, 0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-12)


# -- properties of the law ------------------------------------------------------


def test_matches_with_nullspace_damping_in_the_limit():
    rng = np.random.default_rng(1)
    state = random_state(rng)
    position_ref, orientation_ref = near_reference(rng, state)
    for nullspace in ("dynamic", "kinematic"):
        exact = law(state, position_ref, orientation_ref, nullspace=nullspace)["tau"]
        damped = law(state, position_ref, orientation_ref, nullspace=nullspace,
                     nullspace_damping=1e-9)["tau"]
        assert np.linalg.norm(damped - exact) < 1e-5 * np.linalg.norm(exact)


def test_dynamic_nullspace_does_not_leak_into_the_task():
    """J M^-1 tau_ns, the task acceleration the posture term causes, vanishes."""
    rng = np.random.default_rng(2)
    for _ in range(100):
        state = random_state(rng)
        tau_ns = law(state, *near_reference(rng, state))["tau_nullspace"]
        leak = state["jacobian"] @ np.linalg.solve(state["mass"], tau_ns)
        assert np.linalg.norm(leak) < 1e-9 * np.linalg.norm(tau_ns)


def test_nullspace_armature_is_the_mass_matrix_plus_its_diagonal():
    """The armature enters the posture term as if it were part of M, and only
    there: the task torque does not change."""
    rng = np.random.default_rng(12)
    armature = np.full(7, 0.1)
    for _ in range(20):
        state = random_state(rng)
        ref = near_reference(rng, state)
        ours = law(state, *ref, nullspace_armature=armature)
        heavier = law(dict(state, mass=state["mass"] + np.diag(armature)), *ref)
        np.testing.assert_allclose(ours["tau_nullspace"], heavier["tau_nullspace"], atol=1e-12)
        np.testing.assert_allclose(ours["tau_task"], law(state, *ref)["tau_task"], atol=1e-12)
        kinematic = law(state, *ref, nullspace="kinematic", nullspace_armature=armature)
        np.testing.assert_allclose(kinematic["tau"],
                                   law(state, *ref, nullspace="kinematic")["tau"], atol=1e-12)


def test_nullspace_armature_setter():
    ctrl = TaskImpedance()
    np.testing.assert_array_equal(ctrl.get_nullspace_armature(), 0)
    ctrl.set_nullspace_armature(np.full(7, 0.1))
    np.testing.assert_array_equal(ctrl.get_nullspace_armature(), 0.1)
    with pytest.raises(ValueError):
        ctrl.set_nullspace_armature(np.r_[np.zeros(6), -0.1])


def test_kinematic_nullspace_does_not_move_the_task_kinematically():
    """The kinematic projection gives no task velocity: J tau_ns = 0."""
    rng = np.random.default_rng(3)
    state = random_state(rng)
    tau_ns = law(state, *near_reference(rng, state), nullspace="kinematic")["tau_nullspace"]
    assert np.linalg.norm(state["jacobian"] @ tau_ns) < 1e-9 * np.linalg.norm(tau_ns)


def test_zero_nullspace_stiffness_drops_the_posture_term():
    rng = np.random.default_rng(4)
    state = random_state(rng)
    for nullspace in ("dynamic", "kinematic", "none"):
        out = law(state, *near_reference(rng, state), nullspace=nullspace, k_ns=0.0)
        np.testing.assert_array_equal(out["tau_nullspace"], 0)


def test_alpha_scales_the_spring_but_never_the_damping():
    rng = np.random.default_rng(5)
    state = random_state(rng)
    ref = near_reference(rng, state)
    full, gated = law(state, *ref, nullspace="none"), law(state, *ref, nullspace="none", alpha=0.0)
    np.testing.assert_allclose(gated["wrench_active"], full["wrench_active"])
    np.testing.assert_allclose(
        gated["tau_task"], state["jacobian"].T @ full["wrench_passive"], atol=1e-12
    )


def test_damping_is_critical():
    np.testing.assert_allclose(TaskImpedance.critical_damping(STIFFNESS), 2 * np.sqrt(STIFFNESS))
    np.testing.assert_allclose(TaskImpedance.critical_damping(STIFFNESS, 0.5), np.sqrt(STIFFNESS))


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
    pose = state["pose"]
    out = law(state, pose[:3, 3], xyzw(pose[:3, :3]))
    np.testing.assert_allclose(out["wrench_active"], 0, atol=1e-9)


# -- the controller object -----------------------------------------------------


def test_constructor():
    ctrl = TaskImpedance()
    assert ctrl.frame == "end_effector"
    assert ctrl.telemetry_capacity == 0
    assert ctrl.get_leash() == (float("inf"), float("inf"))
    np.testing.assert_allclose(ctrl.get_damping(), 2 * np.sqrt(ctrl.get_stiffness()))
    offset = np.eye(4)
    offset[2, 3] = 0.088
    ctrl = TaskImpedance(STIFFNESS, frame="flange", frame_transform=offset, nullspace="kinematic",
                         telemetry=1000)
    assert ctrl.frame == "flange"
    assert ctrl.telemetry_capacity == 1000
    np.testing.assert_allclose(ctrl.frame_transform, offset)
    ctrl.set_stiffness(STIFFNESS * 2)
    np.testing.assert_allclose(ctrl.get_stiffness(), STIFFNESS * 2)
    np.testing.assert_allclose(ctrl.get_damping(), 2 * np.sqrt(STIFFNESS * 2))
    ctrl.set_damping_ratio(0.5)
    np.testing.assert_allclose(ctrl.get_damping(), np.sqrt(STIFFNESS * 2))
    ctrl.step_reference([0.01, 0, 0], [0, 0, 0], STIFFNESS)
    np.testing.assert_allclose(ctrl.get_stiffness(), STIFFNESS)
    ctrl.set_leash(0.025, 0.5)
    assert ctrl.get_leash() == (0.025, 0.5)
    assert len(ctrl.read_telemetry()["tick"]) == 0
    with pytest.raises(ValueError):
        ctrl.set_leash(0.0, 0.5)
    with pytest.raises(ValueError):
        TaskImpedance(nullspace="pseudo")
    with pytest.raises(ValueError):
        TaskImpedance(frame="hand")
    with pytest.raises(ValueError):
        TaskImpedance(frame_transform=np.ones((4, 4)))


def test_telemetry_check():
    from panda_py import telemetry  # pylint: disable=import-outside-toplevel

    log = {"tick": np.array([0, 1, 2, 4, 5]), "duration": np.array([0, 1e-3, 1e-3, 3e-3, 1e-3])}
    assert telemetry.check(log) == {"samples": 5, "missing": 1, "lost_cycles": 2, "ok": False}
    log = {"tick": np.arange(5), "duration": np.array([0] + [1e-3] * 4)}
    assert telemetry.check(log)["ok"]


# -- guards ------------------------------------------------------------------


def test_guard_configuration():
    from panda_py import safety  # pylint: disable=import-outside-toplevel

    ctrl = TaskImpedance()
    guard = ctrl.get_guard()
    assert guard["force"] == float("inf") and guard["workspace"] == []
    assert ctrl.guard_state == {"tripped": False, "reason": "none", "time": 0.0,
                                "value": 0.0, "joint": -1}
    box = safety.box_along_axis([0.45, 0.0, 0.067], [0, 0, 1], 0.06, 0.07, 0.1)
    ctrl.set_guard(force=80, force_time=0.05, saturation_time=0.1, speed=0.5,
                   joint_velocity=np.full(7, 2.0), workspace=[box])
    guard = ctrl.get_guard()
    assert (guard["force"], guard["force_time"], guard["saturation_time"], guard["speed"]) == (
        80, 0.05, 0.1, 0.5)
    np.testing.assert_allclose(guard["joint_velocity"], 2.0)
    np.testing.assert_allclose(guard["workspace"][0][0], box[0])
    assert guard["workspace_point"] == "end_effector"
    with pytest.raises(ValueError):
        ctrl.set_guard(workspace=[box] * 9)
    with pytest.raises(ValueError):
        ctrl.set_guard(workspace_point="tip")
    ctrl.set_guard(force=80, force_bias=[0.4, 0.3, 1.7])
    np.testing.assert_allclose(ctrl.get_guard()["force_bias"], [0.4, 0.3, 1.7])
    ctrl.set_guard()
    assert ctrl.get_guard()["speed"] == float("inf")
    np.testing.assert_array_equal(ctrl.get_guard()["force_bias"], 0)


def test_box_along_axis():
    from panda_py import safety  # pylint: disable=import-outside-toplevel

    axis = np.array([0.003, 0.0043, 1.0])
    pose, half = safety.box_along_axis([0.45, -0.007, 0.067], axis, 0.06, 0.07, 0.1)
    z = axis / np.linalg.norm(axis)
    np.testing.assert_allclose(pose[:3, 2], z)
    np.testing.assert_allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(half, [0.06, 0.06, 0.085])
    # The box spans from 70 mm below the origin to 100 mm above, along the axis.
    bottom = pose[:3, 3] - z * half[2]
    top = pose[:3, 3] + z * half[2]
    np.testing.assert_allclose(bottom, np.array([0.45, -0.007, 0.067]) - 0.07 * z)
    np.testing.assert_allclose(top, np.array([0.45, -0.007, 0.067]) + 0.1 * z)


def test_set_collision_thresholds():
    from panda_py import safety  # pylint: disable=import-outside-toplevel

    class Robot:  # pylint: disable=too-few-public-methods
        def set_collision_behavior(self, *args):
            self.args = args  # pylint: disable=attribute-defined-outside-init

    robot = Robot()
    safety.set_collision_thresholds(robot, 100, 30, contact=0.5)
    lower_joint, upper_joint, _, _, lower_wrench, upper_wrench, _, _ = robot.args
    np.testing.assert_allclose(upper_joint, 0.9 * safety.TAU_J_MAX)
    np.testing.assert_allclose(lower_joint, 0.45 * safety.TAU_J_MAX)
    assert upper_wrench == [100, 100, 100, 30, 30, 30]
    assert lower_wrench == [50, 50, 50, 15, 15, 15]
    with pytest.raises(ValueError):
        safety.set_collision_thresholds(robot, 100, 30, contact=0)


# -- tank ----------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["power", "impulse"])
@pytest.mark.parametrize("smooth", [0.25, None])
def test_tank_matches_vic_reference(mode, smooth):
    """Over a simulated trial: the gate, level and draw of every tick."""
    if mode == "impulse" and smooth is None:
        pytest.skip("vic_reference offers impulse mode with the smooth gate only")
    rng = np.random.default_rng(12)
    E0 = 0.05 if mode == "power" else 0.5
    ref_ = vic.VicReference(clip_torque=False, tank_E0=E0, tank_mode=mode,
                            tank_smooth_frac=smooth)
    state = random_state(rng)
    pose = state["pose"]
    ref_.reset(pose[:3, 3], wxyz(pose[:3, :3]))
    level, drawn, ramped = E0, 0.0, False
    for _ in range(40):
        state = random_state(rng)
        pose = state["pose"]
        ref_.process_action(rng.uniform(-1, 1, 6), pose[:3, 3], wxyz(pose[:3, :3]))
        for _ in range(20):
            state["dq"] = rng.normal(scale=0.3, size=7)
            _, tel = reference_torque(ref_, state)
            v6 = state["jacobian"] @ state["dq"]
            alpha, level, drawn = TaskImpedance.tank_step(
                level, drawn, tel["w_active"], v6, 1e-3, E0, mode, smooth or 0.0)
            assert alpha == pytest.approx(tel["alpha"], abs=1e-12)
            assert level == pytest.approx(tel["E_T"], abs=1e-12)
            assert drawn == pytest.approx(ref_.E_drawn, abs=1e-12)
            ramped |= 0 < alpha < 1
    assert ramped, "the trial never reached the gate's ramp"


def test_tank_with_a_huge_budget_changes_nothing():
    """Item 10: with E0 = 1e9 the controller is the plain one."""
    rng = np.random.default_rng(13)
    for _ in range(50):
        w, v = rng.normal(size=6), rng.normal(size=6)
        alpha, _, _ = TaskImpedance.tank_step(1e9, 0.0, w, v, 1e-3, 1e9)
        assert alpha == 1.0


def test_tank_gate_ramps_with_the_level():
    for level in (5.0, 1.25, 0.625, 0.0):
        alpha, _, _ = TaskImpedance.tank_step(level, 0.0, np.zeros(6), np.zeros(6), 1e-3, 5.0)
        assert alpha == pytest.approx(min(level / (0.25 * 5.0), 1.0))


def test_tank_configuration():
    ctrl = TaskImpedance()
    assert ctrl.get_tank() is None
    ctrl.set_tank(33.0, "power")
    assert ctrl.get_tank() == {"E0": 33.0, "mode": "power", "smooth_fraction": 0.25}
    ctrl.set_tank(16.4, "impulse", 0.25)
    assert ctrl.get_tank()["mode"] == "impulse"
    ctrl.set_tank(None)
    assert ctrl.get_tank() is None
    with pytest.raises(ValueError):
        ctrl.set_tank(0.0)
    with pytest.raises(ValueError):
        ctrl.set_tank(1.0, "energy")


# -- joint servo -------------------------------------------------------------------


def test_joint_position_api():
    from panda_py.controllers import JointPosition  # pylint: disable=import-outside-toplevel

    stiffness = np.array([600, 600, 600, 600, 250, 150, 50], float)
    damping = np.array([30, 30, 30, 30, 10, 10, 5], float)
    ctrl = JointPosition(stiffness, damping, telemetry=1000)
    np.testing.assert_array_equal(ctrl.get_stiffness(), stiffness)
    np.testing.assert_array_equal(ctrl.get_damping(), damping)
    ctrl.set_stiffness(stiffness / 2)
    np.testing.assert_array_equal(ctrl.get_stiffness(), stiffness / 2)
    assert ctrl.telemetry_capacity == 1000
    assert len(ctrl.read_telemetry()["tick"]) == 0
    ctrl.step_control(np.full(7, 0.01))
    ctrl.set_guard(force=150, force_time=0.02)
    assert ctrl.get_guard()["force"] == 150
    assert not ctrl.guard_state["tripped"]
