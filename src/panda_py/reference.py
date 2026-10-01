"""
NumPy reference of the task impedance law, for checking
:py:class:`panda_py.controllers.TaskImpedance` against.

It is a line-by-line transcription of the simulator's controller,
``CartesianVicAction.apply_actions`` in
``mjlab_insertion/actions/cartesian_vic.py`` (mjlab-insertion b82edb9), for a
single state instead of a batch, and kept deliberately independent of
panda-py's C++ code: quaternions are scalar-first internally, as in the
simulator, and the projectors are formed the way the simulator forms them.
What the simulator computes and the controller does not, the tank meter and the
learned nullspace action, is left out; alpha is an input.

One difference from the controller is kept on purpose, because the simulator
has it: its axis-angle conversion returns zero below a rotation of 1e-4 rad,
where the controller continues smoothly. The torques differ there by at most
1e-4 times the rotational stiffness.
"""

import numpy as np

__all__ = ["task_impedance"]


def _quat_mul(a, b):
    """Hamilton product, scalar first."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / np.dot(q, q)


def _quat_to_axis_angle(q):
    """cartesian_vic._quat_to_axis_angle for one quaternion."""
    w = np.clip(q[0], -1.0, 1.0)
    angle = 2.0 * np.arccos(abs(w))
    sign = -1.0 if q[0] < 0 else 1.0
    s = np.sqrt(max(1.0 - w * w, 1e-12))
    axis = sign * q[1:4] / s
    return np.zeros(3) if angle < 1e-4 else axis * angle


def _matrix_to_quat(r):
    """Scalar-first unit quaternion of a rotation matrix."""
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


def _damped(a, damping):
    """cartesian_vic._damped: A + lambda I, lambda relative to mean(diag(A))."""
    lam = damping * max(np.mean(np.diag(a)), 1e-12)
    return a + lam * np.eye(a.shape[-1])


def task_impedance(
    q,
    dq,
    pose,
    jacobian,
    mass,
    position_ref,
    orientation_ref,
    stiffness,
    q_nullspace,
    nullspace_stiffness,
    nullspace="dynamic",
    zeta=1.0,
    alpha=1.0,
    nullspace_damping=0.0,
):
    """
    Joint torques of the simulator's law for one state.

    Args:
      q, dq: Joint positions and velocities.
      pose: 4x4 pose of the control frame in the base frame.
      jacobian: 6x7 geometric Jacobian of the control frame, base frame.
      mass: 7x7 joint mass matrix.
      position_ref: Reference position of the control frame.
      orientation_ref: Reference orientation, a scalar-last quaternion as
        everywhere in panda-py.
      stiffness: Diagonal stiffness, translational part first.
      q_nullspace, nullspace_stiffness: Posture target and gain.
      nullspace: ``"dynamic"``, ``"kinematic"`` or ``"none"``.
      zeta: Damping ratio, D = 2 zeta sqrt(K).
      alpha: Tank gate on the active wrench.
      nullspace_damping: The simulator's ``nullspace_damping``.

    Returns:
      dict with ``wrench_active``, ``wrench_passive``, ``tau_task``,
      ``tau_nullspace`` and ``tau``.
    """
    q, dq = np.asarray(q, float), np.asarray(dq, float)
    pose, J, M = np.asarray(pose, float), np.asarray(jacobian, float), np.asarray(mass, float)
    K = np.asarray(stiffness, float)
    x, xyzw = np.asarray(position_ref, float), np.asarray(orientation_ref, float)
    q_ref = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])
    q_ref = q_ref / np.linalg.norm(q_ref)
    pos, quat = pose[:3, 3], _matrix_to_quat(pose[:3, :3])
    vel = J @ dq
    v_lin, v_ang = vel[:3], vel[3:]

    # apply_actions, from "e_p = self._x_ref - pos"
    e_p = x - pos
    e_r = _quat_to_axis_angle(_quat_mul(q_ref, _quat_inv(quat)))
    D = 2.0 * zeta * np.sqrt(K)
    w_active = np.concatenate([K[:3] * e_p, K[3:] * e_r])
    w_passive = np.concatenate([-D[:3] * v_lin, -D[3:] * v_ang])
    wrench = alpha * w_active + w_passive
    tau_task = J.T @ wrench

    k_ns = nullspace_stiffness
    tau_ns = np.zeros(7)
    if nullspace != "none" and k_ns != 0.0:
        JT = J.T
        u_null = k_ns * (np.asarray(q_nullspace, float) - q) - 2.0 * k_ns**0.5 * dq
        eye = np.eye(7)
        if nullspace == "kinematic":
            JJT = _damped(J @ JT, nullspace_damping)
            N = eye - JT @ np.linalg.solve(JJT, J)
            tau_ns = N @ u_null
        elif nullspace == "dynamic":
            MiJT = np.linalg.solve(M, JT)
            A = _damped(J @ MiJT, nullspace_damping)
            M_task = np.linalg.inv(A)
            J_eef_inv = M_task @ MiJT.T
            N = eye - JT @ J_eef_inv
            tau_ns = N @ (M @ u_null)
        else:
            raise ValueError(f"unknown nullspace {nullspace!r}")
    return {
        "wrench_active": w_active,
        "wrench_passive": w_passive,
        "tau_task": tau_task,
        "tau_nullspace": tau_ns,
        "tau": tau_task + tau_ns,
    }
