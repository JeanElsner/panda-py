"""Reference implementation of the simulator's Cartesian VIC law, in NumPy.

A line-by-line port of mjlab_insertion/actions/cartesian_vic.py (the action
term the insertion policies were trained against), for sim-to-hardware
parity tests: feed logged hardware states through `process_action` (50 Hz)
and `torque` (1 kHz) and compare with the torques the arm's controller
commanded. Conventions: quaternions are (w, x, y, z); every vector is in the
arm's base frame; J is the 6x7 geometric Jacobian at the control frame
(flange) with linear rows first; M is the 7x7 joint mass matrix.

    ref = VicReference(variable_stiffness=False)          # fixed gains 400 / 30
    ref.reset(x_flange, q_flange)                          # trial start
    ref.process_action(a, x_flange, q_flange)              # every 20 ms
    tau, tel = ref.torque(x, quat, v, w, q, dq, J, M, dt)  # every 1 ms

Tank variants: VicReference(tank_E0=33.0, tank_mode="power") etc.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

K_TRANS_RANGE = (50.0, 800.0)      # N/m, log-linear in the action
K_ROT_RANGE = (5.0, 150.0)         # N m/rad
POS_STEP, ROT_STEP = 0.01, 0.05    # m, rad per policy step
LEASH_POS, LEASH_ROT = 0.025, 0.5  # m, rad (campaign value; the file default is 0.10 m)
FIXED_K_TRANS, FIXED_K_ROT = 400.0, 30.0
ZETA = 1.0
NULLSPACE_KP = 10.0
POSTURE_ABOVE_HOLE = np.array([0.0, 0.2987, 0.0, -2.5492, 0.0, 2.8479, -0.7853])
START_KEYFRAME = np.array([0.0, 0.2382, 0.0, -2.5591, 0.0, 2.7973, -0.7853])
EFFORT_LIMIT = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / float(np.dot(q, q))


def quat_to_axis_angle(q):
    w = float(np.clip(q[0], -1.0, 1.0))
    angle = 2.0 * math.acos(abs(w))
    sign = -1.0 if q[0] < 0 else 1.0
    s = math.sqrt(max(1.0 - w * w, 1e-12))
    axis = sign * np.asarray(q[1:4]) / s
    return np.zeros(3) if angle < 1e-4 else axis * angle


def axis_angle_to_quat(v):
    angle = max(float(np.linalg.norm(v)), 1e-9)
    return np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * np.asarray(v) / angle])


@dataclass
class VicReference:
    variable_stiffness: bool = False
    k_trans: float = FIXED_K_TRANS
    k_rot: float = FIXED_K_ROT
    zeta: float = ZETA
    leash_pos: float = LEASH_POS
    leash_rot: float = LEASH_ROT
    nullspace_kp: float = NULLSPACE_KP
    nullspace_projection: str = "dynamic"      # "dynamic" (the twins) | "kinematic" (panda-py today)
    q0: np.ndarray = field(default_factory=lambda: POSTURE_ABOVE_HOLE.copy())
    tank_E0: float | None = None
    tank_mode: str = "power"                   # "power" [J] | "impulse" [N s]
    tank_smooth_frac: float | None = 0.25
    clip_torque: bool = True

    def __post_init__(self):
        self.K = np.array([self.k_trans] * 3 + [self.k_rot] * 3, dtype=float)
        self.x_ref = None; self.q_ref = None
        self.E_T = float(self.tank_E0) if self.tank_E0 is not None else None
        self.E_drawn = 0.0
        self._log_lo = (math.log(K_TRANS_RANGE[0]), math.log(K_ROT_RANGE[0]))
        self._log_hi = (math.log(K_TRANS_RANGE[1]), math.log(K_ROT_RANGE[1]))

    # -- trial start -----------------------------------------------------
    def reset(self, x, quat):
        self.x_ref = np.asarray(x, float).copy(); self.q_ref = np.asarray(quat, float).copy()
        if self.tank_E0 is not None:
            self.E_T = float(self.tank_E0); self.E_drawn = 0.0

    # -- 50 Hz: integrate the reference, leash it, map the stiffness ------
    def process_action(self, a, x, quat):
        a = np.clip(np.asarray(a, float), -1.0, 1.0)
        x = np.asarray(x, float); quat = np.asarray(quat, float)
        if self.x_ref is None:
            self.reset(x, quat)
        self.x_ref = self.x_ref + a[0:3] * POS_STEP
        d = self.x_ref - x; dn = max(float(np.linalg.norm(d)), 1e-9)
        self.x_ref = x + d * (min(dn, self.leash_pos) / dn)
        self.q_ref = quat_mul(axis_angle_to_quat(a[3:6] * ROT_STEP), self.q_ref)
        self.q_ref = self.q_ref / np.linalg.norm(self.q_ref)
        e = quat_to_axis_angle(quat_mul(self.q_ref, quat_inv(quat)))
        en = max(float(np.linalg.norm(e)), 1e-9)
        e = e * (min(en, self.leash_rot) / en)
        self.q_ref = quat_mul(axis_angle_to_quat(e), quat)
        if self.variable_stiffness:
            for i in range(2):
                raw = a[6 + 3 * i: 9 + 3 * i]
                log_k = 0.5 * (raw + 1.0) * (self._log_hi[i] - self._log_lo[i]) + self._log_lo[i]
                self.K[3 * i: 3 * i + 3] = np.exp(log_k)
        return self.x_ref.copy(), self.q_ref.copy(), self.K.copy()

    # -- 1 kHz: the impedance law -----------------------------------------
    def torque(self, x, quat, v, w, q, dq, J, M, dt):
        """Returns (tau[7], telemetry dict). v, w: flange linear/angular velocity."""
        x = np.asarray(x, float); quat = np.asarray(quat, float)
        if self.x_ref is None:
            self.reset(x, quat)
        e_p = self.x_ref - x
        e_r = quat_to_axis_angle(quat_mul(self.q_ref, quat_inv(quat)))
        D = 2.0 * self.zeta * np.sqrt(self.K)
        w_active = np.concatenate([self.K[:3] * e_p, self.K[3:] * e_r])
        v6 = np.concatenate([np.asarray(v, float), np.asarray(w, float)])
        w_passive = -D * v6
        alpha, spent = 1.0, 0.0
        if self.E_T is not None:
            P = float(np.linalg.norm(w_active[:3])) if self.tank_mode == "impulse" \
                else max(float(np.dot(w_active, v6)), 0.0)
            if self.tank_smooth_frac is not None:
                alpha = float(np.clip(self.E_T / (self.tank_smooth_frac * self.tank_E0), 0.0, 1.0))
                spent = min(alpha * P * dt, self.E_T)
            else:
                draw = P * dt
                alpha = float(np.clip(self.E_T / max(draw, 1e-12), 0.0, 1.0)) if draw > self.E_T else 1.0
                spent = alpha * draw
            self.E_T -= spent; self.E_drawn += spent
        wrench = alpha * w_active + w_passive
        J = np.asarray(J, float); M = np.asarray(M, float)
        tau_task = J.T @ wrench
        q = np.asarray(q, float); dq = np.asarray(dq, float)
        u = self.nullspace_kp * (self.q0 - q) - 2.0 * math.sqrt(self.nullspace_kp) * dq
        if self.nullspace_kp == 0.0:
            tau_ns = np.zeros(7)
        elif self.nullspace_projection == "kinematic":
            N = np.eye(7) - J.T @ np.linalg.solve(J @ J.T, J)
            tau_ns = N @ u
        else:
            Minv = np.linalg.inv(M)
            A = J @ Minv @ J.T
            N = np.eye(7) - J.T @ np.linalg.solve(A, J @ Minv)
            tau_ns = N @ (M @ u)
        tau = tau_task + tau_ns
        if self.clip_torque:
            tau = np.clip(tau, -EFFORT_LIMIT, EFFORT_LIMIT)
        tel = dict(x_ref=self.x_ref.copy(), q_ref=self.q_ref.copy(), K=self.K.copy(), D=D,
                   w_active=w_active, w_passive=w_passive, alpha=alpha, spent=spent,
                   E_T=self.E_T, tau_task=tau_task, tau_ns=tau_ns)
        return tau, tel


def tank_level(ref: VicReference) -> float:
    """The policy's tank observation: clamp(E_T / E0, 0, 1)."""
    assert ref.tank_E0 is not None
    return float(np.clip(ref.E_T / ref.tank_E0, 0.0, 1.0))


if __name__ == "__main__":
    # Self-test on a random but consistent state: the dynamic nullspace term must
    # not accelerate the task space (J M^-1 tau_ns = 0), the wrench must be K e - D v,
    # and the tank meter must reproduce the integral of the gated active power.
    rng = np.random.default_rng(0)
    A = rng.normal(size=(7, 7)); M = A @ A.T + 7 * np.eye(7)
    J = rng.normal(size=(6, 7))
    q = START_KEYFRAME + rng.normal(scale=0.05, size=7); dq = rng.normal(scale=0.1, size=7)
    x = np.array([0.45, 0.0, 0.18]); quat = np.array([0.0, 1.0, 0.0, 0.0]); v = rng.normal(scale=0.01, size=3); w = rng.normal(scale=0.01, size=3)
    ref = VicReference(); ref.reset(x, quat)
    ref.process_action(np.array([1.0, 0, 0, 0, 0.5, 0]), x, quat)   # 10 mm x step, 25 mrad rotation
    tau, tel = ref.torque(x, quat, v, w, q, dq, J, M, 1e-3)
    leak = np.linalg.norm(J @ np.linalg.solve(M, tel["tau_ns"])) / max(np.linalg.norm(tel["tau_ns"]), 1e-9)
    assert leak < 1e-9, leak
    assert np.allclose(tel["w_active"][:3], 400.0 * (tel["x_ref"] - x))
    assert np.allclose(tel["w_passive"], -2 * np.sqrt(ref.K) * np.concatenate([v, w]))
    print(f"dynamic nullspace leak {leak:.1e}; |tau| {np.linalg.norm(tau):.2f} N m; x_ref step {1e3*(tel['x_ref']-x)} mm")
    t = VicReference(tank_E0=1.0, tank_mode="power"); t.reset(x, quat)
    t.process_action(np.array([1.0, 0, 0, 0, 0, 0]), x, quat)
    drawn = 0.0
    for _ in range(2000):
        tau, tel = t.torque(x, quat, np.array([0.05, 0, 0]), np.zeros(3), q, dq, J, M, 1e-3)
        drawn += tel["spent"]
    assert abs(drawn - t.E_drawn) < 1e-9 and 0.0 <= tank_level(t) <= 1.0
    print(f"tank: E0 1 J, drawn {t.E_drawn:.3f} J in 2 s against a 0.05 m/s wall, alpha now {tel['alpha']:.3f}")
    print("SELF-TEST OK")
