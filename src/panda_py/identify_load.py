"""
``panda-identify-load``: estimates a load that is mounted but not configured.

Anything on the arm beyond the end effector configured in Desk, a camera or a
mount say, adds weight the robot does not compensate. At rest it shows in the
robot's estimate of the external joint torques, which for a load of mass m
with its centre of mass at c (flange frame) is linear in m and m c:

    tau_ext(q) = J(q)^T [m g; (R(q) c) x m g] + b,

J the flange's Jacobian, R its orientation, g gravity in the base frame, and
b a constant offset per joint (the torque sensors' biases). Holding the arm
still in several wrist orientations and solving the stack by least squares
gives m, c and b. The inertia cannot be identified at rest; for the small
loads this is for, a solid box around the centre of mass is close enough.

The result is relative to what the robot already knows: a load set in Desk
or with ``set_load`` is combined with the estimate.

    panda-identify-load <robot-ip> [--out FILE]
"""

import argparse
import json
import sys
import time

import numpy as np

import panda_py
from panda_py import constants

__all__ = ["fit", "main", "POSE_OFFSETS"]

G = np.array([0.0, 0.0, -9.81])
"""Gravity in the base frame: a robot mounted upright on a level surface."""

POSE_OFFSETS = np.array([
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1.0, 0],
    [0, 0, 0, 0, 0, -0.9, 0],
    [0, 0, 0, 0, 0, 1.0, 1.5],
    [0, 0, 0, 0, 0, -0.9, 1.5],
    [0, 0, 0, 0, 0, 1.0, -1.5],
    [0, 0, 0, 0, 0, -0.9, -1.5],
    [0, 0, 0, 0, 1.2, 0.8, 0],
    [0, 0, 0, 0, -1.2, 0.8, 0],
    [0, 0, 0, 0, 1.2, -0.6, 1.0],
    [0, 0, 0, 0, -1.2, -0.6, -1.0],
    [0.4, 0.3, 0, 0.3, 0, 0, 0.8],
    [-0.4, -0.2, 0, -0.3, 0, 0, -0.8],
])
"""
Joint offsets from the start pose of the poses the arm is held still in. The
centre of mass along a direction only shows when that direction is
horizontal, so the flange is tilted well away from pointing down, both ways,
and turned about its axis. The component along the flange's axis is still the
least certain.
"""


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def regressor(q):
    """The 7x4 matrix A with tau_ext = A [m, m c] for a load at the flange."""
    J = panda_py.jacobian(q, F_T_EE=np.eye(4))
    R = panda_py.fk(q, F_T_EE=np.eye(4))[:3, :3]
    A = np.zeros((7, 4))
    A[:, 0] = J[:3].T @ G  # the force m g
    # The moment (R c) x (m g) = -[g]x R (m c).
    A[:, 1:] = J[3:].T @ (-_skew(G) @ R)
    return A


def fit(qs, taus):
    """
    Mass, centre of mass (flange frame) and joint torque offsets from joint
    positions and the external torques held at them, with their standard
    errors and the residual.
    """
    qs, taus = np.asarray(qs, float), np.asarray(taus, float)
    rows = [np.hstack([regressor(q), np.eye(7)]) for q in qs]
    A, y = np.vstack(rows), taus.reshape(-1)
    theta, *_ = np.linalg.lstsq(A, y, rcond=None)
    residual = y - A @ theta
    dof = max(len(y) - A.shape[1], 1)
    sigma2 = residual @ residual / dof
    cov = sigma2 * np.linalg.pinv(A.T @ A)
    m, mc = theta[0], theta[1:4]
    # The sign convention of the external torque flips m and m c together, so
    # the mass is |m| and the centre of mass mc / m either way.
    c = mc / m if abs(m) > 1e-9 else np.zeros(3)
    m_std = float(np.sqrt(cov[0, 0]))
    # First-order error of c = mc / m.
    jac = np.hstack([-mc[:, None] / m**2, np.eye(3) / m]) if abs(m) > 1e-9 else np.zeros((3, 4))
    c_std = np.sqrt(np.clip(np.diag(jac @ cov[:4, :4] @ jac.T), 0, None))
    return {
        "mass": float(abs(m)), "mass_std": m_std,
        "com": c.tolist(), "com_std": c_std.tolist(),
        "offsets": theta[4:].tolist(),
        "residual_rms": float(np.sqrt(np.mean(residual**2))),
        "sign": float(np.sign(m)),
    }


def box_inertia(mass, size=0.06):
    """Inertia of a solid cube of edge `size` about its centre, column-major 3x3."""
    i = mass * size**2 / 6
    return [i, 0, 0, 0, i, 0, 0, 0, i]


def combine(m1, c1, m2, c2):
    """Mass and centre of mass of two point masses."""
    m = m1 + m2
    if m <= 0:
        return 0.0, np.zeros(3)
    return m, (m1 * np.asarray(c1) + m2 * np.asarray(c2)) / m


def collect(panda, samples=100, settle=1.5):
    """Holds each pose still and averages the external joint torques."""
    start = np.asarray(constants.JOINT_POSITION_START)
    limits = panda.limits
    qs, taus = [], []
    for n, offset in enumerate(POSE_OFFSETS, 1):
        target = np.clip(start + offset, limits.q_lower + 0.05, limits.q_upper - 0.05)
        panda.move_to_joint_position(target, speed_factor=0.2)
        time.sleep(settle)
        q_sum, tau_sum = np.zeros(7), np.zeros(7)
        for _ in range(samples):
            state = panda.get_state()
            q_sum += state.q
            tau_sum += state.tau_ext_hat_filtered
            time.sleep(0.01)
        qs.append(q_sum / samples)
        taus.append(tau_sum / samples)
        print(f"  pose {n:2d}/{len(POSE_OFFSETS)}: external torques "
              f"{np.round(taus[-1], 2).tolist()} Nm", flush=True)
    panda.move_to_start(speed_factor=0.2)
    return np.array(qs), np.array(taus)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="panda-identify-load",
                                     description=__doc__.splitlines()[1])
    parser.add_argument("hostname", help="the robot's address")
    parser.add_argument("--out", help="also write the result as JSON")
    parser.add_argument("--yes", action="store_true", help="do not ask before moving")
    args = parser.parse_args(argv)

    if not args.yes:
        print("\n  The robot will move to its start pose and hold still in "
              f"{len(POSE_OFFSETS)} wrist orientations\n  (joints 5 to 7 turn up to 1.5 rad). "
              "Keep the workspace clear and the user stop within reach.")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1

    panda = panda_py.Panda(args.hostname)
    state = panda.get_state()
    known_mass, known_com = float(state.m_load), np.asarray(state.F_x_Cload)
    panda.move_to_start(speed_factor=0.2)
    qs, taus = collect(panda)
    result = fit(qs, taus)
    mass, com = combine(known_mass, known_com, result["mass"], result["com"])
    result.update(known_load_mass=known_mass, known_load_com=known_com.tolist(),
                  total_load_mass=float(mass), total_load_com=com.tolist(),
                  inertia=box_inertia(mass), poses=qs.tolist(), torques=taus.tolist())

    print(f"\n  unconfigured load: {result['mass'] * 1e3:.0f} +- {result['mass_std'] * 1e3:.0f} g, "
          f"centre of mass {np.round(np.asarray(result['com']) * 1e3, 1).tolist()} mm "
          f"+- {np.round(np.asarray(result['com_std']) * 1e3, 1).tolist()} mm (flange frame)")
    print(f"  fit residual {result['residual_rms']:.3f} Nm rms over {len(qs)} poses")
    if known_mass:
        print(f"  with the load already set ({known_mass * 1e3:.0f} g): "
              f"{mass * 1e3:.0f} g at {np.round(com * 1e3, 1).tolist()} mm")
    if result["mass"] < 2 * result["mass_std"] or result["mass"] < 0.02:
        print("  The estimate is within its uncertainty of no load: nothing worth setting.")
    print("\n  For Desk's end-effector settings: load mass "
          f"{mass:.3f} kg, centre of mass {np.round(com, 4).tolist()} m, inertia "
          f"{result['inertia'][0]:.2e} kg m^2 on the diagonal.")
    print("  or in Python, after connecting: panda.get_robot().set_load("
          f"{mass:.3f}, {np.round(com, 4).tolist()}, "
          f"[{result['inertia'][0]:.2e}, 0, 0, 0, {result['inertia'][4]:.2e}, 0, 0, 0, "
          f"{result['inertia'][8]:.2e}])")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=1)
        print(f"  wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
