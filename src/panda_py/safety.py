"""
Safety helpers: the robot's collision thresholds and workspace boxes for the
controller guards (:py:func:`panda_py.controllers.TaskImpedance.set_guard`).

The robot's own collision detection is the outer layer: it stops the arm with
a reflex, independently of panda-py. The controller guards are the inner one:
they act in the 1 kHz loop before a reflex would, by dropping the active
wrench.
"""

import numpy as np

from ._core import _TAU_J_MAX

__all__ = ["set_collision_thresholds", "box_along_axis", "TAU_J_MAX"]

#: The joint torque limits panda-py clips commands to, Nm.
TAU_J_MAX = np.array(_TAU_J_MAX)


def set_collision_thresholds(robot, force, torque, joint_torque=None, contact=None):
    """
    Sets the robot's collision (reflex) thresholds, the same during
    acceleration and at constant velocity.

    Args:
      robot: A :py:class:`panda_py.libfranka.Robot`, e.g. from
        :py:func:`panda_py.Panda.get_robot`. The robot must not be moving.
      force: Translational threshold, N, on each Cartesian axis.
      torque: Rotational threshold, Nm, on each Cartesian axis.
      joint_torque: Per-joint thresholds, Nm, a scalar or 7 values. Defaults
        to 90 % of the joint torque limits.
      contact: Fraction of the collision thresholds at which the robot reports
        contact (``cartesian_contact``, ``joint_contact``) without stopping.
        Defaults to 1, contact and collision alike.

    :py:func:`panda_py.Panda.set_default_behavior` restores panda-py's
    defaults, 10 to 20 N and Nm.
    """
    joint = np.broadcast_to(
        0.9 * TAU_J_MAX if joint_torque is None else np.asarray(joint_torque, float), (7,)
    )
    wrench = np.array([force] * 3 + [torque] * 3, dtype=float)
    fraction = 1.0 if contact is None else float(contact)
    if not 0.0 < fraction <= 1.0:
        raise ValueError("contact must be a fraction in (0, 1].")
    robot.set_collision_behavior(
        (fraction * joint).tolist(), joint.tolist(),
        (fraction * joint).tolist(), joint.tolist(),
        (fraction * wrench).tolist(), wrench.tolist(),
        (fraction * wrench).tolist(), wrench.tolist(),
    )


def box_along_axis(origin, axis, half_width, below, above, reference=(1.0, 0.0, 0.0)):
    """
    A workspace box aligned with an axis, e.g. a bore: square in cross-section,
    ``2 half_width`` wide, from ``below`` under ``origin`` to ``above`` over it
    along ``axis``.

    Args:
      origin: A point on the axis, base frame.
      axis: The axis direction; need not be normalised.
      half_width: Half the box's width across the axis, m.
      below, above: Its extent along the axis from ``origin``, m, both
        positive.
      reference: Fixes the rotation about the axis: the box's x axis is this
        direction projected onto the plane normal to ``axis``.

    Returns:
      ``(pose, half_extents)``, as :py:func:`set_guard` takes in ``workspace``.
    """
    z = np.asarray(axis, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(reference, float)
    x = x - np.dot(x, z) * z
    if np.linalg.norm(x) < 1e-9:
        raise ValueError("reference must not be parallel to axis.")
    x = x / np.linalg.norm(x)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    pose[:3, 3] = np.asarray(origin, float) + z * (above - below) / 2
    return pose, np.array([half_width, half_width, (above + below) / 2])
