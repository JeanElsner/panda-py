"""
Torque controllers that run in panda-py's 1 kHz loop. Instantiate one and
hand it to :py:func:`panda_py.Panda.start_controller`; while it runs, its
``set_reference`` (and the other setters) take effect on the loop's next tick
without ever making the loop wait.

Joint space:

* :py:class:`JointImpedance`: spring and damper per joint to a reference.
* :py:class:`JointVelocity`: joint velocity, integrated into a joint impedance
  reference.
* :py:class:`JointTorque`: feed-forward joint torque and damping.

Task space (at the end effector or a chosen control frame):

* :py:class:`TaskImpedance`: Cartesian impedance with a posture term.
* :py:class:`TaskWrench`: feed-forward wrench and damping.
* :py:class:`TaskForce`: wrench regulation with a PI loop.

Every controller shares the same guards (``set_guard``, ``trip``, ``rearm``,
``guard_state``) and 1 kHz telemetry (``telemetry=<capacity>``,
``read_telemetry``, :py:mod:`panda_py.telemetry`).
"""

# pylint: disable=no-name-in-module
from ._core import (
    JointImpedance,
    JointTorque,
    JointVelocity,
    TaskForce,
    TaskImpedance,
    TaskWrench,
    TorqueController,
)

__all__ = [
    "TorqueController",
    "JointImpedance",
    "JointVelocity",
    "JointTorque",
    "TaskImpedance",
    "TaskWrench",
    "TaskForce",
]
