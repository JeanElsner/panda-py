Guide
=====

Connecting
----------

FCI must be active and the joints unlocked, which Desk does, or
:py:class:`panda_py.Desk` from Python:

.. code-block:: python

   import panda_py

   desk = panda_py.Desk("172.16.0.2", "user", "password")
   desk.unlock()
   desk.activate_fci()

   panda = panda_py.Panda("172.16.0.2")
   print(panda.limits)                    # e.g. RobotLimits(FR3 (robot system >= 5.9.0))
   print(panda.get_robot().server_version())

panda-py is built with libfranka-universal, which speaks every research
interface protocol version (3 to 10): it connects to a Franka Emika Robot (FER)
or a Franka Research 3 (FR3) on any system version and speaks that robot's
protocol. :py:attr:`panda_py.Panda.limits` is the connected robot's
:py:class:`panda_py.RobotLimits`: its type, its joint envelope (the FR3's
changed with robot system 5.9.0), and the joint and Cartesian velocity and
acceleration limits the motion generators plan with.

Panda-py logs to Python's :py:mod:`logging`; ``logging.basicConfig(level=logging.INFO)``
shows what it does.

Moving
------

.. code-block:: python

   panda.move_to_start()
   panda.move_to_joint_position(q)                # or a list of waypoints
   panda.move_to_pose(position, orientation)      # or a 4x4 pose, or lists of either

These plan a time-optimal trajectory within the robot's limits, scaled by
``speed_factor`` (0.2 by default), follow it with an impedance controller and
return whether the robot ended within ``success_threshold`` of the goal.
Orientations are scalar-last quaternions throughout panda-py. The planner is
also available on its own, :py:class:`panda_py.motion.JointTrajectory` and
:py:class:`panda_py.motion.CartesianTrajectory`, for any robot's limits
(``limits=panda.limits``; by default those valid on both).

Kinematics
----------

.. code-block:: python

   pose = panda_py.fk(q)                          # 4x4, base frame
   J = panda_py.jacobian(q)                       # 6x7, [v; omega]
   q = panda_py.ik(pose, q_init=panda.q, limits=panda.limits)

The FER and the FR3 share their link geometry. By default the end effector
is the Franka Hand; pass ``F_T_EE`` (the end effector relative to the flange,
e.g. from the robot state's ``F_T_EE``) for another. :py:func:`panda_py.ik` is
numerical: damped least squares within the joint limits, the arm's redundancy
drawn toward ``q_init``, so the solution is the one near it; pass the current
joint positions to stay in the same configuration. It raises
:py:class:`panda_py.IKError` when no solution lies within the limits.

Controllers
-----------

A controller runs in panda-py's 1 kHz control loop from
:py:func:`panda_py.Panda.start_controller` until
:py:func:`panda_py.Panda.stop_controller`. Python sets its reference and gains
at any rate; the loop applies them on its next tick and never waits for them.

.. list-table::
   :header-rows: 1

   * - Controller
     - Law
     - Reference
   * - :py:class:`~panda_py.controllers.JointImpedance`
     - :math:`K (q_d - q) + D (\dot q_d - \dot q)`
     - ``set_reference(q_d, dq_d)``, ``step_reference(delta)``
   * - :py:class:`~panda_py.controllers.JointVelocity`
     - joint impedance to the integrated velocity
     - ``set_reference(dq_d)``
   * - :py:class:`~panda_py.controllers.JointTorque`
     - :math:`\tau_d - D \dot q`
     - ``set_reference(tau_d)``
   * - :py:class:`~panda_py.controllers.TaskImpedance`
     - :math:`J^\top (K e - D J \dot q) + N M u`
     - ``set_reference(position, orientation)``, ``step_reference(translation, rotation)``
   * - :py:class:`~panda_py.controllers.TaskWrench`
     - :math:`J^\top w_d - D \dot q`
     - ``set_reference(wrench)``
   * - :py:class:`~panda_py.controllers.TaskForce`
     - feed-forward and PI on the wrench
     - ``set_reference(wrench)``

A sinusoid with the end effector:

.. code-block:: python

   from panda_py import controllers
   import numpy as np

   ctrl = controllers.TaskImpedance()
   panda.start_controller(ctrl)
   x0, q0 = panda.get_position(), panda.get_orientation()
   with panda.create_context(frequency=100, max_runtime=10) as ctx:
       while ctx.ok():
           ctrl.set_reference(x0 + [0, 0.1 * np.sin(ctrl.get_time()), 0], q0)

:py:class:`~panda_py.controllers.TaskImpedance` also has a selectable control
frame, a dynamically consistent posture term, a leash that keeps the reference
near the current pose, an energy tank that bounds the work its spring can do,
a joint-space spring and damper per joint, and Coulomb friction compensation.

Guards
------

Every controller evaluates the same guards in its loop, configured with
``set_guard``: the external force, a joint torque at its limit, the speed of
the controller's frame, the joint velocities, and a workspace of up to eight
boxes (:py:func:`panda_py.safety.box_along_axis` builds one around an axis).
When one trips, the controller drops its active term on that tick, the spring
or the commanded torque, and keeps its damping until ``rearm()``.
``guard_state`` says what tripped and when; ``trip()`` trips it from Python.

.. code-block:: python

   ctrl.set_guard(force=10.0, speed=0.3)
   ...
   if ctrl.guard_state["tripped"]:
       print(ctrl.guard_state["reason"], ctrl.guard_state["value"])
       ctrl.rearm()

These complement, not replace, the robot's own collision reflexes
(:py:func:`panda_py.safety.set_collision_thresholds`).

Telemetry
---------

Created with ``telemetry=<capacity>``, a controller records every control tick
into a lock-free buffer: the robot state's main fields, the controller's
reference, gains and terms, the torque its law computed and the torque actually
sent. ``read_telemetry()`` drains it into a dict of arrays;
:py:class:`panda_py.telemetry.Recorder` drains it in the background, and
:py:func:`panda_py.telemetry.check` reports buffer overruns and robot cycles
without a command.

.. code-block:: python

   from panda_py import telemetry

   ctrl = controllers.JointImpedance(telemetry=10000)
   recorder = telemetry.Recorder(ctrl)
   panda.start_controller(ctrl)
   recorder.start()
   ...
   recorder.stop()
   log = recorder.result()      # log["q"], log["tau_cmd"], ...

Checking a robot
----------------

``panda-check <robot-ip>`` (installed with panda-py) exercises the library on a
robot, with small motions around the start pose: the state and the dynamics
model, :py:func:`panda_py.fk` against the robot's own pose, every controller,
the guards, the motion generators, the inverse kinematics, error recovery,
teaching mode and, with ``--gripper``, the Franka Hand. It writes a Markdown
report and a JSON file with every measurement, the robot's type and protocol
version, and panda-py's and libfranka's versions: the first thing to attach
to an issue. ``--desk-user`` unlocks the robot and activates FCI for the
check; ``--no-motion`` runs only what does not move.

Identifying a load
------------------

A camera, a mount or a tool that is not configured in Desk is weight the robot
does not compensate: controllers with zero torque drift, and impedance
controllers settle off their reference. ``panda-identify-load <robot-ip>``
holds the arm still in 13 wrist orientations around the start pose and fits
the load's mass and centre of mass (flange frame) to the external joint
torques the robot estimates, on top of the end effector configured in Desk.
It prints a ``set_load`` call for the load, with the uncertainties, and the
values for the end effector in Desk with the load included, for keeping it
there instead; the inertia, which cannot be identified at rest, is that of a
small box. ``--out FILE`` writes the result, which ``panda-check --load FILE``
sets before checking. A load already set is taken into account, so it can be
run again to check the result.
