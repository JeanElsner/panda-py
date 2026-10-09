Migrating from panda-py 1
=========================

Installing
----------

There is one build for every robot. The per-libfranka-version archives of
panda-py 1 are gone: ``pip install panda-python`` connects to an FER or an FR3
on any system version.

Controllers
-----------

The controllers are named after the space they act in, and every one takes its
commands with ``set_reference``:

.. list-table::
   :header-rows: 1

   * - panda-py 1
     - panda-py 2
   * - ``CartesianImpedance``, ``set_control(position, orientation, q_nullspace)``
     - ``TaskImpedance``, ``set_reference(position, orientation)`` and
       ``set_nullspace_target(q)``
   * - ``JointPosition``, ``set_control(q, dq)``
     - ``JointImpedance``, ``set_reference(q_d, dq_d)``
   * - ``IntegratedVelocity``, ``set_control(dq)``
     - ``JointVelocity``, ``set_reference(dq_d)``
   * - ``AppliedTorque``, ``set_control(tau)``
     - ``JointTorque``, ``set_reference(tau_d)``
   * - ``AppliedForce``, ``set_control(wrench)``
     - ``TaskWrench``, ``set_reference(wrench)``
   * - ``Force``, ``set_control(force)``
     - ``TaskForce``, ``set_reference(wrench)``: a 6-vector, force then torque

Further changes:

* The input filters (``filter_coeff``, ``set_filter``) are gone: a reference
  takes effect on the next tick, as set.
* ``TaskImpedance`` acts on the full rotation angle, where
  ``CartesianImpedance`` acted on the error quaternion's vector part, about half
  of it: the same rotational stiffness is about twice as stiff. Halve it to
  keep the old behaviour.
* ``TaskImpedance`` adds no Coriolis torque unless created with
  ``coriolis=True``, and projects its posture term with the dynamically
  consistent nullspace by default (``nullspace="kinematic"`` for the exact
  kinematic one).
* ``TaskForce`` trips its guard, rather than raising, when the end effector
  leaves ``max_displacement`` (``threshold`` in panda-py 1).
* ``JointVelocity`` clamps its integrated reference to the connected robot's
  joint limits, not the FER's.

Kinematics
----------

* :py:func:`panda_py.ik` is numerical and has no ``q_7`` argument: the arm's
  redundancy is resolved toward ``q_init``. Pass ``limits=panda.limits`` to
  respect the connected robot's envelope. It raises
  :py:class:`panda_py.IKError` instead of returning NaN.
* ``ik_full`` is gone: there are infinitely many solutions; ``ik`` from
  different ``q_init`` finds the one near each.
* :py:func:`panda_py.fk` places the Franka Hand at 0.1034 m from the flange,
  as libfranka does; panda-py 1 used 0.103 m. It takes another end effector
  as ``F_T_EE``.

Motion
------

* The motion generators plan with the connected robot's limits; on an FR3,
  panda-py 1 used the FER's accelerations, above what the FR3 allows.
* ``move_to_joint_position`` reports success by the largest joint error, as
  documented; panda-py 1 compared relative to the size of the joint vector.

Errors
------

:py:class:`panda_py.IncompatibleVersionError` is now only raised for a robot
outside protocol versions 3 to 10, and no longer names a per-version build.
