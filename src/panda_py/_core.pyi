from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import panda_py.libfranka
import typing
__all__: list[str] = ['AppliedForce', 'AppliedTorque', 'CartesianTrajectory', 'Force', 'IntegratedVelocity', 'JointPosition', 'JointTrajectory', 'Panda', 'PandaContext', 'TaskImpedance', 'TorqueController', 'fk', 'ik', 'ik_full', 'realtime_priority_available']
class AppliedForce(TorqueController):
    def __init__(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., filter_coeff: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    def set_control(self, force: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"]) -> None:
        ...
    def set_damping(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_filter(self, filter_coeff: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class AppliedTorque(TorqueController):
    def __init__(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., filter_coeff: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    def set_control(self, torque: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_damping(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_filter(self, filter_coeff: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class CartesianTrajectory:
    @typing.overload
    def __init__(self, positions: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]], orientations: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, max_deviation: typing.SupportsFloat | typing.SupportsIndex = 0, timeout: typing.SupportsFloat | typing.SupportsIndex = 30.0) -> None:
        ...
    @typing.overload
    def __init__(self, poses: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, max_deviation: typing.SupportsFloat | typing.SupportsIndex = 0, timeout: typing.SupportsFloat | typing.SupportsIndex = 30.0) -> None:
        ...
    def get_duration(self) -> float:
        ...
    def get_orientation(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]:
        ...
    def get_pose(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
        ...
    def get_position(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        ...
class Force(TorqueController):
    def __init__(self, k_p: typing.SupportsFloat | typing.SupportsIndex = 1.0, k_i: typing.SupportsFloat | typing.SupportsIndex = 2.0, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01, filter_coeff: typing.SupportsFloat | typing.SupportsIndex = 0.001) -> None:
        ...
    def set_control(self, force: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def set_filter(self, filter_coeff: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_integral_gain(self, k_i: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_proportional_gain(self, k_p: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @property
    def name(self) -> str:
        ...
class IntegratedVelocity(TorqueController):
    def __init__(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> None:
        ...
    def get_qd(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def set_control(self, velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_damping(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_stiffness(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
class JointPosition(TorqueController):
    def __init__(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., telemetry: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
                       Joint position servo,
                       :math:`\\tau = K (q_d - q) + D (\\dot q_d - \\dot q)`. Targets are
                       applied by the loop on its next tick, which never waits for
                       them. On start it holds the current joint positions.
        
                       Args:
                         stiffness: :math:`K`, Nm/rad per joint.
                         damping: :math:`D`, Nm s/rad per joint.
                         telemetry: Capacity of the telemetry buffer in samples; 0
                           records none.
        """
    def get_damping(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def get_guard(self) -> dict:
        ...
    def get_snapshot(self) -> dict:
        """
                       What the loop last did: ``time`` and ``q`` of the latest tick,
                       ``applied_time`` and ``applied_q`` of the tick the latest target
                       was applied at, the target ``q_d`` and ``applied``, the number of
                       targets applied since start.
        """
    def get_stiffness(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def read_telemetry(self) -> dict:
        """
                       The telemetry recorded since the last call, one row per 1 kHz
                       tick: ``tick``, ``time``, ``duration``, ``reference_update``,
                       ``control_command_success_rate``, ``q_d``, ``dq_d``,
                       ``stiffness``, ``damping``, ``tau_active``
                       (:math:`K (q_d - q) + D \\dot q_d`, zero while a guard is
                       tripped), ``tau_passive`` (:math:`-D \\dot q`), ``tau_law``,
                       ``tau_cmd`` (sent), the robot state's ``q``, ``dq``, ``tau_J``,
                       ``tau_J_d``, ``tau_ext_hat_filtered``, ``O_T_EE``, ``F_T_EE``,
                       ``O_F_ext_hat_K``, ``K_F_ext_hat_K``, and ``guard``. See
                       :py:func:`TaskImpedance.read_telemetry`.
        """
    def rearm(self) -> None:
        """
                       Clears a trip on the loop's next tick; the target becomes the
                       joint positions of that tick.
        """
    def set_control(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> None:
        ...
    def set_damping(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_guard(self, force: typing.SupportsFloat | typing.SupportsIndex = ..., force_time: typing.SupportsFloat | typing.SupportsIndex = 0.05, saturation_time: typing.SupportsFloat | typing.SupportsIndex = ..., speed: typing.SupportsFloat | typing.SupportsIndex = ..., joint_velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] | None = None, workspace: collections.abc.Sequence[tuple[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]]] = [], workspace_point: str = 'end_effector') -> None:
        """
                      Guards evaluated in the 1 kHz loop. When one trips, the loop drops
                      the controller's active term (the spring) on that same tick and
                      keeps the damping, until :py:func:`rearm`. Infinite values disable
                      a guard; calling this replaces every setting.
        
                      Args:
                        force: External force norm, N, from ``O_F_ext_hat_K``.
                        force_time: Seconds the force must stay above ``force``.
                        saturation_time: Seconds any sent joint torque may stay at its
                          limit.
                        speed: Speed of the controller's frame (the control frame, or
                          the flange for joint control), m/s.
                        joint_velocity: Per-joint speed limits, rad/s.
                        workspace: Up to eight ``(pose, half_extents)`` boxes, ``pose``
                          a 4x4 transform in the base frame; the guarded point must stay
                          inside at least one. :py:func:`panda_py.safety.box_along_axis`
                          builds one around an axis.
                        workspace_point: ``"end_effector"`` (``O_T_EE``) or
                          ``"control"``, the controller's frame.
        """
    def set_stiffness(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def step_control(self, delta: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        """
                       :math:`q_d = q + \\delta`, :math:`\\dot q_d = 0`, with :math:`q` of
                       the tick it is applied at: one policy step of the simulator's
                       joint servo.
        """
    def trip(self) -> None:
        """
        Trips the guard from outside the loop, on its next tick.
        """
    @property
    def guard_state(self) -> dict:
        """
                      ``tripped``, ``reason`` (``"force"``, ``"saturation"``,
                      ``"speed"``, ``"joint_velocity"``, ``"workspace"``, ``"manual"``
                      or ``"none"``), the robot ``time`` of the trip, the ``value`` that
                      tripped it (N, s, m/s, rad/s, or m outside the workspace) and the
                      ``joint``, where one is at fault. Telemetry's ``guard`` column
                      holds the reason as a number, 0 while armed, in this order.
        """
    @property
    def telemetry_capacity(self) -> int:
        ...
    @property
    def telemetry_dropped(self) -> int:
        """
        Samples lost because the telemetry buffer was full.
        """
class JointTrajectory:
    def __init__(self, waypoints: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, max_deviation: typing.SupportsFloat | typing.SupportsIndex = 0, timeout: typing.SupportsFloat | typing.SupportsIndex = 30.0) -> None:
        ...
    def get_duration(self) -> float:
        ...
    def get_joint_accelerations(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def get_joint_positions(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def get_joint_velocities(self, time: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
class Panda:
    """
    
         The main interface of panda-py to control the robot.
      
    """
    def __init__(self, hostname: str, name: str = 'panda', realtime_config: panda_py.libfranka.RealtimeConfig = panda_py.libfranka.RealtimeConfig.kIgnore) -> None:
        ...
    def create_context(self, frequency: typing.SupportsFloat | typing.SupportsIndex, max_runtime: typing.SupportsFloat | typing.SupportsIndex = 0.0, max_iter: typing.SupportsInt | typing.SupportsIndex = 0) -> PandaContext:
        ...
    def disable_logging(self) -> None:
        ...
    def enable_logging(self, buffer_size: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def get_joint_limits_lower(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        """
                  Lower joint position limits of the connected robot, selected from its
                  server version. The FER and the FR3 have different envelopes, and the
                  FR3's were widened with robot system 5.9.0.
        """
    def get_joint_limits_upper(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        """
                  Upper joint position limits of the connected robot (cf.
                  :py:func:`get_joint_limits_lower`).
        """
    def get_control_options(self) -> dict:
        """
                  The options of :py:func:`set_control_options` in effect, as a dict.
        """
    def get_joint_walls(self) -> bool:
        """
                  Whether the virtual joint walls are on (cf.
                  :py:func:`set_joint_walls`).
        """
    def get_log(self) -> dict[str, list[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]]]:
        ...
    def get_model(self) -> panda_py.libfranka.Model:
        ...
    def get_orientation(self, scalar_first: bool = False) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]:
        """
                       Get current end-effector orientation
                       :math:`\\mathbf q = (\\vec{v},\\ r),~~ \\mathbf q \\in \\mathbb{H},~~ \\vec{v}\\in \\mathbb{R}^3,~~ r \\in \\mathbb{R}`
                       in robot base frame.
        
                       Args:
                         scalar_first: If True returns quaternion in scalar first
                           representation (default: False)
                       
                       Returns:
                         Vector of shape (4,) holding quaternion coefficients.
        """
    def get_pose(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
        ...
    def get_position(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        """
                  Current end-effector position in robot base frame.
        """
    def get_robot(self) -> panda_py.libfranka.Robot:
        """
                       Get a reference to the :py:class:`libfranka.Robot` class behind this instance.
        """
    def get_state(self) -> panda_py.libfranka.RobotState:
        """
                  Get a copy of the last :py:class:`libfranka.RobotState` received from the robot.
        """
    def is_moving(self) -> bool:
        """
                  True while a controller is running, i.e. while the robot is under
                  active control by this instance.
        """
    @typing.overload
    def move_to_joint_position(self, waypoints: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        ...
    @typing.overload
    def move_to_joint_position(self, positions: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        ...
    @typing.overload
    def move_to_pose(self, positions: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]], orientations: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.02, orientation_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.1) -> bool:
        """
                       Moves the end-effector from the current pose through the provided waypoints
                       in piece-wise linear segments. The waypoints are given as lists of positions
                       :math:`\\in \\mathbb{R}^3` and orientations
                       :math:`\\mathbf q = (\\vec{v},\\ r),~~ \\mathbf q \\in \\mathbb{H},~~ \\vec{v}\\in \\mathbb{R}^3,~~ r \\in \\mathbb{R}`,
                       i.e. quaternions with scalar last. The computed trajectory is time-optimal.
        """
    @typing.overload
    def move_to_pose(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.02, orientation_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.1) -> bool:
        """
                       Same as :py:func:`move_to_pose` above, but only one target pose given as
                       position and orientation directly.
        """
    @typing.overload
    def move_to_pose(self, pose: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.02, orientation_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.1) -> bool:
        """
                       Same as :py:func:`move_to_pose` above, but waypoints are given as a list of
                       homogeneous transforms :math:`\\in \\mathbb{R}^{4\\times 4}`.
        """
    @typing.overload
    def move_to_pose(self, pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.02, orientation_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.1) -> bool:
        """
                       Same as :py:func:`move_to_pose` above, but only one target pose given as
                       homogeneous transform :math:`\\in \\mathbb{R}^{4\\times 4}`.
        """
    def move_to_start(self, speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        """
                       Convenience function similar to :py:func:`move_to_pose`, moves the end-effector
                       into the starting position (cf. :py:obj:`constants.JOINT_POSITION_START`).
        """
    def raise_error(self) -> None:
        """
                  Raises a `RuntimeError` in Python when the robot has an active error.
                  As panda-py controllers run asynchroneously, encountered errors don't
                  propagate to the proces' main thread. Use this function or
                  :py:class:`PandaContext` to catch errors.
        """
    def recover(self) -> None:
        ...
    def refresh_state(self) -> None:
        """
                  Reads the robot state once and updates the cached copy. The state
                  getters call this for you when no controller is running; while one is,
                  the control loop already refreshes the state at 1KHz and this is a
                  no-op.
        """
    def set_default_behavior(self) -> None:
        ...
    def set_control_options(self, torque_rate_limit: bool = True, limit_rate: bool = False, cutoff_frequency: typing.SupportsFloat | typing.SupportsIndex = 100.0) -> None:
        """
                  The torque path between a controller and the robot.

                  Args:
                    torque_rate_limit: panda-py's limit of the commanded torque's
                      change to 1 N m per tick and joint (on by default). Takes effect
                      on the next tick.
                    limit_rate: libfranka's own rate limiter (off by default).
                    cutoff_frequency: libfranka's first-order low-pass on the
                      commanded torque, Hz; 1000 turns it off (default 100).

                  ``limit_rate`` and ``cutoff_frequency`` take effect when the next
                  controller starts.
        """
    def set_joint_walls(self, enabled: bool) -> None:
        """
                  Switches the virtual joint walls, the torques that push a joint back
                  as it nears its limit (the last 0.24 rad for joint 1, 0.18 rad for
                  joints 2 to 4, 0.07 rad for the wrist), on top of whatever the
                  controller commands. They are on by default. Off, nothing is added,
                  and a joint that reaches its limit trips the firmware's
                  ``joint_position_limits_violation`` reflex instead. Takes effect on
                  the next tick, also while a controller runs.
        """
    def start_controller(self, controller: TorqueController) -> None:
        ...
    def stop_controller(self) -> None:
        ...
    def teaching_mode(self, active: bool, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> None:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def q(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
class PandaContext:
    def __enter__(self) -> PandaContext:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> bool:
        ...
    def ok(self) -> bool:
        ...
    @property
    def num_ticks(self) -> int:
        ...
    @property
    def time(self) -> float:
        ...
class TaskImpedance(TorqueController):
    @staticmethod
    def compute(q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], dq: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], jacobian: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 7]"], mass: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 7]"], position_ref: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation_ref: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"], damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"], q_nullspace: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex, nullspace: str = 'dynamic', alpha: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_damping: typing.SupportsFloat | typing.SupportsIndex = 0.0, coriolis: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., nullspace_armature: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., joint_spring_stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., joint_spring_damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., q_joint_spring: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> dict:
        """
                       The control law alone, for a given state: what the controller
                       computes in one step. ``pose`` and ``jacobian`` are the control
                       frame's, ``orientation_ref`` a scalar-last quaternion. Returns a
                       dict of ``error``, ``velocity``, ``wrench_active`` (before
                       alpha), ``wrench_passive``, ``tau_task``, ``tau_nullspace`` and
                       ``tau``.
        """
    @staticmethod
    def critical_damping(stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"], damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 1]"]:
        ...
    @staticmethod
    def orientation_error(orientation_ref: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"]:
        """
        Axis-angle vector of the rotation from orientation to orientation_ref.
        """
    @staticmethod
    def shift_jacobian(jacobian: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 7]"], offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 7]"]:
        """
        Moves a geometric Jacobian to a point offset from its origin, base frame.
        """
    @staticmethod
    def step_reference_update(position_ref: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation_ref: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], leash_position: typing.SupportsFloat | typing.SupportsIndex = ..., leash_rotation: typing.SupportsFloat | typing.SupportsIndex = ...) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[3, 1]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]]:
        """
                       The update :py:func:`step_reference` makes in the loop, for a
                       given reference and pose: returns the new position and
                       scalar-last orientation reference.
        """
    @staticmethod
    def tank_step(level: typing.SupportsFloat | typing.SupportsIndex, drawn: typing.SupportsFloat | typing.SupportsIndex, wrench_active: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"], velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"], dt: typing.SupportsFloat | typing.SupportsIndex, E0: typing.SupportsFloat | typing.SupportsIndex, mode: str = 'power', smooth_fraction: typing.SupportsFloat | typing.SupportsIndex = 0.25) -> tuple[float, float, float]:
        """
        One tick of the tank, as the loop runs it: returns (alpha, level, drawn).
        """
    def __init__(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace: str = 'dynamic', nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 10.0, frame: str = 'end_effector', frame_transform: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"] = ..., coriolis: bool = False, nullspace_damping: typing.SupportsFloat | typing.SupportsIndex = 0.0, telemetry: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
                       Impedance in task space at a selectable control frame:
        
                       .. math::
                         \\tau = J^\\top (\\alpha K e - D J \\dot q) + N M u, \\quad
                         u = k_{ns}(q_0 - q) - 2\\sqrt{k_{ns}}\\,\\dot q
        
                       with :math:`e` the position error and the axis-angle orientation
                       error :math:`\\mathrm{axisangle}(q_{ref} q^{-1})` of the control
                       frame, all in the base frame, and :math:`D = 2\\zeta\\sqrt{K}`.
                       There is no gravity term: the robot compensates gravity itself.
                       On start the controller holds the current pose of the control
                       frame, and the current joint positions in the nullspace.
        
                       Args:
                         stiffness: Diagonal stiffness, translational (N/m) then
                           rotational (Nm/rad).
                         damping_ratio: The damping is :math:`2\\zeta\\sqrt{K}` for this
                           ratio :math:`\\zeta`.
                         nullspace: ``"dynamic"`` projects the posture term with
                           :math:`N = I - J^\\top (J M^{-1} J^\\top)^{-1} J M^{-1}` and
                           applies :math:`N M u`, which cannot perturb the task.
                           ``"kinematic"`` applies :math:`(I - J^\\top (J J^\\top)^{-1} J) u`.
                           ``"none"`` drops the posture term.
                         nullspace_stiffness: :math:`k_{ns}`. The two projections need
                           different gains: the dynamic one multiplies by the mass
                           matrix.
                         frame: ``"flange"`` or ``"end_effector"``, the libfranka frame
                           the control frame is attached to.
                         frame_transform: Pose of the control frame relative to
                           ``frame``.
                         coriolis: Add the Coriolis torque.
                         nullspace_damping: Regularises the projector's 6x6 inverse,
                           relative to the mean of its diagonal. 0 is exact.
                         telemetry: Capacity of the telemetry buffer in samples, one per
                           1 kHz tick; 0 records none. Drain it with
                           :py:func:`read_telemetry` faster than it fills.
        
                       Reference commands, :py:func:`set_reference` and
                       :py:func:`step_reference`, are applied by the control loop on its
                       next tick, leashed against the pose of that tick; the loop never
                       waits for them.
        """
    def get_damping(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 1]"]:
        ...
    def get_guard(self) -> dict:
        ...
    def get_leash(self) -> tuple[float, float]:
        ...
    def get_snapshot(self) -> dict:
        """
                       What the loop last did: ``time`` and ``pose`` (control frame) of
                       the latest tick, ``applied_time`` and ``applied_pose`` of the
                       tick the latest reference command was applied at, the reference
                       and stiffness in effect, ``applied``, the number of commands
                       applied since start, and the tank's ``tank_level`` (NaN without
                       one), ``tank_drawn`` and gate ``alpha``. Times are the robot's,
                       in seconds.
        """
    def get_friction_compensation(self) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"], float]:
        """
        ``(friction, deadband)`` of :py:func:`set_friction_compensation`.
        """
    def get_joint_spring(self) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"], typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]]:
        """
        ``(stiffness, damping, q)`` of :py:func:`set_joint_spring`.
        """
    def get_nullspace_armature(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
        ...
    def get_stiffness(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 1]"]:
        ...
    def get_tank(self) -> typing.Any:
        ...
    def read_telemetry(self) -> dict:
        """
                       The telemetry recorded since the last call, a dict of arrays with
                       one row per 1 kHz tick: ``tick`` (counts every tick since start,
                       so a gap is a sample the buffer had no room for), ``time``,
                       ``duration`` (s since the previous tick; above 1 ms the robot
                       ticked without a command), ``reference_update`` (1 where a
                       command was applied), the control frame's ``position`` and
                       ``orientation``, ``position_ref``, ``orientation_ref``,
                       ``stiffness``, ``damping``, ``wrench_active`` (before alpha),
                       ``wrench_passive``, ``alpha`` (the tank's gate, 0 while a guard
                       is tripped), ``tank`` (its level, NaN without one),
                       ``tank_drawn``, ``tau_task``, ``tau_nullspace``, ``tau_law``
                       (the law's torque), ``tau_cmd`` (sent, after the joint walls,
                       rate limit and clipping), the robot state's ``q``, ``dq``,
                       ``tau_J``, ``tau_J_d``, ``tau_ext_hat_filtered``, ``O_T_EE``,
                       ``F_T_EE`` (column-major), ``O_F_ext_hat_K``, ``K_F_ext_hat_K``
                       and ``control_command_success_rate``, ``guard`` (the trip reason
                       as a number, 0 while armed), and the law's ``jacobian`` (6x7) and
                       ``mass`` (7x7, NaN unless the nullspace is dynamic), both
                       column-major, so that every tick can be replayed through
                       :py:func:`compute`. Quaternions are scalar-last.
        """
    def rearm(self) -> None:
        """
                       Clears a trip on the loop's next tick and resets the reference to
                       the pose of that tick, so the active wrench resumes from zero. A
                       reference set before that tick replaces the reset.
        """
    def reset_tank(self) -> None:
        """
        Refills the tank to E0 on the loop's next tick, as at a trial's start.
        """
    def set_damping_ratio(self, damping_ratio: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_guard(self, force: typing.SupportsFloat | typing.SupportsIndex = ..., force_time: typing.SupportsFloat | typing.SupportsIndex = 0.05, saturation_time: typing.SupportsFloat | typing.SupportsIndex = ..., speed: typing.SupportsFloat | typing.SupportsIndex = ..., joint_velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] | None = None, workspace: collections.abc.Sequence[tuple[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]]] = [], workspace_point: str = 'end_effector') -> None:
        """
                      Guards evaluated in the 1 kHz loop. When one trips, the loop drops
                      the controller's active term (the spring) on that same tick and
                      keeps the damping, until :py:func:`rearm`. Infinite values disable
                      a guard; calling this replaces every setting.
        
                      Args:
                        force: External force norm, N, from ``O_F_ext_hat_K``.
                        force_time: Seconds the force must stay above ``force``.
                        saturation_time: Seconds any sent joint torque may stay at its
                          limit.
                        speed: Speed of the controller's frame (the control frame, or
                          the flange for joint control), m/s.
                        joint_velocity: Per-joint speed limits, rad/s.
                        workspace: Up to eight ``(pose, half_extents)`` boxes, ``pose``
                          a 4x4 transform in the base frame; the guarded point must stay
                          inside at least one. :py:func:`panda_py.safety.box_along_axis`
                          builds one around an axis.
                        workspace_point: ``"end_effector"`` (``O_T_EE``) or
                          ``"control"``, the controller's frame.
        """
    def set_leash(self, position: typing.SupportsFloat | typing.SupportsIndex, rotation: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
                       Keeps every reference command within ``position`` (m) and
                       ``rotation`` (rad) of the pose at the tick it is applied.
                       ``float("inf")``, the default, disables it.
        """
    def set_friction_compensation(self, friction: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], deadband: typing.SupportsFloat | typing.SupportsIndex = 0.1) -> None:
        """
                       Coulomb friction compensation: adds, per joint,
                       ``friction * clip(tau / deadband, -1, 1)`` N m, with ``tau`` the
                       law's torque without it (task, posture and joint spring), so a
                       joint gets its breakaway torque in the direction it is pushed
                       and a proportional share below the deadband (N m). Zero
                       friction (the default) leaves it out; off while a guard is
                       tripped. Takes effect on the next tick; logged as
                       ``tau_friction``.
        """
    def set_joint_spring(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        """
                       A joint-space spring outside the task projector, added to the
                       torque after the posture term on every tick:
                       ``stiffness * (q - q_now) - damping * dq`` per joint, N m, with ``q``
                       the spring's target. Zero stiffness and damping (the default) leave
                       it out; a joint with both zero is untouched. It stays on when a
                       guard trips. Takes effect on the next tick; logged as
                       ``tau_joint_spring``.
        """
    def set_nullspace_armature(self, armature: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        """
                       Rotor inertia per joint, kg m^2, added to the diagonal of
                       libfranka's mass matrix for the dynamic posture term only (the
                       projector and N M u); the task law does not use the mass
                       matrix. libfranka's model carries the links and the tool but no
                       rotors, which the drives do not hide. Zero (the default) is
                       libfranka's model; takes effect on the next tick. Telemetry's
                       ``mass`` stays libfranka's.
        """
    def set_nullspace_stiffness(self, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_nullspace_target(self, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_reference(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> None:
        """
                       Position and scalar-last quaternion of the control frame, base
                       frame. Applied, and leashed, by the loop on its next tick; replaces
                       any command not yet applied.
        """
    def set_stiffness(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"]) -> None:
        """
        Also sets the damping, for the current damping ratio.
        """
    def set_tank(self, E0: typing.SupportsFloat | typing.SupportsIndex | None, mode: str = 'power', smooth_fraction: typing.SupportsFloat | typing.SupportsIndex = 0.25) -> None:
        """
                       The insertion simulator's tank, filled to ``E0`` on the loop's
                       next tick. Every tick it meters the active wrench, power
                       :math:`\\max(0, w_{act} \\cdot [v; \\omega])` in ``"power"`` mode
                       (``E0`` in J) or :math:`|w_{act,xyz}|` in ``"impulse"`` mode
                       (``E0`` in N s), and gates it with
                       :math:`\\alpha = \\mathrm{clamp}(E_T / (f E_0), 0, 1)`, ``f`` the
                       ``smooth_fraction``; damping is never scaled. A fraction of 0
                       selects the hard gate, which scales only a draw that would
                       overdraw the tank. ``E0=None`` removes the tank.
        """
    def step_reference(self, translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 1]"] | None = None) -> None:
        """
                       Moves the reference as one policy step does: by ``translation``
                       and by ``rotation``, an axis-angle vector applied on the left,
                       both in the base frame. With ``stiffness``, sets it on the same
                       tick. Applied, and leashed, by the loop on its next tick; steps
                       not yet applied add up.
        """
    def trip(self) -> None:
        """
        Trips the guard from outside the loop, on its next tick.
        """
    @property
    def frame(self) -> str:
        ...
    @property
    def frame_transform(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
        ...
    @property
    def guard_state(self) -> dict:
        """
                      ``tripped``, ``reason`` (``"force"``, ``"saturation"``,
                      ``"speed"``, ``"joint_velocity"``, ``"workspace"``, ``"manual"``
                      or ``"none"``), the robot ``time`` of the trip, the ``value`` that
                      tripped it (N, s, m/s, rad/s, or m outside the workspace) and the
                      ``joint``, where one is at fault. Telemetry's ``guard`` column
                      holds the reason as a number, 0 while armed, in this order.
        """
    @property
    def telemetry_capacity(self) -> int:
        ...
    @property
    def telemetry_dropped(self) -> int:
        """
        Samples lost because the telemetry buffer was full.
        """
class TorqueController:
    """
              Base class for all torque controllers. Torque controllers
              provide the robot with torques at 1KHz and the user with
              an asynchronous interface to provide control signals.
          
    """
    def get_time(self) -> float:
        """
                  Get time in seconds since this controller was started.
        """
def fk(q: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 4]"]:
    """
         Computes end-effector pose in base frame from joint positions.
    """
@typing.overload
def ik(O_T_EE: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], q_init: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., q_7: typing.SupportsFloat | typing.SupportsIndex = 0.7853981633974483) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
    """
              Compute analytical inverse kinematics. 
              Solution is case consistent with configuration  given in `q_init`.
    
              Args:
                O_T_EE: Homogeneous transform :math:`\\mathbb{R}^{4\\times 4}` describing
                  the end-effector pose.
                q_init: Reference joint positions, the result will be consistent
                  with this configuration.
                q_7: Joint 7 is considered the redundant joint, use `q_7` to set the
                  desired joint position (default: :math:`\\frac{\\pi}{4}`).
    
              Returns:
                Vector of shape (7,) containing joint positions.
    """
@typing.overload
def ik(position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], q_init: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., q_7: typing.SupportsFloat | typing.SupportsIndex = 0.7853981633974483) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[7, 1]"]:
    """
              Same as :py:func:`ik` above, but takes position and orientation arguments.
    """
@typing.overload
def ik_full(O_T_EE: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], q_init: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., q_7: typing.SupportsFloat | typing.SupportsIndex = 0.7853981633974483) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 7]"]:
    ...
@typing.overload
def ik_full(position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], q_init: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., q_7: typing.SupportsFloat | typing.SupportsIndex = 0.7853981633974483) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 7]"]:
    ...
def _pose_error(goal_position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], goal_orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]) -> tuple[float, float]:
    """
         Distance in metres and rotation angle in radians between a pose and the
         goal of a move_to_pose, as its success check computes them. Quaternions
         are scalar last. Exposed for the tests.
    """
def realtime_priority_available() -> tuple[bool, str]:
    """
         Whether this process can obtain the realtime scheduling that the
         control loop needs, and the reason it cannot as reported by libfranka.
    
         Returns a tuple of a bool and a message, where the message is empty
         when the answer is True. :py:class:`Panda` logs a warning about this
         on connect; call it directly to check without a robot. See also
         :py:func:`panda_py.libfranka.has_realtime_kernel`, which reports the
         other half of what realtime control needs.
    """
_DTAU_J_MAX: numpy.ndarray  # value = array([1000., 1000., 1000., 1000., 1000., 1000., 1000.])
_JOINT_LIMITS_LOWER: numpy.ndarray  # value = array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_JOINT_LIMITS_LOWER_FR3: numpy.ndarray  # value = array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065,  0.5445, -3.0159])
_JOINT_LIMITS_LOWER_FR3_5_9: numpy.ndarray  # value = array([-2.9007, -1.8361, -2.9007, -3.077 , -2.8763,  0.4398, -3.0508])
_JOINT_LIMITS_UPPER: numpy.ndarray  # value = array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
_JOINT_LIMITS_UPPER_FR3: numpy.ndarray  # value = array([ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159])
_JOINT_LIMITS_UPPER_FR3_5_9: numpy.ndarray  # value = array([ 2.9007,  1.8361,  2.9007, -0.1169,  2.8763,  4.6216,  3.0508])
_JOINT_POSITION_START: numpy.ndarray  # value = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        ,...
_MOVE_TO_POSE_ORIENTATION_THRESHOLD: float  # value = 0.1
_MOVE_TO_POSE_POSITION_THRESHOLD: float  # value = 0.02
_TAU_J_MAX: numpy.ndarray  # value = array([87., 87., 87., 87., 12., 12., 12.])
_Q_MAX_VELOCITY: numpy.ndarray
