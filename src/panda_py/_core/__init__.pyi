from __future__ import annotations
import panda_py._core
import typing
import numpy
import panda_py.libfranka
_Shape = typing.Tuple[int, ...]

__all__ = [
    "AppliedForce",
    "AppliedTorque",
    "CartesianImpedance",
    "CartesianTrajectory",
    "Force",
    "IntegratedVelocity",
    "JointPosition",
    "JointTrajectory",
    "Panda",
    "PandaContext",
    "TorqueController",
    "fk",
    "ik",
    "ik_full"
]


class TorqueController():
    """
    Base class for all torque controllers. Torque controllers
    provide the robot with torques at 1KHz and the user with
    an asynchronous interface to provide control signals.
    """
    def get_time(self) -> float: 
        """
        Get time in seconds since this controller was started.
        """
    pass
class AppliedTorque(TorqueController):
    def __init__(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([0., 0., 0., 0., 0., 0., 0.]), filter_coeff: float = 1.0) -> None: ...
    def set_control(self, torque: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_damping(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_filter(self, filter_coeff: float) -> None: ...
    pass
class CartesianImpedance(TorqueController):
    def __init__(self, impedance: numpy.ndarray[numpy.float64, _Shape[6, 6]] = array([[200., 0., 0., 0., 0., 0.], [ 0., 200., 0., 0., 0., 0.], [ 0., 0., 200., 0., 0., 0.], [ 0., 0., 0., 10., 0., 0.], [ 0., 0., 0., 0., 10., 0.], [ 0., 0., 0., 0., 0., 10.]]), damping_ratio: float = 1.0, nullspace_stiffness: float = 0.5, filter_coeff: float = 1.0) -> None: 
        """
        Cartesian impedance controller. Takes the end-effector pose in robot
        base frame, as well as desired nullspace joint positions as input.

        Args:
          impedance: Cartesian impedance expressed as a matrix
            :math:`\in \mathbb{R}^{6\times 6}`.
          damping_ratio: Cartesian damping is computed based on the given
            impedance and damping ratio.
          nullspace_stiffness: Control gain of the nullspace term.
          filter_coeff: TP1 filter coefficient used to filter input signals.
        """
    def set_control(self, position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], q_nullspace: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816])) -> None: ...
    def set_damping_ratio(self, damping: float) -> None: ...
    def set_filter(self, filter_coeff: float) -> None: ...
    def set_impedance(self, impedance: numpy.ndarray[numpy.float64, _Shape[6, 6]]) -> None: ...
    def set_nullspace_stiffness(self, nullspace_stiffness: float) -> None: ...
    pass
class CartesianTrajectory():
    @typing.overload
    def __init__(self, positions: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], orientations: typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], speed_factor: float = 0.2, max_deviation: float = 0, timeout: float = 30.0) -> None: ...
    @typing.overload
    def __init__(self, poses: typing.List[numpy.ndarray[numpy.float64, _Shape[4, 4]]], speed_factor: float = 0.2, max_deviation: float = 0, timeout: float = 30.0) -> None: ...
    def get_duration(self) -> float: ...
    def get_joint_accelerations(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    def get_joint_positions(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    def get_joint_velocities(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    pass
class Force(TorqueController):
    def __init__(self, k_p: float = 1.0, k_i: float = 2.0, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([1. , 1. , 1. , 1. , 0.33, 0.33, 0.17]), threshold: float = 0.01, filter_coeff: float = 0.001) -> None: ...
    def set_control(self, force: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    def set_filter(self, filter_coeff: float) -> None: ...
    def set_integral_gain(self, k_i: float) -> None: ...
    def set_proportional_gain(self, k_p: float) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    pass
class IntegratedVelocity(TorqueController):
    def __init__(self, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.])) -> None: ...
    def set_control(self, velocity: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_damping(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_stiffness(self, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    pass
class JointPosition(TorqueController):
    def __init__(self, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), filter_coeff: float = 1.0) -> None: ...
    def set_control(self, position: numpy.ndarray[numpy.float64, _Shape[7, 1]], velocity: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([0., 0., 0., 0., 0., 0., 0.])) -> None: ...
    def set_damping(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_filter(self, filter_coeff: float) -> None: ...
    def set_stiffness(self, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    pass
class JointTrajectory():
    def __init__(self, waypoints: typing.List[numpy.ndarray[numpy.float64, _Shape[7, 1]]], speed_factor: float = 0.2, max_deviation: float = 0, timeout: float = 30.0) -> None: ...
    def get_duration(self) -> float: ...
    def get_joint_accelerations(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    def get_joint_positions(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    def get_joint_velocities(self, time: float, q: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]: ...
    pass
class Panda():
    """
    The main interface of panda-py to control the robot.
    """
    def __init__(self, hostname: str, name: str = 'panda', realtime_config: panda_py.libfranka.RealtimeConfig = RealtimeConfig.kIgnore) -> None: ...
    def create_context(self, frequency: float, max_runtime: float = 0.0, max_iter: int = 0) -> PandaContext: ...
    def disable_logging(self) -> None: ...
    def enable_logging(self, buffer_size: int) -> None: ...
    def get_log(self) -> typing.Dict[str, typing.List[numpy.ndarray[numpy.float64, _Shape[m, 1]]]]: ...
    def get_model(self) -> panda_py.libfranka.Model: ...
    def get_orientation(self, scalar_first: bool = False) -> numpy.ndarray[numpy.float64, _Shape[4, 1]]: 
        """
        Get current end-effector orientation
        :math:`\mathbf q = (\vec{v},\ r),~~ \mathbf q \in \mathbb{H},~~ \vec{v}\in \mathbb{R}^3,~~ r \in \mathbb{R}`
        in robot base frame.

        Args:
          scalar_first: If True returns quaternion in scalar first
            representation (default: False)

        Returns:
          Vector of shape (4,) holding quaternion coefficients.
        """
    def get_pose(self) -> numpy.ndarray[numpy.float64, _Shape[4, 4]]: ...
    def get_position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: 
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
    @typing.overload
    def move_to_joint_position(self, waypoints: typing.List[numpy.ndarray[numpy.float64, _Shape[7, 1]]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: ...
    @typing.overload
    def move_to_joint_position(self, positions: numpy.ndarray[numpy.float64, _Shape[7, 1]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: ...
    @typing.overload
    def move_to_pose(self, positions: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], orientations: typing.List[numpy.ndarray[numpy.float64, _Shape[4, 1]]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: 
        """
        Moves the end-effector from the current pose through the provided waypoints
        in piece-wise linear segments. The waypoints are given as lists of positions
        :math:`\in \mathbb{R}^3` and orientations
        :math:`\mathbf q = (\vec{v},\ r),~~ \mathbf q \in \mathbb{H},~~ \vec{v}\in \mathbb{R}^3,~~ r \in \mathbb{R}`,
        i.e. quaternions with scalar last. The computed trajectory is time-optimal.



        Same as :py:func:`move_to_pose` above, but only one target pose given as
        position and orientation directly.



        Same as :py:func:`move_to_pose` above, but waypoints are given as a list of
        homogeneous transforms :math:`\in \mathbb{R}^{4\times 4}`.



        Same as :py:func:`move_to_pose` above, but only one target pose given as
        homogeneous transform :math:`\in \mathbb{R}^{4\times 4}`.
        """
    @typing.overload
    def move_to_pose(self, position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: ...
    @typing.overload
    def move_to_pose(self, position: typing.List[numpy.ndarray[numpy.float64, _Shape[4, 4]]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: ...
    @typing.overload
    def move_to_pose(self, position: numpy.ndarray[numpy.float64, _Shape[4, 4]], speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: ...
    def move_to_start(self, speed_factor: float = 0.2, stiffness: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([600., 600., 600., 600., 250., 150., 50.]), damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([50., 50., 50., 20., 20., 20., 10.]), dq_threshold: float = 0.001, success_threshold: float = 0.01) -> bool: 
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
    def recover(self) -> None: ...
    def set_default_behavior(self) -> None: ...
    def start_controller(self, controller: TorqueController) -> None: ...
    def stop_controller(self) -> None: ...
    def teaching_mode(self, active: bool, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([0., 0., 0., 0., 0., 0., 0.])) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def q(self) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[7, 1]]
        """
    pass
class PandaContext():
    def __enter__(self) -> PandaContext: ...
    def __exit__(self, arg0: object, arg1: object, arg2: object) -> bool: ...
    def ok(self) -> bool: ...
    @property
    def num_ticks(self) -> int:
        """
        :type: int
        """
    @property
    def time(self) -> float:
        """
        :type: float
        """
    pass
class AppliedForce(TorqueController):
    def __init__(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([0., 0., 0., 0., 0., 0., 0.]), filter_coeff: float = 1.0) -> None: ...
    def set_control(self, force: numpy.ndarray[numpy.float64, _Shape[6, 1]]) -> None: ...
    def set_damping(self, damping: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def set_filter(self, filter_coeff: float) -> None: ...
    pass
def fk(q: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> numpy.ndarray[numpy.float64, _Shape[4, 4]]:
    """
    Computes end-effector pose in base frame from joint positions.
    """
@typing.overload
def ik(O_T_EE: numpy.ndarray[numpy.float64, _Shape[4, 4]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
    """
    Compute analytical inverse kinematics. 
    Solution is case consistent with configuration  given in `q_init`.

    Args:
      O_T_EE: Homogeneous transform :math:`\mathbb{R}^{4\times 4}` describing
        the end-effector pose.
      q_init: Reference joint positions, the result will be consistent
        with this configuration.
      q_7: Joint 7 is considered the redundant joint, use `q_7` to set the
        desired joint position (default: :math:`\frac{\pi}{4}`).

    Returns:
      Vector of shape (7,) containing joint positions.



    Same as :py:func:`ik` above, but takes position and orientation arguments.
    """
@typing.overload
def ik(position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
    pass
@typing.overload
def ik_full(O_T_EE: numpy.ndarray[numpy.float64, _Shape[4, 4]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[4, 7]]:
    pass
@typing.overload
def ik_full(position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0. , -0.78539816, 0. , -2.35619449, 0. , 1.57079633, 0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[4, 7]]:
    pass
_JOINT_LIMITS_LOWER: numpy.ndarray # value = array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_JOINT_LIMITS_UPPER: numpy.ndarray # value = array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
_JOINT_POSITION_START: numpy.ndarray # value = 
"""
array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        ,
        1.57079633,  0.78539816])
"""
