from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import panda_py.libfranka
import typing
__all__: list[str] = ['AppliedForce', 'AppliedTorque', 'CartesianImpedance', 'CartesianTrajectory', 'Force', 'IntegratedVelocity', 'JointPosition', 'JointTrajectory', 'Panda', 'PandaContext', 'TorqueController', 'fk', 'ik', 'ik_full']
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
class CartesianImpedance(TorqueController):
    def __init__(self, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 0.5, filter_coeff: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        """
                       Cartesian impedance controller. Takes the end-effector pose in robot
                       base frame, as well as desired nullspace joint positions as input.
        
                       Args:
                         impedance: Cartesian impedance expressed as a matrix
                           :math:`\\in \\mathbb{R}^{6\\times 6}`.
                         damping_ratio: Cartesian damping is computed based on the given
                           impedance and damping ratio.
                         nullspace_stiffness: Control gain of the nullspace term.
                         filter_coeff: TP1 filter coefficient used to filter input signals.
        """
    def set_control(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], q_nullspace: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> None:
        ...
    def set_damping_ratio(self, damping: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_filter(self, filter_coeff: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_impedance(self, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"]) -> None:
        ...
    def set_nullspace_stiffness(self, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex) -> None:
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
    def __init__(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ..., filter_coeff: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    def set_control(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"], velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"] = ...) -> None:
        ...
    def set_damping(self, damping: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
    def set_filter(self, filter_coeff: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_stiffness(self, stiffness: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[7, 1]"]) -> None:
        ...
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
    def move_to_pose(self, positions: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]], orientations: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        """
                       Moves the end-effector from the current pose through the provided waypoints
                       in piece-wise linear segments. The waypoints are given as lists of positions
                       :math:`\\in \\mathbb{R}^3` and orientations
                       :math:`\\mathbf q = (\\vec{v},\\ r),~~ \\mathbf q \\in \\mathbb{H},~~ \\vec{v}\\in \\mathbb{R}^3,~~ r \\in \\mathbb{R}`,
                       i.e. quaternions with scalar last. The computed trajectory is time-optimal.
        """
    @typing.overload
    def move_to_pose(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], orientation: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 1]"], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        """
                       Same as :py:func:`move_to_pose` above, but only one target pose given as
                       position and orientation directly.
        """
    @typing.overload
    def move_to_pose(self, pose: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"]], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
        """
                       Same as :py:func:`move_to_pose` above, but waypoints are given as a list of
                       homogeneous transforms :math:`\\in \\mathbb{R}^{4\\times 4}`.
        """
    @typing.overload
    def move_to_pose(self, pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[4, 4]"], speed_factor: typing.SupportsFloat | typing.SupportsIndex = 0.2, impedance: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[6, 6]"] = ..., damping_ratio: typing.SupportsFloat | typing.SupportsIndex = 1.0, nullspace_stiffness: typing.SupportsFloat | typing.SupportsIndex = 15.0, dq_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.001, success_threshold: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> bool:
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
_DTAU_J_MAX: numpy.ndarray  # value = array([1000., 1000., 1000., 1000., 1000., 1000., 1000.])
_JOINT_LIMITS_LOWER: numpy.ndarray  # value = array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_JOINT_LIMITS_LOWER_FR3: numpy.ndarray  # value = array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065,  0.5445, -3.0159])
_JOINT_LIMITS_LOWER_FR3_5_9: numpy.ndarray  # value = array([-2.9007, -1.8361, -2.9007, -3.077 , -2.8763,  0.4398, -3.0508])
_JOINT_LIMITS_UPPER: numpy.ndarray  # value = array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
_JOINT_LIMITS_UPPER_FR3: numpy.ndarray  # value = array([ 2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159])
_JOINT_LIMITS_UPPER_FR3_5_9: numpy.ndarray  # value = array([ 2.9007,  1.8361,  2.9007, -0.1169,  2.8763,  4.6216,  3.0508])
_JOINT_POSITION_START: numpy.ndarray  # value = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        ,...
_TAU_J_MAX: numpy.ndarray  # value = array([87., 87., 87., 87., 12., 12., 12.])
