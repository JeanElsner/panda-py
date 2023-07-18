from __future__ import annotations
import panda_py.libfranka
import typing
import datetime
import pybind11_stubgen.typing_ext

__all__ = [
    "CartesianPose",
    "CartesianVelocities",
    "ControllerMode",
    "Duration",
    "Errors",
    "Frame",
    "Gripper",
    "GripperState",
    "JointPositions",
    "JointVelocities",
    "MAX_TORQUE_RATE",
    "Model",
    "RealtimeConfig",
    "Robot",
    "RobotMode",
    "RobotState",
    "Torques",
    "has_realtime_kernel",
    "is_homogeneous_transformation",
    "is_valid_elbow",
    "limit_rate",
    "motion_finished",
    "set_current_thread_to_highest_scheduler_priority"
]


class CartesianPose():
    @typing.overload
    def __init__(self, cartesian_pose: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> None: ...
    @typing.overload
    def __init__(self, cartesian_pose: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], elbow: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]) -> None: ...
    @property
    def O_T_EE(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @O_T_EE.setter
    def O_T_EE(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> None:
        pass
    @property
    def elbow(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @elbow.setter
    def elbow(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]) -> None:
        pass
    @property
    def motion_finished(self) -> bool:
        """
        :type: bool
        """
    @motion_finished.setter
    def motion_finished(self, arg0: bool) -> None:
        pass
    pass
class CartesianVelocities():
    @typing.overload
    def __init__(self, cartesian_velocities: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]) -> None: ...
    @typing.overload
    def __init__(self, cartesian_velocities: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)], elbow: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]) -> None: ...
    @property
    def O_dP_EE(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @O_dP_EE.setter
    def O_dP_EE(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]) -> None:
        pass
    @property
    def elbow(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @elbow.setter
    def elbow(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]) -> None:
        pass
    @property
    def motion_finished(self) -> bool:
        """
        :type: bool
        """
    @motion_finished.setter
    def motion_finished(self, arg0: bool) -> None:
        pass
    pass
class ControllerMode():
    """
    Members:

      kCartesianImpedance

      kJointImpedance
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'kCartesianImpedance': <ControllerMode.kCartesianImpedance: 1>, 'kJointImpedance': <ControllerMode.kJointImpedance: 0>}
    kCartesianImpedance: panda_py.libfranka.ControllerMode # value = <ControllerMode.kCartesianImpedance: 1>
    kJointImpedance: panda_py.libfranka.ControllerMode # value = <ControllerMode.kJointImpedance: 0>
    pass
class Duration():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, milliseconds: int) -> None: ...
    @typing.overload
    def __init__(self, duration: datetime.timedelta) -> None: ...
    def to_msec(self) -> int: ...
    def to_sec(self) -> float: ...
    pass
class Errors():
    def __bool__(self) -> bool: ...
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def cartesian_motion_generator_acceleration_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_elbow_limit_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_elbow_sign_inconsistent(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_joint_acceleration_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_joint_position_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_joint_velocity_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_joint_velocity_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_start_elbow_invalid(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_velocity_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_motion_generator_velocity_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_position_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_position_motion_generator_invalid_frame(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_position_motion_generator_start_pose_invalid(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_reflex(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_velocity_profile_safety_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def cartesian_velocity_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def communication_constraints_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def controller_torque_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def force_control_safety_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def force_controller_desired_force_tolerance_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def instability_detected(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_motion_generator_acceleration_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_motion_generator_position_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_motion_generator_velocity_discontinuity(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_motion_generator_velocity_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_move_in_wrong_direction(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_p2p_insufficient_torque_for_planning(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_position_limits_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_position_motion_generator_start_pose_invalid(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_reflex(self) -> bool:
        """
        :type: bool
        """
    @property
    def joint_velocity_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def max_goal_pose_deviation_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def max_path_pose_deviation_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def power_limit_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def self_collision_avoidance_violation(self) -> bool:
        """
        :type: bool
        """
    @property
    def start_elbow_sign_inconsistent(self) -> bool:
        """
        :type: bool
        """
    @property
    def tau_j_range_violation(self) -> bool:
        """
        :type: bool
        """
    pass
class Frame():
    """
    Members:

      kJoint1

      kJoint2

      kJoint3

      kJoint4

      kJoint5

      kJoint6

      kJoint7

      kFlange

      kEndEffector

      kStiffness
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'kJoint1': <Frame.kJoint1: 0>, 'kJoint2': <Frame.kJoint2: 1>, 'kJoint3': <Frame.kJoint3: 2>, 'kJoint4': <Frame.kJoint4: 3>, 'kJoint5': <Frame.kJoint5: 4>, 'kJoint6': <Frame.kJoint6: 5>, 'kJoint7': <Frame.kJoint7: 6>, 'kFlange': <Frame.kFlange: 7>, 'kEndEffector': <Frame.kEndEffector: 8>, 'kStiffness': <Frame.kStiffness: 9>}
    kEndEffector: panda_py.libfranka.Frame # value = <Frame.kEndEffector: 8>
    kFlange: panda_py.libfranka.Frame # value = <Frame.kFlange: 7>
    kJoint1: panda_py.libfranka.Frame # value = <Frame.kJoint1: 0>
    kJoint2: panda_py.libfranka.Frame # value = <Frame.kJoint2: 1>
    kJoint3: panda_py.libfranka.Frame # value = <Frame.kJoint3: 2>
    kJoint4: panda_py.libfranka.Frame # value = <Frame.kJoint4: 3>
    kJoint5: panda_py.libfranka.Frame # value = <Frame.kJoint5: 4>
    kJoint6: panda_py.libfranka.Frame # value = <Frame.kJoint6: 5>
    kJoint7: panda_py.libfranka.Frame # value = <Frame.kJoint7: 6>
    kStiffness: panda_py.libfranka.Frame # value = <Frame.kStiffness: 9>
    pass
class Gripper():
    def __init__(self, franka_address: str) -> None: ...
    def grasp(self, width: float, speed: float, force: float, epsilon_inner: float = 0.005, epsilon_outer: float = 0.005) -> bool: ...
    def homing(self) -> bool: ...
    def move(self, width: float, speed: float) -> bool: ...
    def read_once(self) -> GripperState: ...
    def server_version(self) -> int: ...
    def stop(self) -> bool: ...
    pass
class GripperState():
    @property
    def is_grasped(self) -> bool:
        """
        :type: bool
        """
    @property
    def max_width(self) -> float:
        """
        :type: float
        """
    @property
    def temperature(self) -> int:
        """
        :type: int
        """
    @property
    def time(self) -> Duration:
        """
        :type: Duration
        """
    @property
    def width(self) -> float:
        """
        :type: float
        """
    pass
class JointPositions():
    def __init__(self, joint_positions: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None: ...
    @property
    def motion_finished(self) -> bool:
        """
        :type: bool
        """
    @motion_finished.setter
    def motion_finished(self, arg0: bool) -> None:
        pass
    @property
    def q(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @q.setter
    def q(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None:
        pass
    pass
class JointVelocities():
    def __init__(self, joint_velocities: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None: ...
    @property
    def dq(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @dq.setter
    def dq(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None:
        pass
    @property
    def motion_finished(self) -> bool:
        """
        :type: bool
        """
    @motion_finished.setter
    def motion_finished(self, arg0: bool) -> None:
        pass
    pass
class Model():
    @typing.overload
    def body_jacobian(self, frame: Frame, robot_state: RobotState) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(42)]: ...
    @typing.overload
    def body_jacobian(self, frame: Frame, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], F_T_EE: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], EE_T_K: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(42)]: ...
    @typing.overload
    def coriolis(self, robot_state: RobotState) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]: ...
    @typing.overload
    def coriolis(self, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], dq: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], I_total: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)], m_total: float, F_x_Ctotal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]: ...
    @typing.overload
    def gravity(self, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], m_total: float, F_x_Ctotal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)], gravity_earth: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)] = [0.0, 0.0, -9.81]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]: ...
    @typing.overload
    def gravity(self, robot_state: RobotState, gravity_earth: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)] = [0.0, 0.0, -9.81]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]: ...
    @typing.overload
    def mass(self, robot_state: RobotState) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(49)]: ...
    @typing.overload
    def mass(self, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], I_total: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)], m_total: float, F_x_Ctotal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(49)]: ...
    @typing.overload
    def pose(self, frame: Frame, robot_state: RobotState) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]: ...
    @typing.overload
    def pose(self, frame: Frame, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], F_T_EE: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], EE_T_K: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]: ...
    @typing.overload
    def zero_jacobian(self, frame: Frame, robot_state: RobotState) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(42)]: ...
    @typing.overload
    def zero_jacobian(self, frame: Frame, q: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], F_T_EE: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], EE_T_K: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(42)]: ...
    pass
class RealtimeConfig():
    """
    Members:

      kEnforce

      kIgnore
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'kEnforce': <RealtimeConfig.kEnforce: 0>, 'kIgnore': <RealtimeConfig.kIgnore: 1>}
    kEnforce: panda_py.libfranka.RealtimeConfig # value = <RealtimeConfig.kEnforce: 0>
    kIgnore: panda_py.libfranka.RealtimeConfig # value = <RealtimeConfig.kIgnore: 1>
    pass
class Robot():
    def __init__(self, franka_address: str, realtime_config: RealtimeConfig = RealtimeConfig.kIgnore, log_size: int = 50) -> None: ...
    def automatic_error_recovery(self) -> None: ...
    def control(self, control_callback: typing.Callable[[RobotState, Duration], Torques], limit_rate: bool = True, cutoff_frequency: float = 100.0) -> None: ...
    def load_model(self) -> Model: ...
    def read(self, arg0: typing.Callable[[RobotState], bool]) -> None: ...
    def read_once(self) -> RobotState: ...
    def server_version(self) -> int: ...
    def set_cartesian_impedance(self, K_x: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]) -> None: ...
    @typing.overload
    def set_collision_behavior(self, lower_torque_thresholds_acceleration: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], upper_torque_thresholds_acceleration: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], lower_torque_thresholds_nominal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], upper_torque_thresholds_nominal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], lower_force_thresholds_acceleration: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)], upper_force_thresholds_acceleration: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)], lower_force_thresholds_nominal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)], upper_force_thresholds_nominal: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]) -> None: ...
    @typing.overload
    def set_collision_behavior(self, lower_torque_thresholds: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], upper_torque_thresholds: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], lower_force_thresholds: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)], upper_force_thresholds: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]) -> None: ...
    def set_ee(self, NE_T_EE: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> None: ...
    def set_guiding_mode(self, guiding_mode: typing.Annotated[typing.List[bool], pybind11_stubgen.typing_ext.FixedSize(6)], elbow: bool) -> None: ...
    def set_joint_impedance(self, K_theta: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None: ...
    def set_k(self, EE_T_K: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> None: ...
    def set_load(self, load_mass: float, F_x_Cload: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)], load_inertia: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]) -> None: ...
    def stop(self) -> None: ...
    pass
class RobotMode():
    """
    Members:

      kOther

      kIdle

      kMove

      kGuiding

      kReflex

      kUserStopped

      kAutomaticErrorRecovery
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    __members__: dict # value = {'kOther': <RobotMode.kOther: 0>, 'kIdle': <RobotMode.kIdle: 1>, 'kMove': <RobotMode.kMove: 2>, 'kGuiding': <RobotMode.kGuiding: 3>, 'kReflex': <RobotMode.kReflex: 4>, 'kUserStopped': <RobotMode.kUserStopped: 5>, 'kAutomaticErrorRecovery': <RobotMode.kAutomaticErrorRecovery: 6>}
    kAutomaticErrorRecovery: panda_py.libfranka.RobotMode # value = <RobotMode.kAutomaticErrorRecovery: 6>
    kGuiding: panda_py.libfranka.RobotMode # value = <RobotMode.kGuiding: 3>
    kIdle: panda_py.libfranka.RobotMode # value = <RobotMode.kIdle: 1>
    kMove: panda_py.libfranka.RobotMode # value = <RobotMode.kMove: 2>
    kOther: panda_py.libfranka.RobotMode # value = <RobotMode.kOther: 0>
    kReflex: panda_py.libfranka.RobotMode # value = <RobotMode.kReflex: 4>
    kUserStopped: panda_py.libfranka.RobotMode # value = <RobotMode.kUserStopped: 5>
    pass
class RobotState():
    def __repr__(self) -> str: ...
    @property
    def EE_T_K(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @property
    def F_T_EE(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @property
    def F_x_Cee(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]
        """
    @property
    def F_x_Cload(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]
        """
    @property
    def F_x_Ctotal(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(3)]
        """
    @property
    def I_ee(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]
        """
    @property
    def I_load(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]
        """
    @property
    def I_total(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(9)]
        """
    @property
    def K_F_ext_hat_K(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def O_F_ext_hat_K(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def O_T_EE(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @property
    def O_T_EE_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @property
    def O_T_EE_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]
        """
    @property
    def O_dP_EE_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def O_dP_EE_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def O_ddP_EE_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def cartesian_collision(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def cartesian_contact(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(6)]
        """
    @property
    def control_command_success_rate(self) -> float:
        """
        :type: float
        """
    @property
    def current_errors(self) -> Errors:
        """
        :type: Errors
        """
    @property
    def ddelbow_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def ddq_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def delbow_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def dq(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def dq_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def dtau_J(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def dtheta(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def elbow(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def elbow_c(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def elbow_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]
        """
    @property
    def joint_collision(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def joint_contact(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def last_motion_errors(self) -> Errors:
        """
        :type: Errors
        """
    @property
    def m_ee(self) -> float:
        """
        :type: float
        """
    @property
    def m_load(self) -> float:
        """
        :type: float
        """
    @property
    def m_total(self) -> float:
        """
        :type: float
        """
    @property
    def q(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def q_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def robot_mode(self) -> RobotMode:
        """
        :type: RobotMode
        """
    @property
    def tau_J(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def tau_J_d(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def tau_ext_hat_filtered(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def theta(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @property
    def time(self) -> Duration:
        """
        :type: Duration
        """
    pass
class Torques():
    def __init__(self, torques: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None: ...
    @property
    def motion_finished(self) -> bool:
        """
        :type: bool
        """
    @motion_finished.setter
    def motion_finished(self, arg0: bool) -> None:
        pass
    @property
    def tau_J(self) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
        """
        :type: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]
        """
    @tau_J.setter
    def tau_J(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> None:
        pass
    pass
def has_realtime_kernel() -> bool:
    pass
def is_homogeneous_transformation(transform: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)]) -> bool:
    pass
def is_valid_elbow(elbow: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(2)]) -> bool:
    pass
def limit_rate(arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], arg1: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)], arg2: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]) -> typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(7)]:
    pass
@typing.overload
def motion_finished(command: Torques) -> Torques:
    pass
@typing.overload
def motion_finished(command: JointPositions) -> JointPositions:
    pass
@typing.overload
def motion_finished(command: JointVelocities) -> JointVelocities:
    pass
@typing.overload
def motion_finished(command: CartesianPose) -> CartesianPose:
    pass
@typing.overload
def motion_finished(command: CartesianVelocities) -> CartesianVelocities:
    pass
def set_current_thread_to_highest_scheduler_priority(error_message: str) -> bool:
    pass
MAX_TORQUE_RATE = [999.999, 999.999, 999.999, 999.999, 999.999, 999.999, 999.999]
