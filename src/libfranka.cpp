#include <franka/control_tools.h>
#include <franka/gripper.h>
#include <franka/vacuum_gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>

#define PYBIND11_DETAILED_ERROR_MESSAGES 1

#define def_readonly_franka_robotstate(name) \
  def_readonly(#name, &franka::RobotState::name)
#define def_property_readonly_errors(name) \
  def_property_readonly(                   \
      #name, [](const franka::Errors &errors) { return errors.name; })

namespace py = pybind11;

const std::array<double, 3> gravity_earth = {0., 0., -9.81};

PYBIND11_MODULE(libfranka, m) {
  py::options options;
  //   options.disable_function_signatures();
  //   options.disable_enum_members_docstring();

  py::class_<franka::Errors>(m, "Errors")
      .def(py::init())
      //.def(py::init<const std::array<bool, 37> &>(), py::arg("errors"))
      .def("__repr__",
           [](const franka::Errors &errors) { return std::string(errors); })
      .def("__bool__",
           [](const franka::Errors &errors) { return bool(errors); })
      .def_property_readonly_errors(joint_position_limits_violation)
      .def_property_readonly_errors(cartesian_position_limits_violation)
      .def_property_readonly_errors(self_collision_avoidance_violation)
      .def_property_readonly_errors(joint_velocity_violation)
      .def_property_readonly_errors(cartesian_velocity_violation)
      .def_property_readonly_errors(force_control_safety_violation)
      .def_property_readonly_errors(joint_reflex)
      .def_property_readonly_errors(cartesian_reflex)
      .def_property_readonly_errors(max_goal_pose_deviation_violation)
      .def_property_readonly_errors(max_path_pose_deviation_violation)
      .def_property_readonly_errors(cartesian_velocity_profile_safety_violation)
      .def_property_readonly_errors(
          joint_position_motion_generator_start_pose_invalid)
      .def_property_readonly_errors(
          joint_motion_generator_position_limits_violation)
      .def_property_readonly_errors(
          joint_motion_generator_velocity_limits_violation)
      .def_property_readonly_errors(
          joint_motion_generator_velocity_discontinuity)
      .def_property_readonly_errors(
          joint_motion_generator_acceleration_discontinuity)
      .def_property_readonly_errors(
          cartesian_position_motion_generator_start_pose_invalid)
      .def_property_readonly_errors(
          cartesian_motion_generator_elbow_limit_violation)
      .def_property_readonly_errors(
          cartesian_motion_generator_velocity_limits_violation)
      .def_property_readonly_errors(
          cartesian_motion_generator_velocity_discontinuity)
      .def_property_readonly_errors(
          cartesian_motion_generator_acceleration_discontinuity)
      .def_property_readonly_errors(
          cartesian_motion_generator_elbow_sign_inconsistent)
      .def_property_readonly_errors(
          cartesian_motion_generator_start_elbow_invalid)
      .def_property_readonly_errors(
          cartesian_motion_generator_joint_position_limits_violation)
      .def_property_readonly_errors(
          cartesian_motion_generator_joint_velocity_limits_violation)
      .def_property_readonly_errors(
          cartesian_motion_generator_joint_velocity_discontinuity)
      .def_property_readonly_errors(
          cartesian_motion_generator_joint_acceleration_discontinuity)
      .def_property_readonly_errors(
          cartesian_position_motion_generator_invalid_frame)
      .def_property_readonly_errors(
          force_controller_desired_force_tolerance_violation)
      .def_property_readonly_errors(controller_torque_discontinuity)
      .def_property_readonly_errors(start_elbow_sign_inconsistent)
      .def_property_readonly_errors(communication_constraints_violation)
      .def_property_readonly_errors(power_limit_violation)
      .def_property_readonly_errors(joint_p2p_insufficient_torque_for_planning)
      .def_property_readonly_errors(tau_j_range_violation)
      .def_property_readonly_errors(instability_detected)
      .def_property_readonly_errors(joint_move_in_wrong_direction);

  py::enum_<franka::RobotMode>(m, "RobotMode")
      .value("kOther", franka::RobotMode::kOther)
      .value("kIdle", franka::RobotMode::kIdle)
      .value("kMove", franka::RobotMode::kMove)
      .value("kGuiding", franka::RobotMode::kGuiding)
      .value("kReflex", franka::RobotMode::kReflex)
      .value("kUserStopped", franka::RobotMode::kUserStopped)
      .value("kAutomaticErrorRecovery",
             franka::RobotMode::kAutomaticErrorRecovery);

  // TODO: Add arithmetic and comparison operators
  py::class_<franka::Duration>(m, "Duration")
      .def(py::init())
      .def(py::init<uint64_t>(), py::arg("milliseconds"))
      .def(py::init<std::chrono::duration<uint64_t, std::milli>>(),
           py::arg("duration"))
      .def("to_sec", &franka::Duration::toSec)
      .def("to_msec", &franka::Duration::toMSec);

  py::class_<franka::RobotState>(m, "RobotState")
      .def_readonly_franka_robotstate(O_T_EE)
      .def_readonly_franka_robotstate(O_T_EE_d)
      .def_readonly_franka_robotstate(F_T_EE)
      //   .def_readonly_franka_robotstate(F_T_NE)
      //   .def_readonly_franka_robotstate(NE_T_EE)
      .def_readonly_franka_robotstate(EE_T_K)
      .def_readonly_franka_robotstate(m_ee)
      .def_readonly_franka_robotstate(I_ee)
      .def_readonly_franka_robotstate(F_x_Cee)
      .def_readonly_franka_robotstate(m_load)
      .def_readonly_franka_robotstate(I_load)
      .def_readonly_franka_robotstate(F_x_Cload)
      .def_readonly_franka_robotstate(m_total)
      .def_readonly_franka_robotstate(I_total)
      .def_readonly_franka_robotstate(F_x_Ctotal)
      .def_readonly_franka_robotstate(elbow)
      .def_readonly_franka_robotstate(elbow_d)
      .def_readonly_franka_robotstate(elbow_c)
      .def_readonly_franka_robotstate(delbow_c)
      .def_readonly_franka_robotstate(ddelbow_c)
      .def_readonly_franka_robotstate(tau_J)
      .def_readonly_franka_robotstate(tau_J_d)
      .def_readonly_franka_robotstate(dtau_J)
      .def_readonly_franka_robotstate(q)
      .def_readonly_franka_robotstate(q_d)
      .def_readonly_franka_robotstate(dq)
      .def_readonly_franka_robotstate(dq_d)
      .def_readonly_franka_robotstate(ddq_d)
      .def_readonly_franka_robotstate(joint_contact)
      .def_readonly_franka_robotstate(cartesian_contact)
      .def_readonly_franka_robotstate(joint_collision)
      .def_readonly_franka_robotstate(cartesian_collision)
      .def_readonly_franka_robotstate(tau_ext_hat_filtered)
      .def_readonly_franka_robotstate(O_F_ext_hat_K)
      .def_readonly_franka_robotstate(K_F_ext_hat_K)
      .def_readonly_franka_robotstate(O_dP_EE_d)
      .def_readonly_franka_robotstate(O_T_EE_c)
      .def_readonly_franka_robotstate(O_dP_EE_c)
      .def_readonly_franka_robotstate(O_ddP_EE_c)
      .def_readonly_franka_robotstate(theta)
      .def_readonly_franka_robotstate(dtheta)
      .def_readonly_franka_robotstate(current_errors)
      .def_readonly_franka_robotstate(last_motion_errors)
      .def_readonly_franka_robotstate(control_command_success_rate)
      .def_readonly_franka_robotstate(robot_mode)
      .def_readonly_franka_robotstate(time)
      .def("__repr__", [](const franka::RobotState &s) {
        auto ss = std::stringstream();
        ss << s;
        return ss.str();
      });

  py::enum_<franka::Frame>(m, "Frame")
      .value("kJoint1", franka::Frame::kJoint1)
      .value("kJoint2", franka::Frame::kJoint2)
      .value("kJoint3", franka::Frame::kJoint3)
      .value("kJoint4", franka::Frame::kJoint4)
      .value("kJoint5", franka::Frame::kJoint5)
      .value("kJoint6", franka::Frame::kJoint6)
      .value("kJoint7", franka::Frame::kJoint7)
      .value("kFlange", franka::Frame::kFlange)
      .value("kEndEffector", franka::Frame::kEndEffector)
      .value("kStiffness", franka::Frame::kStiffness);

  py::class_<franka::Model>(m, "Model")
      .def("pose",
           py::overload_cast<franka::Frame, const franka::RobotState &>(
               &franka::Model::pose, py::const_),
           py::arg("frame"), py::arg("robot_state"))
      .def("pose",
           py::overload_cast<franka::Frame, const std::array<double, 7> &,
                             const std::array<double, 16> &,
                             const std::array<double, 16> &>(
               &franka::Model::pose, py::const_),
           py::arg("frame"), py::arg("q"), py::arg("F_T_EE"), py::arg("EE_T_K"))
      .def("body_jacobian",
           py::overload_cast<franka::Frame, const franka::RobotState &>(
               &franka::Model::bodyJacobian, py::const_),
           py::arg("frame"), py::arg("robot_state"))
      .def("body_jacobian",
           py::overload_cast<franka::Frame, const std::array<double, 7> &,
                             const std::array<double, 16> &,
                             const std::array<double, 16> &>(
               &franka::Model::bodyJacobian, py::const_),
           py::arg("frame"), py::arg("q"), py::arg("F_T_EE"), py::arg("EE_T_K"))
      .def("zero_jacobian",
           py::overload_cast<franka::Frame, const franka::RobotState &>(
               &franka::Model::zeroJacobian, py::const_),
           py::arg("frame"), py::arg("robot_state"))
      .def("zero_jacobian",
           py::overload_cast<franka::Frame, const std::array<double, 7> &,
                             const std::array<double, 16> &,
                             const std::array<double, 16> &>(
               &franka::Model::zeroJacobian, py::const_),
           py::arg("frame"), py::arg("q"), py::arg("F_T_EE"), py::arg("EE_T_K"))
      .def("mass",
           py::overload_cast<const franka::RobotState &>(&franka::Model::mass,
                                                         py::const_),
           py::arg("robot_state"))
      .def("mass",
           py::overload_cast<const std::array<double, 7> &,
                             const std::array<double, 9> &, double,
                             const std::array<double, 3> &>(
               &franka::Model::mass, py::const_),
           py::arg("q"), py::arg("I_total"), py::arg("m_total"),
           py::arg("F_x_Ctotal"))
      .def("coriolis",
           py::overload_cast<const franka::RobotState &>(
               &franka::Model::coriolis, py::const_),
           py::arg("robot_state"))
      .def("coriolis",
           py::overload_cast<const std::array<double, 7> &,
                             const std::array<double, 7> &,
                             const std::array<double, 9> &, double,
                             const std::array<double, 3> &>(
               &franka::Model::coriolis, py::const_),
           py::arg("q"), py::arg("dq"), py::arg("I_total"), py::arg("m_total"),
           py::arg("F_x_Ctotal"))
      .def("gravity",
           py::overload_cast<const std::array<double, 7> &, double,
                             const std::array<double, 3> &,
                             const std::array<double, 3> &>(
               &franka::Model::gravity, py::const_),
           py::arg("q"), py::arg("m_total"), py::arg("F_x_Ctotal"),
           py::arg("gravity_earth") = gravity_earth)
      .def("gravity",
           py::overload_cast<const franka::RobotState &,
                             const std::array<double, 3> &>(
               &franka::Model::gravity, py::const_),
           py::arg("robot_state"), py::arg("gravity_earth") = gravity_earth);

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("kEnforce", franka::RealtimeConfig::kEnforce)
      .value("kIgnore", franka::RealtimeConfig::kIgnore);

  py::enum_<franka::ControllerMode>(m, "ControllerMode")
      .value("kCartesianImpedance", franka::ControllerMode::kCartesianImpedance)
      .value("kJointImpedance", franka::ControllerMode::kJointImpedance);

  py::class_<franka::Torques>(m, "Torques")
      .def(py::init<const std::array<double, 7> &>(), py::arg("torques"))
      .def_readwrite("motion_finished", &franka::Torques::motion_finished)
      .def_readwrite("tau_J", &franka::Torques::tau_J);

  py::class_<franka::JointPositions>(m, "JointPositions")
      .def(py::init<const std::array<double, 7> &>(),
           py::arg("joint_positions"))
      .def_readwrite("motion_finished",
                     &franka::JointPositions::motion_finished)
      .def_readwrite("q", &franka::JointPositions::q);

  py::class_<franka::JointVelocities>(m, "JointVelocities")
      .def(py::init<const std::array<double, 7> &>(),
           py::arg("joint_velocities"))
      .def_readwrite("motion_finished",
                     &franka::JointVelocities::motion_finished)
      .def_readwrite("dq", &franka::JointVelocities::dq);

  py::class_<franka::CartesianPose>(m, "CartesianPose")
      .def(py::init<const std::array<double, 16> &>(),
           py::arg("cartesian_pose"))
      .def(py::init<const std::array<double, 16> &,
                    const std::array<double, 2> &>(),
           py::arg("cartesian_pose"), py::arg("elbow"))
      .def_readwrite("motion_finished", &franka::CartesianPose::motion_finished)
      .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE)
      .def_readwrite("elbow", &franka::CartesianPose::elbow);

  py::class_<franka::CartesianVelocities>(m, "CartesianVelocities")
      .def(py::init<const std::array<double, 6> &>(),
           py::arg("cartesian_velocities"))
      .def(py::init<const std::array<double, 6> &,
                    const std::array<double, 2> &>(),
           py::arg("cartesian_velocities"), py::arg("elbow"))
      .def_readwrite("motion_finished",
                     &franka::CartesianVelocities::motion_finished)
      .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
      .def_readwrite("elbow", &franka::CartesianVelocities::elbow);

  /*py::class_<franka::VirtualWallCuboid>(m, "VirtualWallCuboid")
      .def_readonly("id", &franka::VirtualWallCuboid::id)
      .def_readonly("object_world_size",
                    &franka::VirtualWallCuboid::object_world_size)
      .def_readonly("p_frame", &franka::VirtualWallCuboid::p_frame)
      .def_readonly("active", &franka::VirtualWallCuboid::active);*/

  py::class_<franka::Robot>(m, "Robot")
      .def(py::init<const std::string, franka::RealtimeConfig, size_t>(),
           py::arg("franka_address"),
           py::arg("realtime_config") = franka::RealtimeConfig::kIgnore,
           py::arg("log_size") = 50)
      .def("read", &franka::Robot::read)
      .def("read_once", &franka::Robot::readOnce)
      .def("load_model", &franka::Robot::loadModel)
      .def("server_version", &franka::Robot::serverVersion)
      .def("control",
           py::overload_cast<std::function<franka::Torques(
                                 const franka::RobotState &, franka::Duration)>,
                             bool, double>(&franka::Robot::control),
           py::arg("control_callback"), py::arg("limit_rate") = true,
           py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_torque_joint_position",
    //        py::overload_cast<std::function<franka::Torques(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          std::function<franka::JointPositions(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          bool, double>(&franka::Robot::control),
    //        py::arg("control_callback"), py::arg("motion_generator_callback"),
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_torque_joint_velocity",
    //        py::overload_cast<std::function<franka::Torques(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          std::function<franka::JointVelocities(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          bool, double>(&franka::Robot::control),
    //        py::arg("control_callback"), py::arg("motion_generator_callback"),
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_torque_cartesian_pose",
    //        py::overload_cast<std::function<franka::Torques(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          std::function<franka::CartesianPose(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          bool, double>(&franka::Robot::control),
    //        py::call_guard<py::gil_scoped_release>(),
    //        py::arg("control_callback"), py::arg("motion_generator_callback"),
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_torque_cartesian_velocity",
    //        py::overload_cast<std::function<franka::Torques(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          std::function<franka::CartesianVelocities(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          bool, double>(&franka::Robot::control),
    //        py::arg("control_callback"), py::arg("motion_generator_callback"),
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_joint_position",
    //        py::overload_cast<std::function<franka::JointPositions(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          franka::ControllerMode, bool, double>(
    //            &franka::Robot::control),
    //        py::arg("motion_generator_callback"),
    //        py::arg("controller_mode") = franka::ControllerMode::kJointImpedance,
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_joint_velocity",
    //        py::overload_cast<std::function<franka::JointVelocities(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          franka::ControllerMode, bool, double>(
    //            &franka::Robot::control),
    //        py::arg("motion_generator_callback"),
    //        py::arg("controller_mode") = franka::ControllerMode::kJointImpedance,
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_cartesian_pose",
    //        py::overload_cast<std::function<franka::CartesianPose(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          franka::ControllerMode, bool, double>(
    //            &franka::Robot::control),
    //        py::arg("motion_generator_callback"),
    //        py::arg("controller_mode") = franka::ControllerMode::kJointImpedance,
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
    //   .def("control_cartesian_velocity",
    //        py::overload_cast<std::function<franka::CartesianVelocities(
    //                              const franka::RobotState &, franka::Duration)>,
    //                          franka::ControllerMode, bool, double>(
    //            &franka::Robot::control),
    //        py::arg("motion_generator_callback"),
    //        py::arg("controller_mode") = franka::ControllerMode::kJointImpedance,
    //        py::arg("limit_rate") = true,
    //        py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
      //.def("get_virtual_wall", &franka::Robot::getVirtualWall, py::arg("id"))
      .def("set_collision_behavior",
           py::overload_cast<
               const std::array<double, 7> &, const std::array<double, 7> &,
               const std::array<double, 7> &, const std::array<double, 7> &,
               const std::array<double, 6> &, const std::array<double, 6> &,
               const std::array<double, 6> &, const std::array<double, 6> &>(
               &franka::Robot::setCollisionBehavior),
           py::arg("lower_torque_thresholds_acceleration"),
           py::arg("upper_torque_thresholds_acceleration"),
           py::arg("lower_torque_thresholds_nominal"),
           py::arg("upper_torque_thresholds_nominal"),
           py::arg("lower_force_thresholds_acceleration"),
           py::arg("upper_force_thresholds_acceleration"),
           py::arg("lower_force_thresholds_nominal"),
           py::arg("upper_force_thresholds_nominal"))
      .def("set_collision_behavior",
           py::overload_cast<
               const std::array<double, 7> &, const std::array<double, 7> &,
               const std::array<double, 6> &, const std::array<double, 6> &>(
               &franka::Robot::setCollisionBehavior),
           py::arg("lower_torque_thresholds"),
           py::arg("upper_torque_thresholds"),
           py::arg("lower_force_thresholds"), py::arg("upper_force_thresholds"))
      .def("set_joint_impedance", &franka::Robot::setJointImpedance,
           py::arg("K_theta"))
      .def("set_cartesian_impedance", &franka::Robot::setCartesianImpedance,
           py::arg("K_x"))
      .def("set_guiding_mode", &franka::Robot::setGuidingMode,
           py::arg("guiding_mode"), py::arg("elbow"))
      .def("set_k", &franka::Robot::setK, py::arg("EE_T_K"))
      .def("set_ee", &franka::Robot::setEE, py::arg("NE_T_EE"))
      .def("set_load", &franka::Robot::setLoad, py::arg("load_mass"),
           py::arg("F_x_Cload"), py::arg("load_inertia"))
      /*.def("set_filters", &franka::Robot::setFilters,
           py::arg("joint_position_filter_frequency"),
           py::arg("joint_velocity_filter_frequency"),
           py::arg("cartesian_position_filter_frequency"),
           py::arg("cartesian_velocity_filter_frequency"),
           py::arg("controller_filter_frequency"))*/
      .def("automatic_error_recovery", &franka::Robot::automaticErrorRecovery)
      .def("stop", &franka::Robot::stop);

  py::class_<franka::GripperState>(m, "GripperState")
      .def_readonly("width", &franka::GripperState::width)
      .def_readonly("max_width", &franka::GripperState::max_width)
      .def_readonly("is_grasped", &franka::GripperState::is_grasped)
      .def_readonly("temperature", &franka::GripperState::temperature)
      .def_readonly("time", &franka::GripperState::time);

  py::class_<franka::Gripper>(m, "Gripper")
      .def(py::init<std::string>(), py::arg("franka_address"))
      .def("server_version", &franka::Gripper::serverVersion)
      .def("homing", &franka::Gripper::homing,
           py::call_guard<py::gil_scoped_release>())
      .def("grasp", &franka::Gripper::grasp,
           py::call_guard<py::gil_scoped_release>(), py::arg("width"),
           py::arg("speed"), py::arg("force"), py::arg("epsilon_inner") = 0.005,
           py::arg("epsilon_outer") = 0.005)
      .def("move", &franka::Gripper::move,
           py::call_guard<py::gil_scoped_release>(), py::arg("width"),
           py::arg("speed"))
      .def("stop", &franka::Gripper::stop,
           py::call_guard<py::gil_scoped_release>())
      .def("read_once", &franka::Gripper::readOnce);

  py::enum_<franka::VacuumGripperDeviceStatus>(m, "VacuumGripperDeviceStatus")
	.value("kGreen", franka::VacuumGripperDeviceStatus::kGreen)
	.value("kYellow", franka::VacuumGripperDeviceStatus::kYellow)
	.value("kOrange", franka::VacuumGripperDeviceStatus::kOrange)
	.value("kRed", franka::VacuumGripperDeviceStatus::kRed);
  
  py::class_<franka::VacuumGripperState>(m, "VacuumGripperState")
	.def_readonly("in_control_range", &franka::VacuumGripperState::in_control_range)
	.def_readonly("part_detached", &franka::VacuumGripperState::part_detached)
	.def_readonly("part_present", &franka::VacuumGripperState::part_present)
	.def_readonly("device_status", &franka::VacuumGripperState::device_status)
	.def_readonly("actual_power", &franka::VacuumGripperState::actual_power)
	.def_readonly("vacuum", &franka::VacuumGripperState::vacuum)
	.def_readonly("time", &franka::VacuumGripperState::time);

  py::enum_<franka::VacuumGripper::ProductionSetupProfile>(m, "VacuumGripperProductionSetupProfile")
	.value("kP0", franka::VacuumGripper::ProductionSetupProfile::kP0)
	.value("kP1", franka::VacuumGripper::ProductionSetupProfile::kP1)
	.value("kP2", franka::VacuumGripper::ProductionSetupProfile::kP2)
	.value("kP3", franka::VacuumGripper::ProductionSetupProfile::kP3);
  
  py::class_<franka::VacuumGripper>(m, "VacuumGripper")
	.def(py::init<std::string>(), py::arg("franka_address"))
	.def("server_version", &franka::VacuumGripper::serverVersion)
	.def("vacuum", &franka::VacuumGripper::vacuum,
		 py::call_guard<py::gil_scoped_release>(), py::arg("vacuum"),
		 py::arg("timeout"), 
		 py::arg("profile") = franka::VacuumGripper::ProductionSetupProfile::kP0)
	.def("drop_off", &franka::VacuumGripper::dropOff,
		 py::call_guard<py::gil_scoped_release>(), py::arg("timeout"))
	.def("stop", &franka::VacuumGripper::stop,
		 py::call_guard<py::gil_scoped_release>())
	.def("read_once", &franka::VacuumGripper::readOnce);		 
	
  m.def("is_valid_elbow", &franka::isValidElbow, py::arg("elbow"));
  m.def("is_homogeneous_transformation", &franka::isHomogeneousTransformation,
        py::arg("transform"));
  m.def("has_realtime_kernel", &franka::hasRealtimeKernel);
  m.def("set_current_thread_to_highest_scheduler_priority",
        &franka::setCurrentThreadToHighestSchedulerPriority,
        py::arg("error_message"));

  m.def("motion_finished",
        py::overload_cast<franka::Torques>(&franka::MotionFinished),
        py::arg("command"));
  m.def("motion_finished",
        py::overload_cast<franka::JointPositions>(&franka::MotionFinished),
        py::arg("command"));
  m.def("motion_finished",
        py::overload_cast<franka::JointVelocities>(&franka::MotionFinished),
        py::arg("command"));
  m.def("motion_finished",
        py::overload_cast<franka::CartesianPose>(&franka::MotionFinished),
        py::arg("command"));
  m.def("motion_finished",
        py::overload_cast<franka::CartesianVelocities>(&franka::MotionFinished),
        py::arg("command"));

  m.def("limit_rate",
        py::overload_cast<const std::array<double, 7> &,
                          const std::array<double, 7> &,
                          const std::array<double, 7> &>(&franka::limitRate));

  m.attr("MAX_TORQUE_RATE") = franka::kMaxTorqueRate;

  // TODO: lowpass_filter.h
  // TODO: rate_limiting.h
}
