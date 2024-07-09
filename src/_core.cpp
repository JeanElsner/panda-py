#include <franka/exception.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controllers/applied_force.h"
#include "controllers/applied_torque.h"
#include "controllers/cartesian_impedance.h"
#include "controllers/force.h"
#include "controllers/integrated_velocity.h"
#include "controllers/joint_position.h"
#include "kinematics/fk.h"
#include "kinematics/ik.h"
#include "motion/generators.h"
#include "panda.h"

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  py::module::import("panda_py.libfranka");
  py::options options;
  //  options.disable_function_signatures();
  //  options.disable_enum_members_docstring();

  m.attr("_JOINT_POSITION_START") = kJointPositionStart;
  m.attr("_JOINT_LIMITS_LOWER") = kLowerJointLimits;
  m.attr("_JOINT_LIMITS_UPPER") = kUpperJointLimits;
  m.attr("_TAU_J_MAX") = kTauJMax;
  m.attr("_DTAU_J_MAX") = kDTauJMax;

  m.def("ik_full",
        py::overload_cast<Eigen::Matrix<double, 4, 4>, Vector7d, double>(
            &kinematics::ik_full),
        py::arg("O_T_EE"), py::arg("q_init") = kinematics::kQDefault,
        py::arg("q_7") = M_PI_4);
  m.def("ik_full",
        py::overload_cast<const Eigen::Vector3d &, const Eigen::Vector4d &,
                          Vector7d, double>(&kinematics::ik_full),
        py::arg("position"), py::arg("orientation"),
        py::arg("q_init") = kinematics::kQDefault, py::arg("q_7") = M_PI_4);
  m.def("ik",
        py::overload_cast<Eigen::Matrix<double, 4, 4>, Vector7d, double>(
            &kinematics::ik),
        py::arg("O_T_EE"), py::arg("q_init") = kinematics::kQDefault,
        py::arg("q_7") = M_PI_4,
        R"delim(
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
          )delim");
  m.def("ik",
        py::overload_cast<const Eigen::Vector3d &, const Eigen::Vector4d &,
                          Vector7d, double>(&kinematics::ik),
        py::arg("position"), py::arg("orientation"),
        py::arg("q_init") = kinematics::kQDefault, py::arg("q_7") = M_PI_4,
        R"delim(
          Same as :py:func:`ik` above, but takes position and orientation arguments.
          )delim");
  m.def("fk", &kinematics::fk, py::arg("q"), R"delim(
     Computes end-effector pose in base frame from joint positions.
  )delim");

  py::class_<motion::JointTrajectory>(m, "JointTrajectory")
      .def(py::init<const std::vector<Vector7d> &, double, double, double>(),
           py::arg("waypoints"),
           py::arg("speed_factor") = motion::kDefaultJointSpeedFactor,
           py::arg("max_deviation") = 0,
           py::arg("timeout") = motion::kDefaultTimeout)
      .def("get_duration", &motion::JointTrajectory::getDuration)
      .def("get_joint_positions", &motion::JointTrajectory::getJointPositions,
           py::arg("time"))
      .def("get_joint_velocities", &motion::JointTrajectory::getJointVelocities,
           py::arg("time"))
      .def("get_joint_accelerations",
           &motion::JointTrajectory::getJointAccelerations, py::arg("time"));

  py::class_<motion::CartesianTrajectory>(m, "CartesianTrajectory")
      .def(py::init<const std::vector<Eigen::Matrix<double, 3, 1>> &,
                    const std::vector<Eigen::Matrix<double, 4, 1>> &, double,
                    double, double>(),
           py::arg("positions"), py::arg("orientations"),
           py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
           py::arg("max_deviation") = 0,
           py::arg("timeout") = motion::kDefaultTimeout)
      .def(py::init<const std::vector<Eigen::Matrix<double, 4, 4>> &, double,
                    double, double>(),
           py::arg("poses"),
           py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
           py::arg("max_deviation") = 0,
           py::arg("timeout") = motion::kDefaultTimeout)
      .def("get_duration", &motion::CartesianTrajectory::getDuration)
      .def("get_pose",
           &motion::CartesianTrajectory::getPose, py::arg("time"))
      .def("get_position",
           &motion::CartesianTrajectory::getPosition, py::arg("time"))
      .def("get_orientation",
           &motion::CartesianTrajectory::getOrientation, py::arg("time"));

  py::class_<PandaContext>(m, "PandaContext")
      .def("ok", &PandaContext::ok)
      .def("__enter__", &PandaContext::enter)
      .def("__exit__", &PandaContext::exit)
      .def_property_readonly("time", &PandaContext::getTime)
      .def_property_readonly("num_ticks", &PandaContext::getNumTicks);

  py::class_<Panda>(m, "Panda", R"delim(
     The main interface of panda-py to control the robot.
  )delim")
      .def(py::init<std::string, std::string, franka::RealtimeConfig>(),
           /*py::keep_alive<1, 0>(), py::call_guard<py::gil_scoped_release>(),*/
           py::arg("hostname"), py::arg("name") = "panda",
           py::arg("realtime_config") = franka::RealtimeConfig::kIgnore)
      .def_readonly("name", &Panda::name_)
      .def_property_readonly("q", &Panda::getJointPositions)
      .def("teaching_mode", &Panda::teaching_mode, py::arg("active"),
           py::arg("damping") = Panda::kDefaultTeachingDamping,
           py::call_guard<py::gil_scoped_release>())
      .def("create_context", &Panda::createContext, py::arg("frequency"),
           py::arg("max_runtime") = 0.0, py::arg("max_iter") = 0)
      .def("get_robot", &Panda::getRobot,
           py::return_value_policy::reference_internal, R"delim(
               Get a reference to the :py:class:`libfranka.Robot` class behind this instance.
           )delim")
      .def("get_model", &Panda::getModel,
           py::return_value_policy::reference_internal)
      .def("get_state", &Panda::getState, R"delim(
          Get a copy of the last :py:class:`libfranka.RobotState` received from the robot.
      )delim")
      .def("get_position", &Panda::getPosition, R"delim(
          Current end-effector position in robot base frame.
      )delim")
      .def("get_orientation", &Panda::getOrientation,
           py::arg("scalar_first") = false, R"delim(
               Get current end-effector orientation
               :math:`\mathbf q = (\vec{v},\ r),~~ \mathbf q \in \mathbb{H},~~ \vec{v}\in \mathbb{R}^3,~~ r \in \mathbb{R}`
               in robot base frame.

               Args:
                 scalar_first: If True returns quaternion in scalar first
                   representation (default: False)
               
               Returns:
                 Vector of shape (4,) holding quaternion coefficients.
           )delim")
      .def("get_pose", &Panda::getPose)
      .def("enable_logging", &Panda::enableLogging, py::arg("buffer_size"))
      .def("disable_logging", &Panda::disableLogging)
      .def("get_log", &Panda::getLog)
      .def("start_controller", &Panda::startController,
           py::call_guard<py::gil_scoped_release>(), py::arg("controller"))
      .def("stop_controller", &Panda::stopController)
      .def("move_to_joint_position",
           py::overload_cast<std::vector<Vector7d> &, double, const Vector7d &,
                             const Vector7d &, double, double>(
               &Panda::moveToJointPosition),
           py::call_guard<py::gil_scoped_release>(), py::arg("waypoints"),
           py::arg("speed_factor") = motion::kDefaultJointSpeedFactor,
           py::arg("stiffness") = controllers::JointTrajectory::kDefaultStiffness,
           py::arg("damping") = controllers::JointTrajectory::kDefaultDamping,
           py::arg("dq_threshold") =
               controllers::JointTrajectory::kDefaultDqThreshold,
           py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold)
      .def("move_to_joint_position",
           py::overload_cast<const Vector7d &, double, const Vector7d &,
                             const Vector7d &, double, double>(
               &Panda::moveToJointPosition),
           py::call_guard<py::gil_scoped_release>(), py::arg("positions"),
           py::arg("speed_factor") = motion::kDefaultJointSpeedFactor,
           py::arg("stiffness") = controllers::JointTrajectory::kDefaultStiffness,
           py::arg("damping") = controllers::JointTrajectory::kDefaultDamping,
           py::arg("dq_threshold") =
               controllers::JointTrajectory::kDefaultDqThreshold,
           py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold)
      .def(
          "move_to_pose",
          py::overload_cast<std::vector<Eigen::Vector3d> &,
                            std::vector<Eigen::Matrix<double, 4, 1>> &, double,
                            const Eigen::Matrix<double, 6, 6> &,
                            const double &,
                            const double &, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("positions"),
          py::arg("orientations"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold,
          R"delim(
               Moves the end-effector from the current pose through the provided waypoints
               in piece-wise linear segments. The waypoints are given as lists of positions
               :math:`\in \mathbb{R}^3` and orientations
               :math:`\mathbf q = (\vec{v},\ r),~~ \mathbf q \in \mathbb{H},~~ \vec{v}\in \mathbb{R}^3,~~ r \in \mathbb{R}`,
               i.e. quaternions with scalar last. The computed trajectory is time-optimal.
               )delim")
      .def(
          "move_to_pose",
          py::overload_cast<const Eigen::Vector3d &,
                            const Eigen::Matrix<double, 4, 1> &, double,
                            const Eigen::Matrix<double, 6, 6> &,
                            const double &,
                            const double &, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("position"),
          py::arg("orientation"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold,
          R"delim(
               Same as :py:func:`move_to_pose` above, but only one target pose given as
               position and orientation directly.
               )delim")
      .def("move_to_pose",
           py::overload_cast<const std::vector<Eigen::Matrix<double, 4, 4>> &,
                             double,
                             const Eigen::Matrix<double, 6, 6> &,
                             const double &,
                             const double &, double, double>(&Panda::moveToPose),
           py::call_guard<py::gil_scoped_release>(), py::arg("pose"),
           py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
           py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
           py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
           py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
           py::arg("dq_threshold") =
               controllers::JointTrajectory::kDefaultDqThreshold,
           py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold,
           R"delim(
               Same as :py:func:`move_to_pose` above, but waypoints are given as a list of
               homogeneous transforms :math:`\in \mathbb{R}^{4\times 4}`.
               )delim")
      .def(
          "move_to_pose",
          py::overload_cast<const Eigen::Matrix<double, 4, 4> &, double,
                            const Eigen::Matrix<double, 6, 6> &,
                            const double &,
                            const double &, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("pose"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold,
          R"delim(
               Same as :py:func:`move_to_pose` above, but only one target pose given as
               homogeneous transform :math:`\in \mathbb{R}^{4\times 4}`.
               )delim")
      .def("move_to_start", &Panda::moveToStart,
           py::call_guard<py::gil_scoped_release>(),
           py::arg("speed_factor") = motion::kDefaultJointSpeedFactor,
           py::arg("stiffness") = controllers::JointTrajectory::kDefaultStiffness,
           py::arg("damping") = controllers::JointTrajectory::kDefaultDamping,
           py::arg("dq_threshold") =
               controllers::JointTrajectory::kDefaultDqThreshold,
           py::arg("success_threshold") = Panda::kMoveToJointPositionThreshold,
           R"delim(
               Convenience function similar to :py:func:`move_to_pose`, moves the end-effector
               into the starting position (cf. :py:obj:`constants.JOINT_POSITION_START`).
               )delim")
      .def("set_default_behavior", &Panda::setDefaultBehavior)
      .def("raise_error", &Panda::raiseError, R"delim(
          Raises a `RuntimeError` in Python when the robot has an active error.
          As panda-py controllers run asynchroneously, encountered errors don't
          propagate to the proces' main thread. Use this function or
          :py:class:`PandaContext` to catch errors.
      )delim")
      .def("recover", &Panda::recover);

  py::class_<TorqueController, std::shared_ptr<TorqueController>>(
      m, "TorqueController", R"delim(
          Base class for all torque controllers. Torque controllers
          provide the robot with torques at 1KHz and the user with
          an asynchronous interface to provide control signals.
      )delim")
      .def("get_time", &TorqueController::getTime, R"delim(
          Get time in seconds since this controller was started.
      )delim");

  py::class_<IntegratedVelocity, TorqueController,
             std::shared_ptr<IntegratedVelocity>>(m, "IntegratedVelocity")
      .def(py::init<const Vector7d &,
                    const Vector7d &>(), /*py::keep_alive<1, 0>(),*/
           py::arg("stiffness") = IntegratedVelocity::kDefaultStiffness,
           py::arg("damping") = IntegratedVelocity::kDefaultDamping)
      .def("get_qd", &IntegratedVelocity::getQd, py::call_guard<py::gil_scoped_release>())
      .def("set_control", &IntegratedVelocity::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("velocity"))
      .def("set_stiffness", &IntegratedVelocity::setStiffness,
           py::call_guard<py::gil_scoped_release>(), py::arg("stiffness"))
      .def("set_damping", &IntegratedVelocity::setDamping,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"));

  py::class_<JointPosition, TorqueController, std::shared_ptr<JointPosition>>(
      m, "JointPosition")
      .def(py::init<const Vector7d &, const Vector7d &,
                    const double>(), /*py::keep_alive<1, 0>(),*/
           py::arg("stiffness") = JointPosition::kDefaultStiffness,
           py::arg("damping") = JointPosition::kDefaultDamping,
           py::arg("filter_coeff") = JointPosition::kDefaultFilterCoeff)
      .def("set_control", &JointPosition::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("velocity") = JointPosition::kDefaultDqd)
      .def("set_stiffness", &JointPosition::setStiffness,
           py::call_guard<py::gil_scoped_release>(), py::arg("stiffness"))
      .def("set_damping", &JointPosition::setDamping,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"))
      .def("set_filter", &JointPosition::setFilter,
           py::call_guard<py::gil_scoped_release>(), py::arg("filter_coeff"));

  py::class_<CartesianImpedance, TorqueController,
             std::shared_ptr<CartesianImpedance>>(m, "CartesianImpedance")
      .def(py::init<const Eigen::Matrix<double, 6, 6> &, const double &,
                    const double &,
                    const double &>(), /*py::keep_alive<1, 0>(),*/
           py::arg("impedance") = CartesianImpedance::kDefaultImpedance,
           py::arg("damping_ratio") = CartesianImpedance::kDefaultDampingRatio,
           py::arg("nullspace_stiffness") =
               CartesianImpedance::kDefaultNullspaceStiffness,
           py::arg("filter_coeff") = CartesianImpedance::kDefaultFilterCoeff,
           R"delim(
               Cartesian impedance controller. Takes the end-effector pose in robot
               base frame, as well as desired nullspace joint positions as input.

               Args:
                 impedance: Cartesian impedance expressed as a matrix
                   :math:`\in \mathbb{R}^{6\times 6}`.
                 damping_ratio: Cartesian damping is computed based on the given
                   impedance and damping ratio.
                 nullspace_stiffness: Control gain of the nullspace term.
                 filter_coeff: TP1 filter coefficient used to filter input signals.
           )delim")
      .def("set_control", &CartesianImpedance::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("orientation"), py::arg("q_nullspace") = kJointPositionStart)
      .def("set_impedance", &CartesianImpedance::setImpedance,
           py::call_guard<py::gil_scoped_release>(), py::arg("impedance"))
      .def("set_damping_ratio", &CartesianImpedance::setDampingRatio,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"))
      .def("set_nullspace_stiffness",
           &CartesianImpedance::setNullspaceStiffness,
           py::call_guard<py::gil_scoped_release>(),
           py::arg("nullspace_stiffness"))
      .def("set_filter", &CartesianImpedance::setFilter,
           py::call_guard<py::gil_scoped_release>(), py::arg("filter_coeff"));

  py::class_<AppliedTorque, TorqueController, std::shared_ptr<AppliedTorque>>(
      m, "AppliedTorque")
      .def(py::init<const Vector7d &,
                    const double>(), /*py::keep_alive<1, 0>(),*/
           py::arg("damping") = AppliedTorque::kDefaultDamping,
           py::arg("filter_coeff") = AppliedTorque::kDefaultFilterCoeff)
      .def("set_control", &AppliedTorque::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("torque"))
      .def("set_damping", &AppliedTorque::setDamping,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"))
      .def("set_filter", &AppliedTorque::setFilter,
           py::call_guard<py::gil_scoped_release>(), py::arg("filter_coeff"));

  py::class_<AppliedForce, TorqueController, std::shared_ptr<AppliedForce>>(
      m, "AppliedForce")
      .def(py::init<const Vector7d &,
                    const double>(), /*py::keep_alive<1, 0>(),*/
           py::arg("damping") = AppliedForce::kDefaultDamping,
           py::arg("filter_coeff") = AppliedForce::kDefaultFilterCoeff)
      .def("set_control", &AppliedForce::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("force"))
      .def("set_damping", &AppliedForce::setDamping,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"))
      .def("set_filter", &AppliedForce::setFilter,
           py::call_guard<py::gil_scoped_release>(), py::arg("filter_coeff"));

  py::class_<Force, TorqueController, std::shared_ptr<Force>>(m, "Force")
      .def(py::init<const double &, const double &, const Vector7d &,
                    const double &,
                    const double &>(), /*py::keep_alive<1, 0>(),*/
           py::arg("k_p") = Force::kDefaultProportionalGain,
           py::arg("k_i") = Force::kDefaultIntegralGain,
           py::arg("damping") = Force::kDefaultDamping,
           py::arg("threshold") = Force::kDefaultThreshold,
           py::arg("filter_coeff") = Force::kDefaultFilterCoeff)
      .def("set_control", &Force::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("force"))
      .def("set_proportional_gain", &Force::setProportionalGain,
           py::call_guard<py::gil_scoped_release>(), py::arg("k_p"))
      .def("set_integral_gain", &Force::setIntegralGain,
           py::call_guard<py::gil_scoped_release>(), py::arg("k_i"))
      .def("set_filter", &Force::setFilter,
           py::call_guard<py::gil_scoped_release>(), py::arg("filter_coeff"))
      .def_property_readonly("name", &Force::name);
}
