#include <franka/exception.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controllers/applied_force.h"
#include "controllers/applied_torque.h"
#include "controllers/task_impedance.h"
#include "controllers/force.h"
#include "controllers/integrated_velocity.h"
#include "controllers/joint_position.h"
#include "kinematics/fk.h"
#include "kinematics/ik.h"
#include "motion/generators.h"
#include "panda.h"

namespace py = pybind11;

namespace {

task_impedance::Nullspace parseNullspace(const std::string &name) {
  if (name == "dynamic") return task_impedance::Nullspace::kDynamic;
  if (name == "kinematic") return task_impedance::Nullspace::kKinematic;
  if (name == "none") return task_impedance::Nullspace::kNone;
  throw std::invalid_argument(
      "nullspace must be 'dynamic', 'kinematic' or 'none', not '" + name + "'.");
}

TaskImpedance::Frame parseFrame(const std::string &name) {
  if (name == "flange") return TaskImpedance::Frame::kFlange;
  if (name == "end_effector") return TaskImpedance::Frame::kEndEffector;
  throw std::invalid_argument(
      "frame must be 'flange' or 'end_effector', not '" + name + "'.");
}

}  // namespace

PYBIND11_MODULE(_core, m) {
  // clang-format off
  py::module::import("panda_py.libfranka");
  py::options options;
  //  options.disable_function_signatures();
  //  options.disable_enum_members_docstring();

  m.attr("_JOINT_POSITION_START") = kJointPositionStart;
  m.attr("_MOVE_TO_POSE_POSITION_THRESHOLD") = Panda::kMoveToPosePositionThreshold;
  m.attr("_MOVE_TO_POSE_ORIENTATION_THRESHOLD") = Panda::kMoveToPoseOrientationThreshold;
  m.attr("_JOINT_LIMITS_LOWER") = kLowerJointLimits;
  m.attr("_JOINT_LIMITS_UPPER") = kUpperJointLimits;
  m.attr("_JOINT_LIMITS_LOWER_FR3") = kLowerJointLimitsFR3;
  m.attr("_JOINT_LIMITS_UPPER_FR3") = kUpperJointLimitsFR3;
  m.attr("_JOINT_LIMITS_LOWER_FR3_5_9") = kLowerJointLimitsFR3_5_9;
  m.attr("_JOINT_LIMITS_UPPER_FR3_5_9") = kUpperJointLimitsFR3_5_9;
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

  m.def("_pose_error", &Panda::poseError, py::arg("goal_position"),
        py::arg("goal_orientation"), py::arg("position"),
        py::arg("orientation"), R"delim(
     Distance in metres and rotation angle in radians between a pose and the
     goal of a move_to_pose, as its success check computes them. Quaternions
     are scalar last. Exposed for the tests.
  )delim");

  m.def("realtime_priority_available", &realtimePriorityAvailable, R"delim(
     Whether this process can obtain the realtime scheduling that the
     control loop needs, and the reason it cannot as reported by libfranka.

     Returns a tuple of a bool and a message, where the message is empty
     when the answer is True. :py:class:`Panda` logs a warning about this
     on connect; call it directly to check without a robot. See also
     :py:func:`panda_py.libfranka.has_realtime_kernel`, which reports the
     other half of what realtime control needs.
  )delim");

  py::class_<motion::JointTrajectory>(m, "JointTrajectory")
      // Released like the move_to_* methods release it, which construct these
      // internally. This also keeps the Python-side tests on that same path.
      .def(py::init<const std::vector<Vector7d> &, double, double, double>(),
           py::call_guard<py::gil_scoped_release>(), py::arg("waypoints"),
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
           py::call_guard<py::gil_scoped_release>(),
           py::arg("positions"), py::arg("orientations"),
           py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
           py::arg("max_deviation") = 0,
           py::arg("timeout") = motion::kDefaultTimeout)
      .def(py::init<const std::vector<Eigen::Matrix<double, 4, 4>> &, double,
                    double, double>(),
           py::call_guard<py::gil_scoped_release>(), py::arg("poses"),
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
      .def("get_joint_limits_lower", &Panda::getJointLimitsLower, R"delim(
          Lower joint position limits of the connected robot, selected from its
          server version. The FER and the FR3 have different envelopes, and the
          FR3's were widened with robot system 5.9.0.
      )delim")
      .def("get_joint_limits_upper", &Panda::getJointLimitsUpper, R"delim(
          Upper joint position limits of the connected robot (cf.
          :py:func:`get_joint_limits_lower`).
      )delim")
      .def("is_moving", &Panda::isMoving, R"delim(
          True while a controller is running, i.e. while the robot is under
          active control by this instance.
      )delim")
      .def("refresh_state", &Panda::refreshState, R"delim(
          Reads the robot state once and updates the cached copy. The state
          getters call this for you when no controller is running; while one is,
          the control loop already refreshes the state at 1KHz and this is a
          no-op.
      )delim")
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
                            const double &, double, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("positions"),
          py::arg("orientations"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToPosePositionThreshold,
          py::arg("orientation_threshold") = Panda::kMoveToPoseOrientationThreshold,
          R"delim(
               Moves the end-effector from the current pose through the provided waypoints
               in piece-wise linear segments. The waypoints are given as lists of positions
               :math:`\in \mathbb{R}^3` and orientations
               :math:`\mathbf q = (\vec{v},\ r),~~ \mathbf q \in \mathbb{H},~~ \vec{v}\in \mathbb{R}^3,~~ r \in \mathbb{R}`,
               i.e. quaternions with scalar last. The computed trajectory is time-optimal.

               Returns whether the motion finished within ``success_threshold``
               metres and ``orientation_threshold`` radians of the goal. The
               controller is an impedance controller without integral action, so it
               settles a few millimetres and degrees short of the goal wherever
               friction balances its spring; the defaults allow for that at the
               default impedance. Tighten them together with a higher impedance.
               )delim")
      .def(
          "move_to_pose",
          py::overload_cast<const Eigen::Vector3d &,
                            const Eigen::Matrix<double, 4, 1> &, double,
                            const Eigen::Matrix<double, 6, 6> &,
                            const double &,
                            const double &, double, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("position"),
          py::arg("orientation"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToPosePositionThreshold,
          py::arg("orientation_threshold") = Panda::kMoveToPoseOrientationThreshold,
          R"delim(
               Same as :py:func:`move_to_pose` above, but only one target pose given as
               position and orientation directly.
               )delim")
      .def("move_to_pose",
           py::overload_cast<const std::vector<Eigen::Matrix<double, 4, 4>> &,
                             double,
                             const Eigen::Matrix<double, 6, 6> &,
                             const double &,
                             const double &, double, double, double>(&Panda::moveToPose),
           py::call_guard<py::gil_scoped_release>(), py::arg("pose"),
           py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
           py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
           py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
           py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
           py::arg("dq_threshold") =
               controllers::JointTrajectory::kDefaultDqThreshold,
           py::arg("success_threshold") = Panda::kMoveToPosePositionThreshold,
           py::arg("orientation_threshold") = Panda::kMoveToPoseOrientationThreshold,
           R"delim(
               Same as :py:func:`move_to_pose` above, but waypoints are given as a list of
               homogeneous transforms :math:`\in \mathbb{R}^{4\times 4}`.
               )delim")
      .def(
          "move_to_pose",
          py::overload_cast<const Eigen::Matrix<double, 4, 4> &, double,
                            const Eigen::Matrix<double, 6, 6> &,
                            const double &,
                            const double &, double, double, double>(
              &Panda::moveToPose),
          py::call_guard<py::gil_scoped_release>(), py::arg("pose"),
          py::arg("speed_factor") = motion::kDefaultCartesianSpeedFactor,
          py::arg("impedance") = controllers::CartesianTrajectory::kDefaultImpedance,
          py::arg("damping_ratio") = controllers::CartesianTrajectory::kDefaultDampingRatio,
          py::arg("nullspace_stiffness") = controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
          py::arg("dq_threshold") =
              controllers::JointTrajectory::kDefaultDqThreshold,
          py::arg("success_threshold") = Panda::kMoveToPosePositionThreshold,
          py::arg("orientation_threshold") = Panda::kMoveToPoseOrientationThreshold,
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

  py::class_<TaskImpedance, TorqueController,
             std::shared_ptr<TaskImpedance>>(m, "TaskImpedance")
      .def(py::init([](const Vector6d &stiffness, double damping_ratio,
                       const std::string &nullspace, double nullspace_stiffness,
                       const std::string &frame,
                       const Eigen::Matrix4d &frame_transform, bool coriolis,
                       double nullspace_damping) {
             return std::make_shared<TaskImpedance>(
                 stiffness, damping_ratio, parseNullspace(nullspace),
                 nullspace_stiffness, parseFrame(frame), frame_transform,
                 coriolis, nullspace_damping);
           }),
           py::arg("stiffness") = TaskImpedance::kDefaultStiffness,
           py::arg("damping_ratio") = TaskImpedance::kDefaultDampingRatio,
           py::arg("nullspace") = "dynamic",
           py::arg("nullspace_stiffness") =
               TaskImpedance::kDefaultNullspaceStiffness,
           py::arg("frame") = "end_effector",
           py::arg("frame_transform") = Eigen::Matrix4d::Identity(),
           py::arg("coriolis") = false,
           py::arg("nullspace_damping") = 0.0,
           R"delim(
               Impedance in task space at a selectable control frame:

               .. math::
                 \tau = J^\top (\alpha K e - D J \dot q) + N M u, \quad
                 u = k_{ns}(q_0 - q) - 2\sqrt{k_{ns}}\,\dot q

               with :math:`e` the position error and the axis-angle orientation
               error :math:`\mathrm{axisangle}(q_{ref} q^{-1})` of the control
               frame, all in the base frame, and :math:`D = 2\zeta\sqrt{K}`.
               There is no gravity term: the robot compensates gravity itself.
               On start the controller holds the current pose of the control
               frame, and the current joint positions in the nullspace.

               Args:
                 stiffness: Diagonal stiffness, translational (N/m) then
                   rotational (Nm/rad).
                 damping_ratio: The damping is :math:`2\zeta\sqrt{K}` for this
                   ratio :math:`\zeta`.
                 nullspace: ``"dynamic"`` projects the posture term with
                   :math:`N = I - J^\top (J M^{-1} J^\top)^{-1} J M^{-1}` and
                   applies :math:`N M u`, which cannot perturb the task.
                   ``"kinematic"`` applies :math:`(I - J^\top (J J^\top)^{-1} J) u`.
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
           )delim")
      .def("set_reference", &TaskImpedance::setReference,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("orientation"),
           "Position and scalar-last quaternion of the control frame, base frame.")
      .def("set_stiffness", &TaskImpedance::setStiffness,
           py::call_guard<py::gil_scoped_release>(), py::arg("stiffness"),
           "Also sets the damping, for the current damping ratio.")
      .def("set_damping_ratio", &TaskImpedance::setDampingRatio,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping_ratio"))
      .def("set_nullspace_target", &TaskImpedance::setNullspaceTarget,
           py::call_guard<py::gil_scoped_release>(), py::arg("q"))
      .def("set_nullspace_stiffness", &TaskImpedance::setNullspaceStiffness,
           py::call_guard<py::gil_scoped_release>(),
           py::arg("nullspace_stiffness"))
      .def("get_stiffness", &TaskImpedance::getStiffness,
           py::call_guard<py::gil_scoped_release>())
      .def("get_damping", &TaskImpedance::getDamping,
           py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("frame_transform", &TaskImpedance::getFrameTransform)
      .def_property_readonly("frame", [](const TaskImpedance &c) {
             return c.getFrame() == TaskImpedance::Frame::kFlange
                        ? "flange" : "end_effector";
           })
      .def_static("compute",
           [](const Vector7d &q, const Vector7d &dq, const Eigen::Matrix4d &pose,
              const Eigen::Matrix<double, 6, 7> &jacobian,
              const Eigen::Matrix<double, 7, 7> &mass,
              const Eigen::Vector3d &position_ref,
              const Eigen::Vector4d &orientation_ref, const Vector6d &stiffness,
              const Vector6d &damping, const Vector7d &q_nullspace,
              double nullspace_stiffness, const std::string &nullspace,
              double alpha, double nullspace_damping, const Vector7d &coriolis) {
             task_impedance::Inputs in;
             in.q = q;
             in.dq = dq;
             in.pose = pose;
             in.jacobian = jacobian;
             in.mass = mass;
             in.coriolis = coriolis;
             in.position_ref = position_ref;
             in.orientation_ref = Eigen::Quaterniond(orientation_ref).normalized();
             in.stiffness = stiffness;
             in.damping = damping;
             in.alpha = alpha;
             in.q_nullspace = q_nullspace;
             in.nullspace_stiffness = nullspace_stiffness;
             in.nullspace = parseNullspace(nullspace);
             in.nullspace_damping = nullspace_damping;
             const auto out = task_impedance::compute(in);
             py::dict result;
             result["error"] = out.error;
             result["velocity"] = out.velocity;
             result["wrench_active"] = out.wrench_active;
             result["wrench_passive"] = out.wrench_passive;
             result["tau_task"] = out.tau_task;
             result["tau_nullspace"] = out.tau_nullspace;
             result["tau"] = out.tau;
             return result;
           },
           py::arg("q"), py::arg("dq"), py::arg("pose"), py::arg("jacobian"),
           py::arg("mass"), py::arg("position_ref"), py::arg("orientation_ref"),
           py::arg("stiffness"), py::arg("damping"), py::arg("q_nullspace"),
           py::arg("nullspace_stiffness"), py::arg("nullspace") = "dynamic",
           py::arg("alpha") = 1.0, py::arg("nullspace_damping") = 0.0,
           py::arg("coriolis") = Vector7d::Zero(),
           R"delim(
               The control law alone, for a given state: what the controller
               computes in one step. ``pose`` and ``jacobian`` are the control
               frame's, ``orientation_ref`` a scalar-last quaternion. Returns a
               dict of ``error``, ``velocity``, ``wrench_active`` (before
               alpha), ``wrench_passive``, ``tau_task``, ``tau_nullspace`` and
               ``tau``.
           )delim")
      .def_static("critical_damping", &task_impedance::criticalDamping,
           py::arg("stiffness"), py::arg("damping_ratio") = 1.0)
      .def_static("orientation_error",
           [](const Eigen::Vector4d &orientation_ref,
              const Eigen::Vector4d &orientation) {
             return task_impedance::orientationError(
                 Eigen::Quaterniond(orientation_ref).normalized(),
                 Eigen::Quaterniond(orientation).normalized());
           },
           py::arg("orientation_ref"), py::arg("orientation"),
           "Axis-angle vector of the rotation from orientation to orientation_ref.")
      .def_static("shift_jacobian", &task_impedance::shiftJacobian,
           py::arg("jacobian"), py::arg("offset"),
           "Moves a geometric Jacobian to a point offset from its origin, base frame.");

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
  // clang-format on
}
