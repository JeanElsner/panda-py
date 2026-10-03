#include <franka/exception.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>

#include "controllers/applied_force.h"
#include "controllers/applied_torque.h"
#include "controllers/task_impedance.h"
#include "controllers/force.h"
#include "controllers/integrated_velocity.h"
#include "controllers/joint_position.h"
#include "controllers/guard.h"
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


task_impedance::TankMode parseTankMode(const std::string &name) {
  if (name == "power") return task_impedance::TankMode::kPower;
  if (name == "impulse") return task_impedance::TankMode::kImpulse;
  throw std::invalid_argument("mode must be 'power' or 'impulse', not '" + name + "'.");
}

#define UNPACK_FIELD(name, size)                                           \
  {                                                                        \
    py::array_t<double> a(size == 1 ? std::vector<py::ssize_t>{n}          \
                                    : std::vector<py::ssize_t>{n, size});  \
    double *data = a.mutable_data();                                       \
    for (py::ssize_t i = 0; i < n; i++) {                                  \
      std::copy(samples[i].name, samples[i].name + size, data + i * size); \
    }                                                                      \
    d[#name] = a;                                                          \
  }

py::dict unpackTaskImpedance(const std::vector<task_impedance::Sample> &samples) {
  const py::ssize_t n = static_cast<py::ssize_t>(samples.size());
  py::dict d;
  TASK_IMPEDANCE_SAMPLE_FIELDS(UNPACK_FIELD)
  return d;
}

py::dict unpackJointPosition(const std::vector<joint_position::Sample> &samples) {
  const py::ssize_t n = static_cast<py::ssize_t>(samples.size());
  py::dict d;
  JOINT_POSITION_SAMPLE_FIELDS(UNPACK_FIELD)
  return d;
}
#undef UNPACK_FIELD

/// read_telemetry and the telemetry properties of a controller with a ring.
template <typename Controller, typename Sample, typename Class>
void bindTelemetry(Class &cls, py::dict (*unpack)(const std::vector<Sample> &),
                   const char *doc) {
  cls.def("read_telemetry",
          [unpack](Controller &c) {
            std::vector<Sample> samples;
            {
              py::gil_scoped_release release;
              c.readTelemetry(samples);
            }
            return unpack(samples);
          },
          doc)
      .def_property_readonly("telemetry_dropped", &Controller::telemetryDropped,
                             "Samples lost because the telemetry buffer was full.")
      .def_property_readonly("telemetry_capacity", &Controller::telemetryCapacity);
}

/// The guard methods of a controller with a guard::Monitor.
template <typename Class>
void bindGuard(Class &cls, const char *rearm_doc) {
  using Controller = typename Class::type;
  using Boxes = std::vector<std::pair<Eigen::Matrix4d, Eigen::Vector3d>>;
  cls.def("set_guard",
          [](Controller &c, double force, double force_time, double saturation_time,
             double speed, std::optional<Vector7d> joint_velocity,
             const Boxes &workspace, const std::string &workspace_point,
             std::optional<Eigen::Vector3d> force_bias) {
            guard::Config config;
            config.force = force;
            config.force_time = force_time;
            if (force_bias) {
              config.force_bias = *force_bias;
            }
            config.saturation_time = saturation_time;
            config.speed = speed;
            if (joint_velocity) {
              config.joint_velocity = *joint_velocity;
            }
            if (workspace.size() > guard::Config::kMaxBoxes) {
              throw std::invalid_argument("At most 8 workspace boxes.");
            }
            for (size_t i = 0; i < workspace.size(); i++) {
              config.workspace[i].pose = workspace[i].first;
              config.workspace[i].half_extents = workspace[i].second;
            }
            config.workspace_size = workspace.size();
            if (workspace_point != "end_effector" && workspace_point != "control") {
              throw std::invalid_argument(
                  "workspace_point must be 'end_effector' or 'control'.");
            }
            config.workspace_end_effector = workspace_point == "end_effector";
            py::gil_scoped_release release;
            c.setGuard(config);
          },
          py::arg("force") = std::numeric_limits<double>::infinity(),
          py::arg("force_time") = 0.05,
          py::arg("saturation_time") = std::numeric_limits<double>::infinity(),
          py::arg("speed") = std::numeric_limits<double>::infinity(),
          py::arg("joint_velocity") = py::none(), py::arg("workspace") = Boxes(),
          py::arg("workspace_point") = "end_effector",
          py::arg("force_bias") = py::none(),
          R"delim(
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
                force_bias: Subtracted from the force estimate before the
                  force guard compares it, N: its bias, tared in free space.
          )delim")
      .def("get_guard",
           [](Controller &c) {
             guard::Config g;
             {
               py::gil_scoped_release release;
               g = c.getGuard();
             }
             py::dict d;
             d["force"] = g.force;
             d["force_time"] = g.force_time;
             d["force_bias"] = g.force_bias;
             d["saturation_time"] = g.saturation_time;
             d["speed"] = g.speed;
             d["joint_velocity"] = g.joint_velocity;
             py::list boxes;
             for (size_t i = 0; i < g.workspace_size; i++) {
               boxes.append(
                   py::make_tuple(g.workspace[i].pose, g.workspace[i].half_extents));
             }
             d["workspace"] = boxes;
             d["workspace_point"] = g.workspace_end_effector ? "end_effector" : "control";
             return d;
           })
      .def_property_readonly(
          "guard_state",
          [](Controller &c) {
            guard::State g;
            {
              py::gil_scoped_release release;
              g = c.getGuardState();
            }
            py::dict d;
            d["tripped"] = g.tripped();
            d["reason"] = std::string(guard::tripName(g.trip));
            d["time"] = g.time;
            d["value"] = g.value;
            d["joint"] = g.joint;
            return d;
          },
          R"delim(
              ``tripped``, ``reason`` (``"force"``, ``"saturation"``,
              ``"speed"``, ``"joint_velocity"``, ``"workspace"``, ``"manual"``
              or ``"none"``), the robot ``time`` of the trip, the ``value`` that
              tripped it (N, s, m/s, rad/s, or m outside the workspace) and the
              ``joint``, where one is at fault. Telemetry's ``guard`` column
              holds the reason as a number, 0 while armed, in this order.
          )delim")
      .def("trip", &Controller::trip, py::call_guard<py::gil_scoped_release>(),
           "Trips the guard from outside the loop, on its next tick.")
      .def("rearm", &Controller::rearm, py::call_guard<py::gil_scoped_release>(),
           rearm_doc);
}
}  // namespace

PYBIND11_MODULE(_core, m) {
  // clang-format off
  py::module::import("panda_py.libfranka");
  py::options options;
  //  options.disable_function_signatures();
  //  options.disable_enum_members_docstring();

  m.attr("_JOINT_POSITION_START") = kJointPositionStart;
  m.attr("_TAU_J_MAX") = kTauJMax;
  m.attr("_Q_MAX_VELOCITY") = kQMaxVelocity;
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
      .def("set_joint_walls", &Panda::setJointWalls, py::arg("enabled"),
           R"delim(
          Switches the virtual joint walls, the torques that push a joint back
          as it nears its limit (the last 0.24 rad for joint 1, 0.18 rad for
          joints 2 to 4, 0.07 rad for the wrist), on top of whatever the
          controller commands. They are on by default. Off, nothing is added,
          and a joint that reaches its limit trips the firmware's
          ``joint_position_limits_violation`` reflex instead. Takes effect on
          the next tick, also while a controller runs.
      )delim")
      .def("set_control_options", &Panda::setControlOptions,
           py::arg("torque_rate_limit") = true, py::arg("limit_rate") = false,
           py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency,
           R"delim(
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
      )delim")
      .def("get_control_options", &Panda::getControlOptions, R"delim(
          The options of :py:func:`set_control_options` in effect, as a dict.
      )delim")
      .def("get_joint_walls", &Panda::getJointWalls, R"delim(
          Whether the virtual joint walls are on (cf.
          :py:func:`set_joint_walls`).
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

  py::class_<JointPosition, TorqueController, std::shared_ptr<JointPosition>>
      joint_position_class(m, "JointPosition");
  joint_position_class
      .def(py::init<const Vector7d &, const Vector7d &, size_t>(),
           py::arg("stiffness") = JointPosition::kDefaultStiffness,
           py::arg("damping") = JointPosition::kDefaultDamping,
           py::arg("telemetry") = 0,
           R"delim(
               Joint position servo,
               :math:`\tau = K (q_d - q) + D (\dot q_d - \dot q)`. Targets are
               applied by the loop on its next tick, which never waits for
               them. On start it holds the current joint positions.

               Args:
                 stiffness: :math:`K`, Nm/rad per joint.
                 damping: :math:`D`, Nm s/rad per joint.
                 telemetry: Capacity of the telemetry buffer in samples; 0
                   records none.
           )delim")
      .def("set_control", &JointPosition::setControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("velocity") = JointPosition::kDefaultDqd)
      .def("step_control", &JointPosition::stepControl,
           py::call_guard<py::gil_scoped_release>(), py::arg("delta"),
           R"delim(
               :math:`q_d = q + \delta`, :math:`\dot q_d = 0`, with :math:`q` of
               the tick it is applied at: one policy step of the simulator's
               joint servo.
           )delim")
      .def("set_stiffness", &JointPosition::setStiffness,
           py::call_guard<py::gil_scoped_release>(), py::arg("stiffness"))
      .def("set_damping", &JointPosition::setDamping,
           py::call_guard<py::gil_scoped_release>(), py::arg("damping"))
      .def("get_stiffness", &JointPosition::getStiffness,
           py::call_guard<py::gil_scoped_release>())
      .def("get_damping", &JointPosition::getDamping,
           py::call_guard<py::gil_scoped_release>())
      .def("get_snapshot",
           [](JointPosition &c) {
             joint_position::Snapshot snap;
             {
               py::gil_scoped_release release;
               snap = c.getSnapshot();
             }
             py::dict d;
             d["time"] = snap.time;
             d["q"] = snap.q;
             d["applied_time"] = snap.applied_time;
             d["applied_q"] = snap.applied_q;
             d["q_d"] = snap.q_d;
             d["applied"] = snap.applied;
             return d;
           },
           R"delim(
               What the loop last did: ``time`` and ``q`` of the latest tick,
               ``applied_time`` and ``applied_q`` of the tick the latest target
               was applied at, the target ``q_d`` and ``applied``, the number of
               targets applied since start.
           )delim");
  bindGuard(joint_position_class, R"delim(
               Clears a trip on the loop's next tick; the target becomes the
               joint positions of that tick.
           )delim");
  bindTelemetry<JointPosition>(joint_position_class, unpackJointPosition, R"delim(
               The telemetry recorded since the last call, one row per 1 kHz
               tick: ``tick``, ``time``, ``duration``, ``reference_update``,
               ``control_command_success_rate``, ``q_d``, ``dq_d``,
               ``stiffness``, ``damping``, ``tau_active``
               (:math:`K (q_d - q) + D \dot q_d`, zero while a guard is
               tripped), ``tau_passive`` (:math:`-D \dot q`), ``tau_law``,
               ``tau_cmd`` (sent), the robot state's ``q``, ``dq``, ``tau_J``,
               ``tau_J_d``, ``tau_ext_hat_filtered``, ``O_T_EE``, ``F_T_EE``,
               ``O_F_ext_hat_K``, ``K_F_ext_hat_K``, and ``guard``. See
               :py:func:`TaskImpedance.read_telemetry`.
           )delim");

  py::class_<TaskImpedance, TorqueController, std::shared_ptr<TaskImpedance>>
      task_impedance_class(m, "TaskImpedance");
  task_impedance_class
      .def(py::init([](const Vector6d &stiffness, double damping_ratio,
                       const std::string &nullspace, double nullspace_stiffness,
                       const std::string &frame,
                       const Eigen::Matrix4d &frame_transform, bool coriolis,
                       double nullspace_damping, size_t telemetry) {
             return std::make_shared<TaskImpedance>(
                 stiffness, damping_ratio, parseNullspace(nullspace),
                 nullspace_stiffness, parseFrame(frame), frame_transform,
                 coriolis, nullspace_damping, telemetry);
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
           py::arg("telemetry") = 0,
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
                 telemetry: Capacity of the telemetry buffer in samples, one per
                   1 kHz tick; 0 records none. Drain it with
                   :py:func:`read_telemetry` faster than it fills.

               Reference commands, :py:func:`set_reference` and
               :py:func:`step_reference`, are applied by the control loop on its
               next tick, leashed against the pose of that tick; the loop never
               waits for them.
           )delim")
      .def("set_reference", &TaskImpedance::setReference,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("orientation"),
           R"delim(
               Position and scalar-last quaternion of the control frame, base
               frame. Applied, and leashed, by the loop on its next tick; replaces
               any command not yet applied.
           )delim")
      .def("step_reference",
           [](TaskImpedance &c, const Eigen::Vector3d &translation,
              const Eigen::Vector3d &rotation,
              std::optional<Vector6d> stiffness) {
             py::gil_scoped_release release;
             if (stiffness) {
               c.stepReference(translation, rotation, *stiffness);
             } else {
               c.stepReference(translation, rotation);
             }
           },
           py::arg("translation"), py::arg("rotation"),
           py::arg("stiffness") = py::none(),
           R"delim(
               Moves the reference as one policy step does: by ``translation``
               and by ``rotation``, an axis-angle vector applied on the left,
               both in the base frame. With ``stiffness``, sets it on the same
               tick. Applied, and leashed, by the loop on its next tick; steps
               not yet applied add up.
           )delim")
      .def("set_leash", &TaskImpedance::setLeash,
           py::call_guard<py::gil_scoped_release>(), py::arg("position"),
           py::arg("rotation"),
           R"delim(
               Keeps every reference command within ``position`` (m) and
               ``rotation`` (rad) of the pose at the tick it is applied.
               ``float("inf")``, the default, disables it.
           )delim")
      .def("get_leash", &TaskImpedance::getLeash,
           py::call_guard<py::gil_scoped_release>())
      .def("get_snapshot",
           [](TaskImpedance &c) {
             task_impedance::Snapshot snap;
             {
               py::gil_scoped_release release;
               snap = c.getSnapshot();
             }
             py::dict d;
             d["time"] = snap.time;
             d["pose"] = snap.pose;
             d["applied_time"] = snap.applied_time;
             d["applied_pose"] = snap.applied_pose;
             d["position_ref"] = snap.position_ref;
             d["orientation_ref"] = Eigen::Vector4d(snap.orientation_ref.coeffs());
             d["stiffness"] = snap.stiffness;
             d["applied"] = snap.applied;
             d["tank_level"] = snap.tank.level;
             d["tank_drawn"] = snap.tank.drawn;
             d["alpha"] = snap.tank.alpha;
             return d;
           },
           R"delim(
               What the loop last did: ``time`` and ``pose`` (control frame) of
               the latest tick, ``applied_time`` and ``applied_pose`` of the
               tick the latest reference command was applied at, the reference
               and stiffness in effect, ``applied``, the number of commands
               applied since start, and the tank's ``tank_level`` (NaN without
               one), ``tank_drawn`` and gate ``alpha``. Times are the robot's,
               in seconds.
           )delim")
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
      .def("set_nullspace_armature", &TaskImpedance::setNullspaceArmature,
           py::call_guard<py::gil_scoped_release>(), py::arg("armature"),
           R"delim(
               Rotor inertia per joint, kg m^2, added to the diagonal of
               libfranka's mass matrix for the dynamic posture term only (the
               projector and N M u); the task law does not use the mass
               matrix. libfranka's model carries the links and the tool but no
               rotors, which the drives do not hide. Zero (the default) is
               libfranka's model; takes effect on the next tick. Telemetry's
               ``mass`` stays libfranka's.
           )delim")
      .def("get_nullspace_armature", &TaskImpedance::getNullspaceArmature,
           py::call_guard<py::gil_scoped_release>())
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
              double alpha, double nullspace_damping, const Vector7d &coriolis,
              const Vector7d &nullspace_armature) {
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
             in.nullspace_armature = nullspace_armature;
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
           py::arg("nullspace_armature") = Vector7d::Zero(),
           R"delim(
               The control law alone, for a given state: what the controller
               computes in one step. ``pose`` and ``jacobian`` are the control
               frame's, ``orientation_ref`` a scalar-last quaternion. Returns a
               dict of ``error``, ``velocity``, ``wrench_active`` (before
               alpha), ``wrench_passive``, ``tau_task``, ``tau_nullspace`` and
               ``tau``.
           )delim")
      .def_static("step_reference_update",
           [](Eigen::Vector3d position_ref, const Eigen::Vector4d &orientation_ref,
              const Eigen::Vector3d &translation, const Eigen::Vector3d &rotation,
              const Eigen::Vector3d &position, const Eigen::Vector4d &orientation,
              double leash_position, double leash_rotation) {
             Eigen::Quaterniond q_ref = Eigen::Quaterniond(orientation_ref).normalized();
             task_impedance::stepReference(
                 position_ref, q_ref, translation,
                 task_impedance::axisAngleToQuaternion(rotation), position,
                 Eigen::Quaterniond(orientation).normalized(), leash_position,
                 leash_rotation);
             return std::make_pair(position_ref, Eigen::Vector4d(q_ref.coeffs()));
           },
           py::arg("position_ref"), py::arg("orientation_ref"),
           py::arg("translation"), py::arg("rotation"), py::arg("position"),
           py::arg("orientation"),
           py::arg("leash_position") = std::numeric_limits<double>::infinity(),
           py::arg("leash_rotation") = std::numeric_limits<double>::infinity(),
           R"delim(
               The update :py:func:`step_reference` makes in the loop, for a
               given reference and pose: returns the new position and
               scalar-last orientation reference.
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

  bindGuard(task_impedance_class, R"delim(
               Clears a trip on the loop's next tick and resets the reference to
               the pose of that tick, so the active wrench resumes from zero. A
               reference set before that tick replaces the reset.
           )delim");
  bindTelemetry<TaskImpedance>(task_impedance_class, unpackTaskImpedance, R"delim(
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
           )delim");
  task_impedance_class
      .def("set_tank",
           [](TaskImpedance &c, std::optional<double> E0, const std::string &mode,
              double smooth_fraction) {
             task_impedance::TankConfig config;
             config.enabled = E0.has_value();
             config.E0 = E0.value_or(0.0);
             config.mode = parseTankMode(mode);
             config.smooth_fraction = smooth_fraction;
             py::gil_scoped_release release;
             c.setTank(config);
           },
           py::arg("E0"), py::arg("mode") = "power",
           py::arg("smooth_fraction") = 0.25,
           R"delim(
               The insertion simulator's tank, filled to ``E0`` on the loop's
               next tick. Every tick it meters the active wrench, power
               :math:`\max(0, w_{act} \cdot [v; \omega])` in ``"power"`` mode
               (``E0`` in J) or :math:`|w_{act,xyz}|` in ``"impulse"`` mode
               (``E0`` in N s), and gates it with
               :math:`\alpha = \mathrm{clamp}(E_T / (f E_0), 0, 1)`, ``f`` the
               ``smooth_fraction``; damping is never scaled. A fraction of 0
               selects the hard gate, which scales only a draw that would
               overdraw the tank. ``E0=None`` removes the tank.
           )delim")
      .def("get_tank",
           [](TaskImpedance &c) -> py::object {
             task_impedance::TankConfig t;
             {
               py::gil_scoped_release release;
               t = c.getTank();
             }
             if (!t.enabled) {
               return py::none();
             }
             py::dict d;
             d["E0"] = t.E0;
             d["mode"] = t.mode == task_impedance::TankMode::kImpulse ? "impulse" : "power";
             d["smooth_fraction"] = t.smooth_fraction;
             return std::move(d);
           })
      .def("reset_tank", &TaskImpedance::resetTank,
           py::call_guard<py::gil_scoped_release>(),
           "Refills the tank to E0 on the loop's next tick, as at a trial's start.")
      .def_static("tank_step",
           [](double level, double drawn, const Vector6d &wrench_active,
              const Vector6d &velocity, double dt, double E0, const std::string &mode,
              double smooth_fraction) {
             task_impedance::TankConfig config;
             config.enabled = true;
             config.E0 = E0;
             config.mode = parseTankMode(mode);
             config.smooth_fraction = smooth_fraction;
             task_impedance::TankState state;
             state.level = level;
             state.drawn = drawn;
             const double alpha = task_impedance::tankStep(
                 config, state, wrench_active, velocity, dt);
             return py::make_tuple(alpha, state.level, state.drawn);
           },
           py::arg("level"), py::arg("drawn"), py::arg("wrench_active"),
           py::arg("velocity"), py::arg("dt"), py::arg("E0"),
           py::arg("mode") = "power", py::arg("smooth_fraction") = 0.25,
           "One tick of the tank, as the loop runs it: returns (alpha, level, drawn).");

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
