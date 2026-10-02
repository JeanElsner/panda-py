#pragma once
#include <atomic>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "constants.h"
#include "controllers/applied_torque.h"
#include "controllers/cartesian_trajectory.h"
#include "controllers/controller.h"
#include "controllers/joint_limits/virtual_wall_controller.h"
#include "controllers/joint_trajectory.h"
#include "utils.h"

namespace py = pybind11;

/**
 * Whether this process can obtain the realtime scheduling that libfranka
 * requests for the control loop, along with libfranka's message if it cannot.
 *
 * Answered by asking libfranka itself, on a thread of its own, so that a
 * successful attempt does not raise the caller's priority as a side effect.
 */
std::pair<bool, std::string> realtimePriorityAvailable();

class Panda;

class PandaContext {
 public:
  PandaContext(Panda& panda, const double& frequency, const double& t_max = 0,
               const uint64_t& max_ticks = 0);
  const PandaContext& enter();
  bool exit(const py::object& type, const py::object& value,
            const py::object& traceback);
  bool ok();
  uint64_t getNumTicks();
  double getTime();

 private:
  double dt_, t_max_;
  uint64_t num_ticks_, max_ticks_;
  std::chrono::high_resolution_clock::time_point t_start_, t_prev_;
  Panda& panda_;
};

class Panda {
 public:
  static const double kMoveToJointPositionThreshold;
  // Absolute tolerances for move_to_pose, in metres and radians. The Cartesian
  // controller is an impedance controller with no integral action, so it
  // settles where its spring balances friction: a few millimetres and a few
  // degrees off at the default impedance are expected, not a failure.
  static const double kMoveToPosePositionThreshold;
  static const double kMoveToPoseOrientationThreshold;
  // Distance in metres and rotation angle in radians between a pose and a
  // goal, independent of the sign of either quaternion.
  static std::pair<double, double> poseError(
      const Eigen::Vector3d& goal_position,
      const Eigen::Matrix<double, 4, 1>& goal_orientation,
      const Eigen::Vector3d& position,
      const Eigen::Matrix<double, 4, 1>& orientation);
  static const Vector7d kDefaultTeachingDamping;
  Panda(
      std::string hostname, std::string name = "panda",
      franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kIgnore);
  ~Panda();
  const PandaContext createContext(double frequency, double max_runtime = 0.0,
                                   uint64_t max_iter = 0);
  franka::Robot& getRobot();
  franka::Model& getModel();
  franka::RobotState getState();
  void startController(std::shared_ptr<TorqueController> controller);
  void stopController();
  void enableLogging(size_t buffer_size);
  void disableLogging();
  std::map<std::string, std::list<Eigen::VectorXd>> getLog();
  bool moveToJointPosition(
      const Vector7d& position,
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d& stiffness =
          controllers::JointTrajectory::kDefaultStiffness,
      const Vector7d& damping = controllers::JointTrajectory::kDefaultDamping,
      double dq_threshold = controllers::JointTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToJointPosition(
      std::vector<Vector7d>& waypoints,
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d& stiffness =
          controllers::JointTrajectory::kDefaultStiffness,
      const Vector7d& damping = controllers::JointTrajectory::kDefaultDamping,
      double dq_threshold = controllers::JointTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToPose(
      std::vector<Eigen::Vector3d>& positions,
      std::vector<Eigen::Matrix<double, 4, 1>>& orientations,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Eigen::Matrix<double, 6, 6>& impedance =
          controllers::CartesianTrajectory::kDefaultImpedance,
      const double& damping_ratio =
          controllers::CartesianTrajectory::kDefaultDampingRatio,
      const double& nullspace_stiffness =
          controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
      double dq_threshold =
          controllers::CartesianTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToPosePositionThreshold,
      double orientation_threshold = kMoveToPoseOrientationThreshold);
  bool moveToPose(
      const Eigen::Vector3d& position,
      const Eigen::Matrix<double, 4, 1>& orientation,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Eigen::Matrix<double, 6, 6>& impedance =
          controllers::CartesianTrajectory::kDefaultImpedance,
      const double& damping_ratio =
          controllers::CartesianTrajectory::kDefaultDampingRatio,
      const double& nullspace_stiffness =
          controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
      double dq_threshold =
          controllers::CartesianTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToPosePositionThreshold,
      double orientation_threshold = kMoveToPoseOrientationThreshold);
  bool moveToPose(
      const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Eigen::Matrix<double, 6, 6>& impedance =
          controllers::CartesianTrajectory::kDefaultImpedance,
      const double& damping_ratio =
          controllers::CartesianTrajectory::kDefaultDampingRatio,
      const double& nullspace_stiffness =
          controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
      double dq_threshold =
          controllers::CartesianTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToPosePositionThreshold,
      double orientation_threshold = kMoveToPoseOrientationThreshold);
  bool moveToPose(
      const Eigen::Matrix<double, 4, 4>& pose,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Eigen::Matrix<double, 6, 6>& impedance =
          controllers::CartesianTrajectory::kDefaultImpedance,
      const double& damping_ratio =
          controllers::CartesianTrajectory::kDefaultDampingRatio,
      const double& nullspace_stiffness =
          controllers::CartesianTrajectory::kDefaultNullspaceStiffness,
      double dq_threshold =
          controllers::CartesianTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToPosePositionThreshold,
      double orientation_threshold = kMoveToPoseOrientationThreshold);
  bool moveToStart(
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d& stiffness =
          controllers::JointTrajectory::kDefaultStiffness,
      const Vector7d& damping = controllers::JointTrajectory::kDefaultDamping,
      double dq_threshold = controllers::JointTrajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool isMoving();
  void refreshState();
  Vector7d getJointLimitsLower();
  Vector7d getJointLimitsUpper();
  void setJointWalls(bool enabled);
  bool getJointWalls();
  void setControlOptions(bool torque_rate_limit, bool limit_rate,
                         double cutoff_frequency);
  py::dict getControlOptions();
  Eigen::Vector3d getPosition();
  Eigen::Vector4d getOrientation(bool scalar_first = false);
  Eigen::Vector4d getOrientationScalarLast();
  Eigen::Vector4d getOrientationScalarFirst();
  Vector7d getJointPositions();
  Eigen::Matrix<double, 4, 4> getPose();
  void setDefaultBehavior();
  void raiseError();
  void recover();
  void teaching_mode(bool active,
                     const Vector7d& damping = kDefaultTeachingDamping);

  const std::string name_;

 private:
  void _warnIfRealtimeUnavailable();
  void _startController(std::shared_ptr<TorqueController> controller);
  void _runController(TorqueCallback& control);
  void _setState(const franka::RobotState& state);
  template <typename... Args>
  void _log(const std::string level, Args&&... args);
  TorqueCallback _createTorqueCallback();
  std::shared_ptr<franka::Robot> robot_;
  std::shared_ptr<franka::Model> model_;
  franka::RobotState state_;
  std::mutex mux_;
  // Guards the log alone. The control thread only ever try-locks it, so
  // reading a long log cannot stall the 1 kHz loop; it skips logging instead.
  std::mutex log_mux_;
  std::mutex error_mux_;
  std::mutex read_mux_;
  std::shared_ptr<TorqueController> current_controller_;
  std::thread current_thread_;
  std::shared_ptr<controllers::joint_limits::VirtualWallController>
      virtual_walls_;
  JointLimits joint_limits_;
  // Read by the control loop on every tick, set from Python.
  std::atomic<bool> joint_walls_{true};
  // panda-py's own torque rate limit, read every tick; libfranka's
  // limit_rate and low-pass, read when a controller starts.
  std::atomic<bool> torque_rate_limit_{true};
  bool limit_rate_ = false;
  double cutoff_frequency_ = franka::kDefaultCutoffFrequency;
  py::object logger_;
  std::string hostname_;
  std::shared_ptr<franka::Exception> last_error_;
  std::deque<franka::RobotState> log_;
  bool log_enabled_ = false;
  size_t log_size_;
};
