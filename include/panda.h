#pragma once
#include <franka/exception.h>
#include <franka/model.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mutex>
#include <thread>

#include "controllers/controller.h"
#include "controllers/joint_limits/virtual_wall_controller.h"
#include "controllers/trajectory.h"
#include "controllers/applied_torque.h"
#include "utils.h"

namespace py = pybind11;

class Panda;

class PandaContext {
 public:
  PandaContext(Panda &panda, const double &frequency, const double &t_max = 0,
               const uint64_t &max_ticks = 0);
  const PandaContext &enter();
  bool exit(const py::object &type, const py::object &value,
            const py::object &traceback);
  bool ok();
  uint64_t getNumTicks();
  double getTime();

 private:
  double dt_, t_max_;
  uint64_t num_ticks_, max_ticks_;
  std::chrono::high_resolution_clock::time_point t_start_, t_prev_;
  Panda &panda_;
};

class Panda {
 public:
  static const double kMoveToJointPositionThreshold;
  Panda(
      std::string hostname, std::string name = "panda",
      franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kIgnore);
  ~Panda();
  const PandaContext createContext(double frequency, double max_runtime = 0.0,
                                   uint64_t max_iter = 0);
  franka::Robot &getRobot();
  franka::Model &getModel();
  franka::RobotState getState();
  void startController(std::shared_ptr<TorqueController> controller);
  void stopController();
  void enableLogging(size_t buffer_size);
  void disableLogging();
  std::map<std::string, std::list<Eigen::VectorXd>> getLog();
  bool moveToJointPosition(
      const Vector7d &position,
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToJointPosition(
      std::vector<Vector7d> &waypoints,
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToPose(
      std::vector<Eigen::Vector3d> &positions,
      std::vector<Eigen::Matrix<double, 4, 1>> &orientations,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToPose(
      const Eigen::Vector3d &position,
      const Eigen::Matrix<double, 4, 1> &orientation,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToPose(
      const std::vector<Eigen::Matrix<double, 4, 4>> &poses,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToPose(
      const Eigen::Matrix<double, 4, 4> &pose,
      double speed_factor = motion::kDefaultCartesianSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  bool moveToStart(
      double speed_factor = motion::kDefaultJointSpeedFactor,
      const Vector7d &stiffness = controllers::Trajectory::kDefaultStiffness,
      const Vector7d &damping = controllers::Trajectory::kDefaultDamping,
      double dq_threshold = controllers::Trajectory::kDefaultDqThreshold,
      double success_threshold = kMoveToJointPositionThreshold);
  Eigen::Vector3d getPosition();
  Eigen::Vector4d getOrientation(bool scalar_first = false);
  Eigen::Vector4d getOrientationScalarLast();
  Eigen::Vector4d getOrientationScalarFirst();
  Vector7d getJointPositions();
  Eigen::Matrix<double, 4, 4> getPose();
  void setDefaultBehavior();
  void raiseError();
  void recover();
  void teaching_mode(bool active);

  const std::string name_;

 private:
  void _startController(std::shared_ptr<TorqueController> controller);
  void _runController(TorqueCallback &control);
  void _setState(const franka::RobotState &state);
  template <typename... Args>
  void _log(const std::string level, Args &&...args);
  TorqueCallback _createTorqueCallback();
  std::shared_ptr<franka::Robot> robot_;
  std::shared_ptr<franka::Model> model_;
  franka::RobotState state_;
  std::mutex mux_;
  std::shared_ptr<TorqueController> current_controller_;
  std::thread current_thread_;
  std::shared_ptr<controllers::joint_limits::VirtualWallController>
      virtual_walls_;
  py::object logger_;
  std::string hostname_;
  std::shared_ptr<franka::Exception> last_error_;
  std::deque<franka::RobotState> log_;
  bool log_enabled_ = false;
  size_t log_size_;
};