#include "panda.h"

#include <franka/exception.h>

#include <iostream>
#include <typeinfo>

#include "constants.h"
#include "controllers/trajectory.h"
#include "motion/generators.h"

namespace std {
template <typename T, u_long V>
std::ostream &operator<<(std::ostream &os, const std::array<T, V> &vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // namespace std

bool PandaContext::ok() {
  panda_.raiseError();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                     std::chrono::high_resolution_clock::now() - t_prev_)
                     .count() /
                 1e6;
  if (elapsed < dt_) {
    std::this_thread::sleep_for(
        std::chrono::microseconds(int((dt_ - elapsed) * 1e6)));
  }
  t_prev_ = std::chrono::high_resolution_clock::now();
  num_ticks_++;
  if (max_ticks_ > 0 && num_ticks_ - 1 >= max_ticks_) {
    return false;
  } else if (t_max_ > 0.0 && getTime() >= t_max_) {
    return false;
  } else {
    return true;
  }
}

PandaContext::PandaContext(Panda &panda, const double &frequency,
                           const double &t_max, const uint64_t &max_ticks)
    : dt_(1.0 / frequency),
      t_prev_(t_start_),
      max_ticks_(max_ticks),
      t_max_(t_max),
      num_ticks_(0),
      panda_(panda) {}

double PandaContext::getTime() {
  return std::chrono::duration_cast<std::chrono::microseconds>(t_prev_ -
                                                               t_start_)
             .count() *
         1e-6;
}

const PandaContext &PandaContext::enter() {
  t_start_ = std::chrono::high_resolution_clock::now();
  return *this;
}

bool PandaContext::exit(const py::object &type, const py::object &value,
                        const py::object &traceback) {
  return false;
}

uint64_t PandaContext::getNumTicks() { return num_ticks_; }

template <typename... Args>
void Panda::_log(const std::string level, Args &&...args) {
  py::gil_scoped_acquire acquire;
  logger_.attr(level.c_str())(args...);
}

Panda::Panda(std::string hostname, std::string name,
             franka::RealtimeConfig realtime_config)
    : name_(name) {
  py::object logging = py::module_::import("logging");
  logger_ = logging.attr("getLogger")(name);
  py::gil_scoped_release release;
  robot_ = std::shared_ptr<franka::Robot>(
      new franka::Robot(hostname, realtime_config));
  model_ = std::make_shared<franka::Model>(robot_->loadModel());
  hostname_ = hostname;
  _log("info", "Connected to robot (%s).", hostname_);
  _setState(robot_->readOnce());
  virtual_walls_ =
      std::shared_ptr<controllers::joint_limits::VirtualWallController>(
          new controllers::joint_limits::VirtualWallController(
              kUpperJointLimits, kLowerJointLimits, kPDZoneWidth, kDZoneWidth,
              kPDZoneStiffness, kPDZoneDamping, kDZoneDamping));
}

Panda::~Panda() {
  _log("info", "Panda class destructor invoked (%s).", hostname_);
  stopController();
}

const PandaContext Panda::createContext(double frequency, double max_runtime,
                                        uint64_t max_iter) {
  return PandaContext(*this, frequency, max_runtime, max_iter);
}

franka::Robot &Panda::getRobot() { return *robot_; }

franka::Model &Panda::getModel() { return *model_; }

franka::RobotState Panda::getState() {
  std::lock_guard<std::mutex> lock(mux_);
  return state_;
}

void Panda::enableLogging(size_t buffer_size) {
  std::lock_guard<std::mutex> lock(mux_);
  log_enabled_ = true;
  log_size_ = buffer_size;
  log_.clear();
}

void Panda::disableLogging() {
  std::lock_guard<std::mutex> lock(mux_);
  log_enabled_ = false;
}

// std::deque<franka::RobotState> Panda::getLog() { return log_; }

std::map<std::string, std::list<Eigen::VectorXd>> Panda::getLog() {
  std::map<std::string, std::list<Eigen::VectorXd>> log;
  std::list<Eigen::VectorXd> O_T_EE, elbow, tau_J, control_command_success_rate;
  std::lock_guard<std::mutex> lock(mux_);
  for (auto l : log_) {
    O_T_EE.push_back(Eigen::Map<Eigen::VectorXd>(l.O_T_EE.data(), 16, 1));
    elbow.push_back(Eigen::Map<Eigen::VectorXd>(l.elbow.data(), 2, 1));
    tau_J.push_back(Eigen::Map<Eigen::VectorXd>(l.tau_J.data(), 7, 1));
    control_command_success_rate.push_back(
        Eigen::Matrix<double, 1, 1>::Constant(l.control_command_success_rate));
  }
  log.emplace("O_T_EE", O_T_EE);
  log.emplace("elbow", elbow);
  log.emplace("tau_J", tau_J);
  log.emplace("control_command_success_rate", control_command_success_rate);
  return log;
}

Eigen::Vector3d Panda::getPosition() {
  std::lock_guard<std::mutex> lock(mux_);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  return position;
}

Eigen::Vector4d Panda::getOrientation(bool scalar_first) {
  if (scalar_first) {
    return getOrientationScalarFirst();
  }
  return getOrientationScalarLast();
}

Eigen::Vector4d Panda::getOrientationScalarLast() {
  std::lock_guard<std::mutex> lock(mux_);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
  Eigen::Quaterniond orientation(transform.rotation());
  orientation.normalize();
  return orientation.coeffs();
}

Eigen::Vector4d Panda::getOrientationScalarFirst() {
  Eigen::Vector4d orientation = getOrientationScalarLast();
  Eigen::Vector4d tmp;
  tmp[0] = orientation[3];
  tmp.tail(3) << orientation.head(3);
  return tmp;
}

Vector7d Panda::getJointPositions() {
  std::lock_guard<std::mutex> lock(mux_);
  return Eigen::Map<Vector7d>(state_.q.data());
}

Eigen::Matrix4d Panda::getPose() {
  std::lock_guard<std::mutex> lock(mux_);
  return Eigen::Matrix4d::Map(state_.O_T_EE.data());
}

void Panda::_setState(const franka::RobotState &state) {
  std::lock_guard<std::mutex> lock(mux_);
  state_ = state;
  mux_.unlock();
  if (log_enabled_) {
    log_.push_back(state);
    if (log_.size() > log_size_) {
      log_.pop_front();
    }
  }
}

void Panda::startController(std::shared_ptr<TorqueController> controller_ptr) {
  stopController();
  _startController(controller_ptr);
  current_thread_ = std::thread(
      std::bind(&Panda::_runController, this, _createTorqueCallback()));
}

void Panda::_startController(std::shared_ptr<TorqueController> controller_ptr) {
  recover();
  _log("info", "Starting new controller (%s).", controller_ptr->name());
  virtual_walls_->reset();
  this->current_controller_ = controller_ptr;
  current_controller_->setTime(0);
  current_controller_->start(robot_->readOnce(), model_);
}

TorqueCallback Panda::_createTorqueCallback() {
  return TorqueCallback([&](const franka::RobotState &robot_state,
                            franka::Duration duration) -> franka::Torques {
    _setState(robot_state);
    franka::Torques tau = franka::Torques({0, 0, 0, 0, 0, 0, 0});
    if (current_controller_) {
      current_controller_->setTime(current_controller_->getTime() +
                                   duration.toSec());
      tau = current_controller_->step(robot_state, duration);
    }
    // Virtual joint walls
    Array7d tau_virtual_wall;
    virtual_walls_->computeTorque(robot_state.q, robot_state.dq,
                                  tau_virtual_wall);
    for (int i = 0; i < 7; i++) {
      tau.tau_J[i] += tau_virtual_wall[i];
    }
    return tau;
  });
}

void Panda::stopController() {
  if (current_controller_ /*&& current_controller_->isRunning()*/) {
    _log("info", "Stopping active controller (%s).",
         current_controller_->name());
    current_controller_->stop(state_, model_);
  }
  if (current_thread_.joinable()) {
    current_thread_.join();
  }
}

void Panda::recover() {
  auto state = robot_->readOnce();
  if (state.current_errors || state.robot_mode == franka::RobotMode::kReflex ||
      state.robot_mode == franka::RobotMode::kOther) {
    _log("warning",
         "Irregular state detected. Attempting automatic error recovery.");
    robot_->automaticErrorRecovery();
  }
}

void Panda::_runController(TorqueCallback &control_callback) {
  try {
    robot_->control(control_callback);
  } catch (const franka::Exception &e) {
    _log("error", "Control loop interruped: %s", e.what());
    last_error_ = std::make_shared<franka::Exception>(e);
  }
}

void Panda::raiseError() {
  if (last_error_) {
    franka::Exception e = *last_error_;
    last_error_.reset();
    throw e;
  }
}

const double Panda::kMoveToJointPositionThreshold = 1e-2;

bool Panda::moveToJointPosition(const Vector7d &position, double speed_factor,
                                const Vector7d &stiffness,
                                const Vector7d &damping, double dq_threshold,
                                double success_threshold) {
  std::vector<Vector7d> waypoints;
  waypoints.push_back(position);
  return moveToJointPosition(waypoints, speed_factor, stiffness, damping,
                             dq_threshold, success_threshold);
}

const double kDefaultTeachingDampingData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d Panda::kDefaultTeachingDamping =
    Vector7d(kDefaultTeachingDampingData);

void Panda::teaching_mode(bool active, const Vector7d &damping) {
  stopController();
  recover();
  if (!active) {
    return;
  }
  auto ctrl = std::make_shared<AppliedTorque>(damping, 1.0);
  startController(ctrl);
}

bool Panda::moveToJointPosition(std::vector<Vector7d> &waypoints,
                                double speed_factor, const Vector7d &stiffness,
                                const Vector7d &damping, double dq_threshold,
                                double success_threshold) {
  stopController();
  recover();
  _setState(robot_->readOnce());
  _log("info", "Initializing motion generation (moveToJointPosition).");
  waypoints.push_back(getJointPositions());
  std::rotate(waypoints.rbegin(), waypoints.rbegin() + 1, waypoints.rend());
  auto traj =
      std::make_shared<motion::JointTrajectory>(waypoints, speed_factor, 0.02);
  if (traj->getDuration() == 0.0) {
    _log("info", "Already at goal.");
    return true;
  }
  auto ctrl = std::make_shared<controllers::Trajectory>(
      traj, stiffness, damping, dq_threshold);
  _startController(ctrl);
  auto cb = _createTorqueCallback();
  _runController(cb);
  const Vector7d q = Eigen::Map<const Vector7d>(robot_->readOnce().q.data());
  return waypoints.back().isApprox(q, success_threshold);
}

bool Panda::moveToPose(const Eigen::Vector3d &position,
                       const Eigen::Matrix<double, 4, 1> &orientation,
                       double speed_factor, const Vector7d &stiffness,
                       const Vector7d &damping, double dq_threshold,
                       double success_threshold) {
  std::vector<Eigen::Vector3d> positions;
  positions.push_back(position);
  std::vector<Eigen::Matrix<double, 4, 1>> orientations;
  orientations.push_back(orientation);
  return moveToPose(positions, orientations, speed_factor, stiffness, damping,
                    dq_threshold, success_threshold);
}

bool Panda::moveToPose(std::vector<Eigen::Vector3d> &positions,
                       std::vector<Eigen::Matrix<double, 4, 1>> &orientations,
                       double speed_factor, const Vector7d &stiffness,
                       const Vector7d &damping, double dq_threshold,
                       double success_threshold) {
  stopController();
  recover();
  _setState(robot_->readOnce());
  _log("info", "Initializing motion generation (moveToPose).");
  positions.push_back(getPosition());
  orientations.push_back(getOrientation());
  std::rotate(positions.rbegin(), positions.rbegin() + 1, positions.rend());
  std::rotate(orientations.rbegin(), orientations.rbegin() + 1,
              orientations.rend());
  auto traj = std::make_shared<motion::CartesianTrajectory>(
      positions, orientations, speed_factor);
  if (traj->getDuration() == 0.0) {
    _log("info", "Already at goal.");
    return true;
  }
  auto ctrl = std::make_shared<controllers::Trajectory>(
      traj, stiffness, damping, dq_threshold, 1.0);
  _startController(ctrl);
  auto cb = _createTorqueCallback();
  _runController(cb);
  Eigen::Affine3d transform(
      Eigen::Matrix4d::Map(robot_->readOnce().O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  return positions.back().isApprox(position, success_threshold) &&
         orientations.back().isApprox(orientation.coeffs(), success_threshold);
}

bool Panda::moveToPose(const std::vector<Eigen::Matrix<double, 4, 4>> &poses,
                       double speed_factor, const Vector7d &stiffness,
                       const Vector7d &damping, double dq_threshold,
                       double success_threshold) {
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix<double, 4, 1>> orientations;
  for (auto p : poses) {
    positions.push_back(MatrixToPosition(p));
    orientations.push_back(MatrixToOrientation(p));
  }
  return moveToPose(positions, orientations, speed_factor, stiffness, damping,
                    dq_threshold, success_threshold);
}

bool Panda::moveToPose(const Eigen::Matrix<double, 4, 4> &pose,
                       double speed_factor, const Vector7d &stiffness,
                       const Vector7d &damping, double dq_threshold,
                       double success_threshold) {
  std::vector<Eigen::Matrix<double, 4, 4>> poses;
  poses.push_back(pose);
  return moveToPose(poses, speed_factor, stiffness, damping, dq_threshold,
                    success_threshold);
}

bool Panda::moveToStart(double speed_factor, const Vector7d &stiffness,
                        const Vector7d &damping, double dq_threshold,
                        double success_threshold) {
  return moveToJointPosition(kJointPositionStart, speed_factor, stiffness,
                             damping, dq_threshold, success_threshold);
}

void Panda::setDefaultBehavior() {
  recover();
  _log("info", "Resetting impedance and collision behavior.");
  robot_->setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}
