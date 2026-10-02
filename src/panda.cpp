#include "panda.h"

#include <franka/control_tools.h>
#include <franka/exception.h>

#include <cmath>
#include <iostream>
#include <typeinfo>

#include "constants.h"
#include "motion/generators.h"

namespace std {
template <typename T, u_long V>
std::ostream& operator<<(std::ostream& os, const std::array<T, V>& vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // namespace std

std::pair<bool, std::string> realtimePriorityAvailable() {
  // Ask libfranka rather than reimplementing the check. It requests
  // sched_get_priority_max(SCHED_FIFO), so a machine that permits some
  // realtime priority but not the maximum is correctly reported as
  // unavailable, and this cannot drift from what libfranka does.
  //
  // On success the call raises the priority of the thread it runs on, so it
  // runs on a thread of its own: doing that to the caller, which is the
  // interpreter's main thread, would be a side effect of a query. The probe
  // thread inherits the same limits and capabilities, so the answer is the
  // one the control thread would get.
  bool available = false;
  std::string message;
  std::thread probe([&available, &message]() {
    available = franka::setCurrentThreadToHighestSchedulerPriority(&message);
  });
  probe.join();
  return {available, message};
}

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

PandaContext::PandaContext(Panda& panda, const double& frequency,
                           const double& t_max, const uint64_t& max_ticks)
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

const PandaContext& PandaContext::enter() {
  t_start_ = std::chrono::high_resolution_clock::now();
  return *this;
}

bool PandaContext::exit(const py::object& type, const py::object& value,
                        const py::object& traceback) {
  return false;
}

uint64_t PandaContext::getNumTicks() { return num_ticks_; }

template <typename... Args>
void Panda::_log(const std::string level, Args&&... args) {
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
  // The virtual walls throw if the robot is already outside their range, so
  // they have to match the robot actually connected. The FER envelope applied
  // to an FR3 rejects a wide band of perfectly legal configurations, joint 6
  // above 3.7525 rad in particular.
  joint_limits_ = jointLimitsForServerVersion(robot_->serverVersion());
  _log("info", "Using %s joint limits (robot server version %d).",
       joint_limits_.name, robot_->serverVersion());
  virtual_walls_ =
      std::shared_ptr<controllers::joint_limits::VirtualWallController>(
          new controllers::joint_limits::VirtualWallController(
              joint_limits_.upper, joint_limits_.lower, kPDZoneWidth,
              kDZoneWidth, kPDZoneStiffness, kPDZoneDamping, kDZoneDamping));
  _warnIfRealtimeUnavailable();
}

void Panda::_warnIfRealtimeUnavailable() {
  // libfranka always tries to put the control thread on SCHED_FIFO, but only
  // raises RealtimeException about it when the RealtimeConfig is kEnforce.
  // panda-py defaults to kIgnore so that gentle motions work on a stock
  // kernel, which means both of the conditions kEnforce checks fail silently.
  // Report them instead, because the symptom the robot produces,
  // communication_constraints_violation, points at the network rather than at
  // the scheduler.
  //
  // Once per process rather than per instance: the answer cannot differ
  // between two robots in the same interpreter.
  static std::once_flag warned;
  std::call_once(warned, [this]() {
    const std::pair<bool, std::string> priority = realtimePriorityAvailable();
    if (!priority.first) {
      _log("warning",
           "Realtime scheduling is unavailable, so the 1 kHz control loop runs "
           "at normal priority and can miss its deadline when the machine is "
           "busy. The robot reports that as "
           "communication_constraints_violation or a reflex abort. Cause: %s. "
           "To grant the limit, run: echo \"$USER - rtprio 99\" | sudo tee "
           "/etc/security/limits.d/99-realtime.conf, then log out and back in.",
           priority.second);
    }
    if (!franka::hasRealtimeKernel()) {
      _log("warning",
           "The running kernel is not a realtime kernel "
           "(/sys/kernel/realtime is not set). Control usually works, but "
           "latency spikes can abort motions.");
    }
  });
}

Panda::~Panda() {
  _log("info", "Panda class destructor invoked (%s).", hostname_);
  stopController();
}

const PandaContext Panda::createContext(double frequency, double max_runtime,
                                        uint64_t max_iter) {
  return PandaContext(*this, frequency, max_runtime, max_iter);
}

franka::Robot& Panda::getRobot() { return *robot_; }

franka::Model& Panda::getModel() { return *model_; }

franka::RobotState Panda::getState() {
  refreshState();
  std::lock_guard<std::mutex> lock(mux_);
  return state_;
}

void Panda::enableLogging(size_t buffer_size) {
  std::lock_guard<std::mutex> lock(log_mux_);
  log_enabled_ = true;
  log_size_ = buffer_size;
  log_.clear();
}

void Panda::disableLogging() {
  std::lock_guard<std::mutex> lock(log_mux_);
  log_enabled_ = false;
}

// std::deque<franka::RobotState> Panda::getLog() { return log_; }

std::map<std::string, std::list<Eigen::VectorXd>> Panda::getLog() {
  std::map<std::string, std::list<Eigen::VectorXd>> log;
  std::list<Eigen::VectorXd> O_T_EE, elbow, tau_J, control_command_success_rate,
      O_F_ext_hat_K, K_F_ext_hat_K, q, dq, tau_ext_hat_filtered, time;
  std::lock_guard<std::mutex> lock(log_mux_);
  for (auto l : log_) {
    O_T_EE.push_back(Eigen::Map<Eigen::VectorXd>(l.O_T_EE.data(), 16, 1));
    elbow.push_back(Eigen::Map<Eigen::VectorXd>(l.elbow.data(), 2, 1));
    tau_J.push_back(Eigen::Map<Eigen::VectorXd>(l.tau_J.data(), 7, 1));
    control_command_success_rate.push_back(
        Eigen::Matrix<double, 1, 1>::Constant(l.control_command_success_rate));
    O_F_ext_hat_K.push_back(
        Eigen::Map<Eigen::VectorXd>(l.O_F_ext_hat_K.data(), 6, 1));
    K_F_ext_hat_K.push_back(
        Eigen::Map<Eigen::VectorXd>(l.K_F_ext_hat_K.data(), 6, 1));
    q.push_back(Eigen::Map<Eigen::VectorXd>(l.q.data(), 7, 1));
    dq.push_back(Eigen::Map<Eigen::VectorXd>(l.dq.data(), 7, 1));
    tau_ext_hat_filtered.push_back(
        Eigen::Map<Eigen::VectorXd>(l.tau_ext_hat_filtered.data(), 7, 1));
    time.push_back(Eigen::Matrix<double, 1, 1>::Constant(l.time.toMSec()));
  }
  log.emplace("O_T_EE", O_T_EE);
  log.emplace("elbow", elbow);
  log.emplace("tau_J", tau_J);
  log.emplace("control_command_success_rate", control_command_success_rate);
  log.emplace("O_F_ext_hat_K", O_F_ext_hat_K);
  log.emplace("K_F_ext_hat_K", K_F_ext_hat_K);
  log.emplace("q", q);
  log.emplace("dq", dq);
  log.emplace("tau_ext_hat_filtered", tau_ext_hat_filtered);
  log.emplace("time", time);

  return log;
}

Vector7d Panda::getJointLimitsLower() { return joint_limits_.lower; }

Vector7d Panda::getJointLimitsUpper() { return joint_limits_.upper; }

void Panda::setJointWalls(bool enabled) {
  joint_walls_ = enabled;
  _log("info", "Joint walls %s.", enabled ? "on" : "off");
}

bool Panda::getJointWalls() { return joint_walls_; }

void Panda::setControlOptions(bool torque_rate_limit, bool limit_rate,
                              double cutoff_frequency) {
  if (!(cutoff_frequency > 0 && cutoff_frequency <= franka::kMaxCutoffFrequency)) {
    throw std::invalid_argument("cutoff_frequency must be in (0, 1000] Hz.");
  }
  torque_rate_limit_ = torque_rate_limit;
  limit_rate_ = limit_rate;
  cutoff_frequency_ = cutoff_frequency;
  _log("info",
       "Control options: torque rate limit %s, libfranka limit_rate %s, "
       "cutoff %.0f Hz.",
       torque_rate_limit ? "on" : "off", limit_rate ? "on" : "off",
       cutoff_frequency);
}

py::dict Panda::getControlOptions() {
  py::dict options;
  options["torque_rate_limit"] = bool(torque_rate_limit_);
  options["limit_rate"] = limit_rate_;
  options["cutoff_frequency"] = cutoff_frequency_;
  return options;
}

bool Panda::isMoving() {
  return current_controller_ && current_controller_->isRunning();
}

void Panda::refreshState() {
  // While a controller runs, the control loop feeds state_ at 1 kHz and
  // readOnce() must not be called concurrently with control. Outside of that
  // the cached state is only as recent as the last motion, so read the robot
  // once.
  if (isMoving()) {
    return;
  }
  std::lock_guard<std::mutex> lock(read_mux_);
  // The state getters reach this either with the GIL held, when called from
  // Python, or without it, when called internally from move_to_* which releases
  // it for the duration of the motion. Releasing an unheld GIL aborts, so only
  // give it up if this thread actually holds it.
  if (PyGILState_Check()) {
    py::gil_scoped_release release;
    _setState(robot_->readOnce());
  } else {
    _setState(robot_->readOnce());
  }
}

Eigen::Vector3d Panda::getPosition() {
  refreshState();
  std::lock_guard<std::mutex> lock(mux_);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(state_.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  return position;
}

Eigen::Vector4d Panda::getOrientation(bool scalar_first) {
  refreshState();
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
  refreshState();
  std::lock_guard<std::mutex> lock(mux_);
  return Eigen::Map<Vector7d>(state_.q.data());
}

Eigen::Matrix4d Panda::getPose() {
  refreshState();
  std::lock_guard<std::mutex> lock(mux_);
  return Eigen::Matrix4d::Map(state_.O_T_EE.data());
}

void Panda::_setState(const franka::RobotState& state) {
  {
    std::lock_guard<std::mutex> lock(mux_);
    state_ = state;
  }
  // Called from the 1 kHz control loop, which must never wait for get_log()
  // copying a long log. While a read holds the log, this sample is not logged.
  std::unique_lock<std::mutex> log_lock(log_mux_, std::try_to_lock);
  if (log_lock.owns_lock() && log_enabled_) {
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
  return TorqueCallback([&](const franka::RobotState& robot_state,
                            franka::Duration duration) -> franka::Torques {
    _setState(robot_state);
    franka::Torques tau = franka::Torques({0, 0, 0, 0, 0, 0, 0});
    if (current_controller_) {
      current_controller_->setTime(current_controller_->getTime() +
                                   duration.toSec());
      tau = current_controller_->step(robot_state, duration);
    }
    // Virtual joint walls
    Array7d tau_virtual_wall, tau_saturated, tau_clipped;
    if (joint_walls_) {
      virtual_walls_->computeTorque(robot_state.q, robot_state.dq,
                                    tau_virtual_wall);
      for (int i = 0; i < 7; i++) {
        tau.tau_J[i] += tau_virtual_wall[i];
      }
    }
    tau_saturated = torque_rate_limit_
                        ? saturateTorqueRate(tau.tau_J, robot_state.tau_J_d)
                        : tau.tau_J;
    tau_clipped = clipTorques(tau_saturated);
    tau.tau_J = tau_clipped;
    if (current_controller_) {
      current_controller_->commanded(robot_state, tau);
    }
    return tau;
  });
}

void Panda::stopController() {
  if (current_controller_ /*&& current_controller_->isRunning()*/) {
    _log("info", "Stopping active controller (%s).",
         current_controller_->name());
    // getState() copies under the mutex; state_ is written by the control
    // thread at 1 kHz and must not be read directly from here.
    current_controller_->stop(getState(), model_);
  }
  if (current_thread_.joinable()) {
    // The control thread takes the GIL to log, for instance when its loop ends
    // with an error, so it cannot finish while this thread holds the GIL. This
    // is reached with the GIL held from Python and from the destructor.
    if (PyGILState_Check()) {
      py::gil_scoped_release release;
      current_thread_.join();
    } else {
      current_thread_.join();
    }
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

void Panda::_runController(TorqueCallback& control_callback) {
  try {
    robot_->control(control_callback, limit_rate_, cutoff_frequency_);
  } catch (const franka::Exception& e) {
    _log("error", "Control loop interruped: %s", e.what());
    std::lock_guard<std::mutex> lock(error_mux_);
    last_error_ = std::make_shared<franka::Exception>(e);
  }
}

void Panda::raiseError() {
  // Written by the control thread, read here from the caller's thread.
  std::shared_ptr<franka::Exception> error;
  {
    std::lock_guard<std::mutex> lock(error_mux_);
    error = std::move(last_error_);
  }
  if (error) {
    throw *error;
  }
}

const double Panda::kMoveToJointPositionThreshold = 1e-2;
const double Panda::kMoveToPosePositionThreshold = 0.02;
const double Panda::kMoveToPoseOrientationThreshold = 0.1;

std::pair<double, double> Panda::poseError(
    const Eigen::Vector3d& goal_position,
    const Eigen::Matrix<double, 4, 1>& goal_orientation,
    const Eigen::Vector3d& position,
    const Eigen::Matrix<double, 4, 1>& orientation) {
  // Quaternions are given scalar last, which is Eigen's coefficient order.
  // angularDistance is the rotation angle between the two, so q and -q, the
  // same orientation, compare equal.
  const Eigen::Quaterniond goal(goal_orientation.normalized());
  const Eigen::Quaterniond actual(orientation.normalized());
  return {(goal_position - position).norm(), goal.angularDistance(actual)};
}

bool Panda::moveToJointPosition(const Vector7d& position, double speed_factor,
                                const Vector7d& stiffness,
                                const Vector7d& damping, double dq_threshold,
                                double success_threshold) {
  std::vector<Vector7d> waypoints;
  waypoints.push_back(position);
  return moveToJointPosition(waypoints, speed_factor, stiffness, damping,
                             dq_threshold, success_threshold);
}

const double kDefaultTeachingDampingData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d Panda::kDefaultTeachingDamping =
    Vector7d(kDefaultTeachingDampingData);

void Panda::teaching_mode(bool active, const Vector7d& damping) {
  stopController();
  recover();
  if (!active) {
    return;
  }
  auto ctrl = std::make_shared<AppliedTorque>(damping, 1.0);
  startController(ctrl);
}

bool Panda::moveToJointPosition(std::vector<Vector7d>& waypoints,
                                double speed_factor, const Vector7d& stiffness,
                                const Vector7d& damping, double dq_threshold,
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
  auto ctrl = std::make_shared<controllers::JointTrajectory>(
      traj, stiffness, damping, dq_threshold);
  _startController(ctrl);
  auto cb = _createTorqueCallback();
  _runController(cb);
  const Vector7d q = Eigen::Map<const Vector7d>(robot_->readOnce().q.data());
  const bool success = waypoints.back().isApprox(q, success_threshold);
  if (!success) {
    _log("warning",
         "Motion finished %.4f rad from the goal, above the success threshold. "
         "Consider a higher stiffness or a slower speed_factor.",
         (waypoints.back() - q).cwiseAbs().maxCoeff());
  }
  return success;
}

bool Panda::moveToPose(const Eigen::Vector3d& position,
                       const Eigen::Matrix<double, 4, 1>& orientation,
                       double speed_factor,
                       const Eigen::Matrix<double, 6, 6>& impedance,
                       const double& damping_ratio,
                       const double& nullspace_stiffness, double dq_threshold,
                       double success_threshold, double orientation_threshold) {
  std::vector<Eigen::Vector3d> positions;
  positions.push_back(position);
  std::vector<Eigen::Matrix<double, 4, 1>> orientations;
  orientations.push_back(orientation);
  return moveToPose(positions, orientations, speed_factor, impedance,
                    damping_ratio, nullspace_stiffness, dq_threshold,
                    success_threshold, orientation_threshold);
}

bool Panda::moveToPose(std::vector<Eigen::Vector3d>& positions,
                       std::vector<Eigen::Matrix<double, 4, 1>>& orientations,
                       double speed_factor,
                       const Eigen::Matrix<double, 6, 6>& impedance,
                       const double& damping_ratio,
                       const double& nullspace_stiffness, double dq_threshold,
                       double success_threshold, double orientation_threshold) {
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
  auto ctrl = std::make_shared<controllers::CartesianTrajectory>(
      traj, getJointPositions(), impedance, damping_ratio, nullspace_stiffness,
      dq_threshold);
  _startController(ctrl);
  auto cb = _createTorqueCallback();
  _runController(cb);
  Eigen::Affine3d transform(
      Eigen::Matrix4d::Map(robot_->readOnce().O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  const auto error = poseError(positions.back(), orientations.back(), position,
                               orientation.coeffs());
  const bool success =
      error.first <= success_threshold && error.second <= orientation_threshold;
  if (!success) {
    _log("warning",
         "Motion finished %.4f m and %.2f deg from the goal, above the success "
         "threshold of %.4f m and %.2f deg. Consider a higher impedance or a "
         "slower speed_factor.",
         error.first, error.second * 180.0 / M_PI, success_threshold,
         orientation_threshold * 180.0 / M_PI);
  }
  return success;
}

bool Panda::moveToPose(const std::vector<Eigen::Matrix<double, 4, 4>>& poses,
                       double speed_factor,
                       const Eigen::Matrix<double, 6, 6>& impedance,
                       const double& damping_ratio,
                       const double& nullspace_stiffness, double dq_threshold,
                       double success_threshold, double orientation_threshold) {
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix<double, 4, 1>> orientations;
  for (auto p : poses) {
    positions.push_back(MatrixToPosition(p));
    orientations.push_back(MatrixToOrientation(p));
  }
  return moveToPose(positions, orientations, speed_factor, impedance,
                    damping_ratio, nullspace_stiffness, dq_threshold,
                    success_threshold, orientation_threshold);
}

bool Panda::moveToPose(const Eigen::Matrix<double, 4, 4>& pose,
                       double speed_factor,
                       const Eigen::Matrix<double, 6, 6>& impedance,
                       const double& damping_ratio,
                       const double& nullspace_stiffness, double dq_threshold,
                       double success_threshold, double orientation_threshold) {
  std::vector<Eigen::Matrix<double, 4, 4>> poses;
  poses.push_back(pose);
  return moveToPose(poses, speed_factor, impedance, damping_ratio,
                    nullspace_stiffness, dq_threshold, success_threshold,
                    orientation_threshold);
}

bool Panda::moveToStart(double speed_factor, const Vector7d& stiffness,
                        const Vector7d& damping, double dq_threshold,
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
