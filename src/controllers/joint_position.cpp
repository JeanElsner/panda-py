#include "controllers/joint_position.h"

#include <cmath>
#include <limits>

#include "panda.h"

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d JointPosition::kDefaultStiffness =
    Vector7d(kDefaultStiffnessData);

const double kDefaultDqdData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d JointPosition::kDefaultDqd = Vector7d(kDefaultDqdData);

const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d JointPosition::kDefaultDamping = Vector7d(kDefaultDampingData);

JointPosition::JointPosition(const Vector7d& stiffness, const Vector7d& damping,
                             size_t telemetry_capacity)
    : stiffness_(stiffness),
      damping_(damping),
      loop_stiffness_(stiffness),
      loop_damping_(damping),
      q_d_(Vector7d::Zero()),
      dq_d_(Vector7d::Zero()),
      motion_finished_(false),
      telemetry_(telemetry_capacity) {}

franka::Torques JointPosition::step(const franka::RobotState& robot_state,
                                    franka::Duration& duration) {
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const double time = robot_state.time.toSec();
  const double dt = duration.toSec();

  // As in TaskImpedance: the loop only try-locks, and a command waits for
  // the next tick rather than the loop for the command.
  bool updated = false;
  std::unique_lock<std::mutex> lock(mux_, std::try_to_lock);
  if (lock.owns_lock()) {
    loop_stiffness_ = stiffness_;
    loop_damping_ = damping_;
    guard_ = guard_shared_;
    if (command_.pending()) {
      if (command_.rearm) {
        monitor_.reset();
        q_d_ = q;
        dq_d_.setZero();
      }
      if (command_.trip) {
        monitor_.trip(guard::Trip::kManual, time);
      }
      if (command_.absolute) {
        q_d_ = command_.q_d;
        dq_d_ = command_.dq_d;
      }
      if (command_.relative) {
        q_d_ = q + command_.q_d;
        dq_d_.setZero();
      }
      if (command_.absolute || command_.relative) {
        applied_++;
        snapshot_.applied_time = time;
        snapshot_.applied_q = q;
      }
      command_ = Command();
      updated = true;
    }
    guard_state_shared_ = monitor_.state();
    snapshot_.time = time;
    snapshot_.q = q;
    snapshot_.q_d = q_d_;
    snapshot_.applied = applied_;
    lock.unlock();
  }

  // The guard's speed and workspace are those of the flange.
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  if (std::isfinite(guard_.speed) ||
      (guard_.workspace_size > 0 && !guard_.workspace_end_effector)) {
    const Eigen::Affine3d O_T_F(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()) *
        Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.F_T_EE.data()))
            .inverse()
            .matrix());
    position = O_T_F.translation();
    const auto jacobian = model_->zeroJacobian(franka::Frame::kFlange, robot_state);
    velocity = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data())
                   .topRows<3>() *
               dq;
  }
  monitor_.evaluate(guard_, robot_state, position, velocity, dt);

  Vector7d tau_active = loop_stiffness_.cwiseProduct(q_d_ - q) +
                        loop_damping_.cwiseProduct(dq_d_);
  const Vector7d tau_passive = -loop_damping_.cwiseProduct(dq);
  if (monitor_.state().tripped()) {
    tau_active.setZero();
  }
  const Vector7d tau = tau_active + tau_passive;

  sample_ = telemetry_.claim();
  if (sample_) {
    auto& s = *sample_;
    auto put = [](double* to, const auto& from) {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(to, from.size()) =
          Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
              from.data(), from.size());
    };
    s.tick[0] = static_cast<double>(tick_);
    s.time[0] = time;
    s.duration[0] = dt;
    s.reference_update[0] = updated ? 1.0 : 0.0;
    s.control_command_success_rate[0] = robot_state.control_command_success_rate;
    put(s.q_d, q_d_);
    put(s.dq_d, dq_d_);
    put(s.stiffness, loop_stiffness_);
    put(s.damping, loop_damping_);
    put(s.tau_active, tau_active);
    put(s.tau_passive, tau_passive);
    put(s.tau_law, tau);
    put(s.q, robot_state.q);
    put(s.dq, robot_state.dq);
    put(s.tau_J, robot_state.tau_J);
    put(s.tau_J_d, robot_state.tau_J_d);
    put(s.tau_ext_hat_filtered, robot_state.tau_ext_hat_filtered);
    put(s.O_T_EE, robot_state.O_T_EE);
    put(s.F_T_EE, robot_state.F_T_EE);
    put(s.O_F_ext_hat_K, robot_state.O_F_ext_hat_K);
    put(s.K_F_ext_hat_K, robot_state.K_F_ext_hat_K);
    s.guard[0] = static_cast<double>(monitor_.state().trip);
    std::fill(std::begin(s.tau_cmd), std::end(s.tau_cmd),
              std::numeric_limits<double>::quiet_NaN());
  }
  tick_++;

  franka::Torques torques = VectorToArray<7>(tau);
  torques.motion_finished = motion_finished_;
  return torques;
}

void JointPosition::commanded(const franka::RobotState& robot_state,
                              const franka::Torques& torques) {
  monitor_.commanded(torques);
  if (sample_) {
    std::copy(torques.tau_J.begin(), torques.tau_J.end(), sample_->tau_cmd);
    telemetry_.publish();
    sample_ = nullptr;
  }
}

void JointPosition::start(const franka::RobotState& robot_state,
                          std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  model_ = model;
  std::lock_guard<std::mutex> lock(mux_);
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
  loop_stiffness_ = stiffness_;
  loop_damping_ = damping_;
  command_ = Command();
  guard_ = guard_shared_;
  monitor_.reset();
  guard_state_shared_ = guard::State();
  tick_ = applied_ = 0;
  sample_ = nullptr;
  snapshot_ = joint_position::Snapshot();
  snapshot_.time = snapshot_.applied_time = robot_state.time.toSec();
  snapshot_.q = snapshot_.applied_q = snapshot_.q_d = q_d_;
}

void JointPosition::stop(const franka::RobotState& robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool JointPosition::isRunning() { return !motion_finished_; }

const std::string JointPosition::name() { return "Joint Position"; }

void JointPosition::setControl(const Vector7d& position,
                               const Vector7d& velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  command_.absolute = true;
  command_.relative = false;
  command_.q_d = position;
  command_.dq_d = velocity;
}

void JointPosition::stepControl(const Vector7d& delta) {
  std::lock_guard<std::mutex> lock(mux_);
  command_.absolute = false;
  command_.relative = true;
  command_.q_d = delta;
  command_.dq_d.setZero();
}

void JointPosition::setStiffness(const Vector7d& stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  stiffness_ = stiffness;
}

void JointPosition::setDamping(const Vector7d& damping) {
  std::lock_guard<std::mutex> lock(mux_);
  damping_ = damping;
}

Vector7d JointPosition::getStiffness() {
  std::lock_guard<std::mutex> lock(mux_);
  return stiffness_;
}

Vector7d JointPosition::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return damping_;
}

joint_position::Snapshot JointPosition::getSnapshot() {
  std::lock_guard<std::mutex> lock(mux_);
  return snapshot_;
}

void JointPosition::setGuard(const guard::Config& config) {
  std::lock_guard<std::mutex> lock(mux_);
  guard_shared_ = config;
}

guard::Config JointPosition::getGuard() {
  std::lock_guard<std::mutex> lock(mux_);
  return guard_shared_;
}

guard::State JointPosition::getGuardState() {
  std::lock_guard<std::mutex> lock(mux_);
  return guard_state_shared_;
}

void JointPosition::trip() {
  std::lock_guard<std::mutex> lock(mux_);
  command_.trip = true;
}

void JointPosition::rearm() {
  std::lock_guard<std::mutex> lock(mux_);
  command_.rearm = true;
  command_.trip = false;
}

size_t JointPosition::readTelemetry(std::vector<joint_position::Sample>& out) {
  return telemetry_.drain(out);
}

uint64_t JointPosition::telemetryDropped() const { return telemetry_.dropped(); }

size_t JointPosition::telemetryCapacity() const { return telemetry_.capacity(); }
