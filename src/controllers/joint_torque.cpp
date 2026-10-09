#include "controllers/joint_torque.h"

#include <stdexcept>

const Vector7d JointTorque::kDefaultDamping = Vector7d::Zero();

namespace {

void checkDamping(const Vector7d& damping) {
  if (!damping.allFinite() || (damping.array() < 0).any()) {
    throw std::invalid_argument("The damping must be finite and non-negative.");
  }
}

}  // namespace

JointTorque::JointTorque(const Vector7d& damping, size_t telemetry_capacity)
    : Loop(telemetry_capacity), damping_(damping), loop_damping_(damping) {
  checkDamping(damping);
}

void JointTorque::begin(const franka::RobotState& robot_state) {
  tau_d_.setZero();
  tau_d_shared_.setZero();
  pending_ = false;
  loop_damping_ = damping_;
}

bool JointTorque::sync(const franka::RobotState& robot_state, double time) {
  loop_damping_ = damping_;
  if (!pending_) {
    return false;
  }
  tau_d_ = tau_d_shared_;
  pending_ = false;
  return true;
}

void JointTorque::onRearm(const franka::RobotState& robot_state) {
  tau_d_.setZero();
}

Vector7d JointTorque::law(const franka::RobotState& robot_state, double dt,
                          bool tripped) {
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const Vector7d damping = -loop_damping_.cwiseProduct(dq);
  return tripped ? damping : Vector7d(tau_d_ + damping);
}

void JointTorque::record(joint_torque::Sample& s,
                         const franka::RobotState& robot_state) {
  controllers::putField(s.tau_d, tau_d_);
  controllers::putField(s.damping, loop_damping_);
}

void JointTorque::setReference(const Vector7d& tau_d) {
  if (!tau_d.allFinite()) {
    throw std::invalid_argument("The torque must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  tau_d_shared_ = tau_d;
  pending_ = true;
}

void JointTorque::setDamping(const Vector7d& damping) {
  checkDamping(damping);
  std::lock_guard<std::mutex> lock(mux_);
  damping_ = damping;
}

Vector7d JointTorque::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return damping_;
}
