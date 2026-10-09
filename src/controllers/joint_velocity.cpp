#include "controllers/joint_velocity.h"

#include <stdexcept>

namespace {

void checkTimeout(double timeout) {
  if (!(timeout > 0.0)) {
    throw std::invalid_argument("The command timeout must be positive.");
  }
}

}  // namespace

JointVelocity::JointVelocity(const Vector7d& stiffness, const Vector7d& damping,
                             double command_timeout, size_t telemetry_capacity)
    : JointImpedance(stiffness, damping, telemetry_capacity),
      timeout_(command_timeout),
      loop_timeout_(command_timeout) {
  checkTimeout(command_timeout);
}

void JointVelocity::begin(const franka::RobotState& robot_state) {
  JointImpedance::begin(robot_state);
  velocity_.setZero();
  pending_ = false;
  loop_timeout_ = timeout_;
  last_command_time_ = time_ = robot_state.time.toSec();
}

bool JointVelocity::sync(const franka::RobotState& robot_state, double time) {
  const bool updated = JointImpedance::sync(robot_state, time) || pending_;
  loop_timeout_ = timeout_;
  time_ = time;
  if (pending_) {
    dq_d_ = velocity_;
    last_command_time_ = time;
    pending_ = false;
  }
  return updated;
}

Vector7d JointVelocity::law(const franka::RobotState& robot_state, double dt,
                            bool tripped) {
  time_ = robot_state.time.toSec();
  if (time_ - last_command_time_ > loop_timeout_) {
    dq_d_.setZero();
  }
  if (!tripped) {
    q_d_ = (q_d_ + dt * dq_d_)
               .cwiseMin(robotLimits().q_upper)
               .cwiseMax(robotLimits().q_lower);
  }
  return JointImpedance::law(robot_state, dt, tripped);
}

void JointVelocity::setReference(const Vector7d& dq_d) {
  if (!dq_d.allFinite()) {
    throw std::invalid_argument("The reference velocity must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  velocity_ = dq_d;
  pending_ = true;
}

void JointVelocity::setCommandTimeout(double timeout) {
  checkTimeout(timeout);
  std::lock_guard<std::mutex> lock(mux_);
  timeout_ = timeout;
}

double JointVelocity::getCommandTimeout() {
  std::lock_guard<std::mutex> lock(mux_);
  return timeout_;
}
