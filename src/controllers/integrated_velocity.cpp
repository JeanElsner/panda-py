#include "controllers/integrated_velocity.h"
#include "constants.h"
#include <iostream>

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d IntegratedVelocity::kDefaultStiffness =
    Vector7d(kDefaultStiffnessData);
  
const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d IntegratedVelocity::kDefaultDamping =
    Vector7d(kDefaultDampingData);

const double qDefaultDiffLowerData[7] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
const Vector7d IntegratedVelocity::qDefaultDiffLower =
    Vector7d(qDefaultDiffLowerData);

const double qDefaultDiffUpperData[7] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
const Vector7d IntegratedVelocity::qDefaultDiffUpper =
    Vector7d(qDefaultDiffUpperData);

IntegratedVelocity::IntegratedVelocity(const Vector7d &stiffness,
                                       const Vector7d &damping,
                                       const Vector7d &q_diff_upper,
                                       const Vector7d &q_diff_lower) {
  K_p_ = stiffness;
  K_d_ = damping;
  q_diff_upper_ = q_diff_upper;
  q_diff_lower_ = q_diff_lower;
};

franka::Torques IntegratedVelocity::step(const franka::RobotState &robot_state,
                                         franka::Duration &duration) {
  // Observed states
  Vector7d q, dq, dq_d, tau_d, K_p, K_d;
  q = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  // These quantities may be modified outside of the control loop
  mux_.try_lock();
  dq_d = dq_d_;
  K_p = K_p_;
  K_d = K_d_;
  mux_.unlock();
  // PD control
  q_d_ += q + duration.toSec() * dq_d;  // integrate velocity
  q_d_ = q_d_.cwiseMin(kUpperJointLimits).cwiseMax(kLowerJointLimits);
  q_diff_ = q_d_ - q;
  q_diff_ = q_diff_.cwiseMin(q_diff_upper_).cwiseMax(q_diff_lower_);
  tau_d << K_p.asDiagonal() * (dq_d - dq) - K_d.asDiagonal() * dq;
  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void IntegratedVelocity::setControl(const Vector7d &velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  dq_d_ = velocity;
}

void IntegratedVelocity::setStiffness(const Vector7d &stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  K_p_ = stiffness;
}

void IntegratedVelocity::setQDiffUpper(const Vector7d &q_diff_upper) {
  std::lock_guard<std::mutex> lock(mux_);
  q_diff_upper_ = q_diff_upper;
}

void IntegratedVelocity::setQDiffLower(const Vector7d &q_diff_lower) {
  std::lock_guard<std::mutex> lock(mux_);
  q_diff_lower_ = q_diff_lower;
}

void IntegratedVelocity::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_ = damping;
}

void IntegratedVelocity::start(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
}

void IntegratedVelocity::stop(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool IntegratedVelocity::isRunning() {
  return !motion_finished_;
}

const std::string IntegratedVelocity::name() {
  return "Integrated Velocity";
}
