#include "controllers/integrated_velocity.h"
#include "constants.h"
#include <iostream>

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d IntegratedVelocity::kDefaultStiffness =
    Vector7d(kDefaultStiffnessData);
  
const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d IntegratedVelocity::kDefaultDamping =
    Vector7d(kDefaultDampingData);

IntegratedVelocity::IntegratedVelocity(const Vector7d &stiffness,
                                       const Vector7d &damping) {
  K_p_ = stiffness;
  K_d_ = damping;
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
  q_d_ += duration.toSec() * dq_d;  // integrate velocity
  q_d_ = q_d_.cwiseMin(kUpperJointLimits).cwiseMax(kLowerJointLimits);
  tau_d << K_p.asDiagonal() * (q_d_ - q) - K_d.asDiagonal() * dq;
  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

Vector7d IntegratedVelocity::getQd() {
  return q_d_;
}

void IntegratedVelocity::setControl(const Vector7d &velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  dq_d_ = velocity;
}

void IntegratedVelocity::setStiffness(const Vector7d &stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  K_p_ = stiffness;
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