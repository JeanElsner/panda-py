#include "controllers/applied_torque.h"

#include <iostream>

#include "panda.h"

const double kDefaultDampingData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d AppliedTorque::kDefaultDamping = Vector7d(kDefaultDampingData);
const double AppliedTorque::kDefaultFilterCoeff = 1.0;

AppliedTorque::AppliedTorque(const Vector7d &damping, const double filter_coeff)
    : filter_coeff_(filter_coeff), K_d_(damping), K_d_target_(damping){};

franka::Torques AppliedTorque::step(const franka::RobotState &robot_state,
                                    franka::Duration &duration) {
  // Observed states
  Vector7d dq, tau_d, K_d;
  dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  // These quantities may be modified outside of the control loop
  mux_.lock();
  _updateFilter();
  K_d = K_d_;
  tau_d = tau_d_;
  mux_.unlock();
  // Apply torque and damping
  tau_d << tau_d - K_d.asDiagonal() * dq;

  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void AppliedTorque::_updateFilter() {
  tau_d_ = ema_filter(tau_d_, tau_d_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
}

void AppliedTorque::setControl(const Vector7d &torque) {
  std::lock_guard<std::mutex> lock(mux_);
  tau_d_target_ = torque;
}

void AppliedTorque::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_target_ = damping;
}

void AppliedTorque::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void AppliedTorque::start(const franka::RobotState &robot_state,
                          std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  tau_d_.setZero();
  tau_d_target_.setZero();
}

void AppliedTorque::stop(const franka::RobotState &robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool AppliedTorque::isRunning() { return !motion_finished_; }

const std::string AppliedTorque::name() { return "Applied Torque"; }