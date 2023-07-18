#include "controllers/applied_force.h"

#include <iostream>

#include "panda.h"

const double kDefaultDampingData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d AppliedForce::kDefaultDamping = Vector7d(kDefaultDampingData);
const double AppliedForce::kDefaultFilterCoeff = 1.0;

AppliedForce::AppliedForce(const Vector7d &damping, const double filter_coeff)
    : filter_coeff_(filter_coeff), K_d_(damping), K_d_target_(damping){};

franka::Torques AppliedForce::step(const franka::RobotState &robot_state,
                                   franka::Duration &duration) {
  // Observed states
  Vector6d f_d;
  Vector7d dq, tau_d, K_d;
  dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  // These quantities may be modified outside of the control loop
  mux_.lock();
  _updateFilter();
  K_d = K_d_;
  f_d = f_d_;
  mux_.unlock();
  std::array<double, 42> jacobian_array =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  // Apply torque and damping
  tau_d << jacobian.transpose()*f_d - K_d.asDiagonal() * dq;

  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void AppliedForce::_updateFilter() {
  f_d_ = ema_filter(f_d_, f_d_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
}

void AppliedForce::setControl(const Vector6d &force) {
  std::lock_guard<std::mutex> lock(mux_);
  f_d_target_ = force;
}

void AppliedForce::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_target_ = damping;
}

void AppliedForce::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void AppliedForce::start(const franka::RobotState &robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  f_d_.setZero();
  f_d_target_.setZero();
  model_ = model;
}

void AppliedForce::stop(const franka::RobotState &robot_state,
                        std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool AppliedForce::isRunning() { return !motion_finished_; }

const std::string AppliedForce::name() { return "Applied Force"; }