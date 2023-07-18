#include "controllers/joint_position.h"

#include <iostream>

#include "panda.h"

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d JointPosition::kDefaultStiffness =
    Vector7d(kDefaultStiffnessData);

const double kDefaultDqdData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d JointPosition::kDefaultDqd = Vector7d(kDefaultDqdData);

const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d JointPosition::kDefaultDamping = Vector7d(kDefaultDampingData);
const double JointPosition::kDefaultFilterCoeff = 1.0;

JointPosition::JointPosition(const Vector7d &stiffness, const Vector7d &damping,
                             const double filter_coeff) {
  K_p_ = stiffness;
  K_p_target_ = stiffness;
  K_d_ = damping;
  K_d_target_ = damping;
  filter_coeff_ = filter_coeff;
};

franka::Torques JointPosition::step(const franka::RobotState &robot_state,
                                    franka::Duration &duration) {
  // Observed states
  Vector7d q, q_d, dq_d, dq, tau_d, K_p, K_d;
  q = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  // These quantities may be modified outside of the control loop
  mux_.lock();
  _updateFilter();
  K_p = K_p_;
  K_d = K_d_;
  q_d = q_d_;
  dq_d = dq_d_;
  mux_.unlock();
  // PD control
  tau_d << K_p.asDiagonal() * (q_d - q) + K_d.asDiagonal() * (dq_d - dq);
  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void JointPosition::_updateFilter() {
  q_d_ = ema_filter(q_d_, q_d_target_, filter_coeff_, true);
  dq_d_ = ema_filter(dq_d_, dq_d_target_, filter_coeff_, true);
  K_p_ = ema_filter(K_p_, K_p_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
}

void JointPosition::setControl(const Vector7d &position,
                               const Vector7d &velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  q_d_target_ = position;
  dq_d_target_ = velocity;
}

void JointPosition::setStiffness(const Vector7d &stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  K_p_target_ = stiffness;
}

void JointPosition::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_target_ = damping;
}

void JointPosition::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void JointPosition::start(const franka::RobotState &robot_state,
                          std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  q_d_target_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
  dq_d_target_.setZero();
}

void JointPosition::stop(const franka::RobotState &robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool JointPosition::isRunning() { return !motion_finished_; }

const std::string JointPosition::name() { return "Joint Position"; }