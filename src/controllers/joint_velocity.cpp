#include "controllers/joint_velocity.h"

#include <iostream>

#include "panda.h"

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d JointVelocity::kDefaultStiffness =
    Vector7d(kDefaultStiffnessData);

const double kDefaultDqdData[7] = {0, 0, 0, 0, 0, 0, 0};
const Vector7d JointVelocity::kDefaultDqd = Vector7d(kDefaultDqdData);

const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d JointVelocity::kDefaultDamping = Vector7d(kDefaultDampingData);
const double JointVelocity::kDefaultFilterCoeff = 1.0;

const double kDefaultIData[7] = {5, 5, 5, 2, 2, 2, 1};
const Vector7d JointVelocity::kDefaultI = Vector7d(kDefaultIData);

const double velErrorCumMaxDefaultData[7] = {0.5, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1};
const Vector7d JointVelocity::velErrorCumMaxDefault = Vector7d(velErrorCumMaxDefaultData);

const double velErrorCumMinDefaultData[7] = {-0.5, -0.5, -0.5, -0.2, -0.2, -0.2, -0.1};
const Vector7d JointVelocity::velErrorCumMinDefault = Vector7d(velErrorCumMinDefaultData);

JointVelocity::JointVelocity(const Vector7d &stiffness, const Vector7d &damping,
                             const double filter_coeff,
                             const Vector7d &vel_error_cum_max,
                             const Vector7d &vel_error_cum_min,
                             const Vector7d &k_i) {
  K_p_ = stiffness;
  K_p_target_ = stiffness;
  K_d_ = damping;
  K_d_target_ = damping;
  K_i_ = k_i;
  vel_error_cum_max_ = vel_error_cum_max;
  vel_error_cum_min_ = vel_error_cum_min;
  filter_coeff_ = filter_coeff;
};

franka::Torques JointVelocity::step(const franka::RobotState &robot_state,
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
  vel_error = (dq_d - dq);
  vel_error_cum += vel_error;
  vel_error_cum = vel_error_cum.cwiseMax(vel_error_cum_min_).cwiseMin(vel_error_cum_max_);
  // PD control
  //tau_d << K_p.asDiagonal() * (q_d - q) + K_d.asDiagonal() * (dq_d - dq);
  tau_d << K_d.asDiagonal() * vel_error + K_i_.asDiagonal() * vel_error_cum;
  franka::Torques torques = VectorToArray(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void JointVelocity::_updateFilter() {
  q_d_ = ema_filter(q_d_, q_d_target_, filter_coeff_, true);
  dq_d_ = ema_filter(dq_d_, dq_d_target_, filter_coeff_, true);
  K_p_ = ema_filter(K_p_, K_p_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
}

void JointVelocity::setControl(const Vector7d &velocity) {
  std::lock_guard<std::mutex> lock(mux_);
  dq_d_target_ = velocity;
}

void JointVelocity::setStiffness(const Vector7d &stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  K_p_target_ = stiffness;
}

void JointVelocity::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_target_ = damping;
}

void JointVelocity::setKi(const Vector7d &K_i) {
  std::lock_guard<std::mutex> lock(mux_);
  K_i_ = K_i;
}

void JointVelocity::setVelErrorCumMax(const Vector7d &vel_error_cum_max) {
  std::lock_guard<std::mutex> lock(mux_);
  vel_error_cum_max_ = vel_error_cum_max;
}

void JointVelocity::setVelErrorCumMin(const Vector7d &vel_error_cum_min) {
  std::lock_guard<std::mutex> lock(mux_);
  vel_error_cum_min_ = vel_error_cum_min;
}

void JointVelocity::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void JointVelocity::start(const franka::RobotState &robot_state,
                          std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  q_d_target_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
  dq_d_target_.setZero();
}

void JointVelocity::stop(const franka::RobotState &robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool JointVelocity::isRunning() { return !motion_finished_; }

const std::string JointVelocity::name() { return "Joint Position"; }
