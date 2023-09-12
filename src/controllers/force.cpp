#include "controllers/force.h"

#include <franka/exception.h>

#include <iostream>
#include <math.h>

#include "panda.h"

const double kDefaultDampingData[7] = {1.0, 1.0, 1.0, 1.0, 0.33, 0.33, 0.17};
const Vector7d Force::kDefaultDamping = Vector7d(kDefaultDampingData);

const double Force::kDefaultFilterCoeff = 0.001;
const double Force::kDefaultProportionalGain = 1;
const double Force::kDefaultIntegralGain = 2;
const double Force::kDefaultThreshold = 0.01;

Force::Force(const double &k_p, const double &k_i, const Vector7d &damping,
             const double &threshold, const double &filter_coeff)
    : filter_coeff_(filter_coeff),
      k_p_(k_p),
      k_i_(k_i),
      k_p_target_(k_p),
      k_i_target_(k_i),
      K_d_(damping),
      K_d_target_(damping),
      threshold_(threshold),
      threshold_target_(threshold){};

franka::Torques Force::step(const franka::RobotState &robot_state,
                            franka::Duration &duration) {
  // get state variables
  std::array<double, 42> jacobian_array =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  std::array<double, 7> gravity_array = model_->gravity(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Vector7d> tau_measured(
      robot_state.tau_J.data());
  Eigen::Map<const Vector7d> gravity(gravity_array.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Map<const Vector7d> dq(robot_state.dq.data());
  tau_error_integral_maximum << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;

  // These quantities may be modified outside of the control loop
  mux_.lock();
  _updateFilter();
  Eigen::Vector3d f = f_d_;
  double k_p = k_p_;
  double k_i = k_i_;
  double threshold = threshold_;
  Vector7d K_d = K_d_;
  mux_.unlock();

  // Abort condition
  if (getTime() > 0 && (position - position_init_).norm() > threshold) {
    throw franka::Exception(name() + ": Distance threshold exceeded.");
  }

  Eigen::VectorXd tau_d(7), force_torque_d(6), tau_ext(7);
  force_torque_d.setZero();
  force_torque_d.head(3) = f;
  tau_ext << tau_measured - gravity - tau_ext_init_;
  tau_d << jacobian.transpose() * force_torque_d;
  tau_error_integral_ += duration.toSec() * (tau_d - tau_ext);
  tau_error_integral_ = tau_error_integral_.cwiseMax(tau_error_integral_maximum);
  // FF + PI control
  tau_d << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral_ - K_d.asDiagonal() * dq;

  franka::Torques torques = VectorToArray<7>(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void Force::_updateFilter() {
  f_d_ = ema_filter(f_d_, f_d_target_, filter_coeff_, true);
  k_p_ = ema_filter(k_p_, k_p_target_, filter_coeff_, true);
  k_i_ = ema_filter(k_i_, k_i_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
  threshold_ = ema_filter(threshold_, threshold_target_, filter_coeff_, true);
}

void Force::setControl(const Eigen::Vector3d &force) {
  std::lock_guard<std::mutex> lock(mux_);
  f_d_target_ = force;
}

void Force::setFilter(const double &filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void Force::setProportionalGain(const double &k_p) {
  std::lock_guard<std::mutex> lock(mux_);
  k_p_target_ = k_p;
}

void Force::setIntegralGain(const double &k_i) {
  std::lock_guard<std::mutex> lock(mux_);
  k_i_target_ = k_i;
}

void Force::setThreshold(const double &threshold) {
  std::lock_guard<std::mutex> lock(mux_);
  threshold_target_ = threshold;
}

void Force::setDamping(const Vector7d &damping) {
  std::lock_guard<std::mutex> lock(mux_);
  K_d_target_ = damping;
}

void Force::start(const franka::RobotState &robot_state,
                  std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  f_d_.setZero();
  f_d_target_.setZero();

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());

  position_init_ = position;
  model_ = model;

  // Bias torque sensor
  std::array<double, 7> gravity_array = model->gravity(robot_state);
  std::array<double, 7> tau_measured_array = robot_state.tau_J;
  Eigen::Map<Vector7d> initial_tau_measured(
      tau_measured_array.data());
  Eigen::Map<Vector7d> initial_gravity(gravity_array.data());
  tau_ext_init_ = initial_tau_measured - initial_gravity;

  // init integrator
  tau_error_integral_.setZero();
}

void Force::stop(const franka::RobotState &robot_state,
                 std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool Force::isRunning() { return !motion_finished_; }

const std::string Force::name() { return "Force Controller"; }
