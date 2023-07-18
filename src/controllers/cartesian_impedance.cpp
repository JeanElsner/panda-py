#include "controllers/cartesian_impedance.h"

#include <iostream>

#include "panda.h"

// clang-format off
double data[36] = {200,   0,   0,  0,  0,  0,
                     0, 200,   0,  0,  0,  0,
                     0,   0, 200,  0,  0,  0,
                     0,   0,   0, 10,  0,  0,
                     0,   0,   0,  0, 10,  0,
                     0,   0,   0,  0,  0, 10};
// clang-format on
const Eigen::Matrix<double, 6, 6> CartesianImpedance::kDefaultImpedance =
    Eigen::Matrix<double, 6, 6>(data);
const double CartesianImpedance::kDefaultDampingRatio = 1.0;
const double CartesianImpedance::kDefaultNullspaceStiffness = 0.5;
const double CartesianImpedance::kDefaultFilterCoeff = 1.0;

CartesianImpedance::CartesianImpedance(
    const Eigen::Matrix<double, 6, 6> &impedance, const double &damping_ratio,
    const double &nullspace_stiffness, const double &filter_coeff) {
  K_p_ = impedance;
  K_p_target_ = impedance;
  damping_ratio_ = damping_ratio;
  _computeDamping();
  K_d_ = K_d_target_;
  nullspace_stiffness_ = nullspace_stiffness;
  nullspace_stiffnes_target_ = nullspace_stiffness;
  filter_coeff_ = filter_coeff;
};

void CartesianImpedance::_computeDamping() {
  K_d_target_ = damping_ratio_ * 2 * K_p_.cwiseSqrt();
};

franka::Torques CartesianImpedance::step(const franka::RobotState &robot_state,
                                         franka::Duration &duration) {
  Eigen::Vector3d position_d;
  Eigen::Quaterniond orientation_d;
  Vector7d q_nullspace_d;
  Eigen::Matrix<double, 6, 6> K_p, K_d;
  // These quantities may be modified outside of the control loop
  mux_.lock();
  _updateFilter();
  K_p = K_p_;
  K_d = K_d_;
  position_d = position_d_;
  orientation_d = orientation_d_;
  q_nullspace_d = q_nullspace_d_;
  mux_.unlock();

  // get state variables
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  std::array<double, 42> jacobian_array =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

  // convert to Eigen
  Eigen::Map<Vector7d> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  Vector7d tau_J_d = Eigen::Map<const Vector7d>(robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d;

  // orientation error
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(),
      error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-K_p * error - K_d * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_nullspace_d - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;

  franka::Torques torques = VectorToArray<7>(tau_d);
  torques.motion_finished = motion_finished_;
  return torques;
}

void CartesianImpedance::_updateFilter() {
  K_p_ = ema_filter(K_p_, K_p_target_, filter_coeff_, true);
  K_d_ = ema_filter(K_d_, K_d_target_, filter_coeff_, true);
  nullspace_stiffness_ = ema_filter(
      nullspace_stiffness_, nullspace_stiffnes_target_, filter_coeff_, true);
  position_d_ =
      ema_filter(position_d_, position_d_target_, filter_coeff_, true);
  orientation_d_ = orientation_d_.slerp(filter_coeff_, orientation_d_target_);
}

void CartesianImpedance::setControl(const Eigen::Vector3d &position,
                                    const Eigen::Vector4d &orientation,
                                    const Vector7d &q_nullspace) {
  std::lock_guard<std::mutex> lock(mux_);
  position_d_target_ = position;
  orientation_d_target_ = orientation;
  q_nullspace_d_target_ = q_nullspace;
}

void CartesianImpedance::setImpedance(
    const Eigen::Matrix<double, 6, 6> &impedance) {
  std::lock_guard<std::mutex> lock(mux_);
  K_p_target_ = impedance;
  _computeDamping();
}

void CartesianImpedance::setDampingRatio(const double &damping_ratio) {
  std::lock_guard<std::mutex> lock(mux_);
  damping_ratio_ = damping_ratio;
  _computeDamping();
}

void CartesianImpedance::setNullspaceStiffness(
    const double &nullspace_stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  nullspace_stiffnes_target_ = nullspace_stiffness;
}

void CartesianImpedance::setFilter(const double filter_coeff) {
  std::lock_guard<std::mutex> lock(mux_);
  filter_coeff_ = filter_coeff;
}

void CartesianImpedance::start(const franka::RobotState &robot_state,
                               std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  position_d_ = position;
  position_d_target_ = position;
  orientation_d_ = orientation;
  orientation_d_target_ = orientation;
  q_nullspace_d_ = q;
  q_nullspace_d_target_ = q;
  model_ = model;
}

void CartesianImpedance::stop(const franka::RobotState &robot_state,
                              std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool CartesianImpedance::isRunning() { return !motion_finished_; }

const std::string CartesianImpedance::name() { return "Cartesian Impedance"; }