#include "controllers/task_force.h"

#include <stdexcept>

const double TaskForce::kDefaultProportionalGain = 1.0;
const double TaskForce::kDefaultIntegralGain = 2.0;
const double TaskForce::kDefaultMaxDisplacement = 0.01;
const double kDefaultDampingData[7] = {1.0, 1.0, 1.0, 1.0, 0.33, 0.33, 0.17};
const Vector7d TaskForce::kDefaultDamping = Vector7d(kDefaultDampingData);

namespace {

void checkGains(double k_p, double k_i) {
  if (!std::isfinite(k_p) || !std::isfinite(k_i) || k_p < 0 || k_i < 0) {
    throw std::invalid_argument("The gains must be finite and non-negative.");
  }
}

void checkDamping(const Vector7d& damping) {
  if (!damping.allFinite() || (damping.array() < 0).any()) {
    throw std::invalid_argument("The damping must be finite and non-negative.");
  }
}

void checkDisplacement(double max_displacement) {
  if (!(max_displacement > 0.0)) {
    throw std::invalid_argument("The maximum displacement must be positive.");
  }
}

Vector7d externalTorque(const franka::RobotState& robot_state, franka::Model& model) {
  return Eigen::Map<const Vector7d>(robot_state.tau_J.data()) -
         Eigen::Map<const Vector7d>(model.gravity(robot_state).data());
}

}  // namespace

TaskForce::TaskForce(double k_p, double k_i, const Vector7d& damping,
                     double max_displacement, size_t telemetry_capacity)
    : Loop(telemetry_capacity) {
  checkGains(k_p, k_i);
  checkDamping(damping);
  checkDisplacement(max_displacement);
  shared_ = {k_p, k_i, max_displacement, damping};
  loop_ = shared_;
}

void TaskForce::begin(const franka::RobotState& robot_state) {
  loop_ = shared_;
  wrench_d_.setZero();
  wrench_shared_.setZero();
  pending_ = false;
  tau_ext_bias_ = externalTorque(robot_state, *model_);
  integral_.setZero();
  origin_ = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()))
                .translation();
  displacement_ = 0.0;
}

bool TaskForce::sync(const franka::RobotState& robot_state, double time) {
  loop_ = shared_;
  if (!pending_) {
    return false;
  }
  wrench_d_ = wrench_shared_;
  pending_ = false;
  return true;
}

void TaskForce::onRearm(const franka::RobotState& robot_state) {
  wrench_d_.setZero();
  integral_.setZero();
  origin_ = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()))
                .translation();
}

void TaskForce::guardFrame(const franka::RobotState& robot_state,
                           Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
  position = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()))
                 .translation();
  const auto jacobian =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  velocity = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data())
                 .topRows<3>() *
             Eigen::Map<const Vector7d>(robot_state.dq.data());
}

Vector7d TaskForce::law(const franka::RobotState& robot_state, double dt,
                        bool tripped) {
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const Eigen::Vector3d position =
      Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())).translation();
  displacement_ = (position - origin_).norm();
  if (!tripped && displacement_ > loop_.max_displacement) {
    monitor_.trip(guard::Trip::kWorkspace, robot_state.time.toSec(), displacement_);
    tripped = true;
  }
  tau_ext_ = externalTorque(robot_state, *model_) - tau_ext_bias_;
  Vector7d tau = -loop_.damping.cwiseProduct(dq);
  if (tripped) {
    integral_.setZero();
    return tau;
  }
  const auto jacobian =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  const Vector7d tau_d =
      Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data()).transpose() *
      wrench_d_;
  integral_ += dt * (tau_d - tau_ext_);
  tau += tau_d + loop_.k_p * (tau_d - tau_ext_) + loop_.k_i * integral_;
  return tau;
}

void TaskForce::record(task_force::Sample& s, const franka::RobotState& robot_state) {
  controllers::putField(s.wrench_d, wrench_d_);
  controllers::putField(s.tau_ext, tau_ext_);
  controllers::putField(s.tau_error_integral, integral_);
  s.gains[0] = loop_.k_p;
  s.gains[1] = loop_.k_i;
  s.displacement[0] = displacement_;
}

void TaskForce::setReference(const Vector6d& wrench) {
  if (!wrench.allFinite()) {
    throw std::invalid_argument("The wrench must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  wrench_shared_ = wrench;
  pending_ = true;
}

void TaskForce::setGains(double k_p, double k_i) {
  checkGains(k_p, k_i);
  std::lock_guard<std::mutex> lock(mux_);
  shared_.k_p = k_p;
  shared_.k_i = k_i;
}

std::pair<double, double> TaskForce::getGains() {
  std::lock_guard<std::mutex> lock(mux_);
  return {shared_.k_p, shared_.k_i};
}

void TaskForce::setDamping(const Vector7d& damping) {
  checkDamping(damping);
  std::lock_guard<std::mutex> lock(mux_);
  shared_.damping = damping;
}

Vector7d TaskForce::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return shared_.damping;
}

void TaskForce::setMaxDisplacement(double max_displacement) {
  checkDisplacement(max_displacement);
  std::lock_guard<std::mutex> lock(mux_);
  shared_.max_displacement = max_displacement;
}

double TaskForce::getMaxDisplacement() {
  std::lock_guard<std::mutex> lock(mux_);
  return shared_.max_displacement;
}
