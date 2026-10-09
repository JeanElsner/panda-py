#include "controllers/task_wrench.h"

#include <stdexcept>

const Vector7d TaskWrench::kDefaultDamping = Vector7d::Zero();

namespace {

void checkDamping(const Vector7d& damping) {
  if (!damping.allFinite() || (damping.array() < 0).any()) {
    throw std::invalid_argument("The damping must be finite and non-negative.");
  }
}

}  // namespace

TaskWrench::TaskWrench(const Vector7d& damping, size_t telemetry_capacity)
    : Loop(telemetry_capacity), damping_(damping), loop_damping_(damping) {
  checkDamping(damping);
}

void TaskWrench::begin(const franka::RobotState& robot_state) {
  wrench_d_.setZero();
  wrench_shared_.setZero();
  pending_ = false;
  loop_damping_ = damping_;
}

bool TaskWrench::sync(const franka::RobotState& robot_state, double time) {
  loop_damping_ = damping_;
  if (!pending_) {
    return false;
  }
  wrench_d_ = wrench_shared_;
  pending_ = false;
  return true;
}

void TaskWrench::onRearm(const franka::RobotState& robot_state) {
  wrench_d_.setZero();
}

void TaskWrench::guardFrame(const franka::RobotState& robot_state,
                            Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
  position = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()))
                 .translation();
  const auto jacobian =
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  velocity = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data())
                 .topRows<3>() *
             Eigen::Map<const Vector7d>(robot_state.dq.data());
}

Vector7d TaskWrench::law(const franka::RobotState& robot_state, double dt,
                         bool tripped) {
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  Vector7d tau = -loop_damping_.cwiseProduct(dq);
  if (!tripped) {
    const auto jacobian =
        model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
    tau += Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data())
               .transpose() *
           wrench_d_;
  }
  return tau;
}

void TaskWrench::record(task_wrench::Sample& s,
                        const franka::RobotState& robot_state) {
  controllers::putField(s.wrench_d, wrench_d_);
  controllers::putField(s.damping, loop_damping_);
}

void TaskWrench::setReference(const Vector6d& wrench) {
  if (!wrench.allFinite()) {
    throw std::invalid_argument("The wrench must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  wrench_shared_ = wrench;
  pending_ = true;
}

void TaskWrench::setDamping(const Vector7d& damping) {
  checkDamping(damping);
  std::lock_guard<std::mutex> lock(mux_);
  damping_ = damping;
}

Vector7d TaskWrench::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return damping_;
}
