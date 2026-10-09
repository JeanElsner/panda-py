#include "controllers/joint_trajectory.h"

#include <algorithm>
#include <cmath>

using namespace controllers;

const double JointTrajectory::kDefaultDqThreshold = 1e-3;
const double JointTrajectory::kSettleTolerance = 2e-3;
const double JointTrajectory::kSettleTimeout = 1.0;
const double JointTrajectory::kSettleGain = 400.0;
const double JointTrajectory::kSettleTorqueLimit = 2.0;

JointTrajectory::JointTrajectory(
    std::shared_ptr<motion::JointTrajectory> trajectory,
    const Vector7d& stiffness, const Vector7d& damping,
    const double dq_threshold)
    : JointImpedance(stiffness, damping),
      traj_(trajectory),
      dq_threshold_(dq_threshold) {}

void JointTrajectory::prepare(const franka::RobotState& robot_state) {
  q_d_ = traj_->getJointPositions(getTime());
  dq_d_ = traj_->getJointVelocities(getTime());
}

void JointTrajectory::begin(const franka::RobotState& robot_state) {
  JointImpedance::begin(robot_state);
  settle_.setZero();
}

void JointTrajectory::onRearm(const franka::RobotState& robot_state) {
  JointImpedance::onRearm(robot_state);
  settle_.setZero();
}

Vector7d JointTrajectory::law(const franka::RobotState& robot_state, double dt,
                              bool tripped) {
  const Vector7d tau = JointImpedance::law(robot_state, dt, tripped);
  if (tripped || getTime() <= traj_->getDuration()) {
    return tau;
  }
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d error = traj_->getJointPositions(traj_->getDuration()) - q;
  for (int j = 0; j < 7; j++) {
    if (std::abs(error[j]) > kSettleTolerance) {
      settle_[j] = std::clamp(settle_[j] + kSettleGain * error[j] * dt,
                              -kSettleTorqueLimit, kSettleTorqueLimit);
    }
  }
  return tau + settle_;
}

bool JointTrajectory::finished(const franka::RobotState& robot_state) {
  const double overrun = getTime() - traj_->getDuration();
  if (overrun <= 0.0) {
    return false;
  }
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const bool at_rest = dq.cwiseAbs().maxCoeff() <= dq_threshold_;
  // Being at rest is not enough on its own: before the integral term has
  // built up, the robot can rest short of the goal.
  const Vector7d q_goal = traj_->getJointPositions(traj_->getDuration());
  const bool at_goal = (q_goal - q).cwiseAbs().maxCoeff() <= kSettleTolerance;
  return (at_rest && at_goal) || overrun >= kSettleTimeout;
}
