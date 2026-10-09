#include "controllers/joint_trajectory.h"

using namespace controllers;

const double JointTrajectory::kDefaultDqThreshold = 1e-3;
const double JointTrajectory::kSettleTolerance = 2e-3;
const double JointTrajectory::kSettleTimeout = 1.0;

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

bool JointTrajectory::finished(const franka::RobotState& robot_state) {
  const double overrun = getTime() - traj_->getDuration();
  if (overrun <= 0.0) {
    return false;
  }
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  const bool at_rest = dq.cwiseAbs().maxCoeff() <= dq_threshold_;
  // Being at rest is not enough on its own: without an integral term the
  // controller comes to rest wherever the stiffness balances the residual
  // error, which can be well short of the goal.
  const Vector7d q_goal = traj_->getJointPositions(traj_->getDuration());
  const bool at_goal = (q_goal - q).cwiseAbs().maxCoeff() <= kSettleTolerance;
  return (at_rest && at_goal) || overrun >= kSettleTimeout;
}
