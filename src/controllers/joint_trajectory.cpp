#include "controllers/joint_trajectory.h"

using namespace controllers;

const double JointTrajectory::kDefaultDqThreshold = 1e-3;
const double JointTrajectory::kSettleTolerance = 2e-3;
const double JointTrajectory::kSettleTimeout = 1.0;

JointTrajectory::JointTrajectory(std::shared_ptr<motion::JointTrajectory> trajectory,
                       const Vector7d &stiffness, const Vector7d &damping,
                       const double dq_threshold, const double filter_coeff)
    : JointPosition(stiffness, damping, filter_coeff),
      traj_(trajectory),
      dq_threshold_(dq_threshold) {}

franka::Torques JointTrajectory::step(const franka::RobotState &robot_state,
                                 franka::Duration &duration) {
  auto q_d = traj_->getJointPositions(getTime());
  auto dq_d = traj_->getJointVelocities(getTime());
  setControl(q_d, dq_d);
  auto torques = JointPosition::step(robot_state, duration);
  const double overrun = getTime() - traj_->getDuration();
  if (overrun > 0.0) {
    bool at_rest = true;
    for (auto dq : robot_state.dq) {
      if (std::abs(dq) > dq_threshold_) {
        at_rest = false;
      }
    }
    // Being at rest is not enough on its own: without an integral term the
    // controller comes to rest wherever the stiffness balances the residual
    // error, which can be well short of the goal.
    const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
    const Vector7d q_goal = traj_->getJointPositions(traj_->getDuration());
    const bool at_goal =
        (q_goal - q).cwiseAbs().maxCoeff() <= kSettleTolerance;
    if ((at_rest && at_goal) || overrun >= kSettleTimeout) {
      torques.motion_finished = true;
    }
  }
  return torques;
}

const std::string JointTrajectory::name() {
  return "JointTrajectory";
}
