#include "controllers/joint_trajectory.h"

using namespace controllers;

const double JointTrajectory::kDefaultDqThreshold = 1e-3;

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
  if (getTime() > traj_->getDuration()) {
    bool at_rest = true;
    for (auto dq : robot_state.dq) {
      if (std::abs(dq) > dq_threshold_) {
        at_rest = false;
      }
    }
    if (at_rest) {
      torques.motion_finished = true;
    }
  }
  return torques;
}

const std::string JointTrajectory::name() {
  return "JointTrajectory";
}