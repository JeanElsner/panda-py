#include "controllers/trajectory.h"

using namespace controllers;

const double Trajectory::kDefaultDqThreshold = 1e-3;

Trajectory::Trajectory(std::shared_ptr<motion::PandaTrajectory> trajectory,
                       const Vector7d &stiffness, const Vector7d &damping,
                       const double dq_threshold, const double filter_coeff)
    : JointPosition(stiffness, damping, filter_coeff),
      traj_(trajectory),
      dq_threshold_(dq_threshold) {}

franka::Torques Trajectory::step(const franka::RobotState &robot_state,
                                 franka::Duration &duration) {
  Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  auto q_d = traj_->getJointPositions(getTime(), q, q[7]);
  auto dq_d = traj_->getJointVelocities(getTime(), q, q[7]);
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
  return franka::Torques({0,0,0,0,0,0,0});
}

const std::string Trajectory::name() {
  return "Trajectory";
}