#include "controllers/cartesian_trajectory.h"

using namespace controllers;

const double CartesianTrajectory::kDefaultDqThreshold = 1e-3;
const double CartesianTrajectory::kDefaultNullspaceStiffness = 15.0;
// clang-format off
double _data[36] = {800,   0,   0,  0,  0,  0,
                      0, 800,   0,  0,  0,  0,
                      0,   0, 800,  0,  0,  0,
                      0,   0,   0, 40,  0,  0,
                      0,   0,   0,  0, 40,  0,
                      0,   0,   0,  0,  0, 40};
// clang-format on
const Eigen::Matrix<double, 6, 6> CartesianTrajectory::kDefaultImpedance =
    Eigen::Matrix<double, 6, 6>(_data);

CartesianTrajectory::CartesianTrajectory(std::shared_ptr<motion::CartesianTrajectory> trajectory,
             const Eigen::Matrix<double, 6, 6> &impedance,
             const double &damping_ratio,
             const double &nullspace_stiffness,
             const double dq_threshold,
             const double filter_coeff)
    : CartesianImpedance(impedance, damping_ratio, nullspace_stiffness, filter_coeff),
      traj_(trajectory),
      dq_threshold_(dq_threshold) {}

franka::Torques CartesianTrajectory::step(const franka::RobotState &robot_state,
                                 franka::Duration &duration) {
  auto position = traj_->getPosition(getTime());
  auto orientation = traj_->getOrientation(getTime());
  setControl(position, orientation);
  auto torques = CartesianImpedance::step(robot_state, duration);
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

const std::string CartesianTrajectory::name() {
  return "CartesianTrajectory";
}
