#pragma once
#include "controllers/task_impedance.h"
#include "motion/generators.h"

namespace controllers {

// Follows a Cartesian trajectory with the end effector, using task impedance
// at the end-effector frame with a kinematic nullspace and Coriolis
// compensation.
class CartesianTrajectory : public TaskImpedance {
 public:
  static const double kDefaultDqThreshold;
  // Impedance control has no integral term, so the robot settles a little short
  // of the goal. Keep holding the final setpoint until the remaining error is
  // below these tolerances, but give up after kSettleTimeout so that a goal
  // which cannot be reached, because of a payload or an obstacle, still
  // terminates.
  static const double kSettlePositionTolerance;
  static const double kSettleOrientationTolerance;
  static const double kSettleTimeout;
  static const double kDefaultNullspaceStiffness;
  static const double kDefaultDampingRatio;
  // Diagonal; off-diagonal entries are rejected.
  static const Eigen::Matrix<double, 6, 6> kDefaultImpedance;

  CartesianTrajectory(
      std::shared_ptr<motion::CartesianTrajectory> trajectory,
      const Vector7d& q_init,
      const Eigen::Matrix<double, 6, 6>& impedance = kDefaultImpedance,
      const double& damping_ratio = kDefaultDampingRatio,
      const double& nullspace_stiffness = kDefaultNullspaceStiffness,
      const double dq_threshold = kDefaultDqThreshold);

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) override;

  const std::string name() override;

 private:
  std::shared_ptr<motion::CartesianTrajectory> traj_;
  Vector7d q_init_;
  double dq_threshold_;
};

}  // namespace controllers
