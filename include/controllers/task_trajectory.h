#pragma once
#include "controllers/task_impedance.h"
#include "motion/generators.h"

namespace controllers {

/// Follows a Cartesian trajectory with the end effector, using TaskImpedance
/// at the end-effector frame with a kinematic nullspace and Coriolis
/// compensation, the reference taken from the trajectory on every tick, and
/// ends the motion once the robot has settled at the goal.
class TaskTrajectory : public TaskImpedance {
 public:
  static const double kDefaultDqThreshold;
  static const double kSettlePositionTolerance;
  static const double kSettleOrientationTolerance;
  static const double kSettleTimeout;
  static const double kDefaultNullspaceStiffness;
  static const double kDefaultDampingRatio;
  static const Eigen::Matrix<double, 6, 6> kDefaultImpedance;

  TaskTrajectory(std::shared_ptr<motion::CartesianTrajectory> trajectory,
                 const Vector7d& q_init,
                 const Eigen::Matrix<double, 6, 6>& impedance = kDefaultImpedance,
                 const double& damping_ratio = kDefaultDampingRatio,
                 const double& nullspace_stiffness = kDefaultNullspaceStiffness,
                 const double dq_threshold = kDefaultDqThreshold);

  const std::string name() override { return "TaskTrajectory"; }

 protected:
  void prepare(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  bool finished(const franka::RobotState& robot_state) override;

 private:
  std::shared_ptr<motion::CartesianTrajectory> traj_;
  double dq_threshold_;
  Vector7d q_init_;
};

}  // namespace controllers
