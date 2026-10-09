#pragma once
#include "controllers/joint_impedance.h"
#include "motion/generators.h"

namespace controllers {

/// Follows a joint trajectory with JointImpedance, the reference taken from
/// the trajectory on every tick, and ends the motion once the robot has
/// settled at the goal.
class JointTrajectory : public JointImpedance {
 public:
  static const double kDefaultDqThreshold;
  // Impedance control has no integral term, so the robot settles a little short
  // of the goal. Keep holding the final setpoint until the remaining error is
  // below kSettleTolerance, but give up after kSettleTimeout so that a goal
  // which cannot be reached, because of a payload or an obstacle, still
  // terminates.
  static const double kSettleTolerance;
  static const double kSettleTimeout;

  JointTrajectory(std::shared_ptr<motion::JointTrajectory> trajectory,
                  const Vector7d& stiffness = kDefaultStiffness,
                  const Vector7d& damping = kDefaultDamping,
                  const double dq_threshold = kDefaultDqThreshold);

  const std::string name() override { return "JointTrajectory"; }

 protected:
  void prepare(const franka::RobotState& robot_state) override;
  bool finished(const franka::RobotState& robot_state) override;

 private:
  std::shared_ptr<motion::JointTrajectory> traj_;
  double dq_threshold_;
};

}  // namespace controllers
