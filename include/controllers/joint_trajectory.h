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
  // Impedance control alone settles a little short of the goal, where the
  // stiffness balances friction or an unmodelled load. Once the trajectory has
  // ended, a small integral term closes the remaining error: it integrates
  // only while a joint is outside kSettleTolerance, holds its value inside,
  // and is bounded by kSettleTorqueLimit, so pushing against an obstacle
  // stays within a few N m. The motion ends once the robot is at rest at the
  // goal, or after kSettleTimeout, so that a goal which cannot be reached
  // still terminates.
  static const double kSettleTolerance;
  static const double kSettleTimeout;
  static const double kSettleGain;
  static const double kSettleTorqueLimit;

  JointTrajectory(std::shared_ptr<motion::JointTrajectory> trajectory,
                  const Vector7d& stiffness = kDefaultStiffness,
                  const Vector7d& damping = kDefaultDamping,
                  const double dq_threshold = kDefaultDqThreshold);

  const std::string name() override { return "JointTrajectory"; }

 protected:
  void prepare(const franka::RobotState& robot_state) override;
  void begin(const franka::RobotState& robot_state) override;
  void onRearm(const franka::RobotState& robot_state) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  bool finished(const franka::RobotState& robot_state) override;

 private:
  std::shared_ptr<motion::JointTrajectory> traj_;
  double dq_threshold_;
  Vector7d settle_ = Vector7d::Zero();
};

}  // namespace controllers
