#pragma once
#include "controllers/joint_position.h"
#include "motion/generators.h"

namespace controllers {

class JointTrajectory : public JointPosition {
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
                  const double dq_threshold = kDefaultDqThreshold,
                  const double filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) override;

  const std::string name() override;

 private:
  std::shared_ptr<motion::JointTrajectory> traj_;
  double dq_threshold_;
};

}  // namespace controllers
