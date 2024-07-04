#pragma once
#include "controllers/joint_position.h"
#include "motion/generators.h"

namespace controllers {

class JointTrajectory : public JointPosition {
 public:
  static const double kDefaultDqThreshold;

  JointTrajectory(std::shared_ptr<motion::JointTrajectory> trajectory,
             const Vector7d &stiffness = kDefaultStiffness,
             const Vector7d &damping = kDefaultDamping,
             const double dq_threshold = kDefaultDqThreshold,
             const double filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;

  const std::string name() override;

 private:
  std::shared_ptr<motion::JointTrajectory> traj_;
  double dq_threshold_;
};

} // namespace
