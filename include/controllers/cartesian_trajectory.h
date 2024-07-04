#pragma once
#include "controllers/cartesian_impedance.h"
#include "motion/generators.h"

namespace controllers {

class CartesianTrajectory : public CartesianImpedance {
 public:
  static const double kDefaultDqThreshold;

  CartesianTrajectory(std::shared_ptr<motion::CartesianTrajectory> trajectory,
             const Eigen::Matrix<double, 6, 6> &impedance = kDefaultImpedance,
             const double &damping_ratio = kDefaultDampingRatio,
             const double &nullspace_stiffness = kDefaultNullspaceStiffness,
             const double dq_threshold = kDefaultDqThreshold,
             const double filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;

  const std::string name() override;

 private:
  std::shared_ptr<motion::CartesianTrajectory> traj_;
  double dq_threshold_;
};

} // namespace
