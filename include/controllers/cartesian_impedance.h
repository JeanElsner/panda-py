#pragma once
#include <atomic>
#include <mutex>

#include "constants.h"
#include "controllers/controller.h"
#include "utils.h"

class CartesianImpedance : public TorqueController {
 public:
  static const Eigen::Matrix<double, 6, 6> kDefaultImpedance;
  static const double kDefaultDampingRatio;
  static const double kDefaultNullspaceStiffness;
  static const double kDefaultFilterCoeff;

  CartesianImpedance(const Eigen::Matrix<double, 6, 6> &impedance =
                        kDefaultImpedance,
                     const double &damping_ratio = kDefaultDampingRatio,
                     const double &nullspace_stiffness =
                         kDefaultNullspaceStiffness,
                     const double &filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;
  void setControl(const Eigen::Vector3d &position,
                  const Eigen::Vector4d &orientation,
                  const Vector7d &q_nullspace = kJointPositionStart);
  void setImpedance(const Eigen::Matrix<double, 6, 6> &impedance);
  void setDampingRatio(const double &damping_ratio);
  void setNullspaceStiffness(const double &nullspace_stiffness);
  void setFilter(const double filter_coeff);
  void start(const franka::RobotState &robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Eigen::Matrix<double, 6, 6> K_p_, K_d_, K_p_target_, K_d_target_;
  Eigen::Vector3d position_d_, position_d_target_;
  Eigen::Quaterniond orientation_d_, orientation_d_target_;
  Vector7d q_nullspace_d_, q_nullspace_d_target_;
  double filter_coeff_, nullspace_stiffness_, nullspace_stiffnes_target_,
      damping_ratio_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
  std::shared_ptr<franka::Model> model_;

  void _updateFilter();
  void _computeDamping();
};
