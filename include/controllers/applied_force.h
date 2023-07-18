#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "utils.h"

class AppliedForce : public TorqueController {
 public:
  static const double kDefaultFilterCoeff;
  static const Vector7d kDefaultDamping;

  AppliedForce(const Vector7d &damping = kDefaultDamping,
                const double filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;
  void setControl(const Vector6d &force);
  void setDamping(const Vector7d &damping);
  void setFilter(const double filter_coeff);
  void start(const franka::RobotState &robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Vector6d f_d_, f_d_target_;
  Vector7d K_d_, K_d_target_;
  double filter_coeff_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
  std::shared_ptr<franka::Model> model_;

  void _updateFilter();
};
