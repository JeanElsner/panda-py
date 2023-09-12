#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "utils.h"

class IntegratedVelocity : public TorqueController {
 public:
  static const Vector7d kDefaultStiffness;
  static const Vector7d kDefaultDamping;
  static const Vector7d qDefaultDiffLower;
  static const Vector7d qDefaultDiffUpper;

  IntegratedVelocity(const Vector7d &stiffness = kDefaultStiffness,
                     const Vector7d &damping = kDefaultDamping,
                     const Vector7d &q_diff_upper= qDefaultDiffUpper,
                     const Vector7d &q_diff_lower= qDefaultDiffLower);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;

  void setControl(const Vector7d &velocity);
  void setStiffness(const Vector7d &stiffness);
  void setDamping(const Vector7d &damping);
  void setQDiffUpper(const Vector7d &q_diff_upper);
  void setQDiffLower(const Vector7d &q_diff_lower);
  void start(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Vector7d K_p_, K_d_, q_d_, dq_d_, q_diff_, q_diff_upper_, q_diff_lower_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
};
