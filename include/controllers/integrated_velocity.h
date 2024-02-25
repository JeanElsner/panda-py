#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "utils.h"

class IntegratedVelocity : public TorqueController {
 public:
  static const Vector7d kDefaultStiffness;
  static const Vector7d kDefaultDamping;

  IntegratedVelocity(const Vector7d &stiffness = kDefaultStiffness,
                     const Vector7d &damping = kDefaultDamping);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;

  Vector7d getQd();
  void setControl(const Vector7d &velocity);
  void setStiffness(const Vector7d &stiffness);
  void setDamping(const Vector7d &damping);
  void start(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Vector7d K_p_, K_d_, q_d_, dq_d_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
};
