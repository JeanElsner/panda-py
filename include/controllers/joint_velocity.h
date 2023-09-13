#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "utils.h"

class JointVelocity : public TorqueController {
 public:
  static const Vector7d kDefaultStiffness;
  static const Vector7d kDefaultDamping;
  static const Vector7d kDefaultDqd;
  static const Vector7d kDefaultI;
  static const Vector7d velErrorCumMaxDefault;
  static const Vector7d velErrorCumMinDefault;
  static const double kDefaultFilterCoeff;

  JointVelocity(const Vector7d &stiffness = kDefaultStiffness,
                const Vector7d &damping = kDefaultDamping,
                const double filter_coeff = kDefaultFilterCoeff,
                const Vector7d &vel_error_cum_max= velErrorCumMaxDefault,
                const Vector7d &vel_error_cum_min= velErrorCumMinDefault,
                const Vector7d &k_i = kDefaultI);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;
  void setControl(const Vector7d &velocity);
  void setStiffness(const Vector7d &stiffness);
  void setDamping(const Vector7d &damping);
  void setVelErrorCumMax(const Vector7d &vel_error_cum_max);
  void setVelErrorCumMin(const Vector7d &vel_error_cum_min);
  void setKi(const Vector7d &K_i);
  void setFilter(const double filter_coeff);
  void start(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state, std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Vector7d K_p_, K_d_, q_d_, dq_d_, K_p_target_, K_d_target_, q_d_target_, dq_d_target_, K_i_, vel_error_cum_max_, vel_error_cum_min_, vel_error, vel_error_cum;
  double filter_coeff_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;

  void _updateFilter();
};
