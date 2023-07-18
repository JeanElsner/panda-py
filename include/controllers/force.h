#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "utils.h"

class Force : public TorqueController {
 public:
  static const double kDefaultFilterCoeff;
  static const double kDefaultProportionalGain;
  static const double kDefaultIntegralGain;
  static const double kDefaultThreshold;
  static const Vector7d kDefaultDamping;

  Force(const double &k_p = kDefaultProportionalGain,
        const double &k_i = kDefaultIntegralGain,
        const Vector7d &damping = kDefaultDamping,
        const double &threshold = kDefaultThreshold,
        const double &filter_coeff = kDefaultFilterCoeff);

  franka::Torques step(const franka::RobotState &robot_state,
                       franka::Duration &duration) override;
  void setControl(const Eigen::Vector3d &force);
  void setProportionalGain(const double &k_p);
  void setIntegralGain(const double &k_i);
  void setThreshold(const double &threshold);
  void setDamping(const Vector7d &damping);
  void setFilter(const double &filter_coeff);
  void start(const franka::RobotState &robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState &robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

 private:
  Vector7d tau_ext_init_, tau_error_integral_, K_d_, K_d_target_;
  Eigen::Vector3d f_d_, f_d_target_, position_init_;
  double filter_coeff_, k_p_, k_i_, k_p_target_, k_i_target_, threshold_,
      threshold_target_;
  std::mutex mux_;
  std::atomic<bool> motion_finished_;
  std::shared_ptr<franka::Model> model_;

  void _updateFilter();
};
