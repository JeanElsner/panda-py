#pragma once
#include <limits>

#include "controllers/loop.h"

namespace task_force {

#define TASK_FORCE_SAMPLE_FIELDS(X) \
  LOOP_SAMPLE_FIELDS(X)             \
  X(wrench_d, 6)                    \
  X(tau_ext, 7)                     \
  X(tau_error_integral, 7)          \
  X(gains, 2)                       \
  X(displacement, 1)

struct Sample {
  TASK_FORCE_SAMPLE_FIELDS(LOOP_DECLARE_FIELD)
};

}  // namespace task_force

/**
 * Task force: regulates the wrench the end effector exerts, base frame, with
 * feed-forward and a PI loop on the joint torques it maps to,
 *
 *   tau_d = J^T w_d,  tau_ext = tau_J - g(q) - tau_ext(start),
 *   tau = tau_d + k_p (tau_d - tau_ext) + k_i int (tau_d - tau_ext) - D dq,
 *
 * as in libfranka's force control example. The external torque is measured
 * relative to start, so start it in free space or at rest on the surface.
 * If the end effector moves more than the maximum displacement from where it
 * started, the guard trips (workspace). While tripped only the damping
 * remains and the integral is reset.
 */
class TaskForce : public controllers::Loop<task_force::Sample> {
 public:
  static const double kDefaultProportionalGain;
  static const double kDefaultIntegralGain;
  static const double kDefaultMaxDisplacement;
  static const Vector7d kDefaultDamping;

  TaskForce(double k_p = kDefaultProportionalGain,
            double k_i = kDefaultIntegralGain,
            const Vector7d& damping = kDefaultDamping,
            double max_displacement = kDefaultMaxDisplacement,
            size_t telemetry_capacity = 0);

  const std::string name() override { return "TaskForce"; }

  /// The wrench to exert, force then torque, base frame, from the next tick.
  void setReference(const Vector6d& wrench);
  void setGains(double k_p, double k_i);
  std::pair<double, double> getGains();
  void setDamping(const Vector7d& damping);
  Vector7d getDamping();
  void setMaxDisplacement(double max_displacement);
  double getMaxDisplacement();

 protected:
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  void onRearm(const franka::RobotState& robot_state) override;
  void guardFrame(const franka::RobotState& robot_state, Eigen::Vector3d& position,
                  Eigen::Vector3d& velocity) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  void record(task_force::Sample& sample,
              const franka::RobotState& robot_state) override;

 private:
  struct Parameters {
    double k_p, k_i, max_displacement;
    Vector7d damping;
  };
  // Shared, under mux_.
  Parameters shared_;
  Vector6d wrench_shared_ = Vector6d::Zero();
  bool pending_ = false;
  // Loop thread only.
  Parameters loop_;
  Vector6d wrench_d_ = Vector6d::Zero();
  Vector7d tau_ext_bias_ = Vector7d::Zero(), tau_ext_ = Vector7d::Zero(),
           integral_ = Vector7d::Zero();
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  double displacement_ = 0.0;
};
