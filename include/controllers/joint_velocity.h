#pragma once
#include <limits>

#include "controllers/joint_impedance.h"

/**
 * Joint velocity control: the reference velocity is integrated into a
 * JointImpedance reference position every tick, clamped to the robot's joint
 * envelope,
 *
 *   q_d += dq_d dt,  tau = K (q_d - q) + D (dq_d - dq),
 *
 * so the joints follow the velocity and hold their position when it is zero.
 * With a command timeout, the reference velocity falls to zero when no new
 * one has arrived for that long. While a guard is tripped nothing is
 * integrated and the active part is dropped.
 */
class JointVelocity : public JointImpedance {
 public:
  JointVelocity(const Vector7d& stiffness = kDefaultStiffness,
                const Vector7d& damping = kDefaultDamping,
                double command_timeout = std::numeric_limits<double>::infinity(),
                size_t telemetry_capacity = 0);

  const std::string name() override { return "JointVelocity"; }

  /// The reference velocity, from the loop's next tick.
  void setReference(const Vector7d& dq_d);
  /// Seconds without a new reference after which the velocity is zero.
  void setCommandTimeout(double timeout);
  double getCommandTimeout();

 protected:
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;

 private:
  // Shared, under mux_.
  Vector7d velocity_ = Vector7d::Zero();
  bool pending_ = false;
  double timeout_;
  // Loop thread only.
  double loop_timeout_, last_command_time_ = 0.0, time_ = 0.0;
};
