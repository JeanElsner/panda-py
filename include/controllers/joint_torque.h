#pragma once
#include "controllers/loop.h"

namespace joint_torque {

#define JOINT_TORQUE_SAMPLE_FIELDS(X) \
  LOOP_SAMPLE_FIELDS(X)               \
  X(tau_d, 7)                         \
  X(damping, 7)

struct Sample {
  JOINT_TORQUE_SAMPLE_FIELDS(LOOP_DECLARE_FIELD)
};

}  // namespace joint_torque

/**
 * Joint torque: a feed-forward torque per joint and viscous damping,
 *
 *   tau = tau_d - D dq,
 *
 * with tau_d dropped while a guard is tripped. The robot compensates gravity
 * itself; tau_d is what comes on top. Starts with tau_d = 0.
 */
class JointTorque : public controllers::Loop<joint_torque::Sample> {
 public:
  static const Vector7d kDefaultDamping;

  JointTorque(const Vector7d& damping = kDefaultDamping,
              size_t telemetry_capacity = 0);

  const std::string name() override { return "JointTorque"; }

  /// The feed-forward torque, from the loop's next tick.
  void setReference(const Vector7d& tau_d);
  void setDamping(const Vector7d& damping);
  Vector7d getDamping();

 protected:
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  void onRearm(const franka::RobotState& robot_state) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  void record(joint_torque::Sample& sample,
              const franka::RobotState& robot_state) override;

 private:
  Vector7d tau_d_shared_ = Vector7d::Zero(), damping_;
  bool pending_ = false;
  Vector7d tau_d_ = Vector7d::Zero(), loop_damping_;
};
