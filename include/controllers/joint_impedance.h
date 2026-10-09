#pragma once
#include "controllers/loop.h"

namespace joint_impedance {

#define JOINT_IMPEDANCE_SAMPLE_FIELDS(X) \
  LOOP_SAMPLE_FIELDS(X)                  \
  X(q_d, 7)                              \
  X(dq_d, 7)                             \
  X(stiffness, 7)                        \
  X(damping, 7)                          \
  X(tau_active, 7)                       \
  X(tau_passive, 7)

struct Sample {
  JOINT_IMPEDANCE_SAMPLE_FIELDS(LOOP_DECLARE_FIELD)
};

struct Snapshot {
  double time = 0.0;          // robot time of the latest tick, s
  double applied_time = 0.0;  // robot time the latest reference was applied
  Vector7d q = Vector7d::Zero();          // latest tick
  Vector7d applied_q = Vector7d::Zero();  // ... at applied_time
  Vector7d q_d = Vector7d::Zero();
  uint64_t applied = 0;
};

}  // namespace joint_impedance

/**
 * Joint impedance: a spring and damper per joint to a reference,
 *
 *   tau = K (q_d - q) + D (dq_d - dq),
 *
 * with the active part K (q_d - q) + D dq_d dropped while a guard is
 * tripped. On start it holds the current joint positions.
 */
class JointImpedance : public controllers::Loop<joint_impedance::Sample> {
 public:
  static const Vector7d kDefaultStiffness;
  static const Vector7d kDefaultDamping;

  JointImpedance(const Vector7d& stiffness = kDefaultStiffness,
                 const Vector7d& damping = kDefaultDamping,
                 size_t telemetry_capacity = 0);

  const std::string name() override { return "JointImpedance"; }

  /// Reference positions and velocities, from the loop's next tick.
  void setReference(const Vector7d& q_d,
                    const Vector7d& dq_d = Vector7d::Zero());
  /// q_d = q + delta with q of the tick it is applied at, dq_d = 0.
  void stepReference(const Vector7d& delta);
  void setStiffness(const Vector7d& stiffness);
  void setDamping(const Vector7d& damping);
  Vector7d getStiffness();
  Vector7d getDamping();
  joint_impedance::Snapshot getSnapshot();

 protected:
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  void publish(const franka::RobotState& robot_state, double time) override;
  void onRearm(const franka::RobotState& robot_state) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  void record(joint_impedance::Sample& sample,
              const franka::RobotState& robot_state) override;

  /// Loop thread only: the reference the law uses on this tick.
  Vector7d q_d_ = Vector7d::Zero(), dq_d_ = Vector7d::Zero();

 private:
  struct Command {
    bool absolute = false, relative = false;
    Vector7d q_d = Vector7d::Zero(), dq_d = Vector7d::Zero();
  };
  // Shared with the setters, under mux_.
  Vector7d stiffness_, damping_;
  Command command_;
  joint_impedance::Snapshot snapshot_;
  // Loop thread only.
  Vector7d loop_stiffness_, loop_damping_;
  Vector7d tau_active_ = Vector7d::Zero(), tau_passive_ = Vector7d::Zero();
  uint64_t applied_ = 0;
};
