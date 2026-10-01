#pragma once
#include <atomic>
#include <mutex>

#include "controllers/controller.h"
#include "controllers/guard.h"
#include "telemetry.h"
#include "utils.h"

namespace joint_position {

/// One telemetry sample per control tick; see TASK_IMPEDANCE_SAMPLE_FIELDS.
#define JOINT_POSITION_SAMPLE_FIELDS(X) \
  X(tick, 1)                            \
  X(time, 1)                            \
  X(duration, 1)                        \
  X(reference_update, 1)                \
  X(control_command_success_rate, 1)    \
  X(q_d, 7)                             \
  X(dq_d, 7)                            \
  X(stiffness, 7)                       \
  X(damping, 7)                         \
  X(tau_active, 7)                      \
  X(tau_passive, 7)                     \
  X(tau_law, 7)                         \
  X(tau_cmd, 7)                         \
  X(q, 7)                               \
  X(dq, 7)                              \
  X(tau_J, 7)                           \
  X(tau_J_d, 7)                         \
  X(tau_ext_hat_filtered, 7)            \
  X(O_T_EE, 16)                         \
  X(F_T_EE, 16)                         \
  X(O_F_ext_hat_K, 6)                   \
  X(K_F_ext_hat_K, 6)                   \
  X(guard, 1)

struct Sample {
#define JOINT_POSITION_DECLARE(name, size) double name[size];
  JOINT_POSITION_SAMPLE_FIELDS(JOINT_POSITION_DECLARE)
#undef JOINT_POSITION_DECLARE
};

struct Snapshot {
  double time = 0.0;          // robot time of the latest tick, s
  double applied_time = 0.0;  // robot time the latest target was applied
  Vector7d q = Vector7d::Zero();          // latest tick
  Vector7d applied_q = Vector7d::Zero();  // ... at applied_time
  Vector7d q_d = Vector7d::Zero();
  uint64_t applied = 0;
};

}  // namespace joint_position

/// Joint position servo: tau = K (q_d - q) + D (dq_d - dq), with the active
/// part K (q_d - q) + D dq_d dropped while a guard is tripped. Targets are
/// applied by the loop on its next tick; the loop never waits for a setter.
class JointPosition : public TorqueController {
 public:
  static const Vector7d kDefaultStiffness;
  static const Vector7d kDefaultDamping;
  static const Vector7d kDefaultDqd;

  JointPosition(const Vector7d& stiffness = kDefaultStiffness,
                const Vector7d& damping = kDefaultDamping,
                size_t telemetry_capacity = 0);

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) override;
  void start(const franka::RobotState& robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState& robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;
  void commanded(const franka::RobotState& robot_state,
                 const franka::Torques& torques) override;

  /// Target positions and velocities, from the loop's next tick.
  void setControl(const Vector7d& position,
                  const Vector7d& velocity = kDefaultDqd);
  /// q_d = q + delta with q of the tick it is applied at, dq_d = 0: one
  /// policy step of the simulator's joint servo.
  void stepControl(const Vector7d& delta);
  void setStiffness(const Vector7d& stiffness);
  void setDamping(const Vector7d& damping);
  Vector7d getStiffness();
  Vector7d getDamping();
  joint_position::Snapshot getSnapshot();

  void setGuard(const guard::Config& config);
  guard::Config getGuard();
  guard::State getGuardState();
  void trip();
  /// Clears a trip; the target becomes the positions of the next tick.
  void rearm();

  size_t readTelemetry(std::vector<joint_position::Sample>& out);
  uint64_t telemetryDropped() const;
  size_t telemetryCapacity() const;

 private:
  struct Command {
    bool absolute = false, relative = false, rearm = false, trip = false;
    Vector7d q_d = Vector7d::Zero(), dq_d = Vector7d::Zero();
    bool pending() const { return absolute || relative || rearm || trip; }
  };

  std::mutex mux_;
  Vector7d stiffness_, damping_;
  Command command_;
  guard::Config guard_shared_;
  guard::State guard_state_shared_;
  joint_position::Snapshot snapshot_;

  // Loop thread only.
  Vector7d loop_stiffness_, loop_damping_, q_d_, dq_d_;
  guard::Config guard_;
  guard::Monitor monitor_;
  uint64_t tick_ = 0, applied_ = 0;
  std::shared_ptr<franka::Model> model_;

  std::atomic<bool> motion_finished_;
  TelemetryRing<joint_position::Sample> telemetry_;
  joint_position::Sample* sample_ = nullptr;
};
