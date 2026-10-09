#pragma once
#include "controllers/loop.h"

namespace task_wrench {

#define TASK_WRENCH_SAMPLE_FIELDS(X) \
  LOOP_SAMPLE_FIELDS(X)              \
  X(wrench_d, 6)                     \
  X(damping, 7)

struct Sample {
  TASK_WRENCH_SAMPLE_FIELDS(LOOP_DECLARE_FIELD)
};

}  // namespace task_wrench

/**
 * Task wrench: a feed-forward wrench at the end effector, base frame, and
 * viscous joint damping,
 *
 *   tau = J^T w_d - D dq,
 *
 * with w_d (force, then torque) dropped while a guard is tripped. Starts
 * with w_d = 0.
 */
class TaskWrench : public controllers::Loop<task_wrench::Sample> {
 public:
  static const Vector7d kDefaultDamping;

  TaskWrench(const Vector7d& damping = kDefaultDamping,
             size_t telemetry_capacity = 0);

  const std::string name() override { return "TaskWrench"; }

  /// The wrench at the end effector, base frame, from the loop's next tick.
  void setReference(const Vector6d& wrench);
  void setDamping(const Vector7d& damping);
  Vector7d getDamping();

 protected:
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  void onRearm(const franka::RobotState& robot_state) override;
  void guardFrame(const franka::RobotState& robot_state, Eigen::Vector3d& position,
                  Eigen::Vector3d& velocity) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  void record(task_wrench::Sample& sample,
              const franka::RobotState& robot_state) override;

 private:
  Vector6d wrench_shared_ = Vector6d::Zero(), wrench_d_ = Vector6d::Zero();
  Vector7d damping_, loop_damping_;
  bool pending_ = false;
};
