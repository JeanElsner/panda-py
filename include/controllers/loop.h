#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <franka/model.h>

#include "controllers/controller.h"
#include "controllers/guard.h"
#include "telemetry.h"
#include "utils.h"

/// The telemetry every controller records per tick, first in its Sample:
/// the loop's bookkeeping, the torque the law computed and the one sent, and
/// the robot state's main fields. A controller's sample fields macro starts
/// with LOOP_SAMPLE_FIELDS(X).
#define LOOP_SAMPLE_FIELDS(X)         \
  X(tick, 1)                          \
  X(time, 1)                          \
  X(duration, 1)                      \
  X(reference_update, 1)              \
  X(control_command_success_rate, 1)  \
  X(tau_law, 7)                       \
  X(tau_cmd, 7)                       \
  X(q, 7)                             \
  X(dq, 7)                            \
  X(tau_J, 7)                         \
  X(tau_J_d, 7)                       \
  X(tau_ext_hat_filtered, 7)          \
  X(O_T_EE, 16)                       \
  X(F_T_EE, 16)                       \
  X(O_F_ext_hat_K, 6)                 \
  X(K_F_ext_hat_K, 6)                 \
  X(guard, 1)

/// Declares a Sample struct from a fields macro.
#define LOOP_DECLARE_FIELD(name, size) double name[size];

namespace controllers {

/// Copies an Eigen vector or std::array into a sample field.
template <typename From>
void putField(double* to, const From& from) {
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(to, from.size()) =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(from.data(),
                                                                 from.size());
}

/**
 * The 1 kHz loop every panda-py controller runs on.
 *
 * Python sets parameters and commands through setters that only ever take a
 * mutex the loop try-locks: if a setter holds it, the tick runs on what it
 * had, and the command waits for the next tick rather than the loop for the
 * command. Each tick:
 *
 *  1. prepare(): per-tick computation needed before anything else (a control
 *     frame's pose and Jacobian, say).
 *  2. If the lock is free: the shared guard configuration and trip or rearm
 *     requests are taken (onRearm() on a rearm), then sync(), where the controller copies its
 *     parameters and applies pending commands, and publish(), where it
 *     updates what Python may read about the loop.
 *  3. The guards are evaluated at guardFrame(), the flange unless overridden.
 *  4. law() computes the torque; while a guard is tripped the controller
 *     drops its active term (what "active" is, is the controller's).
 *  5. One telemetry sample, if the ring has room: the common fields, then
 *     record() for the controller's own. commanded() completes it with the
 *     torque actually sent, after panda-py's joint walls, rate limit and
 *     clipping.
 */
template <typename Sample>
class Loop : public TorqueController {
 public:
  explicit Loop(size_t telemetry_capacity)
      : motion_finished_(false), telemetry_(telemetry_capacity) {}

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) final {
    const double time = robot_state.time.toSec();
    const double dt = duration.toSec();
    prepare(robot_state);

    bool updated = false;
    std::unique_lock<std::mutex> lock(mux_, std::try_to_lock);
    if (lock.owns_lock()) {
      guard_ = guard_shared_;
      if (trip_requested_) {
        monitor_.trip(guard::Trip::kManual, time);
      }
      if (rearm_requested_) {
        monitor_.reset();
        onRearm(robot_state);
      }
      updated = sync(robot_state, time) || trip_requested_ || rearm_requested_;
      trip_requested_ = rearm_requested_ = false;
      guard_state_shared_ = monitor_.state();
      publish(robot_state, time);
      lock.unlock();
    }

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (std::isfinite(guard_.speed) ||
        (guard_.workspace_size > 0 && !guard_.workspace_end_effector)) {
      guardFrame(robot_state, position, velocity);
    }
    monitor_.evaluate(guard_, robot_state, position, velocity, dt);

    const Vector7d tau = law(robot_state, dt, monitor_.state().tripped());

    sample_ = telemetry_.claim();
    if (sample_) {
      Sample& s = *sample_;
      s.tick[0] = static_cast<double>(tick_);
      s.time[0] = time;
      s.duration[0] = dt;
      s.reference_update[0] = updated ? 1.0 : 0.0;
      s.control_command_success_rate[0] = robot_state.control_command_success_rate;
      putField(s.tau_law, tau);
      std::fill(std::begin(s.tau_cmd), std::end(s.tau_cmd),
                std::numeric_limits<double>::quiet_NaN());
      putField(s.q, robot_state.q);
      putField(s.dq, robot_state.dq);
      putField(s.tau_J, robot_state.tau_J);
      putField(s.tau_J_d, robot_state.tau_J_d);
      putField(s.tau_ext_hat_filtered, robot_state.tau_ext_hat_filtered);
      putField(s.O_T_EE, robot_state.O_T_EE);
      putField(s.F_T_EE, robot_state.F_T_EE);
      putField(s.O_F_ext_hat_K, robot_state.O_F_ext_hat_K);
      putField(s.K_F_ext_hat_K, robot_state.K_F_ext_hat_K);
      s.guard[0] = static_cast<double>(monitor_.state().trip);
      record(s, robot_state);
    }
    tick_++;

    franka::Torques torques = VectorToArray<7>(tau);
    torques.motion_finished = motion_finished_ || finished(robot_state);
    return torques;
  }

  void commanded(const franka::RobotState& robot_state,
                 const franka::Torques& torques) final {
    monitor_.commanded(torques);
    if (sample_) {
      std::copy(torques.tau_J.begin(), torques.tau_J.end(), sample_->tau_cmd);
      telemetry_.publish();
      sample_ = nullptr;
    }
  }

  void start(const franka::RobotState& robot_state,
             std::shared_ptr<franka::Model> model) final {
    motion_finished_ = false;
    model_ = model;
    std::lock_guard<std::mutex> lock(mux_);
    trip_requested_ = rearm_requested_ = false;
    guard_ = guard_shared_;
    monitor_.reset();
    guard_state_shared_ = guard::State();
    tick_ = 0;
    sample_ = nullptr;
    begin(robot_state);
  }

  void stop(const franka::RobotState& robot_state,
            std::shared_ptr<franka::Model> model) final {
    motion_finished_ = true;
  }

  bool isRunning() final { return !motion_finished_; }

  // -- guards ----------------------------------------------------------------

  void setGuard(const guard::Config& config) {
    if (config.workspace_size > guard::Config::kMaxBoxes) {
      throw std::invalid_argument("Too many workspace boxes.");
    }
    std::lock_guard<std::mutex> lock(mux_);
    guard_shared_ = config;
  }
  guard::Config getGuard() {
    std::lock_guard<std::mutex> lock(mux_);
    return guard_shared_;
  }
  guard::State getGuardState() {
    std::lock_guard<std::mutex> lock(mux_);
    return guard_state_shared_;
  }
  /// Trips the guard on the loop's next tick, as if one had fired.
  void trip() {
    std::lock_guard<std::mutex> lock(mux_);
    trip_requested_ = true;
  }
  /// Clears a trip on the loop's next tick; see onRearm().
  void rearm() {
    std::lock_guard<std::mutex> lock(mux_);
    rearm_requested_ = true;
    trip_requested_ = false;
  }

  // -- telemetry -------------------------------------------------------------

  size_t readTelemetry(std::vector<Sample>& out) { return telemetry_.drain(out); }
  uint64_t telemetryDropped() const { return telemetry_.dropped(); }
  size_t telemetryCapacity() const { return telemetry_.capacity(); }

 protected:
  /// Loop thread, every tick, before anything else and without the lock.
  virtual void prepare(const franka::RobotState& robot_state) {}
  /// start(), under the lock: hold where the robot is.
  virtual void begin(const franka::RobotState& robot_state) = 0;
  /// Loop thread, under the lock: copy the parameters, apply pending
  /// commands. Returns whether a command was applied.
  virtual bool sync(const franka::RobotState& robot_state, double time) = 0;
  /// Loop thread, under the lock: update what getters return.
  virtual void publish(const franka::RobotState& robot_state, double time) {}
  /// Loop thread, under the lock, when a trip is cleared: resume from zero
  /// active term, from the state of this tick.
  virtual void onRearm(const franka::RobotState& robot_state) = 0;
  /// The guarded frame's position and linear velocity; the flange's.
  virtual void guardFrame(const franka::RobotState& robot_state,
                          Eigen::Vector3d& position, Eigen::Vector3d& velocity) {
    const Eigen::Affine3d O_T_F(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()) *
        Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.F_T_EE.data()))
            .inverse()
            .matrix());
    position = O_T_F.translation();
    const auto jacobian = model_->zeroJacobian(franka::Frame::kFlange, robot_state);
    velocity = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian.data())
                   .topRows<3>() *
               Eigen::Map<const Vector7d>(robot_state.dq.data());
  }
  /// The torque for this tick, without gravity, which the robot adds.
  virtual Vector7d law(const franka::RobotState& robot_state, double dt,
                       bool tripped) = 0;
  /// The controller's own telemetry fields.
  virtual void record(Sample& sample, const franka::RobotState& robot_state) {}
  /// Ends the motion from inside the loop (a trajectory that has arrived).
  virtual bool finished(const franka::RobotState& robot_state) { return false; }

  std::mutex mux_;
  std::shared_ptr<franka::Model> model_;
  uint64_t tick_ = 0;
  guard::Monitor monitor_;

 private:
  std::atomic<bool> motion_finished_;
  bool trip_requested_ = false, rearm_requested_ = false;
  guard::Config guard_shared_, guard_;
  guard::State guard_state_shared_;
  TelemetryRing<Sample> telemetry_;
  Sample* sample_ = nullptr;
};

}  // namespace controllers
