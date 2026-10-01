#pragma once
#include <franka/control_types.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <array>
#include <limits>

#include "utils.h"

/// Safety guards evaluated in a controller's 1 kHz loop. A controller owns a
/// Monitor, feeds it every tick, and drops its active (spring) term while the
/// guard is tripped; what "active" means is the controller's.
namespace guard {

/// Why a guard tripped.
enum class Trip {
  kNone = 0,
  kForce,          // external force norm above the threshold for long enough
  kSaturation,     // a sent joint torque at its limit for long enough
  kSpeed,          // speed of the guarded frame
  kJointVelocity,  // a joint above its velocity limit
  kWorkspace,      // the guarded point outside every workspace box
  kManual,         // trip() from outside the loop
};

const char* tripName(Trip trip);

/// An oriented box: the point p is inside when |R^T (p - t)| <= half_extents
/// on every axis, with pose = [R t; 0 1] in the base frame.
struct Box {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Vector3d half_extents = Eigen::Vector3d::Zero();
  /// Distance outside the box, 0 inside.
  double outside(const Eigen::Vector3d& point) const;
};

/// Thresholds; infinity disables a guard.
struct Config {
  static constexpr size_t kMaxBoxes = 8;
  double force = std::numeric_limits<double>::infinity();  // N, |O_F_ext_hat_K[0:3]|
  double force_time = 0.05;  // s above it to trip
  double saturation_time = std::numeric_limits<double>::infinity();  // s
  double speed = std::numeric_limits<double>::infinity();  // m/s
  Vector7d joint_velocity =
      Vector7d::Constant(std::numeric_limits<double>::infinity());  // rad/s
  /// The point must stay inside at least one box; none disables the guard.
  std::array<Box, kMaxBoxes> workspace;
  size_t workspace_size = 0;
  /// Guard the end effector (O_T_EE) rather than the controller's frame.
  bool workspace_end_effector = true;
};

struct State {
  Trip trip = Trip::kNone;
  double time = 0.0;   // robot time of the trip
  double value = 0.0;  // what tripped it: N, s, m/s, rad/s or m outside
  int joint = -1;      // the joint, for kSaturation and kJointVelocity
  bool tripped() const { return trip != Trip::kNone; }
};

class Monitor {
 public:
  /// Evaluates the guards for one tick, unless already tripped. `position`
  /// and `velocity` are the controller frame's position and linear velocity.
  void evaluate(const Config& config, const franka::RobotState& robot_state,
                const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                double dt);
  /// The torque sent this tick, for the saturation guard on the next.
  void commanded(const franka::Torques& torques);
  void trip(Trip reason, double time, double value = 0.0, int joint = -1);
  /// Clears a trip and the timers.
  void reset();
  const State& state() const { return state_; }

 private:
  State state_;
  double force_time_ = 0.0, saturation_time_ = 0.0;
  bool saturated_ = false;
};

}  // namespace guard
