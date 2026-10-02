#include "controllers/guard.h"

#include <cmath>

#include "constants.h"

namespace guard {

const char* tripName(Trip trip) {
  switch (trip) {
    case Trip::kNone: return "none";
    case Trip::kForce: return "force";
    case Trip::kSaturation: return "saturation";
    case Trip::kSpeed: return "speed";
    case Trip::kJointVelocity: return "joint_velocity";
    case Trip::kWorkspace: return "workspace";
    case Trip::kManual: return "manual";
  }
  return "unknown";
}

double Box::outside(const Eigen::Vector3d& point) const {
  const Eigen::Vector3d local =
      pose.topLeftCorner<3, 3>().transpose() * (point - pose.topRightCorner<3, 1>());
  return (local.cwiseAbs() - half_extents).cwiseMax(0.0).norm();
}

void Monitor::evaluate(const Config& config, const franka::RobotState& robot_state,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& velocity, double dt) {
  if (state_.tripped()) {
    return;
  }
  const double time = robot_state.time.toSec();
  const double force =
      (Eigen::Map<const Eigen::Vector3d>(robot_state.O_F_ext_hat_K.data()) - config.force_bias)
          .norm();
  force_time_ = force > config.force ? force_time_ + dt : 0.0;
  if (force > config.force && force_time_ >= config.force_time) {
    return trip(Trip::kForce, time, force);
  }
  // Whether last tick's torque was clipped: commanded() runs after step().
  saturation_time_ = saturated_ ? saturation_time_ + dt : 0.0;
  if (saturated_ && saturation_time_ >= config.saturation_time) {
    return trip(Trip::kSaturation, time, saturation_time_);
  }
  const double speed = velocity.norm();
  if (speed > config.speed) {
    return trip(Trip::kSpeed, time, speed);
  }
  for (int i = 0; i < 7; i++) {
    const double dq = std::abs(robot_state.dq[i]);
    if (dq > config.joint_velocity[i]) {
      return trip(Trip::kJointVelocity, time, dq, i);
    }
  }
  if (config.workspace_size > 0) {
    const Eigen::Vector3d point =
        config.workspace_end_effector
            ? Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                              robot_state.O_T_EE[14])
            : position;
    double outside = std::numeric_limits<double>::infinity();
    for (size_t b = 0; b < config.workspace_size; b++) {
      outside = std::min(outside, config.workspace[b].outside(point));
    }
    if (outside > 0.0) {
      return trip(Trip::kWorkspace, time, outside);
    }
  }
}

void Monitor::commanded(const franka::Torques& torques) {
  saturated_ = false;
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(torques.tau_J[i]) >= kTauJMax[i] - 1e-9) {
      saturated_ = true;
    }
  }
}

void Monitor::trip(Trip reason, double time, double value, int joint) {
  if (state_.tripped()) {
    return;
  }
  state_.trip = reason;
  state_.time = time;
  state_.value = value;
  state_.joint = joint;
}

void Monitor::reset() {
  state_ = State();
  force_time_ = saturation_time_ = 0.0;
  saturated_ = false;
}

}  // namespace guard
