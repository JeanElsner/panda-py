#include "controllers/joint_limits/virtual_wall.h"

#include <cmath>
#include <iostream>
#include <sstream>

using controllers::joint_limits::VirtualWall;

VirtualWall::VirtualWall(const double& soft_upper_joint_position_limit,
                     const double& soft_lower_joint_position_limit,
                     const double& PD_zone_width, const double& D_zone_width,
                     const double& PD_zone_stiffness,
                     const double& PD_zone_damping,
                     const double& D_zone_damping)
    : soft_upper_joint_position_limit_(soft_upper_joint_position_limit),
      soft_lower_joint_position_limit_(soft_lower_joint_position_limit),
      PD_zone_width_(PD_zone_width),
      D_zone_width_(D_zone_width),
      PD_zone_stiffness_(PD_zone_stiffness),
      PD_zone_damping_(PD_zone_damping),
      D_zone_damping_(D_zone_damping) {
  PD_zone_width_ = positiveCheck(PD_zone_width);
  D_zone_width_ = positiveCheck(D_zone_width);
  PD_zone_stiffness_ = positiveCheck(PD_zone_stiffness);
  PD_zone_damping_ = positiveCheck(PD_zone_damping);
  D_zone_damping_ = positiveCheck(D_zone_damping);
};

double VirtualWall::computeTorque(const double& q, const double& dq) {
  init(q, dq);
  adjustMovingWall(q, dq);

  double D_zone_boundary_max =
      soft_upper_joint_position_limit_ -
      zone_width_scale_ * (PD_zone_width_ + D_zone_width_);
  double PD_zone_boundary_max =
      soft_upper_joint_position_limit_ - zone_width_scale_ * PD_zone_width_;
  double D_zone_boundary_min =
      soft_lower_joint_position_limit_ +
      zone_width_scale_ * (PD_zone_width_ + D_zone_width_);
  double PD_zone_boundary_min =
      soft_lower_joint_position_limit_ + zone_width_scale_ * PD_zone_width_;

  double torque = 0;
  // In D zone
  if (inRange(D_zone_boundary_max, PD_zone_boundary_max, q) ||
      inRange(PD_zone_boundary_min, D_zone_boundary_min, q)) {
    torque = -D_zone_damping_ * dq;
    // PD zone max
  } else if (q > PD_zone_boundary_max) {
    torque = -PD_zone_damping_ * dq +
             PD_zone_stiffness_ * (PD_zone_boundary_max - q);
    // PD zone min
  } else if (q < PD_zone_boundary_min) {
    torque = -PD_zone_damping_ * dq +
             PD_zone_stiffness_ * (PD_zone_boundary_min - q);
  }

  return torque;
}

void VirtualWall::reset() { initialized_ = false; }

bool VirtualWall::inRange(double low, double high, double x) {
  return (low <= x && x <= high);
};

void VirtualWall::init(const double& q, const double& dq) {
  if (initialized_) {
    return;
  }

  if (q < soft_lower_joint_position_limit_ ||
      q > soft_upper_joint_position_limit_) {
    std::stringstream ss;
    ss << "q " << q << " is beyond the joint wall: ["
       << soft_lower_joint_position_limit_ << ", "
       << soft_upper_joint_position_limit_ << "]";
    throw std::runtime_error(ss.str().c_str());
  }

  switch (getMotionInWall(q, dq)) {
    case MotionInWall::EnteringNormal:
      moving_wall_ = false;
      break;
    case MotionInWall::PenetratingLowerLimit:
    case MotionInWall::LeavingLowerLimit:
      zone_width_scale_ = fabs(q - soft_lower_joint_position_limit_) /
                          (PD_zone_width_ + D_zone_width_);
      moving_wall_ = true;
      break;
    case MotionInWall::PenetratingUpperLimit:
    case MotionInWall::LeavingUpperLimit:
      zone_width_scale_ = fabs(q - soft_upper_joint_position_limit_) /
                          (PD_zone_width_ + D_zone_width_);
      moving_wall_ = true;
      break;
  }
  initialized_ = true;
}

void VirtualWall::adjustMovingWall(const double& q, const double& dq) {
  if (!moving_wall_) {
    return;
  }
  double new_scale;
  switch (getMotionInWall(q, dq)) {
    case MotionInWall::EnteringNormal:
      moving_wall_ = false;
      zone_width_scale_ = 1;
      break;
    case MotionInWall::LeavingLowerLimit:
      new_scale = fabs(q - soft_lower_joint_position_limit_) /
                  (PD_zone_width_ + D_zone_width_);
      zone_width_scale_ = fmax(zone_width_scale_, new_scale);
      break;
    case MotionInWall::LeavingUpperLimit:
      new_scale = fabs(q - soft_upper_joint_position_limit_) /
                  (PD_zone_width_ + D_zone_width_);
      zone_width_scale_ = fmax(zone_width_scale_, new_scale);
      break;
    case MotionInWall::PenetratingLowerLimit:
    case MotionInWall::PenetratingUpperLimit:
      break;
  }
}

double VirtualWall::positiveCheck(double value) {
  if (value < 0) {
    printf(
        "ERROR: VirtualWall expects positive parameters, but got negative. Using "
        "its absolute "
        "value.");
    value = fabs(value);
  }
  return value;
}

VirtualWall::MotionInWall VirtualWall::getMotionInWall(const double& q,
                                                   const double& dq) const {
  double D_zone_boundary_max =
      soft_upper_joint_position_limit_ - PD_zone_width_ - D_zone_width_;
  double D_zone_boundary_min =
      soft_lower_joint_position_limit_ + PD_zone_width_ + D_zone_width_;
  if (q < D_zone_boundary_min) {
    if (dq <= 0) {
      return MotionInWall::PenetratingLowerLimit;
    }
    return MotionInWall::LeavingLowerLimit;
  }
  if (q > D_zone_boundary_max) {
    if (dq >= 0) {
      return MotionInWall::PenetratingUpperLimit;
    }
    return MotionInWall::LeavingUpperLimit;
  }
  return MotionInWall::EnteringNormal;
}
