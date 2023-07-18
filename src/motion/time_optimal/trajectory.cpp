/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "motion/time_optimal/trajectory.h"

#include <fstream>
#include <iostream>
#include <limits>

using namespace Eigen;
using namespace std;
using namespace motion::time_optimal;

constexpr double EPS = 0.000001;

// const double Trajectory::eps = 0.000001;

static double squared(double d) { return d * d; }

Trajectory::Trajectory(const Path& path, const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,
                       double time_step)
    : path_(path),
      max_velocity_(max_velocity),
      max_acceleration_(max_acceleration),
      joint_num_(max_velocity.size()),
      valid_(true),
      time_step_(time_step),
      cached_time_(std::numeric_limits<double>::max()) {
  py::gil_scoped_acquire acquire;
  py::object logging = py::module_::import("logging");
  logger_ = logging.attr("getLogger")("motion");
  py::gil_scoped_release release;
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
  while (valid_ && !integrateForward(trajectory_, after_acceleration) &&
         valid_) {
    double before_acceleration;
    TrajectoryStep switching_point;
    if (getNextSwitchingPoint(trajectory_.back().path_pos_, switching_point,
                              before_acceleration, after_acceleration)) {
      break;
    }
    integrateBackward(trajectory_, switching_point.path_pos_,
                      switching_point.path_vel_, before_acceleration);
  }

  if (valid_) {
    double before_acceleration =
        getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
    integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
  }

  if (valid_) {
    // Calculate timing
    std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
    std::list<TrajectoryStep>::iterator it = previous;
    it->time_ = 0.0;
    ++it;
    while (it != trajectory_.end()) {
      it->time_ =
          previous->time_ + (it->path_pos_ - previous->path_pos_) /
                                ((it->path_vel_ + previous->path_vel_) / 2.0);
      previous = it;
      ++it;
    }
  }
}

Trajectory::~Trajectory() {}

// Returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double path_pos,
                                       TrajectoryStep& next_switching_point,
                                       double& before_acceleration,
                                       double& after_acceleration) {
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration, acceleration_after_acceleration;
  bool acceleration_reached_end;
  do {
    acceleration_reached_end = getNextAccelerationSwitchingPoint(
        acceleration_switching_point.path_pos_, acceleration_switching_point,
        acceleration_before_acceleration, acceleration_after_acceleration);
  } while (
      !acceleration_reached_end &&
      acceleration_switching_point.path_vel_ >
          getVelocityMaxPathVelocity(acceleration_switching_point.path_pos_));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration, velocity_after_acceleration;
  bool velocity_reached_end;
  do {
    velocity_reached_end = getNextVelocitySwitchingPoint(
        velocity_switching_point.path_pos_, velocity_switching_point,
        velocity_before_acceleration, velocity_after_acceleration);
  } while (!velocity_reached_end &&
           velocity_switching_point.path_pos_ <=
               acceleration_switching_point.path_pos_ &&
           (velocity_switching_point.path_vel_ >
                getAccelerationMaxPathVelocity(
                    velocity_switching_point.path_pos_ - EPS) ||
            velocity_switching_point.path_vel_ >
                getAccelerationMaxPathVelocity(
                    velocity_switching_point.path_pos_ + EPS)));

  if (acceleration_reached_end && velocity_reached_end) {
    return true;
  } else if (!acceleration_reached_end &&
             (velocity_reached_end || acceleration_switching_point.path_pos_ <=
                                          velocity_switching_point.path_pos_)) {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  } else {
    next_switching_point = velocity_switching_point;
    before_acceleration = velocity_before_acceleration;
    after_acceleration = velocity_after_acceleration;
    return false;
  }
}

bool Trajectory::getNextAccelerationSwitchingPoint(
    double path_pos, TrajectoryStep& next_switching_point,
    double& before_acceleration, double& after_acceleration) {
  double switching_path_pos = path_pos;
  double switching_path_vel;
  while (true) {
    bool discontinuity;
    switching_path_pos =
        path_.getNextSwitchingPoint(switching_path_pos, discontinuity);

    if (switching_path_pos > path_.getLength() - EPS) {
      return true;
    }

    if (discontinuity) {
      const double before_path_vel =
          getAccelerationMaxPathVelocity(switching_path_pos - EPS);
      const double after_path_vel =
          getAccelerationMaxPathVelocity(switching_path_pos + EPS);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration = getMinMaxPathAcceleration(
          switching_path_pos - EPS, switching_path_vel, false);
      after_acceleration = getMinMaxPathAcceleration(switching_path_pos + EPS,
                                                     switching_path_vel, true);

      if ((before_path_vel > after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos - EPS, switching_path_vel,
                               false) >
               getAccelerationMaxPathVelocityDeriv(switching_path_pos -
                                                   2.0 * EPS)) &&
          (before_path_vel < after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos + EPS, switching_path_vel,
                               true) <
               getAccelerationMaxPathVelocityDeriv(switching_path_pos +
                                                   2.0 * EPS))) {
        break;
      }
    } else {
      switching_path_vel = getAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switching_path_pos - EPS) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + EPS) > 0.0) {
        break;
      }
    }
  }

  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(
    double path_pos, TrajectoryStep& next_switching_point,
    double& before_acceleration, double& after_acceleration) {
  const double step_size = 0.001;
  const double accuracy = 0.000001;

  bool start = false;
  path_pos -= step_size;
  do {
    path_pos += step_size;

    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos),
                            false) >=
        getVelocityMaxPathVelocityDeriv(path_pos)) {
      start = true;
    }
  } while ((!start || getMinMaxPhaseSlope(
                          path_pos, getVelocityMaxPathVelocity(path_pos),
                          false) > getVelocityMaxPathVelocityDeriv(path_pos)) &&
           path_pos < path_.getLength());

  if (path_pos >= path_.getLength()) {
    return true;  // end of trajectory reached
  }

  double before_path_pos = path_pos - step_size;
  double after_path_pos = path_pos;
  while (after_path_pos - before_path_pos > accuracy) {
    path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos),
                            false) >
        getVelocityMaxPathVelocityDeriv(path_pos)) {
      before_path_pos = path_pos;
    } else {
      after_path_pos = path_pos;
    }
  }

  before_acceleration = getMinMaxPathAcceleration(
      before_path_pos, getVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration = getMinMaxPathAcceleration(
      after_path_pos, getVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point = TrajectoryStep(
      after_path_pos, getVelocityMaxPathVelocity(after_path_pos));
  return false;
}

// Returns true if end of path is reached
bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory,
                                  double acceleration) {
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  std::list<std::pair<double, bool>> switching_points =
      path_.getSwitchingPoints();
  std::list<std::pair<double, bool>>::iterator next_discontinuity =
      switching_points.begin();

  while (true) {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos ||
            !next_discontinuity->second)) {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    path_vel += time_step_ * acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);

    if (next_discontinuity != switching_points.end() &&
        path_pos > next_discontinuity->first) {
      // Avoid having a TrajectoryStep with path_pos near a switching point
      // which will cause an almost identical TrajectoryStep get added in the
      // next run (https://github.com/ros-planning/moveit/issues/1665)
      if (path_pos - next_discontinuity->first < EPS) {
        continue;
      }
      path_vel = old_path_vel + (next_discontinuity->first - old_path_pos) *
                                    (path_vel - old_path_vel) /
                                    (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength()) {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    } else if (path_vel < 0.0) {
      valid_ = false;
      py::gil_scoped_acquire acquire;
      logger_.attr("debug")("Negative path velocity while integrating forward.");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos,
                            getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos)) {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) ||
        path_vel > getVelocityMaxPathVelocity(path_pos)) {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > EPS) {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before),
                                false) <=
                getVelocityMaxPathVelocityDeriv(before)) {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint)) {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        } else {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));

      if (getAccelerationMaxPathVelocity(after) <
          getVelocityMaxPathVelocity(after)) {
        if (after > next_discontinuity->first) {
          return false;
        } else if (getMinMaxPhaseSlope(trajectory.back().path_pos_,
                                       trajectory.back().path_vel_, true) >
                   getAccelerationMaxPathVelocityDeriv(
                       trajectory.back().path_pos_)) {
          return false;
        }
      } else {
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_,
                                trajectory_.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_)) {
          return false;
        }
      }
    }
  }
}

void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory,
                                   double path_pos, double path_vel,
                                   double acceleration) {
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  --start2;
  std::list<TrajectoryStep>::iterator start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope;
  assert(start1->path_pos_ <= path_pos);

  while (start1 != start_trajectory.begin() || path_pos >= 0.0) {
    if (start1->path_pos_ <= path_pos) {
      trajectory.push_front(TrajectoryStep(path_pos, path_vel));
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
      acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) /
              (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0) {
        valid_ = false;
        py::gil_scoped_acquire acquire;
        logger_.attr("debug")("Negative path velocity while integrating forward.");
        end_trajectory_ = trajectory;
        return;
      }
    } else {
      --start1;
      --start2;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double start_slope = (start2->path_vel_ - start1->path_vel_) /
                               (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos -
         start_slope * start1->path_pos_) /
        (slope - start_slope);
    if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
        intersection_path_pos <=
            EPS + std::min(start2->path_pos_, trajectory.front().path_pos_)) {
      const double intersection_path_vel =
          start1->path_vel_ +
          start_slope * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(
          TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  py::gil_scoped_acquire acquire;
  logger_.attr("debug")("Did not hit start trajectory while integrating backward.");
  end_trajectory_ = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double path_pos, double path_vel,
                                             bool max) {
  Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i) {
    if (config_deriv[i] != 0.0) {
      max_path_acceleration =
          std::min(max_path_acceleration,
                   max_acceleration_[i] / std::abs(config_deriv[i]) -
                       factor * config_deriv2[i] * path_vel * path_vel /
                           config_deriv[i]);
    }
  }
  return factor * max_path_acceleration;
}

double Trajectory::getMinMaxPhaseSlope(double path_pos, double path_vel,
                                       bool max) {
  return getMinMaxPathAcceleration(path_pos, path_vel, max) / path_vel;
}

double Trajectory::getAccelerationMaxPathVelocity(double path_pos) const {
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  for (unsigned int i = 0; i < joint_num_; ++i) {
    if (config_deriv[i] != 0.0) {
      for (unsigned int j = i + 1; j < joint_num_; ++j) {
        if (config_deriv[j] != 0.0) {
          double a_ij = config_deriv2[i] / config_deriv[i] -
                        config_deriv2[j] / config_deriv[j];
          if (a_ij != 0.0) {
            max_path_velocity = std::min(
                max_path_velocity,
                sqrt((max_acceleration_[i] / std::abs(config_deriv[i]) +
                      max_acceleration_[j] / std::abs(config_deriv[j])) /
                     std::abs(a_ij)));
          }
        }
      }
    } else if (config_deriv2[i] != 0.0) {
      max_path_velocity =
          std::min(max_path_velocity,
                   sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}

double Trajectory::getVelocityMaxPathVelocity(double path_pos) const {
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i) {
    max_path_velocity =
        std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double path_pos) {
  return (getAccelerationMaxPathVelocity(path_pos + EPS) -
          getAccelerationMaxPathVelocity(path_pos - EPS)) /
         (2.0 * EPS);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double path_pos) {
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint;
  for (unsigned int i = 0; i < joint_num_; ++i) {
    const double this_max_path_velocity =
        max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity) {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint] *
           path_.getCurvature(path_pos)[active_constraint]) /
         (tangent[active_constraint] * std::abs(tangent[active_constraint]));
}

bool Trajectory::isValid() const { return valid_; }

double Trajectory::getDuration() const { return trajectory_.back().time_; }

std::list<Trajectory::TrajectoryStep>::const_iterator
Trajectory::getTrajectorySegment(double time) const {
  if (time >= trajectory_.back().time_) {
    std::list<TrajectoryStep>::const_iterator last = trajectory_.end();
    last--;
    return last;
  } else {
    if (time < cached_time_) {
      cached_trajectory_segment_ = trajectory_.begin();
    }
    while (time >= cached_trajectory_segment_->time_) {
      ++cached_trajectory_segment_;
    }
    cached_time_ = time;
    return cached_trajectory_segment_;
  }
}

size_t Trajectory::getTrajectorySegmentIndex(double time) {
  // if (time >= trajectory_.back().time_) {
  //   return path_.section_lengths.size() - 1;
  // }
  size_t idx = 0;
  auto t = getTrajectorySegment(time);
  for (auto l : path_.section_lengths) {
    if (t->path_pos_ <= l) {
      break;
    }
    idx++;
  }
  return std::min(path_.section_lengths.size() - 1, idx);
}

Eigen::VectorXd Trajectory::getPosition(double time) const {
  if (time > trajectory_.back().time_) {
    return getPosition(trajectory_.back().time_);
  }
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 *
      (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) /
      (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos = previous->path_pos_ +
                          time_step * previous->path_vel_ +
                          0.5 * time_step * time_step * acceleration;

  return path_.getConfig(path_pos);
}

Eigen::VectorXd Trajectory::getVelocity(double time) const {
  if (time > trajectory_.back().time_) {
    return getVelocity(trajectory_.back().time_);
  }
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 *
      (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) /
      (time_step * time_step);

  const double path_pos = previous->path_pos_ +
                          time_step * previous->path_vel_ +
                          0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;

  return path_.getTangent(path_pos) * path_vel;
}

Eigen::VectorXd Trajectory::getAcceleration(double time) const {
  if (time > trajectory_.back().time_) {
    return getAcceleration(trajectory_.back().time_);
  }
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 *
      (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) /
      (time_step * time_step);

  const double path_pos = previous->path_pos_ +
                          time_step * previous->path_vel_ +
                          0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;
  Eigen::VectorXd path_acc =
      (path_.getTangent(path_pos) * path_vel -
       path_.getTangent(previous->path_pos_) * previous->path_vel_);
  if (time_step > 0.0) path_acc /= time_step;
  return path_acc;
}
