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

#include "motion/time_optimal/path.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <numeric>

using namespace motion::time_optimal;

class LinearPathSegment : public PathSegment {
 public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
      : PathSegment((end - start).norm()), end_(end), start_(start) {}

  Eigen::VectorXd getConfig(double s) const override {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  Eigen::VectorXd getTangent(double /* s */) const override {
    return (end_ - start_) / length_;
  }

  Eigen::VectorXd getCurvature(double /* s */) const override {
    return Eigen::VectorXd::Zero(start_.size());
  }

  std::list<double> getSwitchingPoints() const override {
    return std::list<double>();
  }

  LinearPathSegment* clone() const override {
    return new LinearPathSegment(*this);
  }

 private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

class CircularPathSegment : public PathSegment {
 public:
  CircularPathSegment(const Eigen::VectorXd& start,
                      const Eigen::VectorXd& intersection,
                      const Eigen::VectorXd& end, double max_deviation) {
    if ((intersection - start).norm() < 0.000001 ||
        (end - intersection).norm() < 0.000001) {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const Eigen::VectorXd start_direction = (intersection - start).normalized();
    const Eigen::VectorXd end_direction = (end - intersection).normalized();
    const double start_dot_end = start_direction.dot(end_direction);

    // catch division by 0 in computations below
    if (start_dot_end > 0.999999 || start_dot_end < -0.999999) {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const double angle = acos(start_dot_end);
    const double start_distance = (start - intersection).norm();
    const double end_distance = (end - intersection).norm();

    // enforce max deviation
    double distance = std::min(start_distance, end_distance);
    distance = std::min(
        distance, max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));

    radius = distance / tan(0.5 * angle);
    length_ = angle * radius;

    center = intersection + (end_direction - start_direction).normalized() *
                                radius / cos(0.5 * angle);
    x = (intersection - distance * start_direction - center).normalized();
    y = start_direction;
  }

  Eigen::VectorXd getConfig(double s) const override {
    const double angle = s / radius;
    return center + radius * (x * cos(angle) + y * sin(angle));
  }

  Eigen::VectorXd getTangent(double s) const override {
    const double angle = s / radius;
    return -x * sin(angle) + y * cos(angle);
  }

  Eigen::VectorXd getCurvature(double s) const override {
    const double angle = s / radius;
    return -1.0 / radius * (x * cos(angle) + y * sin(angle));
  }

  std::list<double> getSwitchingPoints() const override {
    std::list<double> switching_points;
    const double dim = x.size();
    for (unsigned int i = 0; i < dim; ++i) {
      double switching_angle = atan2(y[i], x[i]);
      if (switching_angle < 0.0) {
        switching_angle += M_PI;
      }
      const double switching_point = switching_angle * radius;
      if (switching_point < length_) {
        switching_points.push_back(switching_point);
      }
    }
    switching_points.sort();
    return switching_points;
  }

  CircularPathSegment* clone() const override {
    return new CircularPathSegment(*this);
  }

 private:
  double radius;
  Eigen::VectorXd center;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double max_deviation)
    : length_(0.0) {
  if (path.size() < 2) return;
  std::list<Eigen::VectorXd>::const_iterator path_iterator1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  while (path_iterator2 != path.end()) {
    double section_length = 0.0;
    path_iterator3 = path_iterator2;
    ++path_iterator3;
    if (max_deviation > 0.0 && path_iterator3 != path.end()) {
      CircularPathSegment* blend_segment = new CircularPathSegment(
          0.5 * (*path_iterator1 + *path_iterator2), *path_iterator2,
          0.5 * (*path_iterator2 + *path_iterator3), max_deviation);
      Eigen::VectorXd end_config = blend_segment->getConfig(0.0);
      if ((end_config - start_config).norm() > 0.000001) {
        path_segments_.push_back(
            std::make_unique<LinearPathSegment>(start_config, end_config));
        section_length += path_segments_.back()->getLength();
      }
      path_segments_.emplace_back(blend_segment);
      section_length += path_segments_.back()->getLength()/2.0;
      start_config = blend_segment->getConfig(blend_segment->getLength());
    } else {
      path_segments_.push_back(
          std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      start_config = *path_iterator2;
      section_length = path_segments_.back()->getLength();
    }
    path_iterator1 = path_iterator2;
    ++path_iterator2;
    section_lengths.push_back(section_length);
  }
  std::partial_sum(section_lengths.begin(), section_lengths.end(), section_lengths.begin());

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::unique_ptr<PathSegment>& path_segment : path_segments_) {
    path_segment->position_ = length_;
    std::list<double> local_switching_points =
        path_segment->getSwitchingPoints();
    for (std::list<double>::const_iterator point =
             local_switching_points.begin();
         point != local_switching_points.end(); ++point) {
      switching_points_.push_back(std::make_pair(length_ + *point, false));
    }
    length_ += path_segment->getLength();
    while (!switching_points_.empty() &&
           switching_points_.back().first >= length_)
      switching_points_.pop_back();
    switching_points_.push_back(std::make_pair(length_, true));
  }
  switching_points_.pop_back();
}

Path::Path(const Path& path)
    : length_(path.length_), switching_points_(path.switching_points_) {
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_) {
    path_segments_.emplace_back(path_segment->clone());
  }
  section_lengths = path.section_lengths;
}

double Path::getLength() const { return length_; }

PathSegment* Path::getPathSegment(double& s) const {
  std::list<std::unique_ptr<PathSegment>>::const_iterator it =
      path_segments_.begin();
  std::list<std::unique_ptr<PathSegment>>::const_iterator next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_) {
    it = next;
    ++next;
  }
  s -= (*it)->position_;
  return (*it).get();
}

Eigen::VectorXd Path::getConfig(double s) const {
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const {
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const {
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const {
  std::list<std::pair<double, bool>>::const_iterator it =
      switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s) {
    ++it;
  }
  if (it == switching_points_.end()) {
    discontinuity = true;
    return length_;
  }
  discontinuity = it->second;
  return it->first;
}

std::list<std::pair<double, bool>> Path::getSwitchingPoints() const {
  return switching_points_;
}