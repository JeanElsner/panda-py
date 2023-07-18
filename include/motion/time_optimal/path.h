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

#pragma once

#include <Eigen/Core>
#include <list>
#include <memory>

namespace motion {
namespace time_optimal {

class PathSegment {
 public:
  PathSegment(double length = 0.0) : length_(length) {}
  virtual ~PathSegment()  // is required for destructing derived classes
  {}
  double getLength() const { return length_; }
  virtual Eigen::VectorXd getConfig(double s) const = 0;
  virtual Eigen::VectorXd getTangent(double s) const = 0;
  virtual Eigen::VectorXd getCurvature(double s) const = 0;
  virtual std::list<double> getSwitchingPoints() const = 0;
  virtual PathSegment* clone() const = 0;

  double position_;

 protected:
  double length_;
};

class Path {
 public:
  Path(const std::list<Eigen::VectorXd>& path, double max_deviation = 0.0);
  Path(const Path& path);
  double getLength() const;
  Eigen::VectorXd getConfig(double s) const;
  Eigen::VectorXd getTangent(double s) const;
  Eigen::VectorXd getCurvature(double s) const;

  /** @brief Get the next switching point.
   *  @param[in] s Arc length traveled so far
   *  @param[out] discontinuity True if this switching point is a discontinuity
   *  @return arc length to the switching point
   **/
  double getNextSwitchingPoint(double s, bool& discontinuity) const;

  /// @brief Return a list of all switching points as a pair (arc length to
  /// switching point, discontinuity)
  std::list<std::pair<double, bool>> getSwitchingPoints() const;

  std::list<double> section_lengths;

 private:
  PathSegment* getPathSegment(double& s) const;
  double length_;
  std::list<std::pair<double, bool>> switching_points_;
  std::list<std::unique_ptr<PathSegment>> path_segments_;
};

}  // namespace time_optimal
}  // namespace motion
