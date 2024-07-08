#include "motion/generators.h"

#include <chrono>
#include <iostream>
#include <numeric>

#include "constants.h"

using namespace std;
using namespace Eigen;
using namespace motion;

bool PandaTrajectory::_computeTrajectory(
    const time_optimal::Path &path, const Eigen::VectorXd &max_velocity,
    const Eigen::VectorXd &max_acceleration, double timeout) {
  auto startTime = std::chrono::high_resolution_clock::now();
  bool success = false;
  int i = 0;
  while (true) {
    i++;
    traj_ = std::make_shared<time_optimal::Trajectory>(path, max_velocity,
                                                       max_acceleration, 1e-3);
    if (traj_->isValid() && !isnan(traj_->getDuration())) {
      success = true;
      break;
    }
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        currentTime - startTime)
                        .count();
    if (duration >= timeout) {
      _log("error", "Trajectory computation timed out after %d seconds.",
           duration);
      break;
    }
    _log("debug", "Reattempting trajectory computation. Attempt no. %d.", i);
  }
  return success;
}

JointTrajectory::JointTrajectory(const std::vector<Vector7d> &waypoints,
                                 double speed_factor, double maxDeviation,
                                 double timeout) {
  py::gil_scoped_acquire acquire;
  py::object logging = py::module_::import("logging");
  logger_ = logging.attr("getLogger")("motion");
  py::gil_scoped_release release;

  if (!_computeTrajectory(_convertList(waypoints, maxDeviation),
                          speed_factor * kQMaxVelocity,
                          speed_factor * kQMaxAcceleration, timeout)) {
    throw runtime_error("Trajectory generation faild.");
  }

  if (waypoints.size() == 2) {
    _log("info",
         "Computed joint trajectory: 1 waypoint, duration %.2f seconds.",
         traj_->getDuration());
  } else {
    _log("info",
         "Computed joint trajectory: %d waypoints, duration %.2f seconds.",
         waypoints.size() - 1, traj_->getDuration());
  }
}

time_optimal::Path JointTrajectory::_convertList(
    const std::vector<Vector7d> &list, double maxDeviation) {
  std::list<Eigen::VectorXd> new_list;
  for (auto l : list) {
    new_list.push_back(Eigen::Map<Eigen::VectorXd>(l.data(), 7, 1));
  }
  return time_optimal::Path(new_list, maxDeviation);
}

Vector7d JointTrajectory::getJointPositions(double time) {
  return traj_->getPosition(time);
}

Vector7d JointTrajectory::getJointVelocities(double time) {
  return traj_->getVelocity(time);
}

Vector7d JointTrajectory::getJointAccelerations(double time) {
  return traj_->getAcceleration(time);
}

CartesianTrajectory::CartesianTrajectory(
    const std::vector<Eigen::Matrix<double, 4, 4>> &poses, double speed_factor,
    double maxDeviation, double timeout) {
  std::vector<Eigen::Matrix<double, 3, 1>> positions;
  std::vector<Eigen::Matrix<double, 4, 1>> orientations;
  for (auto p : poses) {
    positions.push_back(MatrixToPosition(p));
    orientations.push_back(MatrixToOrientation(p));
  }
  CartesianTrajectory(positions, orientations, speed_factor, maxDeviation,
                      timeout);
}

CartesianTrajectory::CartesianTrajectory(
    const std::vector<Eigen::Matrix<double, 3, 1>> &positions,
    const std::vector<Eigen::Matrix<double, 4, 1>> &orientations,
    double speed_factor, double maxDeviation, double timeout) {
  py::gil_scoped_acquire acquire;
  py::object logging = py::module_::import("logging");
  logger_ = logging.attr("getLogger")("motion");
  py::gil_scoped_release release;
  angles_.push_back(0);

  for (size_t i = 0; i < orientations.size() - 1; i++) {
    Eigen::Quaterniond q1(orientations.at(i)), q2(orientations.at(i + 1));
    Eigen::AngleAxisd aa(q2 * q1.inverse());
    double angle = aa.angle();
    Eigen::Vector3d axis = aa.axis();
    angles_.push_back(angle);
    axes_.push_back(axis);
    orientations_.push_back(q1);
  }
  orientations_.push_back(Eigen::Quaterniond(orientations.back()));
  std::partial_sum(angles_.begin(), angles_.end(), angles_.begin());
  std::list<Eigen::VectorXd> waypoints;

  for (size_t i = 0; i < orientations.size(); i++) {
    Eigen::VectorXd point;
    point.resize(4);
    point.head(3) = positions.at(i);
    point.coeffRef(3) = angles_.at(i);
    waypoints.push_back(point);
  }

  if (!_computeTrajectory(time_optimal::Path(waypoints, maxDeviation),
                          speed_factor * kXMaxVelocity,
                          speed_factor * kXMaxAcceleration, timeout)) {
    throw runtime_error("Trajectory generation failed.");
  }

  if (orientations.size() == 2) {
    _log("info",
         "Computed Cartesian trajectory: 1 waypoint, duration %.2f seconds.",
         traj_->getDuration());
  } else {
    _log("info",
         "Computed Cartesian trajectory: %d waypoints, duration %.2f seconds.",
         orientations.size() - 1, traj_->getDuration());
  }
}

Eigen::Matrix<double, 4, 4> CartesianTrajectory::getPose(double time) {
  auto pose = traj_->getPosition(time);
  size_t idx = traj_->getTrajectorySegmentIndex(time);
  double angle = pose.coeff(3) - angles_.at(idx);
  Eigen::AngleAxisd aa(angle, axes_.at(idx));
  Eigen::Quaterniond o = Eigen::Quaterniond(aa) * orientations_.at(idx);

  Eigen::Affine3d transform;
  transform = o;
  transform.translation() = pose.head(3);

  return transform.matrix();
}

Eigen::Vector3d CartesianTrajectory::getPosition(double time) {
  auto pose = traj_->getPosition(time);
  return pose.head(3);
}

Eigen::Vector4d CartesianTrajectory::getOrientation(double time) {
  auto pose = traj_->getPosition(time);
  size_t idx = traj_->getTrajectorySegmentIndex(time);
  double angle = pose.coeff(3) - angles_.at(idx);
  Eigen::AngleAxisd aa(angle, axes_.at(idx));
  Eigen::Quaterniond o = Eigen::Quaterniond(aa) * orientations_.at(idx);
  return o.coeffs();
}
