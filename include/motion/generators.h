#ifndef PARABOLIC_BLEND_SMOOTHER_H_
#define PARABOLIC_BLEND_SMOOTHER_H_

#include <pybind11/pybind11.h>

#include <Eigen/Dense>
#include <list>
#include <memory>
#include <vector>

#include "kinematics/ik.h"
#include "motion/time_optimal/trajectory.h"
#include "utils.h"

namespace py = pybind11;
namespace motion {

const double kDefaultTimeout = 30.0;
const double kDefaultJointSpeedFactor = 0.2;
const double kDefaultCartesianSpeedFactor = 0.2;

class PandaTrajectory {
 public:
  double getDuration() { return traj_->getDuration(); }

  virtual Vector7d getJointPositions(double time,
                                     const Vector7d &q = kinematics::kQDefault,
                                     double q7 = M_PI_4) = 0;

  virtual Vector7d getJointVelocities(double time,
                                      const Vector7d &q = kinematics::kQDefault,
                                      double q7 = M_PI_4) = 0;

  virtual Vector7d getJointAccelerations(
      double time, const Vector7d &q = kinematics::kQDefault,
      double q7 = M_PI_4) = 0;

 protected:
  bool _computeTrajectory(const time_optimal::Path &path,
                          const Eigen::VectorXd &max_velocity,
                          const Eigen::VectorXd &max_acceleration,
                          double timeout);

  template <typename... Args>
  void _log(const std::string level, Args &&...args) {
    py::gil_scoped_acquire acquire;
    logger_.attr(level.c_str())(args...);
  }

  py::object logger_;
  std::shared_ptr<time_optimal::Trajectory> traj_;
};

class JointTrajectory : public PandaTrajectory {
 public:
  JointTrajectory(const std::vector<Vector7d> &waypoints,
                  double speed_factor = kDefaultJointSpeedFactor,
                  double maxDeviation = 0.0, double timeout = kDefaultTimeout);

  Vector7d getJointPositions(double time,
                             const Vector7d &q = kinematics::kQDefault,
                             double q7 = M_PI_4) override;

  Vector7d getJointVelocities(double time,
                              const Vector7d &q = kinematics::kQDefault,
                              double q7 = M_PI_4) override;

  Vector7d getJointAccelerations(double time,
                                 const Vector7d &q = kinematics::kQDefault,
                                 double q7 = M_PI_4) override;

 private:
  time_optimal::Path _convertList(const std::vector<Vector7d> &list,
                                  double maxDeviation = 0.0);
};

class CartesianTrajectory : public PandaTrajectory {
 public:
  CartesianTrajectory(
      const std::vector<Eigen::Matrix<double, 3, 1>> &positions,
      const std::vector<Eigen::Matrix<double, 4, 1>> &orientations,
      double speed_factor = kDefaultCartesianSpeedFactor,
      double maxDeviation = 0.0, double timeout = kDefaultTimeout);

  CartesianTrajectory(const std::vector<Eigen::Matrix<double, 4, 4>> &poses,
                      double speed_factor = kDefaultCartesianSpeedFactor,
                      double maxDeviation = 0.0,
                      double timeout = kDefaultTimeout);

  Eigen::Matrix<double, 4, 4> getPose(double time);

  Eigen::Vector3d getPosition(double time);

  Eigen::Quaterniond getOrientation(double time);

  Vector7d getJointPositions(double time,
                             const Vector7d &q = kinematics::kQDefault,
                             double q7 = M_PI_4) override;

  Vector7d getJointVelocities(double time,
                              const Vector7d &q = kinematics::kQDefault,
                              double q7 = M_PI_4) override;

  Vector7d getJointAccelerations(double time,
                                 const Vector7d &q = kinematics::kQDefault,
                                 double q7 = M_PI_4) override;

 private:
  std::vector<double> angles_;
  std::vector<Eigen::Vector3d> axes_;
  std::vector<Eigen::Quaterniond> orientations_;
};

}  // namespace motion

#endif