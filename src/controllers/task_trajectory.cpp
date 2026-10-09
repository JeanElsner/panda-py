#include "controllers/task_trajectory.h"

using namespace controllers;

namespace {

Vector6d diagonal(const Eigen::Matrix<double, 6, 6>& impedance) {
  Eigen::Matrix<double, 6, 6> off = impedance;
  off.diagonal().setZero();
  if (!off.isZero()) {
    throw std::invalid_argument(
        "The impedance must be diagonal: TaskImpedance has diagonal stiffness.");
  }
  return impedance.diagonal();
}

}  // namespace

const double TaskTrajectory::kDefaultDqThreshold = 1e-3;
const double TaskTrajectory::kSettlePositionTolerance = 2e-3;
const double TaskTrajectory::kSettleOrientationTolerance = 2e-3;
const double TaskTrajectory::kSettleTimeout = 1.0;
const double TaskTrajectory::kDefaultNullspaceStiffness = 15.0;
const double TaskTrajectory::kDefaultDampingRatio = 1.0;
// clang-format off
double _data[36] = {800,   0,   0,  0,  0,  0,
                      0, 800,   0,  0,  0,  0,
                      0,   0, 800,  0,  0,  0,
                      0,   0,   0, 20,  0,  0,
                      0,   0,   0,  0, 20,  0,
                      0,   0,   0,  0,  0, 20};
// clang-format on
// The rotational stiffness was 40 until 2.0.0, acting on the vector part of the
// error quaternion, which is about half the rotation angle. TaskImpedance acts
// on the angle itself, so 20 keeps the same stiffness.
const Eigen::Matrix<double, 6, 6> TaskTrajectory::kDefaultImpedance =
    Eigen::Matrix<double, 6, 6>(_data);

TaskTrajectory::TaskTrajectory(
    std::shared_ptr<motion::CartesianTrajectory> trajectory,
    const Vector7d& q_init, const Eigen::Matrix<double, 6, 6>& impedance,
    const double& damping_ratio, const double& nullspace_stiffness,
    const double dq_threshold)
    : TaskImpedance(diagonal(impedance), damping_ratio,
                    task_impedance::Nullspace::kKinematic, nullspace_stiffness,
                    TaskImpedance::Frame::kEndEffector,
                    Eigen::Matrix4d::Identity(), true),
      traj_(trajectory),
      dq_threshold_(dq_threshold),
      q_init_(q_init) {}

void TaskTrajectory::prepare(const franka::RobotState& robot_state) {
  TaskImpedance::prepare(robot_state);
  holdReference(traj_->getPosition(getTime()),
                Eigen::Quaterniond(traj_->getOrientation(getTime())));
}

bool TaskTrajectory::sync(const franka::RobotState& robot_state, double time) {
  const bool updated = TaskImpedance::sync(robot_state, time);
  holdNullspaceTarget(q_init_);
  return updated;
}

bool TaskTrajectory::finished(const franka::RobotState& robot_state) {
  const double overrun = getTime() - traj_->getDuration();
  if (overrun <= 0.0) {
    return false;
  }
  const bool at_rest =
      Eigen::Map<const Vector7d>(robot_state.dq.data()).cwiseAbs().maxCoeff() <=
      dq_threshold_;
  // Being at rest is not enough on its own: without an integral term the
  // controller comes to rest wherever the stiffness balances the residual
  // error, which can be well short of the goal.
  const Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  const double position_error =
      (traj_->getPosition(traj_->getDuration()) - transform.translation()).norm();
  const Eigen::Quaterniond orientation(transform.rotation());
  Eigen::Quaterniond orientation_goal(traj_->getOrientation(traj_->getDuration()));
  if (orientation_goal.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation_goal.coeffs() << -orientation_goal.coeffs();
  }
  const double orientation_error = orientation.angularDistance(orientation_goal);
  const bool at_goal = position_error <= kSettlePositionTolerance &&
                       orientation_error <= kSettleOrientationTolerance;
  return (at_rest && at_goal) || overrun >= kSettleTimeout;
}
