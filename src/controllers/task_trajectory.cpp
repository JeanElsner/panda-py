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
const double TaskTrajectory::kSettlePositionGain = 2000.0;
const double TaskTrajectory::kSettleOrientationGain = 50.0;
const double TaskTrajectory::kSettleForceLimit = 5.0;
const double TaskTrajectory::kSettleTorqueLimit = 0.5;
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

void TaskTrajectory::begin(const franka::RobotState& robot_state) {
  TaskImpedance::begin(robot_state);
  settle_.setZero();
}

void TaskTrajectory::onRearm(const franka::RobotState& robot_state) {
  TaskImpedance::onRearm(robot_state);
  settle_.setZero();
}

Vector6d TaskTrajectory::goalError(const franka::RobotState& robot_state) const {
  const Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  const Eigen::Quaterniond orientation(transform.rotation());
  const Eigen::Quaterniond goal(traj_->getOrientation(traj_->getDuration()));
  const Eigen::AngleAxisd rotation(goal * orientation.inverse());
  double angle = rotation.angle();
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  Vector6d error;
  error.head<3>() = traj_->getPosition(traj_->getDuration()) - transform.translation();
  error.tail<3>() = angle * rotation.axis();
  return error;
}

Vector7d TaskTrajectory::law(const franka::RobotState& robot_state, double dt,
                             bool tripped) {
  const Vector7d tau = TaskImpedance::law(robot_state, dt, tripped);
  if (tripped || getTime() <= traj_->getDuration()) {
    return tau;
  }
  const Vector6d error = goalError(robot_state);
  if (error.head<3>().norm() > kSettlePositionTolerance) {
    settle_.head<3>() += kSettlePositionGain * error.head<3>() * dt;
    const double norm = settle_.head<3>().norm();
    if (norm > kSettleForceLimit) settle_.head<3>() *= kSettleForceLimit / norm;
  }
  if (error.tail<3>().norm() > kSettleOrientationTolerance) {
    settle_.tail<3>() += kSettleOrientationGain * error.tail<3>() * dt;
    const double norm = settle_.tail<3>().norm();
    if (norm > kSettleTorqueLimit) settle_.tail<3>() *= kSettleTorqueLimit / norm;
  }
  const Eigen::Matrix<double, 6, 7> jacobian(
      model_->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
  return tau + jacobian.transpose() * settle_;
}

bool TaskTrajectory::finished(const franka::RobotState& robot_state) {
  const double overrun = getTime() - traj_->getDuration();
  if (overrun <= 0.0) {
    return false;
  }
  const bool at_rest =
      Eigen::Map<const Vector7d>(robot_state.dq.data()).cwiseAbs().maxCoeff() <=
      dq_threshold_;
  // Being at rest is not enough on its own: before the integral term has
  // built up, the robot can rest short of the goal.
  const Vector6d error = goalError(robot_state);
  const bool at_goal = error.head<3>().norm() <= kSettlePositionTolerance &&
                       error.tail<3>().norm() <= kSettleOrientationTolerance;
  return (at_rest && at_goal) || overrun >= kSettleTimeout;
}
