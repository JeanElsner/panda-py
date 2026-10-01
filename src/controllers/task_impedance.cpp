#include "controllers/task_impedance.h"

#include <cmath>

#include "panda.h"

namespace task_impedance {

Eigen::Vector3d orientationError(const Eigen::Quaterniond& orientation_ref,
                                 const Eigen::Quaterniond& orientation) {
  Eigen::Quaterniond error = orientation_ref * orientation.inverse();
  // q and -q are the same rotation; the one with w >= 0 has the angle in
  // [0, pi], the shorter way round.
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  const Eigen::Vector3d v = error.vec();
  const double n = v.norm();
  // angle = 2 atan2(|v|, w); the axis-angle vector tends to 2 v as |v| -> 0.
  if (n < 1e-12) {
    return 2.0 * v;
  }
  return v * (2.0 * std::atan2(n, error.w()) / n);
}

Eigen::Matrix<double, 6, 7> shiftJacobian(
    const Eigen::Matrix<double, 6, 7>& jacobian,
    const Eigen::Vector3d& offset) {
  Eigen::Matrix3d skew;
  skew << 0, -offset.z(), offset.y(), offset.z(), 0, -offset.x(), -offset.y(),
      offset.x(), 0;
  Eigen::Matrix<double, 6, 7> shifted = jacobian;
  // v + w x r = v - r x w
  shifted.topRows<3>() -= skew * jacobian.bottomRows<3>();
  return shifted;
}

Vector6d criticalDamping(const Vector6d& stiffness, double damping_ratio) {
  return 2.0 * damping_ratio * stiffness.cwiseMax(0.0).cwiseSqrt();
}

namespace {

Eigen::Matrix<double, 6, 6> regularised(const Eigen::Matrix<double, 6, 6>& a,
                                        double damping) {
  if (damping == 0.0) {
    return a;
  }
  const double lambda = damping * std::max(a.diagonal().mean(), 1e-12);
  return a + lambda * Eigen::Matrix<double, 6, 6>::Identity();
}

}  // namespace

Outputs compute(const Inputs& in) {
  Outputs out;
  const Eigen::Affine3d transform(in.pose);
  const Eigen::Quaterniond orientation(transform.rotation());
  out.error.head<3>() = in.position_ref - transform.translation();
  out.error.tail<3>() = orientationError(in.orientation_ref, orientation);
  out.velocity = in.jacobian * in.dq;
  out.wrench_active = in.stiffness.cwiseProduct(out.error);
  out.wrench_passive = -in.damping.cwiseProduct(out.velocity);
  out.tau_task = in.jacobian.transpose() *
                 (in.alpha * out.wrench_active + out.wrench_passive);

  out.tau_nullspace.setZero();
  const double k = in.nullspace_stiffness;
  if (in.nullspace != Nullspace::kNone && k != 0.0) {
    const Vector7d u =
        k * (in.q_nullspace - in.q) - 2.0 * std::sqrt(std::max(k, 0.0)) * in.dq;
    const auto& J = in.jacobian;
    const Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    if (in.nullspace == Nullspace::kKinematic) {
      const Eigen::Matrix<double, 6, 6> jjt =
          regularised(J * J.transpose(), in.nullspace_damping);
      const Eigen::Matrix<double, 7, 7> N =
          I - J.transpose() * jjt.ldlt().solve(J);
      out.tau_nullspace = N * u;
    } else {
      const auto mass = in.mass.ldlt();
      const Eigen::Matrix<double, 7, 6> mi_jt = mass.solve(J.transpose());
      const Eigen::Matrix<double, 6, 6> lambda_inv =
          regularised(J * mi_jt, in.nullspace_damping);
      // N = I - J^T (J M^-1 J^T)^-1 J M^-1, with J M^-1 = (M^-1 J^T)^T
      const Eigen::Matrix<double, 7, 7> N =
          I - J.transpose() * lambda_inv.ldlt().solve(mi_jt.transpose());
      out.tau_nullspace = N * (in.mass * u);
    }
  }
  out.tau = out.tau_task + out.tau_nullspace + in.coriolis;
  return out;
}

}  // namespace task_impedance

using task_impedance::Nullspace;

const Vector6d TaskImpedance::kDefaultStiffness =
    (Vector6d() << 600, 600, 600, 30, 30, 30).finished();
const double TaskImpedance::kDefaultDampingRatio = 1.0;
const double TaskImpedance::kDefaultNullspaceStiffness = 10.0;
const Nullspace TaskImpedance::kDefaultNullspace = Nullspace::kDynamic;
const TaskImpedance::Frame TaskImpedance::kDefaultFrame =
    TaskImpedance::Frame::kEndEffector;

TaskImpedance::TaskImpedance(const Vector6d& stiffness, double damping_ratio,
                             Nullspace nullspace, double nullspace_stiffness,
                             Frame frame,
                             const Eigen::Matrix4d& frame_transform,
                             bool coriolis, double nullspace_damping)
    : frame_(frame),
      frame_transform_(frame_transform),
      coriolis_(coriolis),
      nullspace_(nullspace),
      nullspace_damping_(nullspace_damping),
      stiffness_(stiffness),
      damping_(task_impedance::criticalDamping(stiffness, damping_ratio)),
      damping_ratio_(damping_ratio),
      nullspace_stiffness_(nullspace_stiffness),
      position_ref_(Eigen::Vector3d::Zero()),
      orientation_ref_(Eigen::Quaterniond::Identity()),
      q_nullspace_(kJointPositionStart),
      motion_finished_(false) {
  if (!frame_transform.allFinite() ||
      !frame_transform.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)) ||
      !(frame_transform.topLeftCorner<3, 3>() *
        frame_transform.topLeftCorner<3, 3>().transpose())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    throw std::invalid_argument(
        "The frame transform must be a homogeneous transform.");
  }
}

void TaskImpedance::controlFrame(const franka::RobotState& robot_state,
                                 franka::Model& model, Eigen::Matrix4d& pose,
                                 Eigen::Matrix<double, 6, 7>& jacobian) const {
  const franka::Frame base = frame_ == Frame::kFlange
                                 ? franka::Frame::kFlange
                                 : franka::Frame::kEndEffector;
  const Eigen::Matrix4d base_pose =
      Eigen::Matrix4d::Map(model.pose(base, robot_state).data());
  const auto base_jacobian = model.zeroJacobian(base, robot_state);
  pose = base_pose * frame_transform_;
  jacobian = task_impedance::shiftJacobian(
      Eigen::Map<const Eigen::Matrix<double, 6, 7>>(base_jacobian.data()),
      base_pose.topLeftCorner<3, 3>() * frame_transform_.topRightCorner<3, 1>());
}

task_impedance::Inputs TaskImpedance::inputs(
    const franka::RobotState& robot_state) {
  task_impedance::Inputs in;
  {
    std::lock_guard<std::mutex> lock(mux_);
    in.position_ref = position_ref_;
    in.orientation_ref = orientation_ref_;
    in.stiffness = stiffness_;
    in.damping = damping_;
    in.q_nullspace = q_nullspace_;
    in.nullspace_stiffness = nullspace_stiffness_;
  }
  in.nullspace = nullspace_;
  in.nullspace_damping = nullspace_damping_;
  in.q = Eigen::Map<const Vector7d>(robot_state.q.data());
  in.dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  controlFrame(robot_state, *model_, in.pose, in.jacobian);
  if (nullspace_ == Nullspace::kDynamic) {
    in.mass =
        Eigen::Map<const Eigen::Matrix<double, 7, 7>>(model_->mass(robot_state).data());
  }
  if (coriolis_) {
    in.coriolis = Eigen::Map<const Vector7d>(model_->coriolis(robot_state).data());
  }
  return in;
}

franka::Torques TaskImpedance::step(const franka::RobotState& robot_state,
                                    franka::Duration& duration) {
  const auto out = task_impedance::compute(inputs(robot_state));
  franka::Torques torques = VectorToArray<7>(out.tau);
  torques.motion_finished = motion_finished_;
  return torques;
}

void TaskImpedance::start(const franka::RobotState& robot_state,
                          std::shared_ptr<franka::Model> model) {
  motion_finished_ = false;
  model_ = model;
  Eigen::Matrix4d pose;
  Eigen::Matrix<double, 6, 7> jacobian;
  controlFrame(robot_state, *model_, pose, jacobian);
  const Eigen::Affine3d transform(pose);
  std::lock_guard<std::mutex> lock(mux_);
  // Hold where the robot is, with zero active wrench.
  position_ref_ = transform.translation();
  orientation_ref_ = Eigen::Quaterniond(transform.rotation());
  q_nullspace_ = Eigen::Map<const Vector7d>(robot_state.q.data());
}

void TaskImpedance::stop(const franka::RobotState& robot_state,
                         std::shared_ptr<franka::Model> model) {
  motion_finished_ = true;
}

bool TaskImpedance::isRunning() { return !motion_finished_; }

const std::string TaskImpedance::name() { return "Task Impedance"; }

void TaskImpedance::setReference(const Eigen::Vector3d& position,
                                 const Eigen::Vector4d& orientation) {
  std::lock_guard<std::mutex> lock(mux_);
  position_ref_ = position;
  orientation_ref_ = Eigen::Quaterniond(orientation).normalized();
}

void TaskImpedance::setStiffness(const Vector6d& stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  stiffness_ = stiffness;
  damping_ = task_impedance::criticalDamping(stiffness_, damping_ratio_);
}

void TaskImpedance::setDampingRatio(double damping_ratio) {
  std::lock_guard<std::mutex> lock(mux_);
  damping_ratio_ = damping_ratio;
  damping_ = task_impedance::criticalDamping(stiffness_, damping_ratio_);
}

void TaskImpedance::setNullspaceTarget(const Vector7d& q_nullspace) {
  std::lock_guard<std::mutex> lock(mux_);
  q_nullspace_ = q_nullspace;
}

void TaskImpedance::setNullspaceStiffness(double nullspace_stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  nullspace_stiffness_ = nullspace_stiffness;
}

Vector6d TaskImpedance::getStiffness() {
  std::lock_guard<std::mutex> lock(mux_);
  return stiffness_;
}

Vector6d TaskImpedance::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return damping_;
}

Eigen::Matrix4d TaskImpedance::getFrameTransform() const {
  return frame_transform_;
}

TaskImpedance::Frame TaskImpedance::getFrame() const { return frame_; }
