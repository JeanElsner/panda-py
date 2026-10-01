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

void leash(Eigen::Vector3d& position_ref, Eigen::Quaterniond& orientation_ref,
           const Eigen::Vector3d& position,
           const Eigen::Quaterniond& orientation, double leash_position,
           double leash_rotation) {
  const Eigen::Vector3d d = position_ref - position;
  const double dn = d.norm();
  if (dn > leash_position) {
    position_ref = position + d * (leash_position / dn);
  }
  const Eigen::Vector3d e = orientationError(orientation_ref, orientation);
  const double en = e.norm();
  if (en > leash_rotation) {
    orientation_ref =
        Eigen::Quaterniond(Eigen::AngleAxisd(leash_rotation, e / en)) *
        orientation;
  }
}

void stepReference(Eigen::Vector3d& position_ref,
                   Eigen::Quaterniond& orientation_ref,
                   const Eigen::Vector3d& translation,
                   const Eigen::Quaterniond& rotation,
                   const Eigen::Vector3d& position,
                   const Eigen::Quaterniond& orientation, double leash_position,
                   double leash_rotation) {
  position_ref += translation;
  orientation_ref = (rotation * orientation_ref).normalized();
  leash(position_ref, orientation_ref, position, orientation, leash_position,
        leash_rotation);
}

Eigen::Quaterniond axisAngleToQuaternion(const Eigen::Vector3d& rotation) {
  const double angle = rotation.norm();
  return angle > 0.0
             ? Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation / angle))
             : Eigen::Quaterniond::Identity();
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
                             bool coriolis, double nullspace_damping,
                             size_t telemetry_capacity)
    : frame_(frame),
      frame_transform_(frame_transform),
      coriolis_(coriolis),
      nullspace_(nullspace),
      nullspace_damping_(nullspace_damping),
      damping_ratio_(damping_ratio),
      position_ref_(Eigen::Vector3d::Zero()),
      orientation_ref_(Eigen::Quaterniond::Identity()),
      motion_finished_(false),
      telemetry_(telemetry_capacity) {
  if (!frame_transform.allFinite() ||
      !frame_transform.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)) ||
      !(frame_transform.topLeftCorner<3, 3>() *
        frame_transform.topLeftCorner<3, 3>().transpose())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
    throw std::invalid_argument(
        "The frame transform must be a homogeneous transform.");
  }
  shared_.stiffness = stiffness;
  shared_.damping = task_impedance::criticalDamping(stiffness, damping_ratio);
  shared_.nullspace_stiffness = nullspace_stiffness;
  shared_.q_nullspace = kJointPositionStart;
  loop_ = shared_;
}

void TaskImpedance::controlFrame(const franka::RobotState& robot_state,
                                 franka::Model& model, Eigen::Matrix4d& pose,
                                 Eigen::Matrix<double, 6, 7>& jacobian) const {
  const franka::Frame base = frame_ == Frame::kFlange
                                 ? franka::Frame::kFlange
                                 : franka::Frame::kEndEffector;
  // The pose is the robot's own, as in O_T_EE and panda-py's get_pose(), so a
  // reference taken from either is reached exactly; only the Jacobian comes
  // from the model. O_T_EE = O_T_F F_T_EE.
  const Eigen::Matrix4d O_T_EE = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  const Eigen::Matrix4d base_pose =
      frame_ == Frame::kFlange
          ? Eigen::Matrix4d(
                O_T_EE *
                Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.F_T_EE.data()))
                    .inverse()
                    .matrix())
          : O_T_EE;
  const auto base_jacobian = model.zeroJacobian(base, robot_state);
  pose = base_pose * frame_transform_;
  jacobian = task_impedance::shiftJacobian(
      Eigen::Map<const Eigen::Matrix<double, 6, 7>>(base_jacobian.data()),
      base_pose.topLeftCorner<3, 3>() * frame_transform_.topRightCorner<3, 1>());
}

void TaskImpedance::applyCommand(const Command& command,
                                 const Eigen::Matrix4d& pose) {
  if (command.stiffness) {
    loop_.stiffness = command.stiffness_value;
    loop_.damping =
        task_impedance::criticalDamping(loop_.stiffness, damping_ratio_);
  }
  if (!command.absolute && !command.step) {
    return;
  }
  if (command.absolute) {
    position_ref_ = command.position;
    orientation_ref_ = command.orientation;
  }
  const Eigen::Affine3d transform(pose);
  // An absolute reference is a step of zero from it.
  task_impedance::stepReference(
      position_ref_, orientation_ref_, command.translation, command.rotation,
      transform.translation(), Eigen::Quaterniond(transform.rotation()),
      loop_.leash_position, loop_.leash_rotation);
  applied_++;
}

franka::Torques TaskImpedance::step(const franka::RobotState& robot_state,
                                    franka::Duration& duration) {
  task_impedance::Inputs in;
  in.q = Eigen::Map<const Vector7d>(robot_state.q.data());
  in.dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  controlFrame(robot_state, *model_, in.pose, in.jacobian);
  if (nullspace_ == Nullspace::kDynamic) {
    in.mass = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(
        model_->mass(robot_state).data());
  }
  if (coriolis_) {
    in.coriolis =
        Eigen::Map<const Vector7d>(model_->coriolis(robot_state).data());
  }

  // The loop never waits for a setter: if one holds the lock, this tick runs
  // on the previous tick's parameters and any command waits for the next.
  const double time = robot_state.time.toSec();
  bool updated = false;
  std::unique_lock<std::mutex> lock(mux_, std::try_to_lock);
  if (lock.owns_lock()) {
    const Vector6d stiffness = loop_.stiffness, damping = loop_.damping;
    loop_ = shared_;
    // A stiffness set by a command lives in loop_ until the setters see it.
    loop_.stiffness = stiffness;
    loop_.damping = damping;
    if (command_.pending()) {
      applyCommand(command_, in.pose);
      command_ = Command();
      updated = true;
      snapshot_.applied_time = time;
      snapshot_.applied_pose = in.pose;
    }
    shared_.stiffness = loop_.stiffness;
    shared_.damping = loop_.damping;
    snapshot_.time = time;
    snapshot_.pose = in.pose;
    snapshot_.position_ref = position_ref_;
    snapshot_.orientation_ref = orientation_ref_;
    snapshot_.stiffness = loop_.stiffness;
    snapshot_.applied = applied_;
    lock.unlock();
  }

  in.position_ref = position_ref_;
  in.orientation_ref = orientation_ref_;
  in.stiffness = loop_.stiffness;
  in.damping = loop_.damping;
  in.q_nullspace = loop_.q_nullspace;
  in.nullspace_stiffness = loop_.nullspace_stiffness;
  in.nullspace = nullspace_;
  in.nullspace_damping = nullspace_damping_;
  const auto out = task_impedance::compute(in);

  sample_ = telemetry_.claim();
  if (sample_) {
    auto& s = *sample_;
    auto put = [](double* to, const auto& from) {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(to, from.size()) =
          Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(
              from.data(), from.size());
    };
    const Eigen::Affine3d transform(in.pose);
    s.tick[0] = static_cast<double>(tick_);
    s.time[0] = time;
    s.duration[0] = duration.toSec();
    s.reference_update[0] = updated ? 1.0 : 0.0;
    s.control_command_success_rate[0] = robot_state.control_command_success_rate;
    put(s.position, Eigen::Vector3d(transform.translation()));
    put(s.orientation, Eigen::Quaterniond(transform.rotation()).coeffs());
    put(s.position_ref, in.position_ref);
    put(s.orientation_ref, in.orientation_ref.coeffs());
    put(s.stiffness, in.stiffness);
    put(s.damping, in.damping);
    put(s.wrench_active, out.wrench_active);
    put(s.wrench_passive, out.wrench_passive);
    s.alpha[0] = in.alpha;
    s.tank[0] = std::numeric_limits<double>::quiet_NaN();
    put(s.tau_task, out.tau_task);
    put(s.tau_nullspace, out.tau_nullspace);
    put(s.tau_law, out.tau);
    put(s.q, robot_state.q);
    put(s.dq, robot_state.dq);
    put(s.tau_J, robot_state.tau_J);
    put(s.tau_J_d, robot_state.tau_J_d);
    put(s.tau_ext_hat_filtered, robot_state.tau_ext_hat_filtered);
    put(s.O_T_EE, robot_state.O_T_EE);
    put(s.F_T_EE, robot_state.F_T_EE);
    put(s.O_F_ext_hat_K, robot_state.O_F_ext_hat_K);
    put(s.K_F_ext_hat_K, robot_state.K_F_ext_hat_K);
    put(s.jacobian, in.jacobian);
    if (nullspace_ == Nullspace::kDynamic) {
      put(s.mass, in.mass);
    } else {
      std::fill(std::begin(s.mass), std::end(s.mass),
                std::numeric_limits<double>::quiet_NaN());
    }
    // tau_cmd: filled in by commanded(), once the torque is final.
    std::fill(std::begin(s.tau_cmd), std::end(s.tau_cmd),
              std::numeric_limits<double>::quiet_NaN());
  }
  tick_++;

  franka::Torques torques = VectorToArray<7>(out.tau);
  torques.motion_finished = motion_finished_;
  return torques;
}

void TaskImpedance::commanded(const franka::RobotState& robot_state,
                              const franka::Torques& torques) {
  if (sample_) {
    std::copy(torques.tau_J.begin(), torques.tau_J.end(), sample_->tau_cmd);
    telemetry_.publish();
    sample_ = nullptr;
  }
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
  shared_.q_nullspace = Eigen::Map<const Vector7d>(robot_state.q.data());
  loop_ = shared_;
  command_ = Command();
  tick_ = 0;
  applied_ = 0;
  sample_ = nullptr;
  snapshot_ = task_impedance::Snapshot();
  snapshot_.time = robot_state.time.toSec();
  snapshot_.applied_time = snapshot_.time;
  snapshot_.pose = pose;
  snapshot_.applied_pose = pose;
  snapshot_.position_ref = position_ref_;
  snapshot_.orientation_ref = orientation_ref_;
  snapshot_.stiffness = shared_.stiffness;
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
  command_.absolute = true;
  command_.step = false;
  command_.position = position;
  command_.orientation = Eigen::Quaterniond(orientation).normalized();
  command_.translation.setZero();
  command_.rotation.setIdentity();
}

void TaskImpedance::stepReference(const Eigen::Vector3d& translation,
                                  const Eigen::Vector3d& rotation) {
  const Eigen::Quaterniond turn = task_impedance::axisAngleToQuaternion(rotation);
  std::lock_guard<std::mutex> lock(mux_);
  command_.step = true;
  command_.translation += translation;
  command_.rotation = (turn * command_.rotation).normalized();
}

void TaskImpedance::stepReference(const Eigen::Vector3d& translation,
                                  const Eigen::Vector3d& rotation,
                                  const Vector6d& stiffness) {
  stepReference(translation, rotation);
  std::lock_guard<std::mutex> lock(mux_);
  command_.stiffness = true;
  command_.stiffness_value = stiffness;
}

void TaskImpedance::setLeash(double position, double rotation) {
  if (!(position > 0.0) || !(rotation > 0.0)) {
    throw std::invalid_argument("The leash must be positive.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  shared_.leash_position = position;
  shared_.leash_rotation = rotation;
}

std::pair<double, double> TaskImpedance::getLeash() {
  std::lock_guard<std::mutex> lock(mux_);
  return {shared_.leash_position, shared_.leash_rotation};
}

task_impedance::Snapshot TaskImpedance::getSnapshot() {
  std::lock_guard<std::mutex> lock(mux_);
  return snapshot_;
}

void TaskImpedance::setStiffness(const Vector6d& stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  // Through the command, so that the loop takes it like a policy step's.
  command_.stiffness = true;
  command_.stiffness_value = stiffness;
  shared_.stiffness = stiffness;
  shared_.damping = task_impedance::criticalDamping(stiffness, damping_ratio_);
}

void TaskImpedance::setDampingRatio(double damping_ratio) {
  std::lock_guard<std::mutex> lock(mux_);
  damping_ratio_ = damping_ratio;
  const Vector6d stiffness =
      command_.stiffness ? command_.stiffness_value : shared_.stiffness;
  command_.stiffness = true;
  command_.stiffness_value = stiffness;
  shared_.damping = task_impedance::criticalDamping(stiffness, damping_ratio_);
}

void TaskImpedance::setNullspaceTarget(const Vector7d& q_nullspace) {
  std::lock_guard<std::mutex> lock(mux_);
  shared_.q_nullspace = q_nullspace;
}

void TaskImpedance::setNullspaceStiffness(double nullspace_stiffness) {
  std::lock_guard<std::mutex> lock(mux_);
  shared_.nullspace_stiffness = nullspace_stiffness;
}

Vector6d TaskImpedance::getStiffness() {
  std::lock_guard<std::mutex> lock(mux_);
  return command_.stiffness ? command_.stiffness_value : shared_.stiffness;
}

Vector6d TaskImpedance::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return task_impedance::criticalDamping(
      command_.stiffness ? command_.stiffness_value : shared_.stiffness,
      damping_ratio_);
}

Eigen::Matrix4d TaskImpedance::getFrameTransform() const {
  return frame_transform_;
}

TaskImpedance::Frame TaskImpedance::getFrame() const { return frame_; }

size_t TaskImpedance::readTelemetry(std::vector<task_impedance::Sample>& out) {
  return telemetry_.drain(out);
}

uint64_t TaskImpedance::telemetryDropped() const { return telemetry_.dropped(); }

size_t TaskImpedance::telemetryCapacity() const { return telemetry_.capacity(); }
