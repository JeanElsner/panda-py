#include "controllers/joint_impedance.h"

#include <stdexcept>

const double kDefaultStiffnessData[7] = {600, 600, 600, 600, 250, 150, 50};
const Vector7d JointImpedance::kDefaultStiffness = Vector7d(kDefaultStiffnessData);

const double kDefaultDampingData[7] = {50, 50, 50, 20, 20, 20, 10};
const Vector7d JointImpedance::kDefaultDamping = Vector7d(kDefaultDampingData);

namespace {

void checkGains(const Vector7d& gains, const char* what) {
  if (!gains.allFinite() || (gains.array() < 0).any()) {
    throw std::invalid_argument(std::string(what) +
                                " must be finite and non-negative.");
  }
}

}  // namespace

JointImpedance::JointImpedance(const Vector7d& stiffness, const Vector7d& damping,
                               size_t telemetry_capacity)
    : Loop(telemetry_capacity),
      stiffness_(stiffness),
      damping_(damping),
      loop_stiffness_(stiffness),
      loop_damping_(damping) {
  checkGains(stiffness, "The stiffness");
  checkGains(damping, "The damping");
}

void JointImpedance::begin(const franka::RobotState& robot_state) {
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
  loop_stiffness_ = stiffness_;
  loop_damping_ = damping_;
  command_ = Command();
  applied_ = 0;
  snapshot_ = joint_impedance::Snapshot();
  snapshot_.time = snapshot_.applied_time = robot_state.time.toSec();
  snapshot_.q = snapshot_.applied_q = snapshot_.q_d = q_d_;
}

bool JointImpedance::sync(const franka::RobotState& robot_state, double time) {
  loop_stiffness_ = stiffness_;
  loop_damping_ = damping_;
  if (!command_.absolute && !command_.relative) {
    return false;
  }
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  if (command_.absolute) {
    q_d_ = command_.q_d;
    dq_d_ = command_.dq_d;
  } else {
    q_d_ = q + command_.q_d;
    dq_d_.setZero();
  }
  applied_++;
  snapshot_.applied_time = time;
  snapshot_.applied_q = q;
  command_ = Command();
  return true;
}

void JointImpedance::publish(const franka::RobotState& robot_state, double time) {
  snapshot_.time = time;
  snapshot_.q = Eigen::Map<const Vector7d>(robot_state.q.data());
  snapshot_.q_d = q_d_;
  snapshot_.applied = applied_;
}

void JointImpedance::onRearm(const franka::RobotState& robot_state) {
  q_d_ = Eigen::Map<const Vector7d>(robot_state.q.data());
  dq_d_.setZero();
}

Vector7d JointImpedance::law(const franka::RobotState& robot_state, double dt,
                             bool tripped) {
  const Vector7d q = Eigen::Map<const Vector7d>(robot_state.q.data());
  const Vector7d dq = Eigen::Map<const Vector7d>(robot_state.dq.data());
  tau_active_ = loop_stiffness_.cwiseProduct(q_d_ - q) +
                loop_damping_.cwiseProduct(dq_d_);
  tau_passive_ = -loop_damping_.cwiseProduct(dq);
  if (tripped) {
    tau_active_.setZero();
  }
  return tau_active_ + tau_passive_;
}

void JointImpedance::record(joint_impedance::Sample& s,
                            const franka::RobotState& robot_state) {
  controllers::putField(s.q_d, q_d_);
  controllers::putField(s.dq_d, dq_d_);
  controllers::putField(s.stiffness, loop_stiffness_);
  controllers::putField(s.damping, loop_damping_);
  controllers::putField(s.tau_active, tau_active_);
  controllers::putField(s.tau_passive, tau_passive_);
}

void JointImpedance::setReference(const Vector7d& q_d, const Vector7d& dq_d) {
  if (!q_d.allFinite() || !dq_d.allFinite()) {
    throw std::invalid_argument("The reference must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  command_.absolute = true;
  command_.relative = false;
  command_.q_d = q_d;
  command_.dq_d = dq_d;
}

void JointImpedance::stepReference(const Vector7d& delta) {
  if (!delta.allFinite()) {
    throw std::invalid_argument("The step must be finite.");
  }
  std::lock_guard<std::mutex> lock(mux_);
  command_.absolute = false;
  command_.relative = true;
  command_.q_d = delta;
  command_.dq_d.setZero();
}

void JointImpedance::setStiffness(const Vector7d& stiffness) {
  checkGains(stiffness, "The stiffness");
  std::lock_guard<std::mutex> lock(mux_);
  stiffness_ = stiffness;
}

void JointImpedance::setDamping(const Vector7d& damping) {
  checkGains(damping, "The damping");
  std::lock_guard<std::mutex> lock(mux_);
  damping_ = damping;
}

Vector7d JointImpedance::getStiffness() {
  std::lock_guard<std::mutex> lock(mux_);
  return stiffness_;
}

Vector7d JointImpedance::getDamping() {
  std::lock_guard<std::mutex> lock(mux_);
  return damping_;
}

joint_impedance::Snapshot JointImpedance::getSnapshot() {
  std::lock_guard<std::mutex> lock(mux_);
  return snapshot_;
}
