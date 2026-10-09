#pragma once
#include <array>
#include <atomic>
#include <limits>
#include <mutex>
#include <tuple>

#include "constants.h"
#include "controllers/loop.h"
#include "controllers/guard.h"
#include "telemetry.h"
#include "utils.h"

/// The task impedance law as a pure function, so that it can be evaluated
/// and tested without a robot, and compared against a reference
/// implementation on logged states.
namespace task_impedance {

/// How the posture term reaches the joints.
enum class Nullspace {
  /// N M u with N = I - J^T (J M^-1 J^T)^-1 J M^-1. Cannot perturb the
  /// task-space acceleration.
  kDynamic,
  /// N u with N = I - J^T (J J^T)^-1 J. Needs no mass matrix, but does
  /// perturb the task-space acceleration.
  kKinematic,
  /// No posture term.
  kNone,
};

struct Inputs {
  Vector7d q = Vector7d::Zero();
  Vector7d dq = Vector7d::Zero();
  /// Pose of the control frame in the base frame.
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  /// Geometric Jacobian of the control frame's origin, in the base frame.
  Eigen::Matrix<double, 6, 7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();
  Eigen::Matrix<double, 7, 7> mass = Eigen::Matrix<double, 7, 7>::Identity();
  /// Added to the torque as it is. Zero leaves it out.
  Vector7d coriolis = Vector7d::Zero();
  Eigen::Vector3d position_ref = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ref = Eigen::Quaterniond::Identity();
  /// Diagonal stiffness and damping, translational part first.
  Vector6d stiffness = Vector6d::Zero();
  Vector6d damping = Vector6d::Zero();
  /// Scales the active (spring) wrench only, never the damping.
  double alpha = 1.0;
  Vector7d q_nullspace = Vector7d::Zero();
  double nullspace_stiffness = 0.0;
  Nullspace nullspace = Nullspace::kDynamic;
  /// Regularises the 6x6 inverse of the projector, relative to the mean of
  /// its diagonal. Zero is the exact projector.
  double nullspace_damping = 0.0;
  /// Added to the diagonal of ``mass`` for the dynamic posture term only
  /// (projector and N M u): the rotor inertia libfranka's model leaves out.
  /// Zero is libfranka's model.
  Vector7d nullspace_armature = Vector7d::Zero();
  /// A joint-space spring outside the projector, added to the torque as it
  /// is: k o (q_joint_spring - q) - d o dq per joint. Zero leaves it out.
  Vector7d joint_spring_stiffness = Vector7d::Zero();
  Vector7d joint_spring_damping = Vector7d::Zero();
  Vector7d q_joint_spring = Vector7d::Zero();
  /// Coulomb friction compensation: per joint, friction o sat(tau / deadband)
  /// with tau the law's torque without it (task, posture, joint spring, before
  /// Coriolis) and sat clamping to [-1, 1]; a joint pushed with less than the
  /// deadband gets a proportional share. Zero friction leaves it out.
  Vector7d friction = Vector7d::Zero();
  double friction_deadband = 0.1;
};

/// friction o clamp(tau / deadband, -1, 1).
Vector7d frictionCompensation(const Vector7d& friction, double deadband,
                              const Vector7d& tau);

struct Outputs {
  /// [x_ref - x; axisangle(q_ref q^-1)]
  Vector6d error;
  /// [v; omega] of the control frame, J dq.
  Vector6d velocity;
  Vector6d wrench_active;   // K o error, before alpha
  Vector6d wrench_passive;  // -D o velocity
  Vector7d tau_task;        // J^T (alpha w_act + w_pas)
  Vector7d tau_nullspace;
  Vector7d tau_joint_spring;
  Vector7d tau_friction;
  // tau_task + tau_nullspace + tau_joint_spring + tau_friction + coriolis
  Vector7d tau;
};

Outputs compute(const Inputs& in);

/// Axis-angle vector of q_ref q^-1, the rotation from q to q_ref, in the base
/// frame. Its norm is the rotation angle, at most pi.
Eigen::Vector3d orientationError(const Eigen::Quaterniond& orientation_ref,
                                 const Eigen::Quaterniond& orientation);

/// Moves a geometric Jacobian from the origin of frame A to the point
/// A + offset, the offset expressed in the base frame: v_C = v_A + w x offset.
Eigen::Matrix<double, 6, 7> shiftJacobian(
    const Eigen::Matrix<double, 6, 7>& jacobian, const Eigen::Vector3d& offset);

/// D = 2 zeta sqrt(K), elementwise.
Vector6d criticalDamping(const Vector6d& stiffness, double damping_ratio);

/// Limits a reference to within `leash_position` (m) and `leash_rotation`
/// (rad) of the current pose: x_ref = x + d min(1, l / |d|), and the
/// orientation error clipped to l.
void leash(Eigen::Vector3d& position_ref, Eigen::Quaterniond& orientation_ref,
           const Eigen::Vector3d& position,
           const Eigen::Quaterniond& orientation, double leash_position,
           double leash_rotation);

/// An incremental reference update: x_ref += translation,
/// q_ref = rotation q_ref, then the leash against the current pose.
void stepReference(Eigen::Vector3d& position_ref,
                   Eigen::Quaterniond& orientation_ref,
                   const Eigen::Vector3d& translation,
                   const Eigen::Quaterniond& rotation,
                   const Eigen::Vector3d& position,
                   const Eigen::Quaterniond& orientation, double leash_position,
                   double leash_rotation);

/// The rotation of an axis-angle vector.
Eigen::Quaterniond axisAngleToQuaternion(const Eigen::Vector3d& rotation);

// The guards are shared with the other controllers.
using Trip = guard::Trip;
using Box = guard::Box;
using GuardConfig = guard::Config;
using GuardState = guard::State;
using guard::tripName;

/// An energy budget for the active wrench (a passivity tank), or an impulse
/// budget. It meters the active wrench only and gates it with alpha; the
/// damping is never scaled.
enum class TankMode {
  kPower,    // P = max(0, w_act . [v; omega]), W; E0 in J
  kImpulse,  // P = |w_act[0:3]|, N; E0 in N s
};

struct TankConfig {
  bool enabled = false;
  double E0 = 0.0;
  TankMode mode = TankMode::kPower;
  /// alpha = clamp(E_T / (smooth_fraction E0), 0, 1). Zero selects the hard
  /// gate instead, which scales only a draw that would overdraw the tank.
  double smooth_fraction = 0.25;
};

struct TankState {
  double level = 0.0;  // E_T
  double drawn = 0.0;  // drawn since the last reset
  double alpha = 1.0;  // the gate of the latest tick
};

/// One tick of the tank for an active wrench and velocity over dt seconds:
/// sets the gate, draws from the tank and returns the gate, alpha.
double tankStep(const TankConfig& config, TankState& state,
                const Vector6d& wrench_active, const Vector6d& velocity,
                double dt);

/// One telemetry sample per control tick. Every field is a block of doubles,
/// so a table of fields is all the Python side needs to unpack it.
#define TASK_IMPEDANCE_SAMPLE_FIELDS(X) \
  LOOP_SAMPLE_FIELDS(X)                 \
  X(position, 3)                        \
  X(orientation, 4)                     \
  X(position_ref, 3)                    \
  X(orientation_ref, 4)                 \
  X(stiffness, 6)                       \
  X(damping, 6)                         \
  X(wrench_active, 6)                   \
  X(wrench_passive, 6)                  \
  X(alpha, 1)                           \
  X(tank, 1)                            \
  X(tank_drawn, 1)                      \
  X(tau_task, 7)                        \
  X(tau_nullspace, 7)                   \
  X(tau_joint_spring, 7)                \
  X(tau_friction, 7)                    \
  X(jacobian, 42)                       \
  X(mass, 49)

struct Sample {
  TASK_IMPEDANCE_SAMPLE_FIELDS(LOOP_DECLARE_FIELD)
};

/// What the loop last applied, for the 50 Hz side to read.
struct Snapshot {
  double time = 0.0;         // robot time of the latest tick, s
  double applied_time = 0.0; // robot time the latest reference was applied
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();  // control frame, latest tick
  Eigen::Matrix4d applied_pose = Eigen::Matrix4d::Identity();  // ... at applied_time
  Eigen::Vector3d position_ref = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ref = Eigen::Quaterniond::Identity();
  Vector6d stiffness = Vector6d::Zero();
  uint64_t applied = 0;  // reference commands applied since start
  TankState tank;        // NaN level without a tank
};

}  // namespace task_impedance

/// Impedance in task space at a selectable control frame:
///
///   tau = J^T (alpha K o e - D o J dq) + N M u,
///   u = k_ns (q0 - q) - 2 sqrt(k_ns) dq
///
/// with e the position error and the axis-angle orientation error at the
/// control frame. No gravity term: the robot compensates gravity itself.
class TaskImpedance : public controllers::Loop<task_impedance::Sample> {
 public:
  /// The libfranka frame the control frame is attached to.
  enum class Frame { kFlange, kEndEffector };

  static const Vector6d kDefaultStiffness;
  static const double kDefaultDampingRatio;
  static const double kDefaultNullspaceStiffness;
  static const task_impedance::Nullspace kDefaultNullspace;
  static const Frame kDefaultFrame;

  TaskImpedance(
      const Vector6d& stiffness = kDefaultStiffness,
      double damping_ratio = kDefaultDampingRatio,
      task_impedance::Nullspace nullspace = kDefaultNullspace,
      double nullspace_stiffness = kDefaultNullspaceStiffness,
      Frame frame = kDefaultFrame,
      const Eigen::Matrix4d& frame_transform = Eigen::Matrix4d::Identity(),
      bool coriolis = false, double nullspace_damping = 0.0,
      size_t telemetry_capacity = 0);

  const std::string name() override;

  /// Position and scalar-last quaternion of the control frame, base frame.
  /// Applied, and leashed, by the control loop on its next tick, against
  /// the pose of that tick. Replaces any command not yet applied.
  void setReference(const Eigen::Vector3d& position,
                    const Eigen::Vector4d& orientation);
  /// Moves the reference by a translation and a rotation (axis-angle,
  /// applied on the left), both in the base frame, as an incremental action;
  /// with a stiffness, sets it on the same tick. Applied and leashed by the
  /// loop on its next tick; steps not yet applied add up.
  void stepReference(const Eigen::Vector3d& translation,
                     const Eigen::Vector3d& rotation);
  void stepReference(const Eigen::Vector3d& translation,
                     const Eigen::Vector3d& rotation,
                     const Vector6d& stiffness);
  /// Enables the tank, full at E0 from the loop's next tick; the gate scales
  /// the active wrench only. A config with enabled = false removes it.
  void setTank(const task_impedance::TankConfig& config);
  task_impedance::TankConfig getTank();
  /// Refills the tank to E0 on the loop's next tick, as at a trial's start.
  void resetTank();
  /// Infinite disables the leash, which is the default.
  void setLeash(double position, double rotation);
  std::pair<double, double> getLeash();
  task_impedance::Snapshot getSnapshot();
  /// Also sets the damping, critical for the current damping ratio.
  void setStiffness(const Vector6d& stiffness);
  void setDampingRatio(double damping_ratio);
  void setNullspaceTarget(const Vector7d& q_nullspace);
  void setNullspaceStiffness(double nullspace_stiffness);
  void setNullspaceArmature(const Vector7d& armature);
  /// The joint-space spring outside the projector (Inputs); zero stiffness
  /// and damping, the default, leave it out.
  void setJointSpring(const Vector7d& stiffness, const Vector7d& damping,
                      const Vector7d& q);
  std::tuple<Vector7d, Vector7d, Vector7d> getJointSpring();
  /// Coulomb friction compensation (Inputs); zero friction, the default,
  /// leaves it out. Off while a guard is tripped.
  void setFrictionCompensation(const Vector7d& friction, double deadband);
  std::pair<Vector7d, double> getFrictionCompensation();
  Vector7d getNullspaceArmature();
  Vector6d getStiffness();
  Vector6d getDamping();
  Eigen::Matrix4d getFrameTransform() const;
  Frame getFrame() const;

  /// The control frame's pose and Jacobian for a robot state.
  void controlFrame(const franka::RobotState& robot_state,
                    franka::Model& model, Eigen::Matrix4d& pose,
                    Eigen::Matrix<double, 6, 7>& jacobian) const;

 protected:
  // The loop (controllers::Loop). While a guard is tripped the active wrench
  // is dropped (the tank's gate is 0) and the damping, the posture term and
  // the joint spring remain; a rearm resets the reference to the pose of
  // that tick, so the active wrench resumes from zero.
  void prepare(const franka::RobotState& robot_state) override;
  void begin(const franka::RobotState& robot_state) override;
  bool sync(const franka::RobotState& robot_state, double time) override;
  void publish(const franka::RobotState& robot_state, double time) override;
  void onRearm(const franka::RobotState& robot_state) override;
  void guardFrame(const franka::RobotState& robot_state, Eigen::Vector3d& position,
                  Eigen::Vector3d& velocity) override;
  Vector7d law(const franka::RobotState& robot_state, double dt,
               bool tripped) override;
  void record(task_impedance::Sample& sample,
              const franka::RobotState& robot_state) override;

  /// Loop thread only: the reference and nullspace target for this tick,
  /// bypassing the leash, for controllers that compute it every tick.
  void holdReference(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation);
  void holdNullspaceTarget(const Vector7d& q_nullspace);

 private:
  const Frame frame_;
  const Eigen::Matrix4d frame_transform_;
  const bool coriolis_;
  const task_impedance::Nullspace nullspace_;
  const double nullspace_damping_;

  // Shared with the setters, under mux_. The loop only ever try-locks it and
  // keeps its own copy, loop_, for the ticks the lock is taken.
  struct Parameters {
    Vector6d stiffness, damping;
    double nullspace_stiffness;
    Vector7d q_nullspace;
    Vector7d nullspace_armature = Vector7d::Zero();
    Vector7d joint_spring_stiffness = Vector7d::Zero();
    Vector7d joint_spring_damping = Vector7d::Zero();
    Vector7d q_joint_spring = Vector7d::Zero();
    Vector7d friction = Vector7d::Zero();
    double friction_deadband = 0.1;
    double leash_position = std::numeric_limits<double>::infinity();
    double leash_rotation = std::numeric_limits<double>::infinity();
  };
  struct Command {
    bool absolute = false, step = false, stiffness = false, tank_reset = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Vector6d stiffness_value = Vector6d::Zero();
    bool pending() const { return absolute || step || stiffness || tank_reset; }
  };

  Parameters shared_;
  Command command_;
  task_impedance::Snapshot snapshot_;
  double damping_ratio_;

  task_impedance::TankConfig tank_shared_;

  // Loop thread only.
  Parameters loop_;
  task_impedance::TankConfig tank_;
  task_impedance::TankState tank_state_;
  Eigen::Vector3d position_ref_;
  Eigen::Quaterniond orientation_ref_;
  uint64_t applied_ = 0;
  // This tick's inputs and outputs of the law.
  task_impedance::Inputs in_;
  task_impedance::Outputs out_;

  void applyCommand(const Command& command, const Eigen::Matrix4d& pose);
};
