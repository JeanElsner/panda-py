#pragma once
#include <array>
#include <atomic>
#include <limits>
#include <mutex>

#include "constants.h"
#include "controllers/controller.h"
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
};

struct Outputs {
  /// [x_ref - x; axisangle(q_ref q^-1)]
  Vector6d error;
  /// [v; omega] of the control frame, J dq.
  Vector6d velocity;
  Vector6d wrench_active;   // K o error, before alpha
  Vector6d wrench_passive;  // -D o velocity
  Vector7d tau_task;        // J^T (alpha w_act + w_pas)
  Vector7d tau_nullspace;
  Vector7d tau;  // tau_task + tau_nullspace + coriolis
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
/// (rad) of the current pose, as the simulator does after every policy step:
/// x_ref = x + d min(1, l / |d|), and the orientation error clipped to l.
void leash(Eigen::Vector3d& position_ref, Eigen::Quaterniond& orientation_ref,
           const Eigen::Vector3d& position,
           const Eigen::Quaterniond& orientation, double leash_position,
           double leash_rotation);

/// One policy step's reference update: x_ref += translation,
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

/// The energy or impulse tank of the insertion simulator. It meters the
/// active wrench only and gates it with alpha; damping is never scaled.
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
  X(tick, 1)                            \
  X(time, 1)                            \
  X(duration, 1)                        \
  X(reference_update, 1)                \
  X(control_command_success_rate, 1)    \
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
  X(tau_law, 7)                         \
  X(tau_cmd, 7)                         \
  X(q, 7)                               \
  X(dq, 7)                              \
  X(tau_J, 7)                           \
  X(tau_J_d, 7)                         \
  X(tau_ext_hat_filtered, 7)            \
  X(O_T_EE, 16)                         \
  X(F_T_EE, 16)                         \
  X(O_F_ext_hat_K, 6)                   \
  X(K_F_ext_hat_K, 6)                   \
  X(guard, 1)                           \
  X(jacobian, 42)                       \
  X(mass, 49)

struct Sample {
#define TASK_IMPEDANCE_DECLARE(name, size) double name[size];
  TASK_IMPEDANCE_SAMPLE_FIELDS(TASK_IMPEDANCE_DECLARE)
#undef TASK_IMPEDANCE_DECLARE
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
class TaskImpedance : public TorqueController {
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

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) override;
  void start(const franka::RobotState& robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState& robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;
  void commanded(const franka::RobotState& robot_state,
                 const franka::Torques& torques) override;

  /// Position and scalar-last quaternion of the control frame, base frame.
  /// Applied, and leashed, by the control loop on its next tick, against
  /// the pose of that tick. Replaces any command not yet applied.
  void setReference(const Eigen::Vector3d& position,
                    const Eigen::Vector4d& orientation);
  /// Moves the reference by a translation and a rotation (axis-angle,
  /// applied on the left), both in the base frame, as one policy step does;
  /// with a stiffness, sets it on the same tick. Applied and leashed by the
  /// loop on its next tick; steps not yet applied add up.
  void stepReference(const Eigen::Vector3d& translation,
                     const Eigen::Vector3d& rotation);
  void stepReference(const Eigen::Vector3d& translation,
                     const Eigen::Vector3d& rotation,
                     const Vector6d& stiffness);
  /// Sets the guards. When one trips, the loop drops the active (spring)
  /// wrench on that tick and keeps the damping and the nullspace term, until
  /// rearm(). Replaces the previous configuration.
  void setGuard(const task_impedance::GuardConfig& config);
  task_impedance::GuardConfig getGuard();
  task_impedance::GuardState getGuardState();
  /// Trips the guard from outside the loop, e.g. on missed policy deadlines.
  void trip();
  /// Clears a trip on the loop's next tick, with the reference reset to the
  /// pose of that tick, so the active wrench resumes from zero.
  void rearm();
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
  Vector6d getStiffness();
  Vector6d getDamping();
  Eigen::Matrix4d getFrameTransform() const;
  Frame getFrame() const;

  /// Appends the telemetry recorded since the last call.
  size_t readTelemetry(std::vector<task_impedance::Sample>& out);
  uint64_t telemetryDropped() const;
  size_t telemetryCapacity() const;

  /// The control frame's pose and Jacobian for a robot state.
  void controlFrame(const franka::RobotState& robot_state,
                    franka::Model& model, Eigen::Matrix4d& pose,
                    Eigen::Matrix<double, 6, 7>& jacobian) const;

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
    double leash_position = std::numeric_limits<double>::infinity();
    double leash_rotation = std::numeric_limits<double>::infinity();
  };
  struct Command {
    bool absolute = false, step = false, stiffness = false, rearm = false,
         trip = false, tank_reset = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Vector6d stiffness_value = Vector6d::Zero();
    bool pending() const {
      return absolute || step || stiffness || rearm || trip || tank_reset;
    }
  };

  std::mutex mux_;
  Parameters shared_;
  Command command_;
  task_impedance::Snapshot snapshot_;
  double damping_ratio_;

  task_impedance::GuardConfig guard_shared_;
  task_impedance::GuardState guard_state_shared_;
  task_impedance::TankConfig tank_shared_;

  // Loop thread only.
  Parameters loop_;
  task_impedance::GuardConfig guard_;
  guard::Monitor monitor_;
  task_impedance::TankConfig tank_;
  task_impedance::TankState tank_state_;
  Eigen::Vector3d position_ref_;
  Eigen::Quaterniond orientation_ref_;
  uint64_t tick_ = 0;
  uint64_t applied_ = 0;

  std::atomic<bool> motion_finished_;
  std::shared_ptr<franka::Model> model_;
  TelemetryRing<task_impedance::Sample> telemetry_;
  task_impedance::Sample* sample_ = nullptr;

  void applyCommand(const Command& command, const Eigen::Matrix4d& pose,
                    double time);
};
