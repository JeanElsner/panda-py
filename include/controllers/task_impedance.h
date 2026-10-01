#pragma once
#include <atomic>
#include <mutex>

#include "constants.h"
#include "controllers/controller.h"
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
      bool coriolis = false, double nullspace_damping = 0.0);

  franka::Torques step(const franka::RobotState& robot_state,
                       franka::Duration& duration) override;
  void start(const franka::RobotState& robot_state,
             std::shared_ptr<franka::Model> model) override;
  void stop(const franka::RobotState& robot_state,
            std::shared_ptr<franka::Model> model) override;
  bool isRunning() override;
  const std::string name() override;

  /// Position and scalar-last quaternion of the control frame, base frame.
  void setReference(const Eigen::Vector3d& position,
                    const Eigen::Vector4d& orientation);
  /// Also sets the damping, critical for the current damping ratio.
  void setStiffness(const Vector6d& stiffness);
  void setDampingRatio(double damping_ratio);
  void setNullspaceTarget(const Vector7d& q_nullspace);
  void setNullspaceStiffness(double nullspace_stiffness);
  Vector6d getStiffness();
  Vector6d getDamping();
  Eigen::Matrix4d getFrameTransform() const;
  Frame getFrame() const;

  /// The control frame's pose and Jacobian for a robot state.
  void controlFrame(const franka::RobotState& robot_state,
                    franka::Model& model, Eigen::Matrix4d& pose,
                    Eigen::Matrix<double, 6, 7>& jacobian) const;

 protected:
  /// The law's inputs for this state, the reference and gains included.
  task_impedance::Inputs inputs(const franka::RobotState& robot_state);

 private:
  const Frame frame_;
  const Eigen::Matrix4d frame_transform_;
  const bool coriolis_;
  const task_impedance::Nullspace nullspace_;
  const double nullspace_damping_;

  std::mutex mux_;
  Vector6d stiffness_, damping_;
  double damping_ratio_, nullspace_stiffness_;
  Eigen::Vector3d position_ref_;
  Eigen::Quaterniond orientation_ref_;
  Vector7d q_nullspace_;
  std::atomic<bool> motion_finished_;
  std::shared_ptr<franka::Model> model_;
};
