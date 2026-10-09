#pragma once
#include <Eigen/Dense>
#include <limits>
#include <string>

#include "constants.h"

/// Kinematics of the Franka Emika Robot and the Franka Research 3, which
/// share their link geometry (Franka's modified Denavit-Hartenberg
/// parameters), with an arbitrary end effector.
namespace kinematics {

/// The Franka Hand: 0.1034 m along the flange's z, turned by -45 deg about it.
Eigen::Matrix4d handTransform();

/// The flange's pose for joint positions q, base frame.
Eigen::Matrix4d flangePose(const Vector7d& q);

/// The end effector's pose, F_T_EE relative to the flange.
Eigen::Matrix4d pose(const Vector7d& q,
                     const Eigen::Matrix4d& F_T_EE = handTransform());

/// The end effector's geometric Jacobian, [v; omega] in the base frame.
Eigen::Matrix<double, 6, 7> jacobian(
    const Vector7d& q, const Eigen::Matrix4d& F_T_EE = handTransform());

struct IkOptions {
  /// Accepted errors: position (m) and orientation (rad).
  double position_tolerance = 1e-5;
  double orientation_tolerance = 1e-4;
  /// Iterations per start.
  int max_iterations = 200;
  /// Further starts from random joint positions within the limits when the
  /// one from q_init does not converge; deterministic.
  int restarts = 20;
  /// Pull of the redundancy toward q_init, in the Jacobian's nullspace.
  double nullspace_gain = 0.2;
  /// Kept clear of the joint limits, rad.
  double limit_margin = 1e-3;
};

struct IkResult {
  bool success = false;
  Vector7d q = Vector7d::Zero();
  double position_error = std::numeric_limits<double>::infinity();
  double orientation_error = std::numeric_limits<double>::infinity();
  int iterations = 0;
  int starts = 0;
};

/**
 * Numerical inverse kinematics: joint positions within `limits` that put the
 * end effector (F_T_EE relative to the flange) at O_T_EE, by damped least
 * squares from q_init, the redundancy drawn toward q_init, so the solution is
 * the one near it. If that does not converge, further starts are tried.
 * The result says whether it succeeded; q is the best found either way.
 */
IkResult ik(const Eigen::Matrix4d& O_T_EE, const Vector7d& q_init,
            const RobotLimits& limits = conservativeLimits(),
            const Eigen::Matrix4d& F_T_EE = handTransform(),
            const IkOptions& options = IkOptions());

}  // namespace kinematics
