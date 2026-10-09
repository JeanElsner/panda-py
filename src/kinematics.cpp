#include "kinematics/kinematics.h"

#include <array>
#include <cmath>
#include <random>

namespace kinematics {

namespace {

// Franka's modified (Craig) Denavit-Hartenberg parameters, link i from
// i - 1: rot_x(alpha) trans_x(a) rot_z(q_i) trans_z(d). The flange follows
// joint 7 at d = 0.107. The same for the FER and the FR3.
constexpr double kA[7] = {0, 0, 0, 0.0825, -0.0825, 0, 0.088};
constexpr double kD[7] = {0.333, 0, 0.316, 0, 0.384, 0, 0};
constexpr double kAlpha[7] = {0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2};
constexpr double kFlange = 0.107;

Eigen::Matrix4d link(double a, double d, double alpha, double theta) {
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double ct = std::cos(theta), st = std::sin(theta);
  Eigen::Matrix4d T;
  T << ct, -st, 0, a,         //
      st * ca, ct * ca, -sa, -d * sa,  //
      st * sa, ct * sa, ca, d * ca,    //
      0, 0, 0, 1;
  return T;
}

/// The joint frames 1 to 7 and the flange.
void chain(const Vector7d& q, Eigen::Matrix4d frames[8]) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 7; i++) {
    T = T * link(kA[i], kD[i], kAlpha[i], q[i]);
    frames[i] = T;
  }
  frames[7] = T * link(0, kFlange, 0, 0);
}

/// The rotation from R to R_goal as an axis-angle vector, base frame.
Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_goal, const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(R_goal * R.transpose());
  return aa.angle() * aa.axis();
}

}  // namespace

Eigen::Matrix4d handTransform() {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(2, 3) = 0.1034;
  return T;
}

Eigen::Matrix4d flangePose(const Vector7d& q) {
  Eigen::Matrix4d frames[8];
  chain(q, frames);
  return frames[7];
}

Eigen::Matrix4d pose(const Vector7d& q, const Eigen::Matrix4d& F_T_EE) {
  return flangePose(q) * F_T_EE;
}

Eigen::Matrix<double, 6, 7> jacobian(const Vector7d& q, const Eigen::Matrix4d& F_T_EE) {
  Eigen::Matrix4d frames[8];
  chain(q, frames);
  const Eigen::Vector3d p = (frames[7] * F_T_EE).topRightCorner<3, 1>();
  Eigen::Matrix<double, 6, 7> J;
  for (int i = 0; i < 7; i++) {
    // Every joint turns about its frame's z axis.
    const Eigen::Vector3d z = frames[i].block<3, 1>(0, 2);
    const Eigen::Vector3d o = frames[i].topRightCorner<3, 1>();
    J.block<3, 1>(0, i) = z.cross(p - o);
    J.block<3, 1>(3, i) = z;
  }
  return J;
}

IkResult ik(const Eigen::Matrix4d& O_T_EE, const Vector7d& q_init,
            const RobotLimits& limits, const Eigen::Matrix4d& F_T_EE,
            const IkOptions& options) {
  if (!O_T_EE.allFinite() || !q_init.allFinite() || !F_T_EE.allFinite()) {
    throw std::invalid_argument("The pose, q_init and F_T_EE must be finite.");
  }
  const Vector7d lower = limits.q_lower.array() + options.limit_margin;
  const Vector7d upper = limits.q_upper.array() - options.limit_margin;
  const Eigen::Vector3d p_goal = O_T_EE.topRightCorner<3, 1>();
  const Eigen::Matrix3d R_goal = O_T_EE.topLeftCorner<3, 3>();
  const Vector7d q_rest = q_init.cwiseMax(lower).cwiseMin(upper);

  IkResult best;
  std::mt19937 rng(0);
  for (int start = 0; start <= options.restarts; start++) {
    Vector7d q = q_rest;
    if (start > 0) {
      for (int j = 0; j < 7; j++) {
        q[j] = std::uniform_real_distribution<double>(lower[j], upper[j])(rng);
      }
    }
    for (int iteration = 1; iteration <= options.max_iterations; iteration++) {
      const Eigen::Matrix4d T = pose(q, F_T_EE);
      Eigen::Matrix<double, 6, 1> e;
      e.head<3>() = p_goal - T.topRightCorner<3, 1>();
      e.tail<3>() = rotationError(R_goal, T.topLeftCorner<3, 3>());
      const double position_error = e.head<3>().norm();
      const double orientation_error = e.tail<3>().norm();
      const bool converged = position_error <= options.position_tolerance &&
                             orientation_error <= options.orientation_tolerance;
      if (converged || position_error + orientation_error <
                           best.position_error + best.orientation_error) {
        best.q = q;
        best.position_error = position_error;
        best.orientation_error = orientation_error;
        best.iterations = iteration;
        best.starts = start + 1;
      }
      if (converged) {
        best.success = true;
        return best;
      }
      Eigen::Matrix<double, 6, 7> J = jacobian(q, F_T_EE);
      // Damped least squares, the damping growing with the error so that
      // far from the goal and near singularities the step stays small. Close
      // to the goal, Gauss-Newton steps finish it: there the damping and the
      // redundancy's pull would only hold the solution off the goal, as a
      // damped pseudo-inverse's nullspace is not exactly the Jacobian's.
      const bool close = position_error < 5e-3 && orientation_error < 5e-2;
      const double lambda2 = close ? 1e-10 : 1e-4 + 1e-3 * e.squaredNorm();
      // A joint held at a limit while the step would push it further is taken
      // out of the step, so that the others make up for it.
      Vector7d dq = Vector7d::Zero();
      std::array<bool, 7> held{};
      for (int pass = 0; pass < 7; pass++) {
        const Eigen::Matrix<double, 6, 6> JJt =
            J * J.transpose() + lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 7, 6> J_pinv =
            J.transpose() * JJt.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
        dq = J_pinv * e;
        if (!close) {
          // The redundancy: toward q_init, in the task's nullspace.
          Vector7d pull = options.nullspace_gain * (q_rest - q);
          for (int j = 0; j < 7; j++) {
            if (held[j]) pull[j] = 0;
          }
          dq += (Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J) * pull;
        }
        bool changed = false;
        for (int j = 0; j < 7; j++) {
          const bool at_lower = q[j] <= lower[j] + 1e-12 && dq[j] < 0;
          const bool at_upper = q[j] >= upper[j] - 1e-12 && dq[j] > 0;
          if (!held[j] && (at_lower || at_upper)) {
            held[j] = changed = true;
            J.col(j).setZero();
          }
        }
        if (!changed) {
          break;
        }
      }
      for (int j = 0; j < 7; j++) {
        if (held[j]) dq[j] = 0;
      }
      const double largest = dq.cwiseAbs().maxCoeff();
      if (largest > 0.2) {
        dq *= 0.2 / largest;
      }
      q = (q + dq).cwiseMax(lower).cwiseMin(upper);
    }
  }
  return best;
}

}  // namespace kinematics
