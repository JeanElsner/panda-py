#pragma once

#include <franka/control_types.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <cmath>  // for std::abs
#include "constants.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef std::array<double, 7> Array7d;

template <typename ReturnType>
using Callback =
    std::function<ReturnType(const franka::RobotState&, franka::Duration)>;
typedef Callback<franka::Torques> TorqueCallback;

template <int T>
inline Eigen::Matrix<double, T, 1> ArrayToVector(
    const std::array<double, T>& array) {
  Eigen::Matrix<double, T, 1> matrix;
  std::copy(array.begin(), array.end(), matrix.data());
  return matrix;
}

template <int T>
inline std::array<double, T> VectorToArray(
    const Eigen::Matrix<double, T, 1>& matrix) {
  std::array<double, T> array;
  std::copy(matrix.data(), matrix.data() + T, array.data());
  return array;
}

template <int T>
inline std::array<double, T> VectorToArray(const Eigen::VectorXd& vector) {
  std::array<double, T> array;
  std::copy(vector.data(), vector.data() + T, array.data());
  return array;
}

inline Eigen::Matrix4d PositionOrientationToMatrix(const Eigen::Vector3d &position, const Eigen::Vector4d &orientation) {
  Eigen::Quaterniond q(orientation);
  Eigen::Affine3d transform;
  transform = q;
  transform.translation() = position;
  return transform.matrix();
}

inline Eigen::Vector3d MatrixToPosition(const Eigen::Matrix4d &matrix) {
  Eigen::Affine3d transform(matrix);
  return transform.translation();
}

inline Eigen::Vector4d MatrixToOrientation(const Eigen::Matrix4d &matrix) {
  Eigen::Affine3d transform(matrix);
  Eigen::Quaterniond q(transform.rotation());
  return q.coeffs();
}

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                          bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ =
      svd.singularValues();
  Eigen::MatrixXd S_ =
      M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) =
        (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() *
                            svd.matrixU().transpose());
}

inline double kDeltaTauMax = 1.0;

inline Array7d saturateTorqueRate(const Array7d& tau_d_calculated,
                                  const Array7d& tau_J_d) {
  Array7d tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

inline Array7d clipTorques(const Array7d& tau_d_calculdated) {
  Array7d tau_d_clipped{};
  for (size_t i = 0; i < 7; i++) {
    tau_d_clipped[i] = std::max(std::min(tau_d_calculdated[i], kTauJMax[i]), -kTauJMax[i]);
  }
  return tau_d_clipped;
}

template <typename T>
T ema_filter(const T& value_f, const T& value, double alpha,
             bool rounding = false, double threshold = 1e-20);

// Template definition for the general case, i.e. Eigen::Matrix
template <typename EigenMatrix>
inline EigenMatrix ema_filter(const EigenMatrix& value_f,
                              const EigenMatrix& value, double alpha,
                              bool rounding, double threshold) {
  return value_f.binaryExpr(
      value, [alpha, rounding, threshold](const auto v_f, const auto v) {
        return ema_filter(v_f, v, alpha, rounding, threshold);
      });
}

// Template specialization for double
template <>
inline double ema_filter<double>(const double& value_f, const double& value,
                                 double alpha, bool rounding,
                                 double threshold) {
  if (rounding && std::abs(value - value_f) < threshold) {
    return value;
  }
  return alpha * value + (1 - alpha) * value_f;
}