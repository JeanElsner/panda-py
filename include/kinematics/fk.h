#pragma once

#include "Eigen/Dense"

namespace kinematics {

Eigen::Matrix<double, 4, 4> fk(const Eigen::Matrix<double, 7, 1> &q) {
  Eigen::Matrix<double, 4, 4> pose;
  pose.setZero();
  pose.row(0)(0) =
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) *
          std::sin(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[3]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[2]) * std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::cos(q[2]) * std::cos(q[4]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) * std::cos(q[0]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[0]) * std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[1]) * std::sin(q[5]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[0]) * std::cos(q[3]) -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[5]) -
      1.0 * std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[4]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[3]) * std::sin(q[5]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) -
      1.0 * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[6] + M_PI_4) * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]);
  pose.row(0)(1) =
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[3]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::cos(q[2]) * std::cos(q[5]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[6] + M_PI_4) * std::cos(q[2]) *
          std::cos(q[4]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[0]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0]) * std::cos(q[4]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[1]) * std::sin(q[5]) * std::cos(q[0]) * std::cos(q[3]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[2]) * std::sin(q[6] + M_PI_4) * std::cos(q[0]) *
          std::cos(q[1]) * std::cos(q[4]) -
      1.0 * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[4]) * std::sin(q[6] + M_PI_4) * std::cos(q[0]) *
          std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) +
      1.0 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) * std::cos(q[6] + M_PI_4);
  pose.row(0)(2) = -1.0 *
                       (((std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                             std::cos(q[3]) -
                         std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
                            std::cos(q[4]) +
                        (std::sin(q[0]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) -
                   1.0 *
                       ((std::sin(q[0]) * std::sin(q[2]) -
                         std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                            std::sin(q[3]) +
                        std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
                       std::cos(q[5]);
  pose.row(0)(3) =
      -0.21000000000000002 *
          (((std::sin(q[0]) * std::sin(q[2]) -
             std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                std::cos(q[3]) -
            std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
               std::cos(q[4]) +
           (std::sin(q[0]) * std::cos(q[2]) +
            std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
               std::sin(q[4])) *
          std::sin(q[5]) -
      0.087999999999999995 *
          (((std::sin(q[0]) * std::sin(q[2]) -
             std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                std::cos(q[3]) -
            std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
               std::cos(q[4]) +
           (std::sin(q[0]) * std::cos(q[2]) +
            std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
               std::sin(q[4])) *
          std::cos(q[5]) +
      0.087999999999999995 *
          ((std::sin(q[0]) * std::sin(q[2]) -
            std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
               std::sin(q[3]) +
           std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
          std::sin(q[5]) -
      0.21000000000000002 *
          ((std::sin(q[0]) * std::sin(q[2]) -
            std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
               std::sin(q[3]) +
           std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
          std::cos(q[5]) +
      0.38400000000000001 *
          (std::sin(q[0]) * std::sin(q[2]) -
           std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
          std::sin(q[3]) +
      0.082500000000000004 *
          (std::sin(q[0]) * std::sin(q[2]) -
           std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
          std::cos(q[3]) -
      0.082500000000000004 * std::sin(q[0]) * std::sin(q[2]) -
      0.082500000000000004 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0]) +
      0.38400000000000001 * std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3]) +
      0.316 * std::sin(q[1]) * std::cos(q[0]) +
      0.082500000000000004 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]);
  pose.row(1)(0) =
      -1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[5]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[3]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[1]) * std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) * std::cos(q[4]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[0]) * std::sin(q[3]) * std::sin(q[5]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[1]) * std::cos(q[2]) -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[6] + M_PI_4) * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]) -
      1.0 * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) *
          std::sin(q[6] + M_PI_4) * std::cos(q[0]) -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[3]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[2]) * std::sin(q[6] + M_PI_4) * std::cos(q[0]) *
          std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[4]) * std::sin(q[6] + M_PI_4) * std::cos(q[0]) *
          std::cos(q[2]) * std::cos(q[5]) +
      1.0 * std::cos(q[0]) * std::cos(q[2]) * std::cos(q[4]) *
          std::cos(q[6] + M_PI_4);
  pose.row(1)(1) =
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) *
          std::sin(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[5]) * std::cos(q[3]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[1]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[1]) * std::cos(q[4]) -
      1.0 * std::sin(q[0]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) +
      1.0 * std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) * std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[0]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[0]) * std::cos(q[3]) +
      1.0 * std::sin(q[2]) * std::cos(q[0]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[2]) * std::cos(q[5]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[6] + M_PI_4) * std::cos(q[0]) * std::cos(q[2]) *
          std::cos(q[4]);
  pose.row(1)(2) = 1.0 *
                       (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                          std::sin(q[2]) * std::cos(q[0])) *
                             std::cos(q[3]) +
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
                            std::cos(q[4]) -
                        (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
                         std::cos(q[0]) * std::cos(q[2])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) +
                   1.0 *
                       ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0])) *
                            std::sin(q[3]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]);
  pose.row(1)(3) =
      0.21000000000000002 *
          (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
             std::sin(q[2]) * std::cos(q[0])) *
                std::cos(q[3]) +
            std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
               std::cos(q[4]) -
           (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
            std::cos(q[0]) * std::cos(q[2])) *
               std::sin(q[4])) *
          std::sin(q[5]) +
      0.087999999999999995 *
          (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
             std::sin(q[2]) * std::cos(q[0])) *
                std::cos(q[3]) +
            std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
               std::cos(q[4]) -
           (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
            std::cos(q[0]) * std::cos(q[2])) *
               std::sin(q[4])) *
          std::cos(q[5]) -
      0.087999999999999995 *
          ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
            std::sin(q[2]) * std::cos(q[0])) *
               std::sin(q[3]) -
           std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
          std::sin(q[5]) +
      0.21000000000000002 *
          ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
            std::sin(q[2]) * std::cos(q[0])) *
               std::sin(q[3]) -
           std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
          std::cos(q[5]) -
      0.38400000000000001 *
          (std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
           std::sin(q[2]) * std::cos(q[0])) *
          std::sin(q[3]) -
      0.082500000000000004 *
          (std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
           std::sin(q[2]) * std::cos(q[0])) *
          std::cos(q[3]) -
      0.082500000000000004 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) +
      0.38400000000000001 * std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3]) +
      0.316 * std::sin(q[0]) * std::sin(q[1]) +
      0.082500000000000004 * std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
      0.082500000000000004 * std::sin(q[2]) * std::cos(q[0]);
  pose.row(2)(0) = 1.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) *
                       std::sin(q[6] + M_PI_4) * std::cos(q[5]) +
                   1.0 * std::sin(q[1]) * std::sin(q[2]) * std::cos(q[4]) *
                       std::cos(q[6] + M_PI_4) +
                   1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[5]) *
                       std::sin(q[6] + M_PI_4) * std::cos(q[2]) +
                   1.0 * std::sin(q[1]) * std::sin(q[4]) * std::cos(q[2]) *
                       std::cos(q[3]) * std::cos(q[6] + M_PI_4) -
                   1.0 * std::sin(q[1]) * std::sin(q[6] + M_PI_4) *
                       std::cos(q[2]) * std::cos(q[3]) * std::cos(q[4]) *
                       std::cos(q[5]) -
                   1.0 * std::sin(q[3]) * std::sin(q[4]) * std::cos(q[1]) *
                       std::cos(q[6] + M_PI_4) +
                   1.0 * std::sin(q[3]) * std::sin(q[6] + M_PI_4) *
                       std::cos(q[1]) * std::cos(q[4]) * std::cos(q[5]) +
                   1.0 * std::sin(q[5]) * std::sin(q[6] + M_PI_4) *
                       std::cos(q[1]) * std::cos(q[3]);
  pose.row(2)(1) =
      1.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[5]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[4]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[2]) *
          std::cos(q[6] + M_PI_4) -
      1.0 * std::sin(q[1]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[2]) * std::cos(q[3]) -
      1.0 * std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[3]) * std::sin(q[4]) * std::sin(q[6] + M_PI_4) *
          std::cos(q[1]) +
      1.0 * std::sin(q[3]) * std::cos(q[1]) * std::cos(q[4]) * std::cos(q[5]) *
          std::cos(q[6] + M_PI_4) +
      1.0 * std::sin(q[5]) * std::cos(q[1]) * std::cos(q[3]) *
          std::cos(q[6] + M_PI_4);
  pose.row(2)(2) = -1.0 *
                       ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                         std::sin(q[3]) * std::cos(q[1])) *
                            std::cos(q[4]) -
                        std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
                       std::sin(q[5]) -
                   1.0 *
                       (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
                        std::cos(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]);
  pose.row(2)(3) =
      -0.21000000000000002 *
          ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
            std::sin(q[3]) * std::cos(q[1])) *
               std::cos(q[4]) -
           std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
          std::sin(q[5]) -
      0.087999999999999995 *
          ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
            std::sin(q[3]) * std::cos(q[1])) *
               std::cos(q[4]) -
           std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
          std::cos(q[5]) +
      0.087999999999999995 *
          (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
           std::cos(q[1]) * std::cos(q[3])) *
          std::sin(q[5]) -
      0.21000000000000002 *
          (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
           std::cos(q[1]) * std::cos(q[3])) *
          std::cos(q[5]) +
      0.38400000000000001 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
      0.082500000000000004 * std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
      0.082500000000000004 * std::sin(q[1]) * std::cos(q[2]) -
      0.082500000000000004 * std::sin(q[3]) * std::cos(q[1]) +
      0.38400000000000001 * std::cos(q[1]) * std::cos(q[3]) +
      0.316 * std::cos(q[1]) + 0.33300000000000002;
  pose.row(3)(3) = 1.0;
  return pose;
};

}  // namespace kinematics
