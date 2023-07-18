#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>

#include "utils.h"
#include "controllers/joint_limits/virtual_wall.h"

namespace controllers {
namespace joint_limits {

class VirtualWallController {
 public:
  VirtualWallController(
      const Vector7d& soft_upper_joint_position_limits,
      const Vector7d& soft_lower_joint_position_limits,
      const Vector7d& PD_zone_widths,
      const Vector7d& D_zone_widths,
      const Vector7d& PD_zone_stiffnesses,
      const Vector7d& PD_zone_dampings,
      const Vector7d& D_zone_dampings) {
    for (size_t i = 0; i < 7; i++) {
      virtual_walls_.at(i) = std::make_unique<VirtualWall>(
          soft_upper_joint_position_limits[i],
          soft_lower_joint_position_limits[i], PD_zone_widths[i],
          D_zone_widths[i], PD_zone_stiffnesses[i], PD_zone_dampings[i],
          D_zone_dampings[i]);
    }
  }

  VirtualWallController() = delete;

  void computeTorque(const Array7d& q,
                     const Array7d& dq,
                     Array7d& torque) {
    for (size_t i = 0; i < 7; i++) {
      torque[i] = virtual_walls_[i]->computeTorque(q[i], dq[i]);
    };
  }

  void reset() {
    for (auto& jw : virtual_walls_) {
      jw->reset();
    }
  }

 private:
  std::array<std::unique_ptr<VirtualWall>, 7> virtual_walls_;
};
}  // namespace joint_limits
}  // namespace controllers
