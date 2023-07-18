#pragma once

namespace controllers {
namespace joint_limits {

class VirtualWall {
 public:
  VirtualWall() = delete;

  VirtualWall(const double& soft_upper_joint_position_limit,
            const double& soft_lower_joint_position_limit,
            const double& PD_zone_width, const double& D_zone_width,
            const double& PD_zone_stiffness, const double& PD_zone_damping,
            const double& D_zone_damping);

  double computeTorque(const double& q, const double& dq);
  void reset();

 private:
  enum class MotionInWall {
    EnteringNormal,
    PenetratingLowerLimit,
    LeavingLowerLimit,
    PenetratingUpperLimit,
    LeavingUpperLimit
  };
  double soft_upper_joint_position_limit_{0};
  double soft_lower_joint_position_limit_{0};
  double PD_zone_width_{0};
  double D_zone_width_{0};
  double PD_zone_stiffness_{0};
  double PD_zone_damping_{0};
  double D_zone_damping_{0};
  bool initialized_{false};
  bool moving_wall_{false};
  double zone_width_scale_{1};

  static bool inRange(double low, double high, double x);
  static double positiveCheck(double value);
  void init(const double& q, const double& dq);
  void adjustMovingWall(const double& q, const double& dq);
  MotionInWall getMotionInWall(const double& q, const double& dq) const;
};
}  // namespace joint_limits
}  // namespace controllers
