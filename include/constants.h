#pragma once
#include <Eigen/Dense>

using Vector7d = Eigen::Matrix<double, 7, 1>;

const double kTauJMaxData[7] = {87, 87, 87, 87, 12, 12, 12};
const Vector7d kTauJMax(kTauJMaxData);

const double kDTauJMaxData[7] = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
const Vector7d kDTauJMax(kDTauJMaxData);

const double kJointPositionStartData[7] = {0.0, -M_PI_4, 0.0,   -3 * M_PI_4,
                                           0.0, M_PI_2,  M_PI_4};
const Vector7d kJointPositionStart(kJointPositionStartData);

const double kLowerJointLimitsData[7] = {-2.8973, -1.7628, -2.8973, -3.0718,
                                         -2.8973, -0.0175, -2.8973};
const Vector7d kLowerJointLimits(kLowerJointLimitsData);

const double kUpperJointLimitsData[7] = {2.8973, 1.7628, 2.8973, -0.0698,
                                         2.8973, 3.7525, 2.8973};
const Vector7d kUpperJointLimits(kUpperJointLimitsData);

// The FR3 has a different joint envelope from the FER, most obviously on joint
// 6 where it reaches 4.5 rad and does not go below 0.44, while the FER covers
// -0.0175 to 3.7525. Franka also widened the FR3 limits to the datasheet values
// with robot system 5.9.0; the earlier set is a strict subset of the later one.
const double kLowerJointLimitsFR3Data[7] = {-2.7437, -1.7837, -2.9007, -3.0421,
                                            -2.8065, 0.5445,  -3.0159};
const Vector7d kLowerJointLimitsFR3(kLowerJointLimitsFR3Data);

const double kUpperJointLimitsFR3Data[7] = {2.7437, 1.7837, 2.9007, -0.1518,
                                            2.8065, 4.5169, 3.0159};
const Vector7d kUpperJointLimitsFR3(kUpperJointLimitsFR3Data);

const double kLowerJointLimitsFR3_5_9Data[7] = {
    -2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
const Vector7d kLowerJointLimitsFR3_5_9(kLowerJointLimitsFR3_5_9Data);

const double kUpperJointLimitsFR3_5_9Data[7] = {2.9007, 1.8361, 2.9007, -0.1169,
                                                2.8763, 4.6216, 3.0508};
const Vector7d kUpperJointLimitsFR3_5_9(kUpperJointLimitsFR3_5_9Data);

// Motion limits for planning (the motion generators) and the defaults of the
// joint velocity guard. The FER's are libfranka 0.9.2's constants; the FR3's
// are the maxima of its robot description (velocity) and libfranka's rate
// limiter (acceleration), its velocity limits proper depending on the joint
// positions. Cartesian limits are [translation x 3, rotation], m/s and rad/s.
const double kQMaxVelocityFERData[7] = {2.175, 2.175, 2.175, 2.175,
                                        2.61,  2.61,  2.61};
const double kQMaxAccelerationFERData[7] = {15, 7.5, 10, 12.5, 15, 20, 20};
const double kXMaxVelocityFERData[4] = {1.7, 1.7, 1.7, 2.5};
const double kXMaxAccelerationFERData[4] = {13, 13, 13, 25};
const double kQMaxVelocityFR3Data[7] = {2.62, 2.62, 2.62, 2.62,
                                        5.26, 4.18, 5.26};
const double kQMaxAccelerationFR3Data[7] = {10, 10, 10, 10, 10, 10, 10};
const double kXMaxVelocityFR3Data[4] = {3.0, 3.0, 3.0, 2.5};
const double kXMaxAccelerationFR3Data[4] = {9, 9, 9, 17};

enum class RobotType { kFER, kFR3 };

/// Everything panda-py needs to know about the connected robot's envelope.
struct RobotLimits {
  RobotType type;
  const char* name;
  Vector7d q_lower, q_upper;
  Vector7d dq_max, ddq_max;
  Eigen::Vector4d dx_max, ddx_max;
};

// The research interface protocol version identifies the robot generation and,
// for the FR3, the system version closely enough to pick the right envelope:
//
//   <= 5  FER
//    6-9  FR3, robot system before 5.9.0
//   >= 10 FR3, robot system 5.9.0 and later
inline RobotLimits limitsForServerVersion(uint16_t server_version) {
  if (server_version <= 5) {
    return {RobotType::kFER,
            "FER",
            kLowerJointLimits,
            kUpperJointLimits,
            Vector7d(kQMaxVelocityFERData),
            Vector7d(kQMaxAccelerationFERData),
            Eigen::Vector4d(kXMaxVelocityFERData),
            Eigen::Vector4d(kXMaxAccelerationFERData)};
  }
  const bool new_envelope = server_version >= 10;
  return {RobotType::kFR3,
          new_envelope ? "FR3 (robot system >= 5.9.0)" : "FR3 (robot system < 5.9.0)",
          new_envelope ? kLowerJointLimitsFR3_5_9 : kLowerJointLimitsFR3,
          new_envelope ? kUpperJointLimitsFR3_5_9 : kUpperJointLimitsFR3,
          Vector7d(kQMaxVelocityFR3Data),
          Vector7d(kQMaxAccelerationFR3Data),
          Eigen::Vector4d(kXMaxVelocityFR3Data),
          Eigen::Vector4d(kXMaxAccelerationFR3Data)};
}

/// For planning without a robot: per joint the smaller of the FER's and the
/// FR3's, so a trajectory is valid on either.
inline RobotLimits conservativeLimits() {
  const RobotLimits fer = limitsForServerVersion(5), fr3 = limitsForServerVersion(10);
  return {fer.type,
          "FER and FR3",
          fer.q_lower.cwiseMax(kLowerJointLimitsFR3),
          fer.q_upper.cwiseMin(kUpperJointLimitsFR3),
          fer.dq_max.cwiseMin(fr3.dq_max),
          fer.ddq_max.cwiseMin(fr3.ddq_max),
          fer.dx_max.cwiseMin(fr3.dx_max),
          fer.ddx_max.cwiseMin(fr3.ddx_max)};
}

const double kPDZoneWidthData[7] = {0.12,   0.09,   0.09,  0.09,
                                    0.0349, 0.0349, 0.0349};
const Vector7d kPDZoneWidth(kPDZoneWidthData);

const double kDZoneWidthData[7] = {0.12,   0.09,   0.09,  0.09,
                                   0.0349, 0.0349, 0.0349};
const Vector7d kDZoneWidth(kDZoneWidthData);

const double kPDZoneStiffnessData[7] = {2000.0, 2000.0, 1000.0, 1000.0,
                                        500.0,  200.0,  200.0};
const Vector7d kPDZoneStiffness(kPDZoneStiffnessData);

const double kPDZoneDampingData[7] = {30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0};
const Vector7d kPDZoneDamping(kPDZoneDampingData);

const double kDZoneDampingData[7] = {30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0};
const Vector7d kDZoneDamping(kDZoneDampingData);
