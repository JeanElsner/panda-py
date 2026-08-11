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

// The FR3 has a different joint envelope from the FER, most obviously on joint 6
// where it reaches 4.5 rad and does not go below 0.44, while the FER covers
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

const double kUpperJointLimitsFR3_5_9Data[7] = {
    2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
const Vector7d kUpperJointLimitsFR3_5_9(kUpperJointLimitsFR3_5_9Data);

struct JointLimits {
  Vector7d lower;
  Vector7d upper;
  const char *name;
};

// The research interface protocol version identifies the robot generation and,
// for the FR3, the system version closely enough to pick the right envelope:
//
//   <= 5  FER, robot system >= 3.0.0
//    6-9  FR3, robot system 5.2.0 to 5.8.x
//   >= 10 FR3, robot system >= 5.9.0
//
// Falling back to the FER limits for anything unrecognised keeps the previous
// behaviour for robots this mapping does not know about.
inline JointLimits jointLimitsForServerVersion(uint16_t server_version) {
  if (server_version >= 10) {
    return {kLowerJointLimitsFR3_5_9, kUpperJointLimitsFR3_5_9,
            "FR3 (robot system >= 5.9.0)"};
  }
  if (server_version >= 6) {
    return {kLowerJointLimitsFR3, kUpperJointLimitsFR3,
            "FR3 (robot system < 5.9.0)"};
  }
  return {kLowerJointLimits, kUpperJointLimits, "FER"};
}

const double kQMaxVelocityData[7] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100}; //{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
const Vector7d kQMaxVelocity(kQMaxVelocityData);

const double kQMaxAccelerationData[7] = {15, 7.5, 10, 12.5, 15, 20, 20};
const Vector7d kQMaxAcceleration(kQMaxAccelerationData);

const double kXMaxVelocityData[4] = {1.7, 1.7, 1.7, 2.5}; //{1.7, 1.7, 1.7, 2.5};
const Eigen::Vector4d kXMaxVelocity(kXMaxVelocityData);

const double kXMaxAccelerationData[4] = {13, 13, 13, 25};
const Eigen::Vector4d kXMaxAcceleration(kXMaxAccelerationData);

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
