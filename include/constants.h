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
const Vector7d kPDZoneDamping(kPDZoneDamping);

const double kDZoneDampingData[7] = {30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0};
const Vector7d kDZoneDamping(kDZoneDampingData);
