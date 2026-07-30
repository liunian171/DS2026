#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

typedef struct
{
    int32_t wheel_diameter_mm;
    int32_t track_width_mm;
    int32_t counts_per_wheel_revolution;
} ChassisKinematicsConfig;

typedef struct
{
    int32_t distance_um;
    int32_t heading_mdeg;
    int64_t distance_remainder;
    int64_t heading_remainder;
} ChassisOdometry;

void ChassisKinematics_Init(ChassisKinematicsConfig *config,
                            int32_t wheel_diameter_mm,
                            int32_t track_width_mm,
                            int32_t counts_per_wheel_revolution);
void ChassisKinematics_VelocityToWheelCps(
    const ChassisKinematicsConfig *config,
    int32_t linear_mm_per_second,
    int32_t yaw_mdeg_per_second,
    int32_t *left_cps,
    int32_t *right_cps);
int32_t ChassisKinematics_WheelCpsToLinearMmS(
    const ChassisKinematicsConfig *config,
    int32_t left_cps,
    int32_t right_cps);
void ChassisOdometry_Reset(ChassisOdometry *odometry);
void ChassisOdometry_Update(const ChassisKinematicsConfig *config,
                            ChassisOdometry *odometry,
                            int32_t left_delta_counts,
                            int32_t right_delta_counts);

#endif /* CHASSIS_KINEMATICS_H */
