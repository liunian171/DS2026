#include "chassis_kinematics.h"

#define CHASSIS_PI_X1000000 3141593LL

void ChassisKinematics_Init(ChassisKinematicsConfig *config,
                            int32_t wheel_diameter_mm,
                            int32_t track_width_mm,
                            int32_t counts_per_wheel_revolution)
{
    config->wheel_diameter_mm = wheel_diameter_mm;
    config->track_width_mm = track_width_mm;
    config->counts_per_wheel_revolution = counts_per_wheel_revolution;
}

void ChassisKinematics_VelocityToWheelCps(
    const ChassisKinematicsConfig *config,
    int32_t linear_mm_per_second,
    int32_t yaw_mdeg_per_second,
    int32_t *left_cps,
    int32_t *right_cps)
{
    int64_t linear_cps;
    int64_t turn_cps;

    if ((config->wheel_diameter_mm <= 0) ||
        (config->track_width_mm <= 0) ||
        (config->counts_per_wheel_revolution <= 0))
    {
        *left_cps = 0;
        *right_cps = 0;
        return;
    }

    linear_cps = ((int64_t)linear_mm_per_second *
                  config->counts_per_wheel_revolution * 1000000LL) /
                 (CHASSIS_PI_X1000000 * config->wheel_diameter_mm);

    /* PI cancels when wheel arc speed is converted back to encoder CPS. */
    turn_cps = ((int64_t)yaw_mdeg_per_second * config->track_width_mm *
                config->counts_per_wheel_revolution) /
               (360000LL * config->wheel_diameter_mm);

    *left_cps = (int32_t)(linear_cps - turn_cps);
    *right_cps = (int32_t)(linear_cps + turn_cps);
}

int32_t ChassisKinematics_WheelCpsToLinearMmS(
    const ChassisKinematicsConfig *config,
    int32_t left_cps,
    int32_t right_cps)
{
    int64_t average_cps;

    if ((config->wheel_diameter_mm <= 0) ||
        (config->counts_per_wheel_revolution <= 0))
    {
        return 0;
    }

    average_cps = ((int64_t)left_cps + right_cps) / 2LL;
    return (int32_t)((average_cps * CHASSIS_PI_X1000000 *
                      config->wheel_diameter_mm) /
                     ((int64_t)config->counts_per_wheel_revolution *
                      1000000LL));
}

void ChassisOdometry_Reset(ChassisOdometry *odometry)
{
    odometry->distance_um = 0;
    odometry->heading_mdeg = 0;
    odometry->distance_remainder = 0;
    odometry->heading_remainder = 0;
}

void ChassisOdometry_Update(const ChassisKinematicsConfig *config,
                            ChassisOdometry *odometry,
                            int32_t left_delta_counts,
                            int32_t right_delta_counts)
{
    int64_t distance_numerator;
    int64_t distance_denominator;
    int64_t heading_numerator;
    int64_t heading_denominator;
    int64_t distance_increment_um;
    int64_t heading_increment_mdeg;

    if ((config->wheel_diameter_mm <= 0) ||
        (config->track_width_mm <= 0) ||
        (config->counts_per_wheel_revolution <= 0))
    {
        return;
    }

    distance_numerator =
        (int64_t)(left_delta_counts + right_delta_counts) *
        CHASSIS_PI_X1000000 * config->wheel_diameter_mm * 1000LL +
        odometry->distance_remainder;
    distance_denominator =
        2LL * config->counts_per_wheel_revolution * 1000000LL;
    distance_increment_um = distance_numerator / distance_denominator;
    odometry->distance_remainder =
        distance_numerator % distance_denominator;

    heading_numerator =
        (int64_t)(right_delta_counts - left_delta_counts) *
        config->wheel_diameter_mm * 180000LL +
        odometry->heading_remainder;
    heading_denominator =
        (int64_t)config->counts_per_wheel_revolution *
        config->track_width_mm;
    heading_increment_mdeg = heading_numerator / heading_denominator;
    odometry->heading_remainder = heading_numerator % heading_denominator;

    odometry->distance_um += (int32_t)distance_increment_um;
    odometry->heading_mdeg += (int32_t)heading_increment_mdeg;
}
