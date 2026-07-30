#include "chassis_velocity.h"

static int32_t ChassisVelocity_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t ChassisVelocity_StepTowards(int32_t current,
                                           int32_t desired,
                                           int32_t maximum_step)
{
    int32_t difference = desired - current;

    if (difference > maximum_step)
    {
        return current + maximum_step;
    }
    if (difference < -maximum_step)
    {
        return current - maximum_step;
    }
    return desired;
}

void ChassisVelocity_Init(ChassisVelocity *chassis,
                          int32_t max_wheel_cps,
                          int32_t slew_rate_cps_per_second)
{
    chassis->max_wheel_cps = max_wheel_cps;
    chassis->slew_rate_cps_per_second = slew_rate_cps_per_second;
    ChassisVelocity_Reset(chassis);
}

void ChassisVelocity_Reset(ChassisVelocity *chassis)
{
    chassis->desired_left_cps = 0;
    chassis->desired_right_cps = 0;
    chassis->current_left_cps = 0;
    chassis->current_right_cps = 0;
}

void ChassisVelocity_SetSlewRate(ChassisVelocity *chassis,
                                 int32_t slew_rate_cps_per_second)
{
    chassis->slew_rate_cps_per_second =
        (slew_rate_cps_per_second > 0) ?
        slew_rate_cps_per_second : 1;
}

void ChassisVelocity_SetCommand(ChassisVelocity *chassis,
                                int32_t translation_cps,
                                int32_t turn_cps)
{
    int32_t left_cps = translation_cps - turn_cps;
    int32_t right_cps = translation_cps + turn_cps;
    int32_t largest_magnitude = ChassisVelocity_Abs(left_cps);

    if (ChassisVelocity_Abs(right_cps) > largest_magnitude)
    {
        largest_magnitude = ChassisVelocity_Abs(right_cps);
    }

    if ((largest_magnitude > chassis->max_wheel_cps) &&
        (largest_magnitude > 0))
    {
        left_cps = (int32_t)(((int64_t)left_cps * chassis->max_wheel_cps) /
                             largest_magnitude);
        right_cps = (int32_t)(((int64_t)right_cps * chassis->max_wheel_cps) /
                              largest_magnitude);
    }

    chassis->desired_left_cps = left_cps;
    chassis->desired_right_cps = right_cps;
}

void ChassisVelocity_Stop(ChassisVelocity *chassis)
{
    chassis->desired_left_cps = 0;
    chassis->desired_right_cps = 0;
}

void ChassisVelocity_Update(ChassisVelocity *chassis,
                            uint32_t elapsed_ms)
{
    int32_t maximum_step;

    if (elapsed_ms == 0U)
    {
        return;
    }

    maximum_step = (int32_t)(((int64_t)chassis->slew_rate_cps_per_second *
                              elapsed_ms) /
                             1000LL);
    if (maximum_step < 1)
    {
        maximum_step = 1;
    }

    chassis->current_left_cps = ChassisVelocity_StepTowards(
        chassis->current_left_cps,
        chassis->desired_left_cps,
        maximum_step);
    chassis->current_right_cps = ChassisVelocity_StepTowards(
        chassis->current_right_cps,
        chassis->desired_right_cps,
        maximum_step);
}

int32_t ChassisVelocity_GetLeftTarget(const ChassisVelocity *chassis)
{
    return chassis->current_left_cps;
}

int32_t ChassisVelocity_GetRightTarget(const ChassisVelocity *chassis)
{
    return chassis->current_right_cps;
}
