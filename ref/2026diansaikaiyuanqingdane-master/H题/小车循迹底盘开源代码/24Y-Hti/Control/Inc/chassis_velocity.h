#ifndef CHASSIS_VELOCITY_H
#define CHASSIS_VELOCITY_H

#include <stdint.h>

typedef struct
{
    int32_t desired_left_cps;
    int32_t desired_right_cps;
    int32_t current_left_cps;
    int32_t current_right_cps;
    int32_t max_wheel_cps;
    int32_t slew_rate_cps_per_second;
} ChassisVelocity;

void ChassisVelocity_Init(ChassisVelocity *chassis,
                          int32_t max_wheel_cps,
                          int32_t slew_rate_cps_per_second);
void ChassisVelocity_Reset(ChassisVelocity *chassis);
void ChassisVelocity_SetSlewRate(ChassisVelocity *chassis,
                                 int32_t slew_rate_cps_per_second);
void ChassisVelocity_SetCommand(ChassisVelocity *chassis,
                                int32_t translation_cps,
                                int32_t turn_cps);
void ChassisVelocity_Stop(ChassisVelocity *chassis);
void ChassisVelocity_Update(ChassisVelocity *chassis,
                            uint32_t elapsed_ms);
int32_t ChassisVelocity_GetLeftTarget(const ChassisVelocity *chassis);
int32_t ChassisVelocity_GetRightTarget(const ChassisVelocity *chassis);

#endif /* CHASSIS_VELOCITY_H */
