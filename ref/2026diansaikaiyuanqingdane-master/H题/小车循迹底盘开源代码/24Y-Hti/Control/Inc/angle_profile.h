#ifndef ANGLE_PROFILE_H
#define ANGLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t target_heading_mdeg;
    int32_t maximum_yaw_rate_mdeg_s;
    int32_t minimum_yaw_rate_mdeg_s;
    int32_t slowdown_angle_mdeg;
    int32_t tolerance_mdeg;
    bool finished;
} AngleProfile;

void AngleProfile_Init(AngleProfile *profile,
                       int32_t maximum_yaw_rate_mdeg_s,
                       int32_t minimum_yaw_rate_mdeg_s,
                       int32_t slowdown_angle_mdeg,
                       int32_t tolerance_mdeg);
void AngleProfile_Start(AngleProfile *profile,
                        int32_t current_heading_mdeg,
                        int32_t relative_angle_mdeg);
int32_t AngleProfile_Update(AngleProfile *profile,
                            int32_t current_heading_mdeg);
int32_t AngleProfile_GetRemainingMdeg(const AngleProfile *profile,
                                      int32_t current_heading_mdeg);
bool AngleProfile_IsFinished(const AngleProfile *profile);

#endif /* ANGLE_PROFILE_H */
