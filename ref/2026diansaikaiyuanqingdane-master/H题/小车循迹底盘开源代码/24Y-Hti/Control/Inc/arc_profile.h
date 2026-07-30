#ifndef ARC_PROFILE_H
#define ARC_PROFILE_H

#include "angle_profile.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    AngleProfile angle_profile;
    int32_t radius_mm;
} ArcProfile;

void ArcProfile_Init(ArcProfile *profile,
                     int32_t radius_mm,
                     int32_t maximum_linear_speed_mm_s,
                     int32_t minimum_linear_speed_mm_s,
                     int32_t slowdown_angle_mdeg,
                     int32_t tolerance_mdeg);
void ArcProfile_Start(ArcProfile *profile,
                      int32_t current_heading_mdeg,
                      int32_t relative_angle_mdeg);
void ArcProfile_Update(ArcProfile *profile,
                       int32_t current_heading_mdeg,
                       int32_t *linear_speed_mm_s,
                       int32_t *yaw_rate_mdeg_s);
int32_t ArcProfile_GetRemainingMdeg(const ArcProfile *profile,
                                    int32_t current_heading_mdeg);
bool ArcProfile_IsFinished(const ArcProfile *profile);
int32_t ArcProfile_GetExpectedLengthUm(const ArcProfile *profile,
                                       int32_t relative_angle_mdeg);

#endif /* ARC_PROFILE_H */
