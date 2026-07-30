#include "arc_profile.h"

#define ARC_PI_X1000000 3141593LL

static int32_t ArcProfile_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t ArcProfile_LinearToYawRate(int32_t linear_speed_mm_s,
                                          int32_t radius_mm)
{
    if ((linear_speed_mm_s <= 0) || (radius_mm <= 0))
    {
        return 0;
    }
    return (int32_t)(((int64_t)linear_speed_mm_s * 180000LL * 1000000LL) /
                     (ARC_PI_X1000000 * radius_mm));
}

void ArcProfile_Init(ArcProfile *profile,
                     int32_t radius_mm,
                     int32_t maximum_linear_speed_mm_s,
                     int32_t minimum_linear_speed_mm_s,
                     int32_t slowdown_angle_mdeg,
                     int32_t tolerance_mdeg)
{
    profile->radius_mm = radius_mm;
    AngleProfile_Init(
        &profile->angle_profile,
        ArcProfile_LinearToYawRate(maximum_linear_speed_mm_s, radius_mm),
        ArcProfile_LinearToYawRate(minimum_linear_speed_mm_s, radius_mm),
        slowdown_angle_mdeg,
        tolerance_mdeg);
}

void ArcProfile_Start(ArcProfile *profile,
                      int32_t current_heading_mdeg,
                      int32_t relative_angle_mdeg)
{
    AngleProfile_Start(&profile->angle_profile,
                       current_heading_mdeg,
                       relative_angle_mdeg);
}

void ArcProfile_Update(ArcProfile *profile,
                       int32_t current_heading_mdeg,
                       int32_t *linear_speed_mm_s,
                       int32_t *yaw_rate_mdeg_s)
{
    int32_t yaw_rate = AngleProfile_Update(&profile->angle_profile,
                                            current_heading_mdeg);

    *yaw_rate_mdeg_s = yaw_rate;
    if ((yaw_rate == 0) || (profile->radius_mm <= 0))
    {
        *linear_speed_mm_s = 0;
        return;
    }

    *linear_speed_mm_s = (int32_t)(
        ((int64_t)ArcProfile_Abs(yaw_rate) * ARC_PI_X1000000 *
         profile->radius_mm) / (180000LL * 1000000LL));
}

int32_t ArcProfile_GetRemainingMdeg(const ArcProfile *profile,
                                    int32_t current_heading_mdeg)
{
    return AngleProfile_GetRemainingMdeg(&profile->angle_profile,
                                         current_heading_mdeg);
}

bool ArcProfile_IsFinished(const ArcProfile *profile)
{
    return AngleProfile_IsFinished(&profile->angle_profile);
}

int32_t ArcProfile_GetExpectedLengthUm(const ArcProfile *profile,
                                       int32_t relative_angle_mdeg)
{
    return (int32_t)(((int64_t)ArcProfile_Abs(relative_angle_mdeg) *
                      ARC_PI_X1000000 * profile->radius_mm) /
                     (180000LL * 1000LL));
}
