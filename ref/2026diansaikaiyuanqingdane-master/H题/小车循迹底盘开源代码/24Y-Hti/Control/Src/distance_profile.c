#include "distance_profile.h"

static int32_t DistanceProfile_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

void DistanceProfile_Init(DistanceProfile *profile,
                          int32_t maximum_speed_mm_s,
                          int32_t minimum_speed_mm_s,
                          int32_t slowdown_distance_mm,
                          int32_t tolerance_mm)
{
    profile->maximum_speed_mm_s = maximum_speed_mm_s;
    profile->minimum_speed_mm_s = minimum_speed_mm_s;
    profile->slowdown_distance_um = slowdown_distance_mm * 1000;
    profile->tolerance_um = tolerance_mm * 1000;
    profile->target_distance_um = 0;
    profile->finished = true;
}

void DistanceProfile_Start(DistanceProfile *profile,
                           int32_t current_distance_um,
                           int32_t relative_distance_mm)
{
    profile->target_distance_um =
        current_distance_um + relative_distance_mm * 1000;
    profile->finished = false;
}

int32_t DistanceProfile_Update(DistanceProfile *profile,
                               int32_t current_distance_um)
{
    int32_t remaining_um;
    int32_t remaining_abs_um;
    int32_t speed_mm_s;
    int32_t scaling_range_um;

    if (profile->finished)
    {
        return 0;
    }

    remaining_um = profile->target_distance_um - current_distance_um;
    remaining_abs_um = DistanceProfile_Abs(remaining_um);
    if (remaining_abs_um <= profile->tolerance_um)
    {
        profile->finished = true;
        return 0;
    }

    speed_mm_s = profile->maximum_speed_mm_s;
    scaling_range_um =
        profile->slowdown_distance_um - profile->tolerance_um;
    if ((remaining_abs_um < profile->slowdown_distance_um) &&
        (scaling_range_um > 0))
    {
        speed_mm_s = profile->minimum_speed_mm_s +
            (int32_t)(((int64_t)(profile->maximum_speed_mm_s -
                                 profile->minimum_speed_mm_s) *
                       (remaining_abs_um - profile->tolerance_um)) /
                      scaling_range_um);
    }

    return (remaining_um < 0) ? -speed_mm_s : speed_mm_s;
}

int32_t DistanceProfile_GetRemainingUm(const DistanceProfile *profile,
                                       int32_t current_distance_um)
{
    return profile->target_distance_um - current_distance_um;
}

bool DistanceProfile_IsFinished(const DistanceProfile *profile)
{
    return profile->finished;
}
