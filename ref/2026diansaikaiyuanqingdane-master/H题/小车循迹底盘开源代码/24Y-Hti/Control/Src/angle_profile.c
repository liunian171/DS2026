#include "angle_profile.h"

static int32_t AngleProfile_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

void AngleProfile_Init(AngleProfile *profile,
                       int32_t maximum_yaw_rate_mdeg_s,
                       int32_t minimum_yaw_rate_mdeg_s,
                       int32_t slowdown_angle_mdeg,
                       int32_t tolerance_mdeg)
{
    profile->maximum_yaw_rate_mdeg_s = maximum_yaw_rate_mdeg_s;
    profile->minimum_yaw_rate_mdeg_s = minimum_yaw_rate_mdeg_s;
    profile->slowdown_angle_mdeg = slowdown_angle_mdeg;
    profile->tolerance_mdeg = tolerance_mdeg;
    profile->target_heading_mdeg = 0;
    profile->finished = true;
}

void AngleProfile_Start(AngleProfile *profile,
                        int32_t current_heading_mdeg,
                        int32_t relative_angle_mdeg)
{
    profile->target_heading_mdeg =
        current_heading_mdeg + relative_angle_mdeg;
    profile->finished = false;
}

int32_t AngleProfile_Update(AngleProfile *profile,
                            int32_t current_heading_mdeg)
{
    int32_t remaining_mdeg;
    int32_t remaining_abs_mdeg;
    int32_t yaw_rate_mdeg_s;
    int32_t scaling_range_mdeg;

    if (profile->finished)
    {
        return 0;
    }

    remaining_mdeg = profile->target_heading_mdeg - current_heading_mdeg;
    remaining_abs_mdeg = AngleProfile_Abs(remaining_mdeg);
    if (remaining_abs_mdeg <= profile->tolerance_mdeg)
    {
        profile->finished = true;
        return 0;
    }

    yaw_rate_mdeg_s = profile->maximum_yaw_rate_mdeg_s;
    scaling_range_mdeg =
        profile->slowdown_angle_mdeg - profile->tolerance_mdeg;
    if ((remaining_abs_mdeg < profile->slowdown_angle_mdeg) &&
        (scaling_range_mdeg > 0))
    {
        yaw_rate_mdeg_s = profile->minimum_yaw_rate_mdeg_s +
            (int32_t)(((int64_t)(profile->maximum_yaw_rate_mdeg_s -
                                 profile->minimum_yaw_rate_mdeg_s) *
                       (remaining_abs_mdeg - profile->tolerance_mdeg)) /
                      scaling_range_mdeg);
    }

    return (remaining_mdeg < 0) ? -yaw_rate_mdeg_s : yaw_rate_mdeg_s;
}

int32_t AngleProfile_GetRemainingMdeg(const AngleProfile *profile,
                                      int32_t current_heading_mdeg)
{
    return profile->target_heading_mdeg - current_heading_mdeg;
}

bool AngleProfile_IsFinished(const AngleProfile *profile)
{
    return profile->finished;
}
