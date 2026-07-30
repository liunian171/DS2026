#include "line_follower.h"

void LineFollower_Init(LineFollower *controller,
                       int32_t kp,
                       int32_t maximum_yaw_rate_mdeg_s)
{
    controller->kp = kp;
    controller->maximum_yaw_rate_mdeg_s = maximum_yaw_rate_mdeg_s;
}

int32_t LineFollower_Update(const LineFollower *controller,
                            int32_t line_position_x1000)
{
    /* D2 is left and has a negative position. A line on the left therefore
       requires positive (left) yaw. */
    int32_t yaw_rate_mdeg_s = -controller->kp * line_position_x1000;

    if (yaw_rate_mdeg_s > controller->maximum_yaw_rate_mdeg_s)
    {
        yaw_rate_mdeg_s = controller->maximum_yaw_rate_mdeg_s;
    }
    else if (yaw_rate_mdeg_s < -controller->maximum_yaw_rate_mdeg_s)
    {
        yaw_rate_mdeg_s = -controller->maximum_yaw_rate_mdeg_s;
    }
    return yaw_rate_mdeg_s;
}
