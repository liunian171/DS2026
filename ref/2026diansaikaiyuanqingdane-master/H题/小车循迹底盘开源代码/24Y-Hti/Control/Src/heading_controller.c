#include "heading_controller.h"

void HeadingController_Init(HeadingController *controller,
                            int32_t kp_x1000,
                            int32_t maximum_yaw_rate_mdeg_s)
{
    controller->target_heading_mdeg = 0;
    controller->kp_x1000 = kp_x1000;
    controller->maximum_yaw_rate_mdeg_s = maximum_yaw_rate_mdeg_s;
}

void HeadingController_Start(HeadingController *controller,
                             int32_t current_heading_mdeg)
{
    controller->target_heading_mdeg = current_heading_mdeg;
}

int32_t HeadingController_GetErrorMdeg(const HeadingController *controller,
                                       int32_t current_heading_mdeg)
{
    return controller->target_heading_mdeg - current_heading_mdeg;
}

int32_t HeadingController_Update(const HeadingController *controller,
                                 int32_t current_heading_mdeg)
{
    int32_t error_mdeg = HeadingController_GetErrorMdeg(
        controller, current_heading_mdeg);
    int32_t yaw_rate_mdeg_s = (int32_t)(
        ((int64_t)controller->kp_x1000 * error_mdeg) / 1000LL);

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
