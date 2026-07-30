#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    int32_t target_heading_mdeg;
    int32_t kp_x1000;
    int32_t maximum_yaw_rate_mdeg_s;
} HeadingController;

void HeadingController_Init(HeadingController *controller,
                            int32_t kp_x1000,
                            int32_t maximum_yaw_rate_mdeg_s);
void HeadingController_Start(HeadingController *controller,
                             int32_t current_heading_mdeg);
int32_t HeadingController_Update(const HeadingController *controller,
                                 int32_t current_heading_mdeg);
int32_t HeadingController_GetErrorMdeg(const HeadingController *controller,
                                       int32_t current_heading_mdeg);

#endif /* HEADING_CONTROLLER_H */
