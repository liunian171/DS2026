#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>

typedef struct
{
    int32_t kp;
    int32_t maximum_yaw_rate_mdeg_s;
} LineFollower;

void LineFollower_Init(LineFollower *controller,
                       int32_t kp,
                       int32_t maximum_yaw_rate_mdeg_s);
int32_t LineFollower_Update(const LineFollower *controller,
                            int32_t line_position_x1000);

#endif /* LINE_FOLLOWER_H */
