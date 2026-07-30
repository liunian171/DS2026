#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t left_delta_counts;
    int32_t right_delta_counts;
    int32_t left_counts_per_second;
    int32_t right_counts_per_second;
    int32_t left_rpm_x10;
    int32_t right_rpm_x10;
    uint32_t elapsed_ms;
} WheelSpeedSample;

typedef struct
{
    int32_t previous_left_count;
    int32_t previous_right_count;
    uint32_t previous_ms;
    uint32_t sample_period_ms;
    int32_t counts_per_revolution;
} WheelSpeedEstimator;

void WheelSpeed_Init(WheelSpeedEstimator *estimator,
                     uint32_t sample_period_ms,
                     int32_t counts_per_revolution,
                     int32_t left_count,
                     int32_t right_count,
                     uint32_t now_ms);

bool WheelSpeed_Update(WheelSpeedEstimator *estimator,
                       int32_t left_count,
                       int32_t right_count,
                       uint32_t now_ms,
                       WheelSpeedSample *sample);

#endif /* WHEEL_SPEED_H */
