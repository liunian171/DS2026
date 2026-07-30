#include "wheel_speed.h"

static int32_t WheelSpeed_CountsPerSecond(int32_t delta_counts,
                                          uint32_t elapsed_ms)
{
    return (int32_t)(((int64_t)delta_counts * 1000LL) /
                     (int64_t)elapsed_ms);
}

static int32_t WheelSpeed_RpmX10(int32_t counts_per_second,
                                 int32_t counts_per_revolution)
{
    return (int32_t)(((int64_t)counts_per_second * 600LL) /
                     (int64_t)counts_per_revolution);
}

void WheelSpeed_Init(WheelSpeedEstimator *estimator,
                     uint32_t sample_period_ms,
                     int32_t counts_per_revolution,
                     int32_t left_count,
                     int32_t right_count,
                     uint32_t now_ms)
{
    estimator->previous_left_count = left_count;
    estimator->previous_right_count = right_count;
    estimator->previous_ms = now_ms;
    estimator->sample_period_ms = sample_period_ms;
    estimator->counts_per_revolution = counts_per_revolution;
}

bool WheelSpeed_Update(WheelSpeedEstimator *estimator,
                       int32_t left_count,
                       int32_t right_count,
                       uint32_t now_ms,
                       WheelSpeedSample *sample)
{
    uint32_t elapsed_ms = now_ms - estimator->previous_ms;

    if ((elapsed_ms < estimator->sample_period_ms) ||
        (elapsed_ms == 0U) ||
        (estimator->counts_per_revolution <= 0))
    {
        return false;
    }

    sample->left_delta_counts = left_count - estimator->previous_left_count;
    sample->right_delta_counts = right_count - estimator->previous_right_count;
    sample->left_counts_per_second =
        WheelSpeed_CountsPerSecond(sample->left_delta_counts, elapsed_ms);
    sample->right_counts_per_second =
        WheelSpeed_CountsPerSecond(sample->right_delta_counts, elapsed_ms);
    sample->left_rpm_x10 =
        WheelSpeed_RpmX10(sample->left_counts_per_second,
                          estimator->counts_per_revolution);
    sample->right_rpm_x10 =
        WheelSpeed_RpmX10(sample->right_counts_per_second,
                          estimator->counts_per_revolution);
    sample->elapsed_ms = elapsed_ms;

    estimator->previous_left_count = left_count;
    estimator->previous_right_count = right_count;
    estimator->previous_ms = now_ms;
    return true;
}
