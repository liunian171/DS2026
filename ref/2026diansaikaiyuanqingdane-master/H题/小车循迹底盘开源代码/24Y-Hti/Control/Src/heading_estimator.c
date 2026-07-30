#include "heading_estimator.h"

#define HEADING_BLEND_SCALE       1000
#define HEADING_HALF_TURN_MDEG  180000
#define HEADING_FULL_TURN_MDEG  360000

static int32_t HeadingEstimator_WrappedDelta(int32_t current_mdeg,
                                             int32_t previous_mdeg)
{
    int64_t delta = (int64_t)current_mdeg - previous_mdeg;

    while (delta > HEADING_HALF_TURN_MDEG)
    {
        delta -= HEADING_FULL_TURN_MDEG;
    }
    while (delta < -HEADING_HALF_TURN_MDEG)
    {
        delta += HEADING_FULL_TURN_MDEG;
    }
    return (int32_t)delta;
}

void HeadingEstimator_Init(HeadingEstimator *estimator,
                           int32_t gyro_sign,
                           int32_t gyro_weight_permille)
{
    if (gyro_weight_permille < 0)
    {
        gyro_weight_permille = 0;
    }
    else if (gyro_weight_permille > HEADING_BLEND_SCALE)
    {
        gyro_weight_permille = HEADING_BLEND_SCALE;
    }

    estimator->heading_mdeg = 0;
    estimator->last_encoder_heading_mdeg = 0;
    estimator->last_gyro_heading_mdeg = 0;
    estimator->gyro_sign = (gyro_sign < 0) ? -1 : 1;
    estimator->gyro_weight_permille = gyro_weight_permille;
    estimator->blend_remainder = 0;
    estimator->initialized = false;
    estimator->gyro_tracking = false;
}

int32_t HeadingEstimator_ConvertGyroMdeg(const HeadingEstimator *estimator,
                                         int32_t gyro_raw_heading_mdeg)
{
    return estimator->gyro_sign * gyro_raw_heading_mdeg;
}

void HeadingEstimator_Update(HeadingEstimator *estimator,
                             int32_t encoder_heading_mdeg,
                             int32_t gyro_raw_heading_mdeg,
                             bool gyro_valid)
{
    int32_t gyro_heading_mdeg = HeadingEstimator_ConvertGyroMdeg(
        estimator, gyro_raw_heading_mdeg);
    int32_t encoder_delta_mdeg;
    int32_t gyro_delta_mdeg;
    int64_t blended_numerator;

    if (!estimator->initialized)
    {
        estimator->last_encoder_heading_mdeg = encoder_heading_mdeg;
        estimator->last_gyro_heading_mdeg = gyro_heading_mdeg;
        estimator->initialized = true;
        estimator->gyro_tracking = gyro_valid;
        return;
    }

    encoder_delta_mdeg = encoder_heading_mdeg -
                         estimator->last_encoder_heading_mdeg;
    estimator->last_encoder_heading_mdeg = encoder_heading_mdeg;

    if (!gyro_valid)
    {
        estimator->heading_mdeg += encoder_delta_mdeg;
        estimator->gyro_tracking = false;
        return;
    }

    if (!estimator->gyro_tracking)
    {
        estimator->last_gyro_heading_mdeg = gyro_heading_mdeg;
        estimator->heading_mdeg += encoder_delta_mdeg;
        estimator->gyro_tracking = true;
        return;
    }

    gyro_delta_mdeg = HeadingEstimator_WrappedDelta(
        gyro_heading_mdeg, estimator->last_gyro_heading_mdeg);
    estimator->last_gyro_heading_mdeg = gyro_heading_mdeg;
    blended_numerator =
        (int64_t)estimator->gyro_weight_permille * gyro_delta_mdeg +
        (int64_t)(HEADING_BLEND_SCALE - estimator->gyro_weight_permille) *
        encoder_delta_mdeg + estimator->blend_remainder;
    estimator->heading_mdeg +=
        (int32_t)(blended_numerator / HEADING_BLEND_SCALE);
    estimator->blend_remainder =
        (int32_t)(blended_numerator % HEADING_BLEND_SCALE);
}

int32_t HeadingEstimator_GetHeadingMdeg(const HeadingEstimator *estimator)
{
    return estimator->heading_mdeg;
}
