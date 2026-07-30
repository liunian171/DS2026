#ifndef HEADING_ESTIMATOR_H
#define HEADING_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t heading_mdeg;
    int32_t last_encoder_heading_mdeg;
    int32_t last_gyro_heading_mdeg;
    int32_t gyro_sign;
    int32_t gyro_weight_permille;
    int32_t blend_remainder;
    bool initialized;
    bool gyro_tracking;
} HeadingEstimator;

void HeadingEstimator_Init(HeadingEstimator *estimator,
                           int32_t gyro_sign,
                           int32_t gyro_weight_permille);
void HeadingEstimator_Update(HeadingEstimator *estimator,
                             int32_t encoder_heading_mdeg,
                             int32_t gyro_raw_heading_mdeg,
                             bool gyro_valid);
int32_t HeadingEstimator_GetHeadingMdeg(const HeadingEstimator *estimator);
int32_t HeadingEstimator_ConvertGyroMdeg(const HeadingEstimator *estimator,
                                         int32_t gyro_raw_heading_mdeg);

#endif /* HEADING_ESTIMATOR_H */
