#ifndef WHEEL_SPEED_PI_H
#define WHEEL_SPEED_PI_H

#include <stdint.h>

typedef struct
{
    int32_t kp_x1000;
    int32_t ki_x1000_per_second;
    int32_t feedforward_gain_x1000;
    int32_t static_feedforward_permille;
    int32_t static_feedforward_cutoff_cps;
    int32_t high_speed_feedforward_gain_x1000;
    int32_t high_speed_feedforward_start_cps;
    int32_t output_min_permille;
    int32_t output_max_permille;
    int32_t integral_limit_permille;
    int32_t integral_x1000;
} WheelSpeedPI;

void WheelSpeedPI_Init(WheelSpeedPI *controller,
                       int32_t kp_x1000,
                       int32_t ki_x1000_per_second,
                       int32_t feedforward_gain_x1000,
                       int32_t static_feedforward_permille,
                       int32_t static_feedforward_cutoff_cps,
                       int32_t high_speed_feedforward_gain_x1000,
                       int32_t high_speed_feedforward_start_cps,
                       int32_t output_min_permille,
                       int32_t output_max_permille,
                       int32_t integral_limit_permille);
void WheelSpeedPI_Reset(WheelSpeedPI *controller);
int16_t WheelSpeedPI_Update(WheelSpeedPI *controller,
                            int32_t target_counts_per_second,
                            int32_t measured_counts_per_second,
                            uint32_t elapsed_ms);

#endif /* WHEEL_SPEED_PI_H */
