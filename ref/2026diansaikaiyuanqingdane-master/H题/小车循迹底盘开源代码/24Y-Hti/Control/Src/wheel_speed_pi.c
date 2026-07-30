#include "wheel_speed_pi.h"

static int32_t WheelSpeedPI_Clamp(int32_t value,
                                  int32_t minimum,
                                  int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

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
                       int32_t integral_limit_permille)
{
    controller->kp_x1000 = kp_x1000;
    controller->ki_x1000_per_second = ki_x1000_per_second;
    controller->feedforward_gain_x1000 = feedforward_gain_x1000;
    controller->static_feedforward_permille =
        static_feedforward_permille;
    controller->static_feedforward_cutoff_cps =
        static_feedforward_cutoff_cps;
    controller->high_speed_feedforward_gain_x1000 =
        high_speed_feedforward_gain_x1000;
    controller->high_speed_feedforward_start_cps =
        high_speed_feedforward_start_cps;
    controller->output_min_permille = output_min_permille;
    controller->output_max_permille = output_max_permille;
    controller->integral_limit_permille = integral_limit_permille;
    controller->integral_x1000 = 0;
}

void WheelSpeedPI_Reset(WheelSpeedPI *controller)
{
    controller->integral_x1000 = 0;
}

int16_t WheelSpeedPI_Update(WheelSpeedPI *controller,
                            int32_t target_counts_per_second,
                            int32_t measured_counts_per_second,
                            uint32_t elapsed_ms)
{
    int32_t error;
    int32_t proportional_x1000;
    int32_t integral_delta_x1000;
    int32_t candidate_integral_x1000;
    int32_t output_x1000;
    int32_t output_permille;
    int32_t integral_limit_x1000;
    int32_t feedforward_x1000;
    int32_t static_feedforward_x1000 = 0;
    int32_t high_speed_feedforward_x1000 = 0;
    int32_t direction_sign = 1;

    if ((target_counts_per_second == 0) || (elapsed_ms == 0U))
    {
        WheelSpeedPI_Reset(controller);
        return 0;
    }

    if (target_counts_per_second < 0)
    {
        direction_sign = -1;
        target_counts_per_second = -target_counts_per_second;
        measured_counts_per_second = -measured_counts_per_second;
    }

    error = target_counts_per_second - measured_counts_per_second;
    if ((controller->static_feedforward_cutoff_cps > 0) &&
        (target_counts_per_second <
         controller->static_feedforward_cutoff_cps))
    {
        static_feedforward_x1000 = (int32_t)(
            ((int64_t)controller->static_feedforward_permille * 1000LL *
             (controller->static_feedforward_cutoff_cps -
              target_counts_per_second)) /
            controller->static_feedforward_cutoff_cps);
    }
    if ((controller->high_speed_feedforward_start_cps > 0) &&
        (target_counts_per_second >
         controller->high_speed_feedforward_start_cps))
    {
        high_speed_feedforward_x1000 =
            (target_counts_per_second -
             controller->high_speed_feedforward_start_cps) *
            controller->high_speed_feedforward_gain_x1000;
    }
    feedforward_x1000 =
        target_counts_per_second * controller->feedforward_gain_x1000 +
        static_feedforward_x1000 + high_speed_feedforward_x1000;
    proportional_x1000 = controller->kp_x1000 * error;
    integral_delta_x1000 =
        (int32_t)(((int64_t)controller->ki_x1000_per_second *
                   (int64_t)error * (int64_t)elapsed_ms) /
                  1000LL);
    integral_limit_x1000 = controller->integral_limit_permille * 1000;
    candidate_integral_x1000 = WheelSpeedPI_Clamp(
        controller->integral_x1000 + integral_delta_x1000,
        -integral_limit_x1000,
        integral_limit_x1000);

    output_x1000 = feedforward_x1000 +
                   proportional_x1000 + candidate_integral_x1000;

    /* Do not integrate farther into output saturation. */
    if (!(((output_x1000 > controller->output_max_permille * 1000) &&
           (error > 0)) ||
          ((output_x1000 < controller->output_min_permille * 1000) &&
           (error < 0))))
    {
        controller->integral_x1000 = candidate_integral_x1000;
    }

    output_x1000 = feedforward_x1000 +
                   proportional_x1000 + controller->integral_x1000;
    output_permille = WheelSpeedPI_Clamp(output_x1000 / 1000,
                                         controller->output_min_permille,
                                         controller->output_max_permille);
    return (int16_t)(direction_sign * output_permille);
}
