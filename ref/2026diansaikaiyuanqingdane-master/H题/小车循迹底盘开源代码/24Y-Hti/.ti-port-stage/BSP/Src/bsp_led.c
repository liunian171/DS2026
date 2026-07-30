#include "bsp_led.h"

#include "ti_msp_dl_config.h"

void BSP_LED_Set(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_MOTOR_SAFE_PORT,
                        GPIO_MOTOR_SAFE_STATUS_LED_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_SAFE_PORT,
                          GPIO_MOTOR_SAFE_STATUS_LED_PIN);
    }
}

void BSP_LED_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_MOTOR_SAFE_PORT,
                       GPIO_MOTOR_SAFE_STATUS_LED_PIN);
}
