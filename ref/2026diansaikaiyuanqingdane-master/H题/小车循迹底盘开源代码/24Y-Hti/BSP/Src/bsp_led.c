#include "bsp_led.h"

#include "main.h"

void BSP_LED_Set(bool on)
{
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BSP_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}
