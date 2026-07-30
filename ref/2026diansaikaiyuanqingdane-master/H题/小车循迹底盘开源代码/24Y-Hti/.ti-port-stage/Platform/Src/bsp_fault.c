#include "bsp_fault.h"

#include "ti_msp_dl_config.h"

void BSP_Fault(void)
{
    DL_GPIO_clearPins(GPIO_MOTOR_SAFE_PORT,
        GPIO_MOTOR_SAFE_M1_IN1_PIN | GPIO_MOTOR_SAFE_M1_IN2_PIN |
        GPIO_MOTOR_SAFE_M2_IN1_PIN | GPIO_MOTOR_SAFE_M2_IN2_PIN |
        GPIO_MOTOR_SAFE_MOTOR12_STBY_PIN |
        GPIO_MOTOR_SAFE_MOTOR_VM_EN_PIN);

    __disable_irq();
    while (1) {
        __WFI();
    }
}
