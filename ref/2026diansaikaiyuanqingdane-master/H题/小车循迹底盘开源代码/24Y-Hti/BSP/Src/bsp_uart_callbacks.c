#include "bsp_gyro_uart.h"
#include "bsp_line_uart.h"
#include "bsp_stepper_uart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    BSP_GyroUART_RxCpltCallback(uart);
    BSP_LineUART_RxCpltCallback(uart);
    BSP_StepperUART_RxCpltCallback(uart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    BSP_StepperUART_TxCpltCallback(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    BSP_GyroUART_ErrorCallback(uart);
    BSP_LineUART_ErrorCallback(uart);
    BSP_StepperUART_ErrorCallback(uart);
}
