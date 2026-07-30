#ifndef BSP_GYRO_UART_H
#define BSP_GYRO_UART_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_gyro_uart_rx_count;
extern volatile uint32_t g_gyro_uart_overflow_count;
extern volatile uint32_t g_gyro_uart_error_count;

bool BSP_GyroUART_Init(void);
uint16_t BSP_GyroUART_Available(void);
bool BSP_GyroUART_ReadByte(uint8_t *byte);
bool BSP_GyroUART_Write(const uint8_t *data, uint16_t length);
void BSP_GyroUART_RxCpltCallback(UART_HandleTypeDef *uart);
void BSP_GyroUART_ErrorCallback(UART_HandleTypeDef *uart);

#endif /* BSP_GYRO_UART_H */
