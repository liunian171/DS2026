#ifndef BSP_LINE_UART_H
#define BSP_LINE_UART_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t g_line_uart_rx_count;
extern volatile uint32_t g_line_uart_overflow_count;
extern volatile uint32_t g_line_uart_error_count;

bool BSP_LineUART_Init(void);
uint16_t BSP_LineUART_Available(void);
bool BSP_LineUART_ReadByte(uint8_t *byte);
void BSP_LineUART_RxCpltCallback(UART_HandleTypeDef *uart);
void BSP_LineUART_ErrorCallback(UART_HandleTypeDef *uart);

#endif /* BSP_LINE_UART_H */
