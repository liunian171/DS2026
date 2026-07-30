#ifndef BSP_STEPPER_UART_H
#define BSP_STEPPER_UART_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool BSP_StepperUART_Init(void);
bool BSP_StepperUART_Write(const uint8_t *data, uint16_t length);
bool BSP_StepperUART_ReadByte(uint8_t *byte);
uint16_t BSP_StepperUART_Available(void);
bool BSP_StepperUART_IsTxBusy(void);

void BSP_StepperUART_RxCpltCallback(UART_HandleTypeDef *uart);
void BSP_StepperUART_TxCpltCallback(UART_HandleTypeDef *uart);
void BSP_StepperUART_ErrorCallback(UART_HandleTypeDef *uart);

extern volatile uint32_t g_stepper_uart_tx_frame_count;
extern volatile uint32_t g_stepper_uart_rx_byte_count;
extern volatile uint32_t g_stepper_uart_rx_overflow_count;
extern volatile uint32_t g_stepper_uart_error_count;

#endif /* BSP_STEPPER_UART_H */
