#include "bsp_gyro_uart.h"

#include "usart.h"

#include <stddef.h>

#define GYRO_UART_RX_BUFFER_SIZE 128U
#define GYRO_UART_RX_BUFFER_MASK (GYRO_UART_RX_BUFFER_SIZE - 1U)

static uint8_t gyro_rx_buffer[GYRO_UART_RX_BUFFER_SIZE];
static uint8_t gyro_rx_byte;
static volatile uint16_t gyro_rx_head;
static volatile uint16_t gyro_rx_tail;

volatile uint32_t g_gyro_uart_rx_count;
volatile uint32_t g_gyro_uart_overflow_count;
volatile uint32_t g_gyro_uart_error_count;

bool BSP_GyroUART_Init(void)
{
    gyro_rx_head = 0U;
    gyro_rx_tail = 0U;
    g_gyro_uart_rx_count = 0U;
    g_gyro_uart_overflow_count = 0U;
    g_gyro_uart_error_count = 0U;
    return HAL_UART_Receive_IT(&huart1, &gyro_rx_byte, 1U) == HAL_OK;
}

uint16_t BSP_GyroUART_Available(void)
{
    return (uint16_t)((gyro_rx_head - gyro_rx_tail) &
                      GYRO_UART_RX_BUFFER_MASK);
}

bool BSP_GyroUART_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (gyro_rx_tail == gyro_rx_head))
    {
        return false;
    }

    *byte = gyro_rx_buffer[gyro_rx_tail];
    gyro_rx_tail = (uint16_t)((gyro_rx_tail + 1U) &
                              GYRO_UART_RX_BUFFER_MASK);
    return true;
}

bool BSP_GyroUART_Write(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return false;
    }
    return HAL_UART_Transmit(&huart1, data, length, 100U) == HAL_OK;
}

void BSP_GyroUART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1)
    {
        uint16_t next_head = (uint16_t)((gyro_rx_head + 1U) &
                                        GYRO_UART_RX_BUFFER_MASK);
        if (next_head != gyro_rx_tail)
        {
            gyro_rx_buffer[gyro_rx_head] = gyro_rx_byte;
            gyro_rx_head = next_head;
            g_gyro_uart_rx_count++;
        }
        else
        {
            g_gyro_uart_overflow_count++;
        }
        (void)HAL_UART_Receive_IT(&huart1, &gyro_rx_byte, 1U);
    }
}

void BSP_GyroUART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1)
    {
        g_gyro_uart_error_count++;
        (void)HAL_UART_Receive_IT(&huart1, &gyro_rx_byte, 1U);
    }
}
