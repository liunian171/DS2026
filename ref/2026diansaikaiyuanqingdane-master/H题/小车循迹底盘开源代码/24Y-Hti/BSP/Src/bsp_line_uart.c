#include "bsp_line_uart.h"

#include "usart.h"

#include <stddef.h>

#define LINE_UART_RX_BUFFER_SIZE 128U
#define LINE_UART_RX_BUFFER_MASK (LINE_UART_RX_BUFFER_SIZE - 1U)

static uint8_t line_rx_buffer[LINE_UART_RX_BUFFER_SIZE];
static uint8_t line_rx_byte;
static volatile uint16_t line_rx_head;
static volatile uint16_t line_rx_tail;

volatile uint32_t g_line_uart_rx_count;
volatile uint32_t g_line_uart_overflow_count;
volatile uint32_t g_line_uart_error_count;

bool BSP_LineUART_Init(void)
{
    line_rx_head = 0U;
    line_rx_tail = 0U;
    g_line_uart_rx_count = 0U;
    g_line_uart_overflow_count = 0U;
    g_line_uart_error_count = 0U;
    return HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1U) == HAL_OK;
}

uint16_t BSP_LineUART_Available(void)
{
    return (uint16_t)((line_rx_head - line_rx_tail) &
                      LINE_UART_RX_BUFFER_MASK);
}

bool BSP_LineUART_ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || (line_rx_tail == line_rx_head))
    {
        return false;
    }

    *byte = line_rx_buffer[line_rx_tail];
    line_rx_tail = (uint16_t)((line_rx_tail + 1U) &
                              LINE_UART_RX_BUFFER_MASK);
    return true;
}

void BSP_LineUART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2)
    {
        uint16_t next_head = (uint16_t)((line_rx_head + 1U) &
                                        LINE_UART_RX_BUFFER_MASK);
        if (next_head != line_rx_tail)
        {
            line_rx_buffer[line_rx_head] = line_rx_byte;
            line_rx_head = next_head;
            g_line_uart_rx_count++;
        }
        else
        {
            g_line_uart_overflow_count++;
        }
        (void)HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1U);
    }
}

void BSP_LineUART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2)
    {
        g_line_uart_error_count++;
        (void)HAL_UART_Receive_IT(&huart2, &line_rx_byte, 1U);
    }
}
