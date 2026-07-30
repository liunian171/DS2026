#include "bsp_stepper_uart.h"

#include "usart.h"

#define STEPPER_UART_RX_BUFFER_SIZE 64U
#define STEPPER_UART_RX_BUFFER_MASK (STEPPER_UART_RX_BUFFER_SIZE - 1U)
#define STEPPER_UART_TX_BUFFER_SIZE 32U

static uint8_t stepper_rx_buffer[STEPPER_UART_RX_BUFFER_SIZE];
static volatile uint16_t stepper_rx_head;
static volatile uint16_t stepper_rx_tail;
static uint8_t stepper_rx_byte;
static uint8_t stepper_tx_buffer[STEPPER_UART_TX_BUFFER_SIZE];
static volatile bool stepper_tx_busy;

volatile uint32_t g_stepper_uart_tx_frame_count;
volatile uint32_t g_stepper_uart_rx_byte_count;
volatile uint32_t g_stepper_uart_rx_overflow_count;
volatile uint32_t g_stepper_uart_error_count;

bool BSP_StepperUART_Init(void)
{
    stepper_rx_head = 0U;
    stepper_rx_tail = 0U;
    stepper_rx_byte = 0U;
    stepper_tx_busy = false;
    g_stepper_uart_tx_frame_count = 0U;
    g_stepper_uart_rx_byte_count = 0U;
    g_stepper_uart_rx_overflow_count = 0U;
    g_stepper_uart_error_count = 0U;
    return HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1U) == HAL_OK;
}

bool BSP_StepperUART_Write(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint16_t index;
    HAL_StatusTypeDef status;

    if ((data == 0) || (length == 0U) ||
        (length > STEPPER_UART_TX_BUFFER_SIZE))
    {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (stepper_tx_busy)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }
    for (index = 0U; index < length; index++)
    {
        stepper_tx_buffer[index] = data[index];
    }
    stepper_tx_busy = true;
    status = HAL_UART_Transmit_IT(&huart3, stepper_tx_buffer, length);
    if (status != HAL_OK)
    {
        stepper_tx_busy = false;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (status != HAL_OK)
    {
        return false;
    }
    return true;
}

bool BSP_StepperUART_ReadByte(uint8_t *byte)
{
    if ((byte == 0) || (stepper_rx_head == stepper_rx_tail))
    {
        return false;
    }
    *byte = stepper_rx_buffer[stepper_rx_tail];
    stepper_rx_tail = (uint16_t)((stepper_rx_tail + 1U) &
                                 STEPPER_UART_RX_BUFFER_MASK);
    return true;
}

uint16_t BSP_StepperUART_Available(void)
{
    return (uint16_t)((stepper_rx_head - stepper_rx_tail) &
                      STEPPER_UART_RX_BUFFER_MASK);
}

bool BSP_StepperUART_IsTxBusy(void)
{
    return stepper_tx_busy;
}

void BSP_StepperUART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART3)
    {
        return;
    }
    stepper_tx_busy = false;
    g_stepper_uart_tx_frame_count++;
}

void BSP_StepperUART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next_head;

    if (uart->Instance != USART3)
    {
        return;
    }
    next_head = (uint16_t)((stepper_rx_head + 1U) &
                           STEPPER_UART_RX_BUFFER_MASK);
    if (next_head == stepper_rx_tail)
    {
        g_stepper_uart_rx_overflow_count++;
    }
    else
    {
        stepper_rx_buffer[stepper_rx_head] = stepper_rx_byte;
        stepper_rx_head = next_head;
        g_stepper_uart_rx_byte_count++;
    }
    (void)HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1U);
}

void BSP_StepperUART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART3)
    {
        return;
    }
    g_stepper_uart_error_count++;
    stepper_tx_busy = false;
    (void)HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1U);
}
