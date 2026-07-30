#include "bsp_debug_uart.h"

void BSP_DebugUART_Write(const char *text)
{
    /* USART3 (PB10/PB11) is reserved for the ZDT stepper driver. */
    (void)text;
}
