#include "usergpio_platform.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

void usergpio_stm32_write(void *hgpio_port, uint16_t gpio_pin, uint8_t state)
{
    GPIO_TypeDef *port   = (GPIO_TypeDef *)hgpio_port;
    uint16_t pin_mask    = (uint16_t)(1 << gpio_pin);
    HAL_GPIO_WritePin(port, pin_mask, (GPIO_PinState)state);
}

void usergpio_stm32_toggle(void *hgpio_port, uint16_t gpio_pin)
{
    GPIO_TypeDef *port   = (GPIO_TypeDef *)hgpio_port;
    uint16_t pin_mask    = (uint16_t)(1 << gpio_pin);
    HAL_GPIO_TogglePin(port, pin_mask);
}

uint8_t usergpio_stm32_read(void *hgpio_port, uint16_t gpio_pin)
{
    GPIO_TypeDef *port   = (GPIO_TypeDef *)hgpio_port;
    uint16_t pin_mask    = (uint16_t)(1 << gpio_pin);
    return (uint8_t)HAL_GPIO_ReadPin(port, pin_mask);
}

const UserGPIO_PlatformOps_t usergpio_platform_ops_stm32 = {
    .write  = usergpio_stm32_write,
    .toggle = usergpio_stm32_toggle,
    .read   = usergpio_stm32_read,
};
