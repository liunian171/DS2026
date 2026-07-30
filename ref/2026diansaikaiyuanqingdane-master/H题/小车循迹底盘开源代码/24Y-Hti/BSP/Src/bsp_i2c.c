#include "bsp_i2c.h"

#include "i2c.h"

static bool BSP_I2C_IsAddressValid(uint8_t address_7bit)
{
    return address_7bit <= 0x7FU;
}

static bool BSP_I2C_IsRegisterSizeValid(uint16_t size)
{
    return (size == I2C_MEMADD_SIZE_8BIT) ||
           (size == I2C_MEMADD_SIZE_16BIT);
}

bool BSP_I2C_IsDeviceReady(uint8_t address_7bit,
                           uint32_t trials,
                           uint32_t timeout_ms)
{
    if (!BSP_I2C_IsAddressValid(address_7bit))
    {
        return false;
    }
    return HAL_I2C_IsDeviceReady(&hi2c1,
                                 (uint16_t)(address_7bit << 1),
                                 trials, timeout_ms) == HAL_OK;
}

bool BSP_I2C_MasterTransmit(uint8_t address_7bit,
                            const uint8_t *data,
                            uint16_t length,
                            uint32_t timeout_ms)
{
    if (!BSP_I2C_IsAddressValid(address_7bit) ||
        (data == 0) || (length == 0U))
    {
        return false;
    }
    return HAL_I2C_Master_Transmit(&hi2c1,
                                   (uint16_t)(address_7bit << 1),
                                   (uint8_t *)data, length,
                                   timeout_ms) == HAL_OK;
}

bool BSP_I2C_MasterReceive(uint8_t address_7bit,
                           uint8_t *data,
                           uint16_t length,
                           uint32_t timeout_ms)
{
    if (!BSP_I2C_IsAddressValid(address_7bit) ||
        (data == 0) || (length == 0U))
    {
        return false;
    }
    return HAL_I2C_Master_Receive(&hi2c1,
                                  (uint16_t)(address_7bit << 1),
                                  data, length, timeout_ms) == HAL_OK;
}

bool BSP_I2C_MemoryWrite(uint8_t address_7bit,
                         uint16_t register_address,
                         uint16_t register_address_size,
                         const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
    if (!BSP_I2C_IsAddressValid(address_7bit) ||
        !BSP_I2C_IsRegisterSizeValid(register_address_size) ||
        (data == 0) || (length == 0U))
    {
        return false;
    }
    return HAL_I2C_Mem_Write(&hi2c1,
                             (uint16_t)(address_7bit << 1),
                             register_address, register_address_size,
                             (uint8_t *)data, length,
                             timeout_ms) == HAL_OK;
}

bool BSP_I2C_MemoryRead(uint8_t address_7bit,
                        uint16_t register_address,
                        uint16_t register_address_size,
                        uint8_t *data,
                        uint16_t length,
                        uint32_t timeout_ms)
{
    if (!BSP_I2C_IsAddressValid(address_7bit) ||
        !BSP_I2C_IsRegisterSizeValid(register_address_size) ||
        (data == 0) || (length == 0U))
    {
        return false;
    }
    return HAL_I2C_Mem_Read(&hi2c1,
                            (uint16_t)(address_7bit << 1),
                            register_address, register_address_size,
                            data, length, timeout_ms) == HAL_OK;
}
