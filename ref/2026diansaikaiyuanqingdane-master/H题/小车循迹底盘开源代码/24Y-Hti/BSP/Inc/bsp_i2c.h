#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

bool BSP_I2C_IsDeviceReady(uint8_t address_7bit,
                           uint32_t trials,
                           uint32_t timeout_ms);
bool BSP_I2C_MasterTransmit(uint8_t address_7bit,
                            const uint8_t *data,
                            uint16_t length,
                            uint32_t timeout_ms);
bool BSP_I2C_MasterReceive(uint8_t address_7bit,
                           uint8_t *data,
                           uint16_t length,
                           uint32_t timeout_ms);
bool BSP_I2C_MemoryWrite(uint8_t address_7bit,
                         uint16_t register_address,
                         uint16_t register_address_size,
                         const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms);
bool BSP_I2C_MemoryRead(uint8_t address_7bit,
                        uint16_t register_address,
                        uint16_t register_address_size,
                        uint8_t *data,
                        uint16_t length,
                        uint32_t timeout_ms);

#endif /* BSP_I2C_H */
