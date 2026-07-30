#include "cyz_sensor.h"
#include "i2c.h"

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c != 0) && (hi2c->Instance == I2C1))
    {
        CYZ_Sensor_I2CMemoryReadComplete();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c != 0) && (hi2c->Instance == I2C1))
    {
        CYZ_Sensor_I2CError();
    }
}
