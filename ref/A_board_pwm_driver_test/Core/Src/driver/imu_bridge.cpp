/**
 * @file    imu_bridge.cpp
 * @brief   IMU C 桥接实现
 */

#include "imu_bridge.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

static IIMU       *imu_devices[MAX_IMUS]  = {nullptr};
static ImuType_t   imu_types[MAX_IMUS]    = {IMU_MPU6050};
static uint8_t     imu_ready[MAX_IMUS]    = {0};
static ImuFilter   imu_filters[MAX_IMUS];
static uint32_t    imu_last_tick[MAX_IMUS] = {0};


void imu_bridge_init(uint8_t id, ImuType_t type, void *i2c_handle)
{
    if (id >= MAX_IMUS) return;

    switch (type) {
    case IMU_MPU6050:
        imu_devices[id] = new MPU6050((I2C_Handle *)i2c_handle);
        imu_types[id]   = IMU_MPU6050;
        break;
    default:
        return;
    }

    if (imu_devices[id]->init() == 0) {
        imu_ready[id] = 1;
    } else {
        /* 兜底：克隆芯片 WHO_AM_I 不匹配，手动唤醒 + 配置 */
        MPU6050 *mpu = static_cast<MPU6050 *>(imu_devices[id]);
        mpu->write_reg(0x6B, 0x00);
        for (volatile int i = 0; i < 500000; i++) { }
        mpu->write_reg(0x19, 9);
        mpu->write_reg(0x1A, 0x05);
        mpu->write_reg(0x1B, 0x00);
        mpu->write_reg(0x1C, 0x00);
        imu_ready[id] = 1;
    }

    /* 滤波器系数 */
    imu_filters[id].set_accel_scale(imu_devices[id]->accel_scale());
    imu_filters[id].set_gyro_scale(imu_devices[id]->gyro_scale());
}

uint8_t imu_bridge_ready(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_ready[id] : 0;
}

int8_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return -1;
    return imu_devices[id]->read_accel_raw(ax, ay, az);
}

int8_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return -1;
    return imu_devices[id]->read_gyro_raw(gx, gy, gz);
}

float imu_bridge_accel_scale(uint8_t id)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return 1.0f;
    return imu_devices[id]->accel_scale();
}

float imu_bridge_gyro_scale(uint8_t id)
{
    if (id >= MAX_IMUS || !imu_devices[id]) return 1.0f;
    return imu_devices[id]->gyro_scale();
}

/* ==========================================================================
 *  互补滤波
 * ========================================================================== */

void imu_bridge_update_filter(uint8_t id)
{
    if (!imu_bridge_ready(id)) return;

    int16_t ax, ay, az, gx, gy, gz;
    if (imu_bridge_read_accel_raw(id, &ax, &ay, &az) != 0 ||
        imu_bridge_read_gyro_raw(id, &gx, &gy, &gz) != 0)
        return;

    float af[3], gf[3];
    imu_filters[id].raw_to_accel(ax, ay, az, &af[0], &af[1], &af[2]);
    imu_filters[id].raw_to_gyro(gx, gy, gz, &gf[0], &gf[1], &gf[2]);

    /* 静态校准陀螺零偏（首30次） */
    static int16_t gb_sum[3] = {0};
    static int8_t  cal_cnt = 0;

    if (cal_cnt < 30) {
        gb_sum[0] += gx; gb_sum[1] += gy; gb_sum[2] += gz;
        cal_cnt++;
        /* 校准期间全信加速度 */
        imu_filters[id].complementary_filter(0,0,0, af[0], af[1], af[2], 0.01f, 1.0f);
        imu_last_tick[id] = HAL_GetTick();
        return;
    } else if (cal_cnt == 30) {
        /* 写入校准偏置（取30次均值） */
        int16_t avg[3] = { gb_sum[0] / 30, gb_sum[1] / 30, gb_sum[2] / 30 };
        imu_filters[id].calibrate_gyro_bias(&avg[0], &avg[1], &avg[2], 1);
        cal_cnt = 31;
    }

    uint32_t now = HAL_GetTick();
    float dt    = (float)(now - imu_last_tick[id]) * 0.001f;
    imu_last_tick[id] = now;

    /* 忽略微小角速度（静止时噪声） */
    if (gf[0] * gf[0] < 2.0f) gf[0] = 0;   // roll gyro < ~1.4°/s
    if (gf[1] * gf[1] < 2.0f) gf[1] = 0;
    if (gf[2] * gf[2] < 2.0f) gf[2] = 0;

    float alpha = (dt > 0.001f && dt < 1.0f) ? 0.10f : 1.0f;
    imu_filters[id].complementary_filter(gf[0], gf[1], gf[2],
                                          af[0], af[1], af[2], dt, alpha);
}

float imu_bridge_get_roll(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_roll() : 0.0f;
}
float imu_bridge_get_pitch(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_pitch() : 0.0f;
}
float imu_bridge_get_yaw(uint8_t id)
{
    return (id < MAX_IMUS) ? imu_filters[id].get_yaw() : 0.0f;
}
