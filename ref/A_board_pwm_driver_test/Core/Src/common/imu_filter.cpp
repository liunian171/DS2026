/**
 * @file    imu_filter.cpp
 * @brief   IMU 数据处理实现
 */

#include "imu_filter.h"
#include <math.h>

/* ==========================================================================
 *  构造
 * ========================================================================== */

ImuFilter::ImuFilter()
    : accel_scale_(16384.0f)
    , gyro_scale_(131.0f)
    , accel_bx_(0), accel_by_(0), accel_bz_(0)
    , gyro_bx_(0), gyro_by_(0), gyro_bz_(0)
    , roll_(0), pitch_(0), yaw_(0)
{}

void ImuFilter::set_accel_scale(float lsb_per_g) { accel_scale_ = lsb_per_g; }
void ImuFilter::set_gyro_scale(float lsb_per_dps) { gyro_scale_ = lsb_per_dps; }


/* ==========================================================================
 *  raw → 物理量
 * ========================================================================== */

void ImuFilter::raw_to_accel(int16_t rx, int16_t ry, int16_t rz,
                             float *ax, float *ay, float *az) const
{
    *ax = (float)(rx - accel_bx_) / accel_scale_;
    *ay = (float)(ry - accel_by_) / accel_scale_;
    *az = (float)(rz - accel_bz_) / accel_scale_;
}

void ImuFilter::raw_to_gyro(int16_t rx, int16_t ry, int16_t rz,
                            float *gx, float *gy, float *gz) const
{
    *gx = (float)(rx - gyro_bx_) / gyro_scale_;
    *gy = (float)(ry - gyro_by_) / gyro_scale_;
    *gz = (float)(rz - gyro_bz_) / gyro_scale_;
}


/* ==========================================================================
 *  姿态角（加速度 → roll/pitch）
 * ========================================================================== */

void ImuFilter::calc_roll_pitch(float ax, float ay, float az,
                                float *roll, float *pitch) const
{
    *roll  = atan2f(ay, az) * 57.29578f;   // radians → degrees
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
}


/* ==========================================================================
 *  互补滤波
 *
 *  roll  = alpha × accel_angle + (1-alpha) × (last_roll  + gyro_gx × dt)
 *  pitch = alpha × accel_angle + (1-alpha) × (last_pitch + gyro_gy × dt)
 *
 *  alpha 越大 → 更信任加速度计 (响应快, 高频噪声大)
 *  alpha 越小 → 更信任陀螺仪 (平滑, 但会累积漂移)
 * ========================================================================== */

void ImuFilter::complementary_filter(float gx, float gy, float gz,
                                     float ax, float ay, float az,
                                     float dt_sec, float alpha)
{
    float accel_roll, accel_pitch;
    calc_roll_pitch(ax, ay, az, &accel_roll, &accel_pitch);

    /* 陀螺仪积分 */
    float gyro_roll  = roll_  + gx * dt_sec;
    float gyro_pitch = pitch_ + gy * dt_sec;

    /* 互补融合 */
    roll_  = alpha * accel_roll  + (1.0f - alpha) * gyro_roll;
    pitch_ = alpha * accel_pitch + (1.0f - alpha) * gyro_pitch;

    /* yaw 无磁力计参考，纯积分 */
    yaw_  += gz * dt_sec;
}


/* ==========================================================================
 *  偏置校准：静止采样 N 次取均值
 * ========================================================================== */

static int16_t mean(const int16_t *samples, uint16_t count)
{
    int32_t sum = 0;
    for (uint16_t i = 0; i < count; i++) sum += samples[i];
    return (int16_t)(sum / count);
}

void ImuFilter::calibrate_accel_bias(const int16_t *sx, const int16_t *sy,
                                     const int16_t *sz, uint16_t count)
{
    accel_bx_ = mean(sx, count);
    accel_by_ = mean(sy, count);
    accel_bz_ = mean(sz, count) - (int16_t)accel_scale_;  // 减 1g
}

void ImuFilter::calibrate_gyro_bias(const int16_t *sx, const int16_t *sy,
                                    const int16_t *sz, uint16_t count)
{
    gyro_bx_ = mean(sx, count);
    gyro_by_ = mean(sy, count);
    gyro_bz_ = mean(sz, count);
}
