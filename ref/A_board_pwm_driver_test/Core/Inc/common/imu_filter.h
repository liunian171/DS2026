/**
 * @file    imu_filter.h
 * @brief   通用 IMU 数据处理 — 换算、姿态、互补滤波
 *
 * 零硬件依赖，纯数学运算，跨芯片复用。
 */

#ifndef __IMU_FILTER_H__
#define __IMU_FILTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

class ImuFilter {
public:
    ImuFilter();

    /** @brief 注入芯片参数 */
    void set_accel_scale(float lsb_per_g);
    void set_gyro_scale(float lsb_per_dps);

    /* ---- raw → 物理量 ---- */
    void raw_to_accel(int16_t rx, int16_t ry, int16_t rz,
                      float *ax, float *ay, float *az) const;
    void raw_to_gyro(int16_t rx, int16_t ry, int16_t rz,
                     float *gx, float *gy, float *gz) const;

    /* ---- 姿态 ---- */
    void calc_roll_pitch(float ax, float ay, float az,
                         float *roll, float *pitch) const;

    /**
     * @brief 互补滤波 — 陀螺仪积分 + 加速度修正
     * @param gx/gy/gz  陀螺仪物理量 (°/s)
     * @param ax/ay/az  加速度物理量 (g)
     * @param dt_sec    距上次调用间隔 (秒)
     * @param alpha     加速度权重 (0~1, 默认 0.02)
     */
    void complementary_filter(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float dt_sec, float alpha = 0.02f);

    /* ---- 偏置校准 ---- */
    void calibrate_accel_bias(const int16_t *sx, const int16_t *sy,
                              const int16_t *sz, uint16_t count);
    void calibrate_gyro_bias(const int16_t *sx, const int16_t *sy,
                             const int16_t *sz, uint16_t count);

    /* ---- 读取结果 ---- */
    float get_roll()  const { return roll_; }
    float get_pitch() const { return pitch_; }
    float get_yaw()   const { return yaw_; }

    int16_t accel_bias_x() const { return accel_bx_; }
    int16_t accel_bias_y() const { return accel_by_; }
    int16_t accel_bias_z() const { return accel_bz_; }

private:
    float accel_scale_;       ///< LSB/g
    float gyro_scale_;        ///< LSB/(°/s)

    int16_t accel_bx_, accel_by_, accel_bz_;
    int16_t gyro_bx_,  gyro_by_,  gyro_bz_;

    float roll_, pitch_, yaw_;  ///< 姿态角 (度)
};

#endif /* __cplusplus */

#endif /* __IMU_FILTER_H__ */
