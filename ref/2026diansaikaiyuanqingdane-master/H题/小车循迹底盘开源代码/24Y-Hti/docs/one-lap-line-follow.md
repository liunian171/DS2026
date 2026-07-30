# 一圈纯循迹测试

## 当前行为

1. 复位后等待 10 秒，PC13 闪烁；巡线数据有效时起步。
2. D2～D7 对连续黑线进行闭环循迹，以 250 mm/s² 缓加速到 350 mm/s（0.35 m/s）。
3. 达到目标速度后保持 350 mm/s 恒速指令，此阶段加速度指令为 0。
4. 编码器累计转向达到 320° 后，以 250 mm/s² 缓减速到 100 mm/s，低速寻找停车线。
5. 如果小车起步时压在宽停车线上，程序先忽略该线。连续 5 帧回到普通轨迹线后，终点检测才会使能。
6. 再次检测到至少 4 个通道同时为黑色并持续 2 帧时，判定完成一圈并以 250 mm/s² 缓减速到 0。
7. 左右轮连续 5 个采样周期低于 120 counts/s 后关闭电机。

本模式不使用赛道直线长度、圆弧半径、直径或预设总里程。120 秒仅作为未检测到停车线时的安全超时。

## 停车线要求

停车线需要横跨 D2～D7 的有效探测宽度。普通轨迹通常只触发 1～3 个通道，停车线至少需要稳定触发 4 个通道。

如果现场停车线宽度不足，可在 `App/Inc/app_config.h` 调整：

- `APP_LINE_STOP_ACTIVE_COUNT`：同时压黑的最少通道数；
- `APP_LINE_STOP_DETECT_STABLE_FRAMES`：停车线确认帧数；
- `APP_LINE_STOPPED_MAX_CPS`：确认车辆停止的轮速阈值。

## 速度与加速度

加速度由左右编码器平均线速度每 20 ms 差分得到，单位均为 SI 常用工程单位：

- `g_vehicle_linear_speed_mm_s`：车辆纵向速度，mm/s；
- `g_vehicle_acceleration_raw_mm_s2`：未滤波纵向加速度，mm/s²；
- `g_vehicle_acceleration_mm_s2`：四点等效低通后的加速度，mm/s²；
- `g_vehicle_acceleration_max_mm_s2`：本圈最大正加速度；
- `g_vehicle_acceleration_min_mm_s2`：本圈最大减速度，数值为负；
- `g_vehicle_profile_speed_mm_s`：缓启动/恒速/缓停止速度曲线指令；
- `g_vehicle_acceleration_command_mm_s2`：速度曲线加速度指令；
- `g_lap_elapsed_ms`、`g_lap_distance_um`：本圈时间和编码器累计里程；
- `g_lap_heading_progress_mdeg`：本圈编码器累计转向角；
- `g_line_motion_phase`：0 等待、1 加速、2 恒速、3 接近、4 停车、5 完成；
- `g_lap_stop_line_armed`、`g_lap_stop_line_detected`：停车线检测状态。
- `g_lap_line_active_count_max`：终点检测武装后，本圈最大同时压黑路数；
- `g_lap_stop_candidate_max_frames`：达到停车线通道阈值的最大连续帧数。

固件另外以 100 ms 周期保存最多 192 个运动曲线点，约覆盖 19.2 秒。每个点包含阶段、规划/实测速度、指令/实测加速度、循迹转向量和累计转角。运行 `tools/read-motion-curve.ps1` 可导出 `build/Debug/motion-curve.csv`。

恒速阶段还会按循迹转向量自动分为直线与转弯，分别累计：

- 实测速度最小值、最大值；
- 滤波加速度最小值、最大值；
- 有效样本数。

烧录后可运行 `tools/read-wheel-speed.ps1` 读取以上变量。加速度换算依赖 `Control/Inc/chassis_config.h` 中的轮径，后续实测轮径变化时需要同步修正。

## 首次上车建议

将驱动轮架空，确认起步倒计时、停车线状态变量和电机停止逻辑；随后以当前 350 mm/s 在地面测试。调速、加速度和提前减速角度集中在 `App/Inc/app_config.h`，后续地图变化只需标定提前减速角度，无需修改状态机。
