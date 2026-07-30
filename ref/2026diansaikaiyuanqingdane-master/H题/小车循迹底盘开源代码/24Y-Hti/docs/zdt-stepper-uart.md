# 张大头 V5 步进驱动串口接口

## 接线与串口参数

- MCU：STM32F103C8T6
- 串口：USART3（原调试串口，现专供步进驱动）
- PB10 / USART3_TX -> 驱动器 RX
- PB11 / USART3_RX <- 驱动器 TX
- MCU GND 与驱动器 GND 必须共地
- 115200 bit/s，8 数据位，无校验，1 停止位（8N1）
- 默认驱动地址：1

PB11 已配置内部上拉，串口采用中断收发。USART3 的调试文本输出已禁用，
避免 ASCII 启动日志被驱动器误识别为控制命令。

如果驱动器接口是 RS485 而不是 TTL UART，不能直接连接 PB10/PB11，必须增加
RS485 收发器和方向控制。

## 已提供接口

`zdt_stepper.h` 提供：

- `ZDT_Stepper_Enable()`：使能或失能
- `ZDT_Stepper_Stop()`：立即停止
- `ZDT_Stepper_MoveAbsolutePulses()`：绝对位置
- `ZDT_Stepper_MoveRelativePulses()`：相对位置
- `ZDT_Stepper_SetAngleMdeg()`：以毫度设置电机轴绝对角度
- `ZDT_Stepper_RequestPosition()`：请求实时位置
- `ZDT_Stepper_Process()`：非阻塞接收解析，由主循环调用

固件上电只初始化串口，不会自动使能或转动电机。

当 `APP_ZDT_STEPPER_TEST_ENABLED=1` 时例外：测试固件会在上电 3 秒后使能，
以 60 RPM、加速度档 20 相对运行 3200 脉冲一次，并在 2.5 秒后读取当前位置。
此测试模式下车辆循迹功能必须关闭。

## 位置单位

当前位置回复 `0x36` 使用每圈 65536 个原始单位。程序同时输出：

- `g_zdt_stepper_position_raw`：驱动器原始位置
- `g_zdt_stepper_position_pulses`：按 3200 脉冲/圈换算的位置
- `g_zdt_stepper_angle_mdeg`：电机轴角度，单位 0.001 度

当前 `ZDT_Stepper_SetAngleMdeg()` 默认电机轴和小球轨道为 1:1。安装完成后需要根据
减速比、连杆长度、轨道允许倾角及机械零点，再增加“轨道角度 -> 电机角度”的标定层
和软件限位，之后才能接入小球位置闭环。

## V5 帧

- 使能：`ADDR F3 AB STATE 00 6B`
- 位置：`ADDR FD DIR VEL_H VEL_L ACC POS_31..0 ABS SYNC 6B`
- 停止：`ADDR FE 98 00 6B`
- 读取当前位置：`ADDR 36 6B`

协议来自 `ARM_V1.5.1/User/Motor/ZDT_Motor/Emm_V5.c`。
