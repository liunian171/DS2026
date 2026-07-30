# 天猛星 MSPM0G3507 教学小车引脚表

本表与 `config/board.syscfg` 一致，并已通过 TI SysConfig 1.28 冲突检查。

## 双轮小车核心接口

| 功能 | 引脚 | 外设 |
|---|---|---|
| M1 左电机 PWM | PA26 | TIMG7 CCP0，20 kHz |
| M2 右电机 PWM | PA27 | TIMG7 CCP1，20 kHz |
| M1 IN1 / IN2 | PB17 / PB18 | GPIO |
| M2 IN1 / IN2 | PB19 / PB20 | GPIO |
| TB6612 STBY | PB24 | GPIO，复位默认低 |
| VM_EN | PB25 | GPIO，复位默认低 |
| M1 编码器 A / B | PB10 / PB11 | TIMG6 CCP0 / CCP1 |
| M2 编码器 A / B | PB15 / PB16 | TIMG8 CCP0 / CCP1 |
| 板载状态 LED | PB22 | 高电平点亮 |

## 通信与教学扩展

| 功能 | 引脚 | 外设 |
|---|---|---|
| 调试串口 TX / RX | PA10 / PA11 | UART0，115200 8N1 |
| 陀螺仪串口 TX / RX | PA8 / PA9 | UART1，115200 8N1 |
| 扩展串口 TX / RX | PA23 / PA24 | UART2，115200 8N1 |
| 八路循迹 TX / RX | PB12 / PB13 | UART3，115200 8N1 |
| I2C SDA / SCL | PB3 / PB2 | I2C1，400 kHz |
| SPI SCLK / MOSI / MISO / CS | PB9 / PB8 / PB7 / PB6 | SPI1，1 MHz，Mode 0 |

## 舵机与预留资源

| 功能 | 引脚 | 外设 |
|---|---|---|
| SERVO1 | PA0 | TIMA0 CCP0，50 Hz |
| SERVO2 | PA1 | TIMA0 CCP1，50 Hz |
| SERVO3 | PB4 | TIMA0 CCP2，50 Hz |
| SERVO4 | PA28 | TIMA0 CCP3，50 Hz |
| CAN TX / RX（预留） | PA12 / PA13 | MCAN0，需外接收发器 |
| ADC1 CH4..CH7（预留） | PA14 / PA15 / PA16 / PA17 | 模拟输入 |

舵机使用独立 5 V 电源并与 MCU 共地，不得由开发板 3.3 V 供电。I2C SDA/SCL
需要 2.2 kΩ～4.7 kΩ 上拉到 3.3 V。所有 UART 均为 3.3 V 电平。

## 下载与特殊引脚

| 功能 | 引脚 |
|---|---|
| SWDIO / SWCLK | PA19 / PA20 |
| 板载按键 | PB21 |
| BSL 启动 | PA18，避免普通外设占用 |

## 安全自检固件

`debug` 预设为安全板级自检：电机和舵机不启动，PB22 每 250 ms 翻转，并启动
两组编码器及陀螺仪/循迹 UART 接收计数。主要 OpenOCD 观察符号如下：

- `g_board_selftest_heartbeat`
- `g_board_selftest_status`
- `g_encoder_left_count` / `g_encoder_right_count`
- `g_gyro_uart_rx_count` / `g_line_uart_rx_count`
- 各 UART 的 `error_count` 和 `overflow_count`

只有完成安全自检并架空车轮后，才允许烧录带电机动作的测试固件。
