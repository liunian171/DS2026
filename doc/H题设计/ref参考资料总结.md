# ref参考资料与新确认信息总结

> 日期: 2026-07-29
> 说明: ref目录仅供参考，实际项目规划以用户明确信息为准

---

## 一、用户本次新确认的实事信息

| 项目 | 确认信息 | 之前文档错误 |
|------|----------|-------------|
| TMC2209 | 芯片丝印"tmc2209-la1935 82600 germany"，商品页面信息在步进电机驱动图片中(无法读取) | 之前写的通用规格需以实际为准 |
| 步进电机型号 | **17PM-J347-G2VS** | 之前只写17PM，需补全型号 |
| 直流电机型号 | **MG513XP28_12V** (不是MG513P-28) | 之前写错型号 |
| 红外传感器 | **5路~6路红外灰度传感器** | 之前写5-7路 |
| 轮径 | **66mm** (全径) | 之前写3.25cm(错) |
| 三人分工 | 1=建模+规划+算法, 2=底层移植, 3=视觉识别 | 之前自行分配任务(错) |
| OLED位置 | **在A板上** | 之前已确认 |

---

## 二、ref目录新增参考资料

### 2.1 BluePill_Car — 完整循迹小车工程(F103C8T6)

**高度相关**: A板是F103ZET6(同系列)，可直接参考移植

**硬件配置**:
- MCU: STM32F103C8T6, 72MHz, HAL库, GCC+CMake+Ninja
- 串口: USART2, 115200-8N1 (PA2 TX, PA3 RX)
- I2C: I2C2, OLED(0x3C) + MPU6050(0x68)
- 电机: TIM1_CH1/CH2 PWM(20kHz), PB12-PB15 GPIO方向(TB6612)
- 编码器: TIM2(编码器模式, PPR=1320), TIM3(编码器模式)
- 灰度: 5路 GPIO输入(PB3-PB7)
- 舵机: TIM4_CH3 PWM(50Hz)
- OLED: SSD1315(I2C)

**核心设计文档**:
- `doc/巡线逻辑设计文档.md` — 完整巡线状态机: FOLLOW/LINE_EXIT/TURNING/SEARCH, 三层阻尼机制, 直角弯判定, PD控制
- `doc/工程文档.md` — 引脚分配, 软件架构, 串口协议, 初始化流程
- `doc/调试总结.md`

**串口协议(可直接参考用于A板→B板通信)**:
- 二进制帧: `0xAA | cmd | id | 参数 | 0xFF 0xFF`
- 命令枚举: CMD_MOTOR_SET_RPM(0x01), CMD_MOTOR_BRAKE(0x03), CMD_SERVO_SET_ANGLE(0x10), CMD_PING(0xF0)等
- 状态枚举: STATUS_OK(0xA0), STATUS_ERROR(0xA1)等

**巡线逻辑(可直接参考)**:
- 5路灰度加权: pos = (-2*S1 - S2 + 0*S3 + S4 + 2*S5) / sum
- PD控制: corr = Kp*pos + Kd*d_pos
- 三层阻尼: 回中反向阻尼 + 逐次衰减g_decay + 死区迟滞
- 参数: Kp=6.0, Kd=2.0, 巡航30RPM, 转弯60RPM, 50ms周期

**软件架构(C++桥接模式)**:
- driver层: encoder/motor/servo/pwm/uart/usergpio/i2c_hardware_ops/oled_driver
- common层: ringbuf/pid/imu_filter/tool
- C++类 + extern "C" bridge, 平台ops表模式

### 2.2 A_board_pwm_driver_test — F427电机/串口/PWM架构

**高度相关**: B板是RM A型板(STM32F427IIHx)，可直接参考

**已有设计文档**:
- `motor_arch_design.md` — TB6612电机四层架构(bridge→Motor→IMotorProtocol→TB6612MotorProtocol→pwm/usergpio)
- `UART_Serial_Design.md` — UART驱动+环形缓冲区+二进制命令解析(与BluePill_Car协议一致)
- `PWM_Driver_Design.md` — PWM抽象层
- `Servo_Driver_Design.md` — 舵机驱动
- `State_Transition_Reference.md` — 状态转换参考
- `IMU_Driver_Design.md` — IMU驱动设计
- `Porting_Guide.md` / `Bluepill_Porting_Plan.md` — 移植指南

**电机架构(可直接用于B板STEP脉冲)**:
- 四层分离: motor_bridge(C接口) → Motor类 → IMotorProtocol → TB6612MotorProtocol → pwm/usergpio
- set_speed_rpm/set_speed_mps/brake/stop
- 千分比速率(0E3)统一接口
- PWM: TIM5_CH4(PI0), 20kHz

**串口协议(与BluePill_Car完全一致)**:
- 同样的二进制帧格式和命令枚举
- 环形缓冲区无锁设计
- 中断只写ringbuf+重使能接收(<2μs)

### 2.3 ESP32-CAM资料

**产品规格书(已提取文本到`doc/赛题原文/ESP32-CAM_规格书提取.txt`)**:
- 模块: ESP32-CAM, 尺寸27×40.5×4.5mm
- CPU: 双核32位, 主频240MHz, 600DMIPS
- RAM: 内部520KB + 外部4M PSRAM
- SPI Flash: 32Mbit
- WiFi: 802.11b/g/n, 支持STA/AP/STA+AP
- 蓝牙: 4.2 BR/EDR + BLE
- 摄像头: 支持OV2640和OV7670(注意:规格书写OV2640/OV7670，用户实际用OV3660)
- 供电: 5V
- 功耗: 关闪光180mA, 深睡眠6mA
- IO口: 9个
- 串口速率: 默认115200bps
- 图像格式: JPEG/BMP/GRAYSCALE
- TF卡: 最大4G
- 内置闪光灯
- 重量: 10g

**其他资料**:
- `esp32_datasheet_cn.pdf` — ESP32芯片数据手册
- `esp32_technical_reference_manual_cn.pdf` — 技术参考手册
- `摄像头ov2640_ds_1.8_.pdf` — OV2640数据手册(用户实际用OV3660,此手册仅供参考)
- `出厂默认固件ai-thinker_esp32-cam_dio_v1.0_20180825.zip` — 出厂固件
- `ESP32_CAMERA_QR-master.zip` — QR识别示例

### 2.4 RM_databook — RM A板手册(已提取)

**使用说明(已提取文本到`doc/H题设计/RM_A板_使用说明提取.txt`)**:
- 芯片: STM32F427IIH6
- 最大电压: 26V
- 支持电池: 4~6S LiPo
- 最大持续电流: 20A
- 尺寸: 85×58mm, 重48g
- 板载接口: CAN1/CAN2, PWM×8, USB, UART, OLED, DBUS, SWD, GPIO×18, TF卡, 用户按键, LED×10
- 电源: XT30供电, 板载LM25116(12V@10A), MP2233(3.3V@3A), TPS54540(可调5~12V@5A), 12V输出×3, 5V输出, 3.3V输出
- 支持DFU固件更新

---

## 三、根据实事信息可明确的内容

### 3.1 器件型号明确

| 器件 | 明确型号 |
|------|----------|
| 步进电机 | 美蓓亚 17PM-J347-G2VS |
| 步进驱动 | TMC2209 (丝印确认: tmc2209-la1935 82600 germany) |
| 直流电机 | MG513XP28_12V |
| 轮径 | 66mm |
| 红外 | 5~6路灰度传感器 |
| A板 | STM32F103ZET6 |
| B板 | RM A型板 (STM32F427IIH6) |
| OLED | SSD1306 0.96" SPI 4线 (在A板) |
| 视觉 | 庐山派Lite K230D (GC2093) |
| 图传 | ESP32-CAM (OV3660) |
| 电池 | INR18650 4S2P (14.8V) |
| 图传接收 | 手机 |

### 3.2 参考工程可直接复用

| 参考工程 | 对应板 | 可复用内容 |
|----------|--------|-----------|
| BluePill_Car | A板(F103) | 巡线状态机、PD控制、串口协议、PID、编码器、TB6612驱动、OLED |
| A_board_pwm_driver_test | B板(F427) | 电机架构、UART驱动、PWM驱动、GPIO抽象、移植指南 |

### 3.3 通信协议已有参考

二进制帧格式(0xAA帧头 + cmd + id + 参数 + 0xFF 0xFF帧尾)已在两个参考工程中实现，可直接用于A板→B板通信。

### 3.4 三人分工明确

| 成员 | 职责 |
|------|------|
| 成员1 | 建模+规划+算法(球杆系统建模、PID/LQR、前馈补偿、速度规划) |
| 成员2 | 底层移植(驱动代码、电机/编码器/UART/PWM/OLED移植) |
| 成员3 | 视觉识别(K230D球位置检测算法、ESP32-CAM图传配置) |

---

## 四、仍有疑问/信息缺失

### 4.1 器件规格缺失

| 器件 | 缺失信息 | 说明 |
|------|----------|------|
| 17PM-J347-G2VS | 步距角/额定电压/电流/保持扭矩 | 需查手册或用户提供 |
| MG513XP28_12V | 减速比/编码器PPR/空载RPM | 12V已确认，其余需查手册 |
| TMC2209紫板 | 具体电气参数 | 图片无法读取，丝印已确认芯片为TMC2209 |
| 红外灰度传感器 | 具体型号/间距/输出电平 | 5~6路已确认，型号未知 |

### 4.2 ESP32-CAM摄像头疑问

ESP32-CAM规格书写的是"支持OV2640和OV7670"，但用户说摄像头是OV3660。**OV3660是否兼容ESP32-CAM?** 需确认用户的具体ESP32-CAM模块是否为定制版或 OV3660 接口兼容版。

### 4.3 RM A板IMU使用

RM A板板载MPU6500(IMU)，是否用于检测摆杆倾角? 官方问答Q40/Q41说"参照命题要求"，管壁不允许传感器，但IMU在支撑台上。**是否使用IMU需用户确认。**

### 4.4 供电方案

RM A板支持4~6S LiPo(最高26V)，用户电池是INR18650 4S2P(14.8V)。RM A板自带电源管理(LM25116 12V输出、MP2233 3.3V、TPS54540可调5~12V)。**B板供电是否直接用RM A板板载电源管理?** A板(F103ZET6)供电方案待确认。

### 4.5 BluePill_Car的OLED

BluePill_Car用的是SSD1315(I2C)，用户A板用的是SSD1306(SPI 4线)。**驱动代码不能直接复用，需适配。**

### 4.6 编码器PPR

BluePill_Car工程文档写PPR=1320，但那是BluePill配的编码器。MG513XP28_12V的编码器PPR可能不同，**需确认。**
