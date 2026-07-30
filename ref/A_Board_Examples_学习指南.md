
# RoboMaster A Board STM32 官方例程学习指南

> 本文档基于 RoboMaster 开发板 (A Board) 的 7 个官方示例工程，旨在帮助初学者系统性地学习 STM32 嵌入式开发。
> 目标芯片: **STM32F427IIHx** | 开发环境: **Keil MDK (uvprojx)** | HAL 库版本: **STM32F4xx_HAL_Driver**

---

## 目录

1. [通用工程结构分析](#1-通用工程结构分析)
2. [PWM 输出例程](#2-pwm-输出例程)
3. [IMU 陀螺仪数据读取例程](#3-imu-陀螺仪数据读取例程)
4. [RemoteControl 遥控器接收解码例程](#4-remotecontrol-遥控器接收解码例程)
5. [RM_OLED 显示屏例程](#5-rm_oled-显示屏例程)
6. [SDCard 文件系统例程](#6-sdcard-文件系统例程)
7. [USB 虚拟串口例程](#7-usb-虚拟串口例程)
8. [UWB 定位模块数据读取例程](#8-uwb-定位模块数据读取例程)
9. [总结与进阶建议](#9-总结与进阶建议)

---

## 1. 通用工程结构分析

所有例程均采用 STM32CubeMX 生成的 **HAL 库** 工程结构，代码框架高度统一。

### 1.1 目录结构约定

```
ProjectName/
├── Drivers/                  # 底层驱动（不修改）
│   ├── CMSIS/                # ARM Cortex-M 内核抽象层
│   └── STM32F4xx_HAL_Driver/ # ST 官方 HAL 硬件抽象层驱动
├── Inc/                      # 用户头文件（CubeMX 自动生成 + 用户添加）
│   ├── main.h                # 主头文件（引脚定义、宏定义）
│   ├── gpio.h                # GPIO 初始化
│   ├── usart.h               # 串口初始化（如用到）
│   ├── tim.h                 # 定时器初始化（如用到）
│   ├── spi.h                 # SPI 初始化（如用到）
│   ├── can.h                 # CAN 初始化（如用到）
│   └── ...
├── Src/                      # 用户源文件（CubeMX 自动生成 + 用户添加）
│   ├── main.c                # 主程序入口
│   ├── gpio.c                # GPIO 初始化实现
│   ├── usart.c               # 串口初始化实现
│   └── ...
├── MDK-ARM/                  # Keil 工程文件
│   ├── bsp/                  # 板级支持包（手动添加，非 CubeMX 生成）
│   │   ├── bsp_imu.c/h       # IMU 驱动
│   │   ├── bsp_uart.c/h      # 遥控器 UART 驱动
│   │   └── ...
│   ├── app/                  # 应用层代码（手动添加）
│   │   └── sd_card.c/h       # SD 卡测试应用
│   ├── Project.uvprojx       # Keil 工程文件
│   └── startup_stm32f427xx.s # 启动文件
├── Project.ioc               # STM32CubeMX 配置文件
└── README.md                 # 工程说明
```

### 1.2 工程配置通用模板

所有例程的 main.c 都遵循同样的框架：

```c
int main(void)
{
    /* STEP 1: HAL 库初始化 */
    HAL_Init();

    /* STEP 2: 系统时钟配置 */
    SystemClock_Config();

    /* STEP 3: 外设初始化（CubeMX 自动生成的函数） */
    MX_GPIO_Init();         // GPIO 初始化
    MX_USART6_UART_Init();  // 串口6初始化（调试输出）
    // ... 其他外设

    /* STEP 4: 用户自定义初始化 */
    led_off();              // 关闭所有 LED
    // ... 其他 BSP 初始化

    /* STEP 5: 主循环 */
    while (1)
    {
        // 用户业务逻辑
        // ...
        HAL_Delay(500);     // 延时 500ms
        HAL_GPIO_TogglePin(...); // 翻转 LED
    }
}
```

### 1.3 系统时钟配置详解

所有例程的时钟配置基本一致（USB 工程有细微差别），统一为：

| 参数 | 值 | 说明 |
|------|-----|------|
| HSE 晶振 | 8 MHz | 外部高速晶振 |
| PLLM | 6 | PLL 输入分频 |
| PLLN | 168 | PLL 倍频 |
| PLLP | 2 | PLL 输出分频 |
| SYSCLK | 168 MHz | 系统主频 |
| AHB (HCLK) | 168 MHz | AHB 总线时钟 |
| APB1 (PCLK1) | 42 MHz | APB1 外设时钟 (168/4) |
| APB2 (PCLK2) | 84 MHz | APB2 外设时钟 (168/2) |

> **学习要点**：系统时钟是 MCU 的心脏，通过 PLL 锁相环将外部 8MHz 晶振倍频到 168MHz。APB1 和 APB2 的时钟不同，外设挂载时需注意总线频率限制。

### 1.4 通用外设: LED 和调试串口

每个例程都使用了以下通用硬件资源：

- **LED 指示灯**：
  - LED_RED: PE11 (红色)
  - LED_GREEN: PF14 (绿色)
  - 低电平点亮，通过 `led_off()` 函数初始化关闭所有 LED
  - 常用函数：`HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)`

- **调试串口 USART6**：
  - 用于打印调试信息到串口助手
  - 发送函数：`HAL_UART_Transmit(&huart6, buf, len, timeout)`

---

## 2. PWM 输出例程

### 2.1 功能概述

演示 STM32 定时器的 PWM 输出功能，同时驱动 **4 组定时器共 16 路 PWM 通道** 输出，适用于舵机/电机控制。

### 2.2 使用的外设

| 外设 | 功能 | 引脚映射 |
|------|------|----------|
| TIM2 | PWM 输出 (CH1~CH4) | PA0~PA3 |
| TIM4 | PWM 输出 (CH1~CH4) | PD12~PD15 |
| TIM5 | PWM 输出 (CH1~CH4) | PH10~PH12, PI0 |
| TIM8 | PWM 输出 (CH1~CH4) | PI2, PI5~PI7 |

### 2.3 核心参数

```c
#define PWM_FREQUENCE   50      // PWM 频率 50Hz（标准舵机频率）
#define PWM_RESOLUTION  10000   // PWM 分辨率（计数值）
#define APB1_TIMER_CLOCKS 84000000  // APB1 时钟 (TIM2,4,5)
#define APB2_TIMER_CLOCKS 168000000 // APB2 时钟 (TIM8)
```

- **PWM 周期计算**: 频率 = 50Hz，周期 = 20ms
- **分辨率**: 10000 步，即占空比精度为 0.01%
- **预分频计算**: `TIM_PSC_APB1 = (84MHz / 50Hz / 10000) - 1`

### 2.4 核心代码分析

**PWM 占空比设置函数**:
```c
void PWM_SetDuty(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty)
{
    switch(tim_channel) {
        case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_3: tim->Instance->CCR3 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_4: tim->Instance->CCR4 = (PWM_RESOLUTION * duty) - 1; break;
    }
}
```

> **核心思想**: 通过直接操作定时器的 CCR (Capture/Compare Register) 寄存器来设置占空比。`duty` 为 0.0~1.0 的浮点数，乘以分辨率得到具体的计数值。

**启动 PWM 输出**:
```c
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // 启动指定定时器的 PWM 通道
```

### 2.5 学习要点

1. **定时器工作原理**: 定时器通过递增计数器 CNT，与 CCR 比较，当 CNT < CCR 时输出高电平，反之输出低电平。
2. **预分频器 PSC**: 将定时器时钟分频到合适频率，`定时器频率 = 总线时钟 / (PSC + 1)`。
3. **自动重装载 ARR**: 决定 PWM 周期，`PWM 频率 = 定时器频率 / (ARR + 1)`。
4. **占空比控制**: 通过 CCR / ARR 的比值控制。

### 2.6 典型应用场景

- 舵机控制 (50Hz, 占空比 5%~10% 对应 0~180 度)
- 直流电机调速
- LED 呼吸灯效果
- 无刷电机 ESC 信号

---

## 3. IMU 陀螺仪数据读取例程

### 3.1 功能概述

读取 MPU6500（六轴陀螺仪+加速度计）和 IST8310（磁力计）的数据，通过 **Mahony 姿态解算算法** 计算飞行器/机器人的 Roll (横滚)、Pitch (俯仰)、Yaw (偏航) 角度，并通过串口输出。

### 3.2 使用的外设

| 外设 | 功能 |
|------|------|
| SPI5 | 与 MPU6500 通信 |
| USART6 | 打印姿态数据到串口 |

### 3.3 硬件连接

- **MPU6500** 通过 SPI5 连接 (NSS: PF6)
- **IST8310** 磁力计通过 MPU6500 的 **I2C Master** 接口连接（辅助 I2C）

### 3.4 核心数据结构

```c
// MPU6500 原始数据（含零偏）
typedef struct {
    int16_t ax, ay, az;      // 加速度计原始值
    int16_t mx, my, mz;      // 磁力计原始值
    int16_t temp;             // 温度原始值
    int16_t gx, gy, gz;      // 陀螺仪原始值（已减零偏）
    int16_t ax_offset, ay_offset, az_offset;  // 加速度零偏
    int16_t gx_offset, gy_offset, gz_offset;  // 陀螺仪零偏
} mpu_data_t;

// IMU 解算后的姿态数据
typedef struct {
    int16_t ax, ay, az;      // 加速度计
    int16_t mx, my, mz;      // 磁力计
    float temp;               // 温度 (摄氏度)
    float wx, wy, wz;         // 角速度 (rad/s)
    float vx, vy, vz;         // 加速度 (归一化)
    float rol, pit, yaw;      // Roll/Pitch/Yaw (度)
} imu_t;
```

### 3.5 核心代码分析

#### MPU6500 初始化流程

```c
uint8_t mpu_device_init(void)
{
    // 1. 读取 WHO_AM_I 寄存器，验证设备 ID (应为 0x70)
    id = mpu_read_byte(MPU6500_WHO_AM_I);

    // 2. 配置 MPU6500 寄存器
    //    - PWR_MGMT_1: 复位 -> 选择陀螺仪 Z 轴为时钟源
    //    - PWR_MGMT_2: 使能加速度计和陀螺仪
    //    - CONFIG: 设置数字低通滤波器 (DLPF) 41Hz
    //    - GYRO_CONFIG: 量程 ±2000dps
    //    - ACCEL_CONFIG: 量程 ±8G
    //    - USER_CTRL: 使能 I2C Master 模式（用于访问 IST8310）

    // 3. 初始化 IST8310 磁力计
    ist8310_init();

    // 4. 采集零偏数据（采样 300 次取平均）
    mpu_offset_call();
}
```

#### 零偏校准

```c
void mpu_offset_call(void)
{
    for (i = 0; i < 300; i++) {
        // 读取原始数值并累加
        mpu_data.gx_offset += gx_raw;
        // ...
        HAL_Delay(5);
    }
    // 取平均值作为零偏
    mpu_data.gx_offset /= 300;
}
```

> **为什么需要零偏？** 陀螺仪和加速度计在静止时输出值并不是 0，存在系统偏差。通过采集多次数据取平均可以得到静态零偏，后续读取数据时减去这个零偏即可得到真实的角速度/加速度。

#### 数据读取和姿态解算

```c
void mpu_get_data(void)
{
    // 1. 通过 SPI 读取 14 字节原始数据（加速度+温度+陀螺仪）
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    // 2. 将原始字节拼接为 16 位整数
    mpu_data.ax = mpu_buff[0] << 8 | mpu_buff[1];
    // ...

    // 3. 读取磁力计数据（通过 MPU6500 的 I2C Master 自动采集）
    ist8310_get_data(ist_buff);

    // 4. 单位转换
    //    陀螺仪: 2000dps / 32768 = 16.384 LSB/dps -> rad/s (除以 57.3)
    imu.wx = mpu_data.gx / 16.384f / 57.3f;

    //    温度: 值 / 333.87 + 21 = 摄氏度
    imu.temp = 21 + mpu_data.temp / 333.87f;
}

void imu_ahrs_update(void)
{
    // Mahony 滤波算法（互补滤波）
    // 1. 归一化加速度计和磁力计数据
    // 2. 计算重力参考方向 vs 测量方向的误差
    // 3. PI 控制器修正陀螺仪偏差
    // 4. 四元数更新（一阶龙格-库塔积分）
    // 5. 四元数归一化
}

void imu_attitude_update(void)
{
    // 从四元数解算欧拉角
    imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 57.3;
    imu.pit = -asin(-2*q1*q3 + 2*q0*q2) * 57.3;
    imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.3;
}
```

### 3.6 Mahony 滤波算法理解

这是整个 IMU 例程最核心的部分，一定要理解其思想：

1. **陀螺仪**：动态响应快，但有积分漂移（长时间会偏离真实值）
2. **加速度计**：静态精度高，但动态响应慢，受振动干扰大
3. **磁力计**：提供航向参考，但易受磁场干扰

**互补滤波思想**：融合三种传感器的优势，用加速度计和磁力计修正陀螺仪的积分误差。

```
                   +------------------+
                   |   陀螺仪 (角速度)  |----+
                   +------------------+    |   +----------+
                                           +-->| 四元数更新 |--> 欧拉角
                   +------------------+    |   +----------+
                   | 加速度计 (重力方向) |----+
                   +------------------+    |
                                           |   (误差修正)
                   +------------------+    |
                   |  磁力计 (航向)    |----+
                   +------------------+
```

### 3.7 学习要点

1. **SPI 通信**：全双工同步通信，通过 NSS 片选、SCK 时钟、MOSI/MISO 数据线通信
2. **传感器数据读取**：注意字节序（大端/小端），原始值到物理值的换算
3. **姿态解算**：四元数与欧拉角的转换，互补滤波/Mahony 算法
4. **I2C Master 模式**：MPU6500 的辅助 I2C 主机模式，可直接读取外部传感器

---

## 4. RemoteControl 遥控器接收解码例程

### 4.1 功能概述

接收 RoboMaster 遥控器接收机的 **DBUS 协议** 数据，解码出遥控器的 4 个摇杆通道值和 2 个拨杆开关状态，并通过串口打印。

### 4.2 使用的外设

| 外设 | 功能 |
|------|------|
| USART1 | 接收遥控器 DBUS 数据（通过 DMA） |
| USART6 | 打印解码后的遥控数据到串口 |
| DMA2 Stream2 | USART1 RX 的 DMA 传输 |

### 4.3 DBUS 协议解析

DBUS 协议是 RoboMaster 遥控器使用的自定义串口协议：

- **波特率**: 100000 bps
- **数据帧长度**: 18 字节
- **采用 DMA + IDLE 中断** 方式接收完整帧

### 4.4 核心数据结构

```c
typedef struct {
    int16_t ch1;   // 右摇杆 X 轴 (-660 ~ +660)
    int16_t ch2;   // 右摇杆 Y 轴 (-660 ~ +660)
    int16_t ch3;   // 左摇杆 Y 轴 (-660 ~ +660)
    int16_t ch4;   // 左摇杆 X 轴 (-660 ~ +660)
    uint8_t sw1;   // 左侧拨杆开关 (0/1/2/3)
    uint8_t sw2;   // 右侧拨杆开关 (0/1/2/3)
} rc_info_t;
```

### 4.5 核心代码分析

#### 数据接收机制 (DMA + IDLE 中断)

```c
void dbus_uart_init(void)
{
    // 1. 使能 USART1 的 IDLE 中断（空闲中断）
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

    // 2. 启动 DMA 接收（不使能 DMA 传输完成中断）
    uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
```

> **DMA + IDLE 接收方式**：DMA 负责将串口数据自动搬运到缓冲区，当串口接收到一帧数据后出现空闲状态，触发 IDLE 中断。在 IDLE 中断中处理数据，然后重置 DMA 继续接收。这种方式效率高，适合不定长帧接收。

#### IDLE 中断处理

```c
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
    // 1. 清除 IDLE 标志
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    // 2. 计算接收到的数据长度
    //    DBUS_MAX_LEN - DMA 剩余计数值 = 已接收的字节数
    if ((DBUS_MAX_LEN - dma_current_data_counter(...)) == DBUS_BUFLEN) {
        // 3. 解码数据
        rc_callback_handler(&rc, dbus_buf);
    }

    // 4. 重置 DMA 继续接收
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}
```

#### 遥控器数据解码

```c
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    // 每个摇杆通道占 11 位（0~2047），分布在连续的字节中
    // 通过位运算提取对应位
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;  // 取低 11 位
    rc->ch1 -= 1024;  // 减去中值 1024，得到有符号值 (-1024~+1023)

    // 拨杆开关: 从第 5 字节的高 4 位中提取
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    // 安全检查: 任何通道绝对值超过 660 则认为数据异常，清零
    if (abs(rc->ch1) > 660 || ...) {
        memset(rc, 0, sizeof(rc_info_t));
    }
}
```

### 4.6 学习要点

1. **DMA + IDLE 中断**：高效的串口接收方案，适合接收不定长数据帧
2. **位操作解码**：掌握位与(&)、位或(|)、左移(<<)等操作进行数据提取
3. **数据校验**：通过范围检查判断数据是否有效（安全检查机制）
4. **波特率 100000**：非标准波特率，需要在 CubeMX 中手动设置

### 4.7 典型应用场景

- RoboMaster 机器人遥控控制
- 自定义遥控器协议接收
- 任何需要高效串口接收的场景

---

## 5. RM_OLED 显示屏例程

### 5.1 功能概述

驱动开发板上的 0.96 寸 OLED 显示屏 (128x64 像素，SSD1306 控制器)，显示 DJI Logo 图片。

### 5.2 使用的外设

| 外设 | 功能 |
|------|------|
| SPI1 | 与 OLED 通信 |
| ADC1 | (已配置但未在 main 中使用，预留) |

### 5.3 OLED 驱动核心

#### 显存管理

```c
static uint8_t OLED_GRAM[128][8];  // 显存缓冲区
```

OLED 的物理分辨率为 128x64 像素，在内存中按 **页 (Page)** 组织：

```
    列: 0        1        2      ...  127
页0: [0-7位]   [0-7位]   [0-7位]  ...  [0-7位]   (第 0~7 行)
页1: [0-7位]   [0-7位]   [0-7位]  ...  [0-7位]   (第 8~15 行)
...     ...       ...       ...    ...   ...
页7: [0-7位]   [0-7位]   [0-7位]  ...  [0-7位]   (第 56~63 行)
```

每个字节的 8 个位对应一列中连续的 8 个像素点。

#### SPI 命令/数据写入

```c
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd != 0)
        OLED_CMD_Set();   // DC = 1: 数据
    else
        OLED_CMD_Clr();   // DC = 0: 命令

    HAL_SPI_Transmit(&hspi1, &dat, 1, 10);  // SPI 发送
}
```

> OLED 的 DC 引脚控制传输的是命令还是数据。写命令用于配置 OLED 的工作模式，写数据用于刷新显示内容。

#### OLED 初始化序列

初始化是通过 SPI 发送一系列配置命令，包括：
1. 关闭显示
2. 设置列/行地址映射
3. 设置对比度
4. 设置电荷泵
5. 设置显示时钟分频
6. 开启显示

#### 刷新显示

```c
void oled_refresh_gram(void)
{
    for (i = 0; i < 8; i++) {          // 遍历 8 页
        oled_set_pos(0, i);             // 设置光标到页首
        for (n = 0; n < 128; n++) {     // 遍历 128 列
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);  // 写入显存数据
        }
    }
}
```

#### 驱动函数库

该例程提供了完善的 OLED 图形驱动库：

| 函数 | 功能 |
|------|------|
| `oled_init()` | OLED 初始化 |
| `oled_drawpoint(x, y, pen)` | 画点（写/清除/反转） |
| `oled_drawline(x1,y1,x2,y2,pen)` | 画线 |
| `oled_showchar(row, col, chr)` | 显示字符 (6x12 字体) |
| `oled_shownum(row, col, num, mode, len)` | 显示数字 |
| `oled_showstring(row, col, chr)` | 显示字符串 |
| `oled_printf(row, col, fmt, ...)` | 格式化输出（类似 printf） |
| `oled_LOGO()` | 显示 DJI Logo 位图 |
| `oled_clear(pen)` | 清屏 |

### 5.4 学习要点

1. **SPI 驱动 OLED**：掌握通过 SPI 总线控制外设的方法
2. **显存操作**：修改本地显存缓冲区，一次性刷新到硬件
3. **字库原理**：`oledfont.h` 中存储了 ASCII 字符的点阵数据
4. **位图显示**：从位图数组中逐位解析并绘制像素点
5. **Pen 模式**：支持写入(置1)、清除(置0)、反转(异或)三种操作模式

---

## 6. SDCard 文件系统例程

### 6.1 功能概述

演示如何通过 **SDIO 接口** 驱动 SD 卡，并使用 **FatFS 文件系统** 进行文件的创建、写入和读取操作。

### 6.2 使用的外设

| 外设 | 功能 |
|------|------|
| SDIO | 与 SD 卡通信（4 位数据线模式） |
| USART6 | 打印测试结果到串口 |
| DMA2 Stream3/6 | SDIO 数据传输（可选） |

### 6.3 核心代码分析

#### 初始化流程

```c
// main.c 中
BSP_SD_Init();   // 初始化 SD 卡硬件
sd_test();       // 执行 SD 卡读写测试
```

#### 文件系统测试流程

```c
void sd_test(void)
{
    FATFS SDFatFs;    // 文件系统对象
    FIL SDFile;       // 文件对象

    // STEP 1: 挂载文件系统（注册逻辑驱动器）
    f_mount(&SDFatFs, SDPath, 0);

    // STEP 2: 格式化 SD 卡（创建 FAT 文件系统）
    f_mkfs(SDPath, FM_ANY, 0, buffer, sizeof(buffer));

    // STEP 3: 创建并打开文件（写模式）
    f_open(&SDFile, "robomaster.txt", FA_CREATE_ALWAYS | FA_WRITE);

    // STEP 4: 写入数据
    f_write(&SDFile, " Welcome to Robomaster! ", sizeof(wtext), &byteswritten);

    // STEP 5: 关闭文件
    f_close(&SDFile);

    // STEP 6: 打开文件（读模式）
    f_open(&SDFile, "robomaster.txt", FA_READ);

    // STEP 7: 读取数据
    f_read(&SDFile, rtext, sizeof(rtext), &bytesread);

    // STEP 8: 关闭文件
    f_close(&SDFile);

    // STEP 9: 卸载文件系统
    FATFS_UnLinkDriver(SDPath);
}
```

#### 错误指示

通过红色 LED 闪烁次数指示不同的错误状态：
- **闪烁 1 次**: 文件系统挂载/格式化失败
- **闪烁 2 次**: 文件打开失败
- **LED 灭**: 读写成功

### 6.4 FatFS 文件系统要点

| 函数 | 功能 | 返回值 |
|------|------|--------|
| `f_mount` | 注册/注销一个工作区（必须先调用） | FR_OK 或错误码 |
| `f_mkfs` | 格式化逻辑驱动器（创建 FAT 文件系统） | FR_OK 或错误码 |
| `f_open` | 打开/创建文件 | FR_OK 或错误码 |
| `f_write` | 写入数据到文件 | FR_OK 或错误码 |
| `f_read` | 从文件读取数据 | FR_OK 或错误码 |
| `f_close` | 关闭文件 | FR_OK 或错误码 |

### 6.5 学习要点

1. **SDIO 协议**：了解 SD 卡的初始化流程（CMD0, CMD8, ACMD41, CMD2, CMD3）
2. **FatFS 移植**：需要提供底层的磁盘 I/O 函数（disk_read, disk_write, disk_status, disk_initialize）
3. **文件操作流程**：挂载 -> 打开 -> 读写 -> 关闭 -> 卸载
4. **错误处理**：程序中的错误处理机制（LED 闪烁指示）
5. **CubeMX 配置**：SDIO 的时钟分频、数据线宽度、SDIO 中断优先级等

---

## 7. USB 虚拟串口例程

### 7.1 功能概述

实现 **USB 虚拟串口 (CDC - Communication Device Class)** 功能，让开发板通过 USB 线连接到电脑后，在电脑上显示为一个虚拟 COM 端口，实现 **USB <-> 串口 双向透传**。

### 7.2 使用的外设

| 外设 | 功能 |
|------|------|
| USB_OTG_FS | USB 全速设备接口 |
| USART6 | 物理串口（与 USB 桥接） |
| DMA | USART6 的 DMA 传输 |

### 7.3 系统架构

```
PC (串口助手) <--USB--> STM32 (USB CDC) <--USART6--> 外部串口设备
```

数据流向：
- **接收方向 (PC -> 外部设备)**: USB OUT endpoint -> CDC_Receive_FS() -> HAL_UART_Transmit_DMA(&huart6, ...)
- **发送方向 (外部设备 -> PC)**: USART6 RX IDLE 中断 -> UART_RxCplCallback() -> CDC_Transmit_FS() -> USB IN endpoint

### 7.4 核心代码分析

#### USB CDC 接收回调（PC 发来的数据）

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    // 1. 准备下一个 USB 接收缓冲区
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    // 2. 通过串口 DMA 转发出去
    HAL_UART_Transmit_DMA(&huart6, Buf, *Len);

    return USBD_OK;
}
```

#### 串口接收完成回调（外部设备发来的数据）

```c
void UART_RxCplCallback(UART_HandleTypeDef* uart)
{
    // 通过 USB 发送到 PC
    CDC_Transmit_FS(uart->pRxBuffPtr, uart->RxXferCount);
}
```

#### 动态串口参数配置

当 PC 端修改串口号参数（波特率、停止位、校验位等），USB CDC 会收到 `CDC_SET_LINE_CODING` 控制命令，`ComPort_Config()` 函数根据新参数重新初始化 USART6：

```c
static void ComPort_Config(void)
{
    // 1. 停止 DMA，反初始化串口
    HAL_UART_DeInit(&huart6);

    // 2. 根据 PC 端设置的参数重新配置
    huart6.Init.BaudRate  = SET_LineCoding.bitrate;
    huart6.Init.StopBits  = 根据 format 字段设置;
    huart6.Init.Parity    = 根据 paritytype 字段设置;
    huart6.Init.WordLength = 根据 datatype 字段设置;

    // 3. 重新初始化串口
    HAL_UART_Init(&huart6);
}
```

### 7.5 USB CDC 协议层次

```
应用程序层: CDC_Receive_FS / CDC_Transmit_FS
    |
USB 设备类层: USBD_CDC (usbd_cdc.c/h)
    |
USB 核心层: USBD_Core (usbd_core.c)
    |
USB 设备底层: STM32 USB_OTG 外设驱动
    |
硬件: USB D+ / D- 差分信号线
```

### 7.6 学习要点

1. **USB CDC 类**：最为常用的 USB 通信类之一，Windows/macOS/Linux 原生支持
2. **端点 (Endpoint)**：USB CDC 使用 3 个端点（IN 发送、OUT 接收、NOTIFY 控制）
3. **描述符 (Descriptor)**：设备描述符、配置描述符、接口描述符、端点描述符
4. **双向透传实现**：理解 USB 到串口的数据桥接逻辑
5. **USART IDLE 中断 + DMA**：高效接收不定长数据

### 7.7 典型应用场景

- 开发板调试接口（替代 FT232/CH340）
- 与 PC 端的上位机软件通信
- 固件升级（DFU 模式）

---

## 8. UWB 定位模块数据读取例程

### 8.1 功能概述

接收 RoboMaster UWB 定位模块通过 **CAN 总线** 发送的数据，解析出机器人的 X/Y 坐标、偏航角 Yaw 和信号强度等信息，并通过串口打印。

### 8.2 使用的外设

| 外设 | 功能 |
|------|------|
| CAN1 | 接收 UWB 模块数据 |
| CAN2 | (已初始化但未在主循环中使用，预留) |
| USART6 | 打印定位数据到串口 |

### 8.3 UWB 数据结构

```c
typedef struct {
    int16_t  coor_x;          // X 坐标 (mm)
    int16_t  corr_y;          // Y 坐标 (mm)
    uint16_t yaw;             // 偏航角 (0.1 度为单位)
    int16_t  distance[6];     // 到 6 个基站的测距值
    uint16_t err_mask  : 14;  // 错误掩码
    uint16_t sig_level : 2;   // 信号强度等级 (0-3)
    uint16_t reserved;        // 保留
} uwb_info_t;
```

### 8.4 CAN 过滤器配置

```c
HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;           // 滤波器组编号
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32 位宽度
    sFilterConfig.FilterIdHigh = 0x0000;    // 不筛选特定 ID
    sFilterConfig.FilterMaskIdHigh = 0x0000; // 掩码全 0 = 接收所有
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14; // CAN2 从 14 号滤波器开始

    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    return HAL_OK;
}
```

> **滤波模式**：这里设置为接收所有 CAN 消息（掩码全 0），在实际产品中应设置具体的 CAN ID 过滤器以减轻 CPU 负担。

### 8.5 CAN 数据接收与拼包

UWB 模块的数据通过多条 CAN 帧发送，需要在接收中断中拼包：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    if (RxHeader.StdId != UWB_CAN_RX_ID)  // 非心跳包
    {
        if (RxHeader.DLC == 8)  // 8 字节数据帧：数据包前半部分
        {
            memcpy(p, RxData, 8);
            p += 8;
            unpackstep++;
        }
        else if (RxHeader.DLC == 6)  // 6 字节数据帧：数据包最后部分
        {
            memcpy(p, RxData, 6);
            // 拼包完成，拷贝到结构体
            memcpy(&uwb_data, &uwb_rx_data_buf, sizeof(uwb_info_t));
            can_rx_flag++;
        }
    }
}
```

### 8.6 CAN 总线基础

| 概念 | 说明 |
|------|------|
| CAN 标准帧 | 11 位标识符 (ID) + 数据段 (0~8 字节) |
| CAN 扩展帧 | 29 位标识符 |
| DLC | Data Length Code，数据长度 |
| FIFO | 硬件接收缓冲区，有 FIFO0 和 FIFO1 |
| 滤波器 | 硬件级报文过滤，可以接收/屏蔽特定 ID 的消息 |

### 8.7 学习要点

1. **CAN 总线协议**：差分信号传输，具有高可靠性、实时性和抗干扰能力
2. **CAN 过滤器**：硬件级的报文筛选，可配置为掩码模式或列表模式
3. **CAN 中断接收**：使用 FIFO 消息等待中断，高效接收数据
4. **多帧拼包**：较长数据需要通过多条 CAN 帧分段传输，需要拼包逻辑

### 8.8 典型应用场景

- RoboMaster UWB 定位系统
- 机器人内部通信（电机、传感器等）
- 工业现场总线控制

---

## 9. 总结与进阶建议

### 9.1 各例程核心技术总结

| 例程 | 核心外设 | 关键技术 | 难度 |
|------|----------|----------|------|
| PWM | 定时器 | PWM 生成、定时器配置 | ⭐ |
| IMU | SPI, I2C | 传感器驱动、姿态解算、Mahony 滤波 | ⭐⭐⭐⭐⭐ |
| RemoteControl | USART, DMA | DMA+IDLE 接收、位操作解码 | ⭐⭐⭐ |
| RM_OLED | SPI | 显存管理、图形驱动、字库 | ⭐⭐ |
| SDCard | SDIO, FatFS | 文件系统移植、磁盘 I/O | ⭐⭐⭐⭐ |
| USB | USB OTG, CDC | USB 协议栈、虚拟串口 | ⭐⭐⭐⭐ |
| UWB | CAN | CAN 协议、过滤器配置、多帧拼包 | ⭐⭐⭐ |

### 9.2 学习路线建议

```
第一阶段：基础外设
  1. PWM  ->  理解定时器和 GPIO 的基本使用
  2. RM_OLED -> 掌握 SPI 通信和图形驱动原理

第二阶段：数据通信
  3. RemoteControl -> 掌握 UART + DMA 高效接收
  4. UWB -> 了解 CAN 总线通信
  5. USB -> 理解 USB CDC 虚拟串口

第三阶段：传感器与算法
  6. IMU -> 传感器融合与姿态解算（最复杂）

第四阶段：存储系统
  7. SDCard -> 文件系统移植与应用

第五阶段：综合项目
  结合多个例程，例如：
  - IMU + PWM -> 四轴飞行器/平衡车
  - RemoteControl + PWM -> 遥控机器人
  - UWB + OLED -> 定位显示终端
  - USB + 所有 -> 上位机通信调试
```

### 9.3 调试技巧

1. **串口调试**：每个例程都通过 USART6 输出调试信息，连接串口助手观察输出
2. **LED 指示**：通过绿色 LED 闪烁状态判断程序是否正常运行
3. **HAL_Delay**：基于 SysTick 的毫秒级延时，需注意不能在中断中长时间调用
4. **错误处理**：大部分例程都有 `_Error_Handler()` 错误处理函数，出现异常会停在 while(1) 循环中

### 9.4 深入学习建议

1. 阅读 STM32F4xx 参考手册和 HAL 库源码
2. 学习使用 STM32CubeMX 进行外设配置
3. 研究 FreeRTOS 实时操作系统在多任务场景下的应用
4. 学习 PID 控制算法，结合 PWM 和 IMU 实现电机闭环控制
5. 深入学习 CANOpen 或 ROS，用于更复杂的机器人系统
