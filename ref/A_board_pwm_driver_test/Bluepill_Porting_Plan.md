# Bluepill (STM32F103C8T6) 移植计划

> 从 STM32F427 移植到 STM32F103C8T6（Bluepill）的完整执行清单。
> 策略层 + 功能层代码零修改，只动平台绑定层。
>
> **引脚分配基于 `BluePill_Car` 小车主板实际布局。**

---

## 总览

| 项目 | 值 |
|---|---|
| 源芯片 | STM32F427 (Cortex-M4, 180MHz) |
| 目标芯片 | STM32F103C8T6 (Cortex-M3, 72MHz) |
| Flash | 64KB (当前固件 ~11.5KB，预期 ~30KB) |
| RAM | 20KB (当前 ~1.7KB，预期 ~8KB) |
| 移植方式 | 复用 CubeMX 生成工程 `BluePill_Car` + 迁入驱动代码 |
| 预计时间 | **2~3 小时** |

---

## 第一阶段：CubeMX 工程 — 已完成

`BluePill_Car.ioc` 已配置好以下外设：

### 引脚分配

| 功能 | Bluepill 引脚 | 外设 | 配置 |
|---|---|---|---|
| 舵机 PWM | **PB1** | TIM3_CH4 | PWM Generation CH4 |
| 电机A PWM | **PA8** | TIM1_CH1 | PWM Generation CH1 |
| 电机B PWM | **PA9** | TIM1_CH2 | PWM Generation CH2 |
| 电机A IN1 | **PB13** | GPIO_Output | 推挽输出 |
| 电机A IN2 | **PB12** | GPIO_Output | 推挽输出 |
| 电机B IN1 | **PB14** | GPIO_Output | 推挽输出 |
| 电机B IN2 | **PB15** | GPIO_Output | 推挽输出 |
| STBY | **5V** | 硬接高电平 | 不占 GPIO |
| 编码器 CH1 | **PA0** | TIM2_CH1 | Encoder Mode TI1+TI2 |
| 编码器 CH2 | **PA1** | TIM2_CH2 | Encoder Mode TI1+TI2 |
| 调试串口 TX | **PA2** | USART2_TX | 异步 9600bps |
| 调试串口 RX | **PA3** | USART2_RX | 异步 9600bps |
| 软件 I2C SCL | **PB10** | GPIO_Output_OD | 软件 I2C |
| 软件 I2C SDA | **PB11** | GPIO_Output_OD | 软件 I2C |
| I2C 时基 | **TIM4** | 基本定时器 | PSC=0, ARR=719 (10μs) |
| 寻迹传感器 | **PB3~PB7** | GPIO_Input | 5 路输入，与 I2C 无冲突 |
| SWDIO | PA13 | SYS_JTMS-SWDIO | Serial Wire |
| SWCLK | PA14 | SYS_JTCK-SWCLK | Serial Wire |

### 时钟树

- HSE = 8MHz → PLL (x9) → SYSCLK = **72MHz**
- APB1 = 36MHz (Tim = 72MHz)
- APB2 = 72MHz

### 外设参数

| 外设 | 参数 |
|---|---|
| TIM1 | PSC=0, Period=65535, CH1 + CH2 PWM（电机A/B） |
| TIM2 | EncoderMode=TI12, IC1Filter=15, IC2Filter=15, Period=65535 (16bit) |
| TIM3 | PSC=0, Period=65535, CH4 PWM（舵机） |
| TIM4 | PSC=0, Period=719, 基本定时器（软件 I2C 时基，10μs tick） |
| USART2 | 9600bps, 8N1, 中断使能 |
| 软件 I2C | PB10(SCL), PB11(SDA), TIM4 时基 |

### 已使能的中断

- USART2_IRQn (抢占 0, 子 0)

### CubeMX 待改项

1. **TIM4** → 勾选 `Activated` + NVIC 使能 TIM4 中断
2. **USART2** → 波特率 9600 → 建议改 **115200**
3. **TIM1** → PSC 和 Period 需调整（见 2.7）
4. **TIM3** → PSC 和 Period 需调整（见 2.7）

---

## 第二阶段：AI Agent 移植（代码修改）

### 2.1 替换 HAL 库 — 已完成

`BluePill_Car` 工程已包含 `STM32F1xx_HAL_Driver`，无需替换。

### 2.2 更新 CMakeLists.txt

从 `A_board_pwm_driver_test\CMakeLists.txt` 拷贝驱动配置到 `BluePill_Car\CMakeLists.txt`：

```cmake
# 在原 CMakeLists.txt 基础上添加

# 启用 C++ 支持
enable_language(CXX)

# 添加驱动源文件
file(GLOB USER_SOURCES
    Core/Src/driver/*.c
    Core/Src/driver/*.cpp
    Core/Src/common/*.c
    Core/Src/common/*.cpp
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# 添加 include 路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/driver
    Core/Inc/common
)

# 链接 C++ 标准库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    stdc++
)
```

### 2.3 迁入驱动代码（策略层 + 功能层 — 零修改）

从 `A_board_pwm_driver_test` 拷贝以下文件到 `BluePill_Car` 对应目录：

```
Core/Src/driver/     → 全部 .c / .cpp
Core/Inc/driver/     → 全部 .h
Core/Inc/common/     → ringbuf.h, tool.h 等
Core/Src/common/     → ringbuf.c 等
```

### 2.4 重写平台 ops（5 个文件）

所有平台 ops 文件只需改一行 include：

| 文件 | 旧 include | 新 include |
|---|---|---|
| `pwm_platform_ops.c` | `"stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `encoder_platform_ops.c` | `"stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `uart_platform_ops.c` | `"stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `usergpio_platform.c` | `"stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |
| `useri2c_ops.h` | `"stm32f4xx_hal.h"` | `"stm32f1xx_hal.h"` |

HAL API 名全兼容，无需其他改动。

### 2.5 迁入 main.c 用户代码段

#### 2.5.1 UART 句柄

```c
UART_Handle uart_debug = {
    .huart = &huart2,           // USART2
    .ops   = &uart_platform_ops_stm32,
};
```

#### 2.5.2 软件 I2C 上下文

```c
// PB10(SCL), PB11(SDA), TIM4 时基
I2C_SoftwareContext g_software_i2c = {
    .scl_pin = { .hgpio_port = GPIOB, .gpio_pin = 10,
                 .ops = &usergpio_platform_ops_stm32 },
    .sda_pin = { .hgpio_port = GPIOB, .gpio_pin = 11,
                 .ops = &usergpio_platform_ops_stm32 },
    .timer_handle = &htim4,
    .macro_state = I2C_MACRO_IDLE,
    .micro_state = I2C_MICRO_IDLE,
};
```

#### 2.5.3 编码器（不变）

```c
Encoder_Handle henc;
henc.htim  = &htim2;
henc.ops   = encoder_platform_get_ops();
henc.ppr   = 1320;
henc.position = 0;
```

#### 2.5.4 PWM 实例 + 电机初始化

需要在 `pwm.c` 末尾（或新建 `pwm_instance.c`）新增实例：

```c
// Bluepill 的 PWM 实例
PWM_Handle pwm_tim1_ch1 = {
    .htim=&htim1, .ops=&pwm_platform_ops_stm32,
    .Channel=TIM_CHANNEL_1, .Ch_State=PWM_Ch_State_OK,
};
PWM_Handle pwm_tim1_ch2 = {
    .htim=&htim1, .ops=&pwm_platform_ops_stm32,
    .Channel=TIM_CHANNEL_2, .Ch_State=PWM_Ch_State_OK,
};
PWM_Handle pwm_tim3_ch4 = {
    .htim=&htim3, .ops=&pwm_platform_ops_stm32,
    .Channel=TIM_CHANNEL_4, .Ch_State=PWM_Ch_State_OK,
};
```

main.c 中电机初始化：

```c
UserGPIO_Handle motor_a_in1 = { GPIOB, 13, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_a_in2 = { GPIOB, 12, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in1 = { GPIOB, 14, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_b_in2 = { GPIOB, 15, &usergpio_platform_ops_stm32 };

motor_bridge_init(0, &pwm_tim1_ch1, &motor_a_in1, &motor_a_in2, NULL, 200.0f, 33.1f);
motor_bridge_init(1, &pwm_tim1_ch2, &motor_b_in1, &motor_b_in2, NULL, 200.0f, 33.1f);
```

> **注意**：STBY 硬接 5V，`stby_gpio` 传 `NULL`。需确认 `motor_protocol.cpp` 中 `stop()` 对 NULL 指针的处理。

#### 2.5.5 MX_ 调用顺序

```c
// Bluepill main()
MX_GPIO_Init();
MX_TIM1_Init();      // 电机A PWM (CH1) + 电机B PWM (CH2)
MX_TIM2_Init();      // 编码器
MX_TIM3_Init();      // 舵机 PWM (CH4)
MX_TIM4_Init();      // 软件 I2C 时基
MX_USART2_UART_Init(); // 调试串口
```

### 2.6 更新中断处理（stm32f1xx_it.c）

```c
/* USER CODE BEGIN Includes */
#include "useri2c_ops.h"
/* USER CODE END Includes */

/* USER CODE BEGIN EV */
extern I2C_SoftwareContext g_software_i2c;
/* USER CODE END EV */

// 添加 TIM4 中断（软件 I2C 时基）
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        i2c_software_timer_isr(&g_software_i2c);
    }
    HAL_TIM_IRQHandler(&htim4);
}
```

### 2.7 硬件参数对照表

| 参数 | F427 原值 | Bluepill 当前值 | 需改为 |
|---|---|---|---|
| 电机A/B PWM 频率 | 20kHz (TIM5, PSC=89, ARR=100) | ~1.1kHz (TIM1, PSC=0, ARR=65535) | PSC=0, ARR=3600 (20kHz) |
| 舵机 PWM 频率 | 50Hz (TIM1, PSC=71, ARR=19999) | ~1.1kHz (TIM3, PSC=0, ARR=65535) | PSC=71, ARR=19999 (50Hz) |
| 编码器 Period | 0xFFFFFFFF (32bit) | 65535 (16bit) | 保持 0xFFFF，软件处理溢出 |
| 软件 I2C 时基 | TIM6, PSC=0, ARR=899 (10μs) | TIM4, PSC=0, ARR=719 (10μs) | ✅ 已对 |
| USART 波特率 | 115200 | 9600 | 建议改 115200 |

### 2.8 需注意的 F103 差异

| 差异 | 说明 | 影响 |
|---|---|---|
| **TIM2 是 16bit** | Period 最大 65535 | 编码器会频繁溢出，需软件处理 |
| **无 TIM5/TIM6** | F103 无这两个定时器 | 电机 PWM 用 TIM1，I2C 时基用 TIM4 |
| **USART2 引脚** | PA2=TX, PA3=RX | 改 `huart2` 即可 |
| **9600bps** | CubeMX 默认值 | 建议改 115200 |
| **C++ 支持** | 原 CMakeLists 只有 `C ASM` | 必须加 CXX 和 stdc++ |

---

## 第三阶段：验证

### 3.1 编译验证

- [ ] 首次构建，修复编译错误
- [ ] 常见问题：include 路径、TIM2 16bit、C++ 未启用

### 3.2 功能验证

| 序号 | 测试项 | 验证方法 |
|---|---|---|
| 1 | 串口通信 | 上电打印 `"UART2 Ready!\r\n"` |
| 2 | CMD_PING | 发 `0xAA 0xF0 0xFF 0xFF`，收响应 |
| 3 | 编码器 | 手动旋转，打印计数值变化 |
| 4 | 舵机 | 发 `CMD_SERVO_SET_ANGLE` |
| 5 | 电机 A/B | 发 `CMD_MOTOR_SET_RPM` 验证正反转 |
| 6 | IMU | 发 `CMD_IMU_GET_DATA` 读欧拉角 |
| 7 | I2C | 逻辑分析仪抓 PB6/PB7 时序 |

### 3.3 清理

- [ ] 更新 CMakeLists.txt
- [ ] 更新文档

---

## 风险与注意事项

| 风险 | 说明 | 应对 |
|---|---|---|
| **TIM2 16bit** | Period 只有 65535，编码器会频繁溢出 | 软件中处理溢出 |
| **PB13 共用** | AIN1 和 BIN2 都接 PB13，两个电机不能独立刹车 | IN1/IN2=11 是刹车，A 刹车时 B 也刹车，正反转控制不受影响 |
| **PB15 未配** | BIN2=PB15，但 CubeMX 中 PB15 未配置 | 需要在 CubeMX 中把 PB15 设为 Output Push Pull |
| **STBY=NULL** | STBY 硬接 5V，bridge 传 NULL | 确认 `motor_protocol.cpp` `stop()` 对 NULL 的处理 |
| **Flash 64KB** | 预期 ~30KB | 够用，加 `-fno-exceptions -fno-rtti` |
| **SWD 锁死** | 误配 GPIO 为 SWD 引脚 | CubeMX 已开 `Serial Wire` |
| **USART2 9600bps** | 命令响应慢 | 建议改 115200 |

---

## 待决策事项

1. **STBY=NULL**：是否需要改 `motor_protocol.cpp` 支持 NULL stby？
2. **USART2 波特率**：保留 9600 还是改 115200？
3. **PB10/PB11 外部上拉**：软件 I2C 的 SCL/SDA 需要外部 4.7kΩ 上拉电阻，板子上是否有？

---

## 建议执行顺序

```
1. CubeMX: 配 TIM4 + 调参数 + 重新生成         ───── 15min
2. 更新 CMakeLists.txt (加 CXX + 驱动)          ───── 10min
3. 拷入驱动代码                                  ───── 10min
4. 改 5 个平台 ops (只改 include)               ───── 5min
5. 迁入 main.c + 改引脚/句柄                     ───── 20min
6. 编译 + 修错                                   ───── 30~60min
7. 上板验证                                      ───── 30min
                                                ─────────
                                      总计:     ~2~3h
```
