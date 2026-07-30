# Bluepill 移植踩坑记录

> 从 STM32F427 移植到 STM32F103C8T6（Bluepill）过程中遇到的所有问题及解决方案。

---

## 1. TIM3 共用 CNT 冲突

### 问题

TIM3 同时配置了编码器模式（CH1/CH2）和 PWM 输出（CH4），但两者共用同一个 CNT 计数器。编码器转动时 CNT 不断变化，导致 PWM 周期不稳定；同时舵机需要的 50Hz（ARR=19999）和编码器防溢出（ARR=65535）无法兼得。

### 解决

将舵机 PWM 从 TIM3_CH4（PB1）改为 **TIM4_CH3（PB8）**，需飞线 1 根。最终分配：

| 定时器 | 用途 |
|---|---|
| TIM1 | 电机A/B PWM（CH1/CH2） |
| TIM2 | 编码器1 |
| TIM3 | 编码器2 |
| TIM4 | 舵机 PWM（CH3） |

---

## 2. 硬件 I2C vs 软件 I2C 方案切换

### 问题

原始工程使用软件 I2C（GPIO 比特流 + 定时器中断），需要 TIM4 做时基。但 F103C8T6 只有 4 个定时器（TIM1~4），不够分。

### 解决

改用 **硬件 I2C2**（PB10/PB11），100kHz 标准模式。不再需要定时器做 I2C 时基，TIM4 专供舵机 PWM。
- `oled_driver.cpp` 从 `i2c_write_reg_async` 改为 `HAL_I2C_Mem_Write`
- IMU 驱动 `imu_bridge_init` 直接传入 `&hi2c2`
- 删除 `useri2c_ops.h` 中 TIM4 中断相关代码

---

## 3. TIM1 配成 OC 模式（Output Compare）不是 PWM

### 问题

CubeMX 中 TIM1 的 CH1/CH2 默认配置为 `Output Compare`，生成的代码使用 `HAL_TIM_OC_Init` + `TIM_OCMODE_TIMING`，不会输出 PWM 波形，电机无法驱动。

### 解决

CubeMX 中 TIM1 → Channel1/Channel2 改为 `PWM Generation CH1` / `PWM Generation CH2`，重新生成后代码使用 `HAL_TIM_PWM_Init` + `TIM_OCMODE_PWM1`。

---

## 4. F103 定时器全是 16bit

### 问题

F427 的 TIM2/TIM5 是 32 位定时器（Period 可达 0xFFFFFFFF），但 F103 所有定时器都是 16 位，最大 Period=65535。`pwm_platform_ops.c` 中判断 `htim->Instance == TIM2 || TIM5` 做 32 位写入，F103 没有 TIM5 导致编译报错。

### 解决

- `pwm_platform_ops.c` 中去掉 TIM5/TIM6/TIM7/TIM12/TIM13/TIM14 引用
- `set_ccr` 函数统一用 `(uint16_t)ccr` 强转
- 编码器 Period 保持 65535，软件处理溢出

---

## 5. pwm_platform_ops.h 遗留 stm32f4xx_hal.h 引用

### 问题

`pwm_platform_ops.h` 中 `#include "stm32f4xx_hal.h"` 被遗漏，编译时报 `fatal error: stm32f4xx_hal.h: No such file or directory`。

### 解决

全局搜索所有 `.h`/`.c` 文件中的 `stm32f4xx` 引用，改为 `stm32f1xx`。涉及文件：
- `pwm_platform_ops.h`
- `useri2c_ops.h`
- `usergpio_platform.c`
- `imu_bridge.cpp`
- `imu_uart_handler.cpp`

---

## 6. pwm.c 末尾残留 F427 实例定义

### 问题

`pwm.c` 末尾定义了 `PWM_Handle pwm_tim5_ch4 = { .htim=&htim5, ... }`，F103 没有 `htim5`，编译报错。

### 解决

删除 `pwm.c` 末尾的旧实例定义。已新建 `pwm_instance.c` 存放 Bluepill 的 3 个 PWM 实例（`pwm_tim1_ch1`, `pwm_tim1_ch2`, `pwm_tim4_ch3`），同时更新 `pwm.h` 中的 `extern` 声明。

---

## 7. uart_cmd_parser.c 中 huart7 → huart2

### 问题

`uart_cmd_parser.c` 中 `UART_BASE_HANDLE` 定义为 `huart7`（F427 的 UART7），F103 没有 UART7，只有 USART2（`huart2`）。

### 解决

```c
#define UART_BASE_HANDLE    huart2  // 原为 huart7
```

---

## 8. servo_bridge_init 接口不匹配

### 问题

原始 `main.c` 中调用方式为 F427 旧接口：
```c
servo_bridge_init(0, &pwm_tim4_ch3, 500, 2500, 0, 180);
```
但 `servo_bridge.h` 的接口签名是：
```c
void servo_bridge_init(uint8_t id, ServoProtocol_t protocol, void *handle);
```

### 解决

改为：
```c
servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim4_ch3);
```

---

## 9. mingw64 编译器不识别 ARM 汇编

### 问题

直接 `cmake -B build` 时使用了系统 PATH 中的 `mingw64/bin/gcc.exe`（x86_64 架构），无法解析启动文件中的 ARM 汇编指令（`.syntax`, `cpsid`, `wfi` 等），大量汇编错误。

### 解决

工程已有 `cmake/gcc-arm-none-eabi.cmake` 工具链文件，需通过 preset 或手动指定工具链配置：
```bash
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
```
工具链路径为 `STM32CubeCLT` 自带的 `arm-none-eabi-gcc`。

---

## 10. syscalls.c 中 sys/times.h 在 mingw 下不存在

### 问题

CubeMX 生成的 `syscalls.c` 中 `#include <sys/times.h>` 在 ARM 交叉编译器下正常，但 cmake 重配置时若检测到 mingw 会报错。实际使用 ARM 工具链编译时该文件也会产生 `struct tms` 的 warning。

### 解决

注释掉 `#include <sys/times.h>` 行。CubeMX 重新生成后需要重新注释。

---

## 11. F103 无 TIM5/TIM6/TIM7 等定时器

### 问题

`pwm_platform_ops.c` 中 `get_clk_freq` 函数判断 APB1 定时器时列出了 F427 才有的 TIM5/6/7/12/13/14，F103 编译时报 `undeclared` 错误。

### 解决

APB1 定时器列表改为 F103 实际有的：
```c
if (htim->Instance == TIM2 || htim->Instance == TIM3 ||
    htim->Instance == TIM4)
```

---

## 12. CubeMX 重新生成后 TIM4 中断丢失

### 问题

CubeMX 重新生成 `.ioc` 后，`tim.c` 中 TIM4 的 `HAL_TIM_Base_MspInit` 不再使能中断（`HAL_NVIC_EnableIRQ(TIM4_IRQn)`），`stm32f1xx_it.c` 中也没有 `TIM4_IRQHandler` 函数。但改用硬件 I2C2 后 TIM4 中断不再需要，所以这不是问题。

---

## 总结

| 类别 | 数量 | 典型问题 |
|---|---|---|
| **引脚/定时器冲突** | 2 | TIM3 共用 CNT、定时器不够分 |
| **HAL 库差异** | 3 | F4→F1 的 include、定时器 16bit、OC vs PWM |
| **接口不匹配** | 2 | servo_bridge 参数、F427 实例残留 |
| **工具链** | 2 | ARM 汇编 vs mingw、sys/times.h |
| **遗漏修改** | 3 | pwm_platform_ops.h include、uart huart7、TIM5 引用 |

**核心经验：移植的关键是全面扫描所有与芯片型号绑定的代码——平台 ops、include、定时器列表、实例定义、CubeMX 生成文件中的引用。**
