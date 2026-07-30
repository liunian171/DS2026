# A_Board-Examples 源码总结

> 来源: `ref/A_Board-Examples/` 官方样例(Keil MDK工程)
> 芯片: STM32F427IIH6, HAL库, 168MHz
> 日期: 2026-07-29
> 说明: 仅供B板开发参考，实际以用户明确信息为准

---

## 一、保留的样例(4个)

| 样例 | B板相关度 | 关键文件 |
|------|-----------|----------|
| **PWM** | ⭐⭐⭐⭐⭐ | `Src/main.c` — PWM_SetDuty函数 |
| **RM_OLED** | ⭐⭐⭐ | `BSP/oled.c` — SSD1306 SPI驱动 |
| **Imu** | ⭐⭐⭐⭐ | `MDK-ARM/bsp/bsp_imu.c` — MPU6500驱动 |
| **RemoteControl** | ⭐⭐⭐⭐ | `MDK-ARM/bsp/bsp_uart.c` — DMA+IDLE串口 |

> 已删除: SDCard、USB、UWB(与H题无关)

---

## 二、PWM样例 — STEP脉冲/电机控制

### 2.1 PWM配置

```c
#define PWM_FREQUENCE   50      // 频率50Hz
#define PWM_RESOLUTION  10000   // 分辨率10000
```

**4组定时器，16路PWM**:
| 定时器 | 通道 | 引脚 | 总线 | 时钟 |
|--------|------|------|------|------|
| TIM2 | CH1-4 | PA0~PA3 | APB1 | 84MHz |
| TIM4 | CH1-4 | PD12~PD15 | APB1 | 84MHz |
| TIM5 | CH1-4 | PH10~PH12, PI0 | APB1 | 84MHz |
| TIM8 | CH1-4 | PI2, PI5~PI7 | APB2 | 168MHz |

### 2.2 核心函数

```c
void PWM_SetDuty(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty) {
    switch(tim_channel) {
        case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_3: tim->Instance->CCR3 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_4: tim->Instance->CCR4 = (PWM_RESOLUTION * duty) - 1; break;
    }
}
```

**B板使用**: 用一路TIM PWM输出STEP脉冲给TMC2209，改变频率控制步进速度。

---

## 三、RM_OLED样例 — SSD1306驱动

### 3.1 硬件配置

| 参数 | 值 |
|------|-----|
| 控制器 | SSD1306 |
| 分辨率 | 128×64 |
| 接口 | SPI1 |
| 显存 | `uint8_t OLED_GRAM[128][8]` (128列×8页) |

### 3.2 核心函数

| 函数 | 功能 |
|------|------|
| `oled_init()` | 初始化SSD1306 |
| `oled_write_byte(uint8_t dat, uint8_t cmd)` | SPI写字节(cmd=0命令, cmd=1数据) |
| `oled_refresh_gram()` | 刷新显存到屏幕 |
| `oled_drawpoint(uint8_t x, uint8_t y, uint8_t pen)` | 画点 |
| `oled_showchar(uint8_t row, uint8_t col, uint8_t chr)` | 显示字符 |
| `oled_printf(uint8_t row, uint8_t col, const char *fmt, ...)` | 格式化输出 |
| `oled_clear(uint8_t pen)` | 清屏 |

### 3.3 B板使用注意

- RM A板OLED是**SPI1**接口
- A板(STM32F103ZET6)的OLED是**I2C**接口
- **B板可直接使用此驱动代码**(若用板载OLED调试)
- A板需另写I2C版SSD1306驱动(可参考BluePill_Car的SSD1315 I2C驱动框架)

---

## 四、Imu样例 — MPU6500驱动

### 4.1 硬件配置

| 参数 | 值 |
|------|-----|
| 器件 | MPU6500(6轴陀螺仪+加速度计) + IST8310(磁力计) |
| 接口 | SPI5 |
| CS(NSS) | PF6 |
| IST8310 | 经MPU6500辅助I2C Master连接 |

### 4.2 数据结构

```c
typedef struct {
    float wx, wy, wz;      // 角速度 (rad/s)
    float vx, vy, vz;      // 加速度 (归一化)
    float rol, pit, yaw;    // Roll/Pitch/Yaw (度)
} imu_t;
```

### 4.3 核心功能

- SPI5读取MPU6500寄存器(陀螺仪+加速度计原始数据)
- MPU6500辅助I2C读取IST8310(磁力计)
- **Mahony互补滤波**解算姿态角(Roll/Pitch/Yaw)
- 调试输出: `printf`通过USART6输出姿态数据

### 4.4 B板使用注意

- 用户确认RM A板**不挂在管子上**，IMU不用于检测摆杆倾角
- 但板载MPU6500可用于检测**小车整体姿态**(如转弯离心加速度)，辅助前馈
- 若不需要IMU，可不用此样例

---

## 五、RemoteControl样例 — DMA+IDLE串口接收

### 5.1 硬件配置

| 参数 | 值 |
|------|-----|
| 接口 | USART1 + DMA |
| 协议 | DBUS(大疆遥控器, 100kbps 8E1) |
| 缓冲区 | `uint8_t dbus_buf[DBUS_BUFLEN]` |

### 5.2 核心技术: DMA + IDLE中断

**初始化**:
```c
void dbus_uart_init(void) {
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);           // 使能IDLE中断
    uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN); // 启动DMA接收
}
```

**IDLE中断回调**(一帧数据接收完成时触发):
```c
static void uart_rx_idle_callback(UART_HandleTypeDef* huart) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);                          // 清除IDLE标志
    __HAL_DMA_DISABLE(huart->hdmarx);                          // 停止DMA
    // 处理接收到的数据...
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);        // 重置DMA计数
    __HAL_DMA_ENABLE(huart->hdmarx);                           // 重启DMA
}
```

**中断处理函数**:
```c
void uart_receive_handler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
        uart_rx_idle_callback(huart);
    }
}
```

### 5.3 B板使用方案

此DMA+IDLE技术**直接用于B板接收两路UART数据**:

| 数据源 | UART | 用途 |
|--------|------|------|
| K230D球位置 | USARTx | DMA+IDLE接收球坐标帧 |
| A板运动状态 | USARTy | DMA+IDLE接收速度/加速度帧 |

**优势**: 中断只写ringbuf+重使能接收(<2μs)，不阻塞控制循环。

---

## 六、工程结构(Keil MDK)

每个样例工程结构一致:
```
ExampleName/
├── Drivers/              # HAL库+CMSIS (CubeMX生成,不修改)
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── Inc/                  # 头文件 (CubeMX生成)
├── Src/                  # 源文件 (CubeMX生成)
│   ├── main.c            # 主程序(含用户代码)
│   ├── stm32f4xx_it.c    # 中断处理
│   ├── tim.c/gpio.c/...  # 外设初始化
│   └── system_stm32f4xx.c
├── MDK-ARM/              # Keil工程
│   ├── bsp/              # 板级支持包(用户代码,关键!)
│   │   ├── bsp_imu.c     # IMU驱动
│   │   └── bsp_uart.c    # UART驱动
│   ├── Project.uvprojx    # Keil工程文件
│   └── startup_stm32f427xx.s
├── PWM.ioc               # CubeMX配置文件
└── keilkill.bat
```

> RM_OLED的BSP代码在`BSP/`目录(非`MDK-ARM/bsp/`)

---

## 七、B板开发参考路径

| B板功能 | 参考样例 | 参考文件 | 适配说明 |
|---------|----------|----------|----------|
| STEP脉冲 | PWM | `Src/main.c` PWM_SetDuty | 改PWM频率控制步进速度 |
| 球位置UART接收 | RemoteControl | `MDK-ARM/bsp/bsp_uart.c` | DMA+IDLE，改协议解析 |
| A板状态UART接收 | RemoteControl | `MDK-ARM/bsp/bsp_uart.c` | 同上，第二路UART |
| 调试OLED(可选) | RM_OLED | `BSP/oled.c` | B板板载OLED直接用 |
| IMU(可选) | Imu | `MDK-ARM/bsp/bsp_imu.c` | 检测小车姿态(非摆杆) |

> **注意**: 这些样例是Keil MDK工程，用户开发环境是CubeIDE for VSCode + CubeMX。B板工程需用CubeMX重新生成F427IIH6的初始化代码，然后移植BSP层代码。
