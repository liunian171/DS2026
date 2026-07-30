# RM A型板 资源提取与B板开发指南

> 来源: A_Board-Examples样例源码 + 官方手册
> 提取日期: 2026-07-29
> 用途: B板(摆杆控制主控)开发参考

---

## 一、芯片确认（重要更正）

**RM A型板芯片为 STM32F427IIHx**（非F407，之前文档有误，已更正）

| 参数 | 值 |
|------|-----|
| 芯片 | STM32F427IIHx |
| 内核 | Cortex-M4F (带单精度FPU) |
| 主频 | 168MHz |
| Flash | 1MB (F427) |
| SRAM | 192KB |
| 封装 | UFBGA176 |
| HSE晶振 | 8MHz |
| HAL库 | STM32F4xx_HAL_Driver |
| 开发环境 | Keil MDK (uvprojx) |
| 配置工具 | STM32CubeMX (.ioc) |

### 时钟配置
| 参数 | 值 |
|------|-----|
| HSE | 8MHz |
| PLLM | 6 |
| PLLN | 168 |
| PLLP | 2 |
| SYSCLK | 168MHz |
| AHB (HCLK) | 168MHz |
| APB1 (PCLK1) | 42MHz |
| APB2 (PCLK2) | 84MHz |

---

## 二、板载硬件资源

### 2.1 LED
| LED | 引脚 | 电平 |
|-----|------|------|
| LED_RED | PE11 | 低电平点亮 |
| LED_GREEN | PF14 | 低电平点亮 |

### 2.2 调试串口
- USART6 (用于printf调试输出)
- `HAL_UART_Transmit(&huart6, buf, len, timeout)`

### 2.3 板载IMU (MPU6500 + IST8310)
| 器件 | 接口 | 引脚 |
|------|------|------|
| MPU6500 (6轴陀螺仪+加速度计) | SPI5 | NSS: PF6 |
| IST8310 (磁力计) | MPU6500辅助I2C Master | - |

> **对本题价值**: MPU6500可检测摆杆倾角(Pitch)，作为球位置控制的辅助反馈。需确认赛题是否允许（Q40-Q41提到"参照命题要求"，摆杆管壁不允许加装传感器，但IMU装在小车/支撑台上不在管壁上，可能合规）。

### 2.4 板载OLED (SSD1306)
| 参数 | 值 |
|------|-----|
| 控制器 | SSD1306 |
| 分辨率 | 128×64 |
| 尺寸 | 0.96寸 |
| 接口 | SPI1 |

> **重要**: RM A板板载OLED也是SSD1306！但B板不需要计时显示(那是A板的任务)。B板可用板载OLED做调试显示。

---

## 三、样例工程与本题目相关度

### 3.1 相关度排序

| 样例 | B板可用性 | 相关功能 |
|------|-----------|----------|
| **PWM** | ⭐⭐⭐⭐⭐ | STEP脉冲驱动TMC2209步进电机 |
| **RM_OLED** | ⭐⭐⭐ | 调试显示(SSD1306 SPI驱动可直接复用) |
| **IMU** | ⭐⭐⭐⭐ | MPU6500检测摆杆倾角(若合规) |
| **RemoteControl** | ⭐⭐⭐⭐ | DMA+IDLE接收K230D/A板UART数据 |
| USB | ⭐⭐ | 调试串口(可选) |
| SDCard | ⭐ | 本题不需要 |
| UWB | ❌ | 本题不需要 |

### 3.2 PWM样例 → 步进电机STEP脉冲

RM A板PWM样例配置了4组定时器共16路PWM：
- TIM2 (CH1~CH4): PA0~PA3 (APB1, 84MHz)
- TIM4 (CH1~CH4): PD12~PD15 (APB1, 84MHz)
- TIM5 (CH1~CH4): PH10~PH12, PI0 (APB1, 84MHz)
- TIM8 (CH1~CH4): PI2, PI5~PI7 (APB2, 168MHz)

**PWM核心代码**:
```c
#define PWM_FREQUENCE   50      // 频率
#define PWM_RESOLUTION  10000   // 分辨率

void PWM_SetDuty(TIM_HandleTypeDef *tim, uint32_t tim_channel, float duty) {
    switch(tim_channel) {
        case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION * duty) - 1; break;
        case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION * duty) - 1; break;
        // ...
    }
}
```

> **B板STEP脉冲方案**: 用一路TIM PWM输出STEP信号，通过改变PWM频率控制步进速度，通过计数脉冲数控制位置。DIR/EN用GPIO控制。

### 3.3 RM_OLED样例 → SSD1306驱动

**RM A板OLED与我们的SSD1306完全相同！** 驱动代码可直接复用。

关键驱动函数:
| 函数 | 功能 |
|------|------|
| `oled_init()` | 初始化 |
| `oled_write_byte(dat, cmd)` | SPI写字节(cmd=0命令, cmd=1数据) |
| `oled_refresh_gram()` | 刷新显存到屏幕 |
| `oled_drawpoint(x, y, pen)` | 画点 |
| `oled_showchar(row, col, chr)` | 显示字符 |
| `oled_printf(row, col, fmt, ...)` | 格式化输出 |
| `oled_clear(pen)` | 清屏 |

显存: `uint8_t OLED_GRAM[128][8]` (128列×8页=128×64像素)

接口: SPI1, DC引脚控制命令/数据

### 3.4 IMU样例 → MPU6500姿态检测

**MPU6500数据结构**:
```c
typedef struct {
    float wx, wy, wz;    // 角速度 (rad/s)
    float vx, vy, vz;    // 加速度 (归一化)
    float rol, pit, yaw;  // Roll/Pitch/Yaw (度)
} imu_t;
```

**Mahony滤波解算姿态**: 陀螺仪积分 + 加速度计/磁力计修正

> **B板可用IMU的Pitch角检测摆杆倾角**，作为步进电机角度的开环参考或闭环反馈。

### 3.5 RemoteControl样例 → DMA+IDLE UART接收

**DMA + IDLE中断接收** (适合K230D和A板通信):
```c
// 初始化: 使能IDLE中断 + 启动DMA接收
__HAL_UART_ENABLE_IT(&huart, UART_IT_IDLE);
HAL_UART_Receive_DMA(&huart, buf, len);

// IDLE中断回调: 一帧接收完成
static void uart_rx_idle_callback(UART_HandleTypeDef* huart) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    // 处理数据...
    // 重置DMA继续接收
}
```

> **B板用此方式接收K230D球位置数据和A板运动状态数据**。

---

## 四、B板引脚分配规划（基于RM A板）

### 4.1 已用板载资源

| 功能 | 外设 | 引脚 | 来源 |
|------|------|------|------|
| LED_RED | GPIO | PE11 | 板载 |
| LED_GREEN | GPIO | PF14 | 板载 |
| 调试串口 | USART6 | - | 板载 |
| MPU6500 | SPI5 | NSS:PF6 | 板载 |
| IST8310 | I2C(经MPU6500) | - | 板载 |
| 板载OLED | SPI1 | - | 板载(可选使用) |

### 4.2 B板新增外设分配

| 功能 | 外设 | 建议引脚 | 说明 |
|------|------|----------|------|
| STEP脉冲(TMC2209) | TIMx_CHx | PA0(TIM2_CH1)或PD12(TIM4_CH1) | PWM输出脉冲 |
| DIR方向(TMC2209) | GPIO | 待定 | 高低电平控制方向 |
| EN使能(TMC2209) | GPIO | 待定 | 低电平有效 |
| K230D通信 | UART | USART1或USART3 | DMA+IDLE接收球位置 |
| A板通信 | UART | 另一路USART | DMA+IDLE接收运动状态 |
| 状态LED | GPIO | PE11/PF14(板载) | 用板载LED即可 |

> **注意**: 需避开MPU6500(SPI5/PF6)和板载OLED(SPI1)已占用的引脚。具体可用引脚需查RM A板原理图确认。

---

## 五、工程结构

### 5.1 样例工程目录结构
```
ProjectName/
├── Drivers/                  # HAL库(不修改)
│   ├── CMSIS/
│   └── STM32F4xx_HAL_Driver/
├── Inc/                      # 头文件(CubeMX生成)
├── Src/                      # 源文件(CubeMX生成)
├── MDK-ARM/                  # Keil工程
│   ├── bsp/                  # 板级支持包(手动添加)
│   ├── app/                  # 应用层(手动添加)
│   ├── Project.uvprojx       # Keil工程文件
│   └── startup_stm32f427xx.s # 启动文件
└── Project.ioc               # CubeMX配置文件
```

### 5.2 main.c框架
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();        // 168MHz
    MX_GPIO_Init();
    MX_USART6_UART_Init();       // 调试串口
    // ... 其他外设初始化

    // 用户初始化
    led_off();

    while (1) {
        // 业务逻辑
        HAL_Delay(500);
    }
}
```

---

## 六、开发板对比总结（AB板最终方案）

| 对比项 | A板 (循迹) | B板 (摆杆控制) |
|--------|-----------|---------------|
| 开发板 | STM32F103ZET6 | RM A型板 |
| 芯片 | STM32F103ZET6 | STM32F427IIHx |
| 内核 | Cortex-M3 | Cortex-M4F |
| 主频 | 72MHz | 168MHz |
| FPU | ❌ | ✅ |
| Flash | 512KB | 1MB |
| SRAM | 64KB | 192KB |
| GPIO | 112 | 多(BGA176) |
| 板载IMU | ❌ | ✅ MPU6500 |
| 板载OLED | ❌ | ✅ SSD1306 |
| 开发环境 | Keil/CubeIDE | Keil MDK |
| 库 | HAL/标准库 | HAL库 |

### 分配理由
- **A板=F103ZET6**: 循迹吃引脚数量(红外阵列+编码器+电机+OLED+按键+UART)，F103的112个GPIO最多；循迹PID不需FPU
- **B板=RM A型板**: 摆杆控制吃计算性能(FPU做LQR矩阵运算)；板载MPU6500可检测摆杆倾角；板载SSD1306可做调试显示；168MHz快

---

## 七、ref目录文件索引

| 文件 | 内容 |
|------|------|
| `A_Board_Examples_学习指南.md` | 7个官方样例详细分析(含代码) |
| `A板官方样例学习指南.md` | 同上(Obsidian版) |
| `A_Board_Examples_README.md` | 样例列表 |
| `RM_A板_相关链接.md` | 官方链接 |
| `开发板 用户手册.pdf` | RM A板用户手册 |
| `开发板A型使用说明.pdf` | A板使用说明 |
| `开发板OLED使用说明.pdf` | 板载OLED说明 |
| `RoboMaster OLED 原理图.pdf` | OLED原理图 |

> 样例源码在 `C:\Users\liunian\Desktop\32模块资料\A_Board-Examples\` 下各子目录(PWM/Imu/RM_OLED等)
