# PWM 驱动层设计

---

## 一、架构设计（三层分离）

```
应用层（调用方）
  pwm_set_freq(hpwm, 50) / pwm_set_duty_0E3(hpwm, 500)
         │
         ▼
┌─ pwm.c / pwm.h ────────────────────────────┐
│  策略层（跨平台通用，与芯片无关）              │
│  职责：组合调用 ops + 纯算法计算              │
│  换平台时：此文件零修改                        │
└──────────────────────┬──────────────────────┘
                       │ hpwm->ops->xxx()
                       ▼
┌─ pwm_platform_ops.c / pwm_platform_ops.h ──┐
│  平台实现层（每款芯片一套）                    │
│  职责：直接操作硬件寄存器 / HAL 库             │
│  换平台时：新增一套 .c/.h，接口签名不变         │
└──────────────────────────────────────────────┘

┌─ tim.c / tim.h ────────────────────────────┐
│  HAL 配置层（STM32CubeMX 生成）              │
│  职责：定时器时钟使能、GPIO 复用、PWM 输出     │
│  换平台时：重新生成或适配                      │
└──────────────────────────────────────────────┘
```

### 核心解耦手段

| 机制 | 作用 |
|---|---|
| `void *htim` | 隐藏具体芯片的句柄类型 |
| `PWM_PlatformOps_t *ops` | 操作表（函数指针），多态调用 |
| `PWM_Handle` 句柄 | 串联 `htim` + `ops` + `Channel` |

---

## 二、文件结构

| 文件 | 层 | 职责 |
|---|---|---|
| `Core/Inc/driver/pwm.h` | 抽象层头文件 | `PWM_Handle` 结构体、`PWM_PlatformOps_t` 接口定义、策略函数声明 |
| `Core/Inc/driver/pwm_platform_ops.h` | 平台层头文件 | STM32 ops 函数声明及操作表 extern |
| `Core/Src/driver/pwm.c` | 策略层 | 频率设置、占空比读写、ARR 查表、PSC 计算 |
| `Core/Src/driver/pwm_platform_ops.c` | 平台实现层 | STM32 HAL 寄存器级操作的实现 |
| `Core/Src/tim.c` | HAL 配置层 | CubeMX 生成的 TIM1/TIM5 初始化代码 |

---

## 三、数据结构

### `PWM_Handle`（核心句柄）

```c
typedef struct PWM_Handle {
    void *htim;                     // 平台句柄（STM32 中为 TIM_HandleTypeDef*）
    const PWM_PlatformOps_t *ops;  // 平台操作表
    uint32_t Channel;               // PWM 通道号（如 TIM_CHANNEL_4）
    PWM_Ch_State Ch_State;          // 通道状态
    uint32_t PWM_CCR;               // 缓存当前比较值
} PWM_Handle;
```

### `PWM_Ch_State`（通道状态）

| 值 | 含义 |
|---|---|
| `PWM_Ch_State_OK` | 正常 |
| `PWM_Ch_State_Using` | 正在使用 |
| `PWM_Ch_State_Uninit` | 未初始化 |
| `PWM_Ch_State_Error` | 错误状态 |

### `PWM_PlatformOps_t`（操作表）

| 操作 | 签名 | 说明 |
|---|---|---|
| `start` | `void (*start)(void *htim, uint32_t ch)` | 启动通道 PWM 输出 |
| `stop` | `void (*stop)(void *htim, uint32_t ch)` | 停止通道 PWM 输出 |
| `get_psc` | `uint16_t (*get_psc)(void *htim)` | 读取预分频器 |
| `set_psc` | `void (*set_psc)(void *htim, uint32_t psc)` | 写入预分频器 |
| `get_arr` | `uint32_t (*get_arr)(void *htim)` | 读取自动重装载值 |
| `set_arr` | `void (*set_arr)(void *htim, uint32_t arr)` | 写入自动重装载值 |
| `get_clk_freq` | `uint32_t (*get_clk_freq)(void *htim)` | 获取定时器时钟频率 |
| `get_ccr` | `uint32_t (*get_ccr)(void *htim, uint32_t ch)` | 读取比较值 |
| `set_ccr` | `void (*set_ccr)(void *htim, uint32_t ch, uint32_t ccr)` | 写入比较值 |

---

## 四、策略层 API

### 占空比操作

```c
// 设置占空比（千分比 0～1000），0=0%，500=50%，1000=100%
void pwm_set_duty_0E3(PWM_Handle *hpwm, uint16_t duty_cycle);

// 读取当前占空比（千分比 0～1000）
uint16_t pwm_get_duty_0E3(PWM_Handle *hpwm);
```

### 频率操作

```c
// 设置频率（单位 Hz）
void pwm_set_freq(PWM_Handle *hpwm, uint32_t freq);
```

---

## 五、频率范围与 ARR 预设

| 频率范围 | ARR | 适用场景 | 选择原因 |
|---|---|---|---|
| 20 Hz ～ 1 kHz | 19999 | 舵机控制 | 高分辨率（20000 步），精确脉宽 |
| 1 kHz ～ 100 kHz | 1999 | 电机控制 | 2000 步分辨率，满足一般需求 |

PSC 根据 `PSC = clk_freq / ((ARR + 1) × freq) - 1` 自动计算。

> ARR 固定模式下，频率精确当且仅当 `clk / ((ARR+1) × freq)` 为整数。当前电机段 ARR=1999，90MHz/(2000×20kHz)=2.25 非整数，实际输出 ~22.5kHz（偏差 12.5%）。该偏差不影响电机运行（>20kHz 已避开听觉范围）。若需精确 20kHz，可将 ARR 改为 2249。

---

## 六、当前硬件配置（STM32F427）

| 定时器 | 总线 | 时钟 | PSC | ARR | 通道 | 引脚 | 初始频率 |
|---|---|---|---|---|---|---|---|
| TIM1 | APB2 | 180 MHz | 89 | 999 | CH2 | PE11 | 2 kHz |
| TIM1 | APB2 | 180 MHz | 89 | 999 | CH4 | PE14 | 2 kHz |
| TIM5 | APB1 | 90 MHz | 179 | 9999 | CH4 | PI0 | 50 Hz |

> **注意**：`pwm_stm32_get_clk_freq()` 当前 APB2 分支直接返回 `HAL_RCC_GetPCLK2Freq()` 而未 ×2（APB2 预分频为 /2，实际 TIM1 时钟应为 180 MHz，但函数返回 90 MHz）。使用时需注意该 Bug 对 TIM1 频率精度的影响。

---

## 七、用法示例

```c
#include "pwm.h"

// 1. 定义全局实例（当前在 pwm.c 底部，待移至 pwm_instance.c）
extern PWM_Handle pwm_tim5_ch4;

// 2. 设置占空比 50%
pwm_set_duty_0E3(&pwm_tim5_ch4, 500);

// 3. 设置频率 200 Hz（同区间内只改 PSC，ARR 不变）
pwm_set_freq(&pwm_tim5_ch4, 200);

// 4. 读取当前占空比
uint16_t duty = pwm_get_duty_0E3(&pwm_tim5_ch4);
```

---

## 八、STM32 定时器时钟树

### 时钟关系

```
HSE/HSI → PLL → SYSCLK(180MHz) → AHB(180MHz)
                                     ├→ APB1(45MHz)  /4 → TIM2~7,12~14 时钟
                                     └→ APB2(90MHz)  /2 → TIM1,8~11 时钟
```

**关键规则**（STM32F4xx 参考手册）：

当 APBx 预分频器 ≠ 1 时，定时器时钟 = APBx 频率 × 2。

工程配置：
- `APB1CLKDivider = RCC_HCLK_DIV4`（/4）→ PCLK1 = 45 MHz → TIM5 时钟 = **90 MHz** ✅
- `APB2CLKDivider = RCC_HCLK_DIV2`（/2）→ PCLK2 = 90 MHz → TIM1 时钟 = **180 MHz** ✅（已修 Bug，现返回 ×2）

---

## 九、影子寄存器与更新事件（UEV）

### PSC 和 ARR 的双缓冲机制

STM32 定时器的 PSC 和 ARR 都是**影子寄存器**（双缓冲）：

```
软件写入 TIMx->PSC ──→ 预装载寄存器（随时可写）
软件写入 TIMx->ARR ──→ 预装载寄存器（随时可写）
                            ↓
                CNT 溢出 → 更新事件（UEV）
                            ↓
                   预装载 → 工作寄存器（新值生效）
                            ↓
                   下一个周期按新参数运行
```

### 当前驱动行为

驱动层写入寄存器后**不强制触发 UEV**，而是等待当前周期自然结束：

```c
// set_psc / set_arr 的操作流程：
__HAL_TIM_SET_PRESCALER(htim, psc);     // 写入预装载寄存器
__HAL_TIM_SET_AUTORELOAD(htim, arr);    // 写入预装载寄存器
                                        // 等待 CNT 计满 → 自动加载
```

**优点**：新参数在周期边界同步切换，输出无毛刺。
**缺点**：同周期内修改立即生效的场景（如故障保护）需要强制触发 UEV。

### CCR 的预装载

CubeMX 生成的代码中 `TIM_OC_InitTypeDef` 未显式设置 `OCPreload`，HAL 默认 `TIM_OCPRELOAD_ENABLE`。CCR 也是双缓冲的，写入后下次 UEV 才生效。

---

## 十、频率切换的边界条件

### 同 ARR 区间（仅改 PSC）

```
写入 TIMx->PSC ← 新的预分频值
     ↓
当前 CNT 继续向上计数
     ↓
CNT == ARR → 产生 UEV → PSC 影子加载
     ↓
新频率从下一周期开始生效
```

### 跨 ARR 区间（改 ARR + PSC）

```
写入 TIMx->PSC → TIMx->ARR
     ↓
当前 CNT 何时溢出取决于新旧 ARR 值：
     ↓
情况 A：新 ARR < 当前 CNT → CNT 立即溢出
        （例：CNT=15000，新 ARR=1999 → 超范围，立即 UEV）
情况 B：新 ARR ≥ 当前 CNT → 继续计数到新 ARR 后溢出
        （例：CNT=500，新 ARR=1999 → 正常计数到 1999）
     ↓
影子加载 → 新频率从下一周期开始生效
```

### 特殊场景：ARR 大幅降低

当新 ARR 小于当前 CNT 值时，CNT 会**立即**溢出并产生 UEV，导致当前周期被意外截断，产生一个不完整的短脉冲。

**建议**：若对切换时刻的脉冲完整度有严格要求，可在改 ARR 前将占空比置 0，改完再恢复。

---

## 十一、16 位 vs 32 位定时器

| 定时器 | 位宽 | ARR 范围 | PSC 范围 | CCR 范围 |
|---|---|---|---|---|
| TIM2 / TIM5 | 32 位 | 0 ~ 4,294,967,295 | 0 ~ 65535 | 0 ~ 4,294,967,295 |
| 其他（TIM1 等） | 16 位 | 0 ~ 65535 | 0 ~ 65535 | 0 ~ 65535 |

**频率下限估算**（TIM1 @ 180 MHz）：

| ARR | PSC 最大 65535 | 最低频率 |
|---|---|---|
| 65535 | 65535 | 180M / (65536 × 65536) ≈ 0.042 Hz |

**频率上限估算**（TIM1 @ 180 MHz）：

| ARR | PSC | 最高频率 |
|---|---|---|
| 1 | 0 | 180M / (2 × 1) = 90 MHz |

---

## 十二、TIM1 高级定时器特性

TIM1 是高级控制定时器，具备以下额外特性（当前未使用）：

### 互补输出与死区

| 通道 | 主输出 | 互补输出 |
|---|---|---|
| CH1 | PE9 | PE8 |
| CH2 | PE11 | PE10 |
| CH3 | PE13 | PE12 |
| CH4 | PE14 | **无互补** |

当前 `DeadTime = 0`、`BreakState = DISABLE`，驱动层无相关 API。

### 刹车输入（Break）

TIM1 支持刹车输入（BKIN 引脚），用于电机驱动等安全关键场景的硬件级输出关断。当前未使用。

### 主输出使能（MOE）

TIM1 的 BDTR 寄存器中 MOE 位控制所有 PWM 输出通道。当刹车事件发生时，MOE 硬件自动清零，输出进入预设空闲状态。

---

## 十三、GPIO 复用映射

| 信号 | 引脚 | 复用功能 | 定时器通道 | 当前用途 |
|---|---|---|---|---|
| TIM1_CH1 | PA8 | AF1 | 高级定时器通道 1 | 电机 PWM（备选） |
| TIM5_CH4 | PI0 | AF2 | 通用定时器通道 4 | 电机 PWM（当前 20kHz） |

---

## 十四、使用注意事项

1. **首次使用前确保 HAL 初始化已执行**：`MX_TIM1_Init()` / `MX_TIM5_Init()` 必须在操作 PWM 句柄前调用。
2. **频率范围保护**：`pwm_get_pre_arr()` 在频率不在预设区间内时返回 0，`pwm_get_psc_calculate()` 会因此产生除零。调用 `pwm_set_freq()` 前应确保频率在 20 ~ 100000 Hz 范围内。
3. **占空比钳位**：`pwm_set_duty_0E3()` 当前不对 `duty_cycle` 做限幅，传入 >1000 的值会导致 CCR > ARR，输出恒高电平。
4. **CNT 清 0 风险**：写入新 ARR 后，若新 ARR < 当前 CNT，CNT 立即溢出。在需要保持完整周期的场景中，应注意此行为。
5. **同一个定时器多通道同步**：TIM1 的 CH2 和 CH4 共享同一组 PSC/ARR。`pwm_set_freq()` 修改的是整个定时器的参数，影响该定时器所有通道。
6. **AHB 频率限制**：当前 SYSCLK=180 MHz，超出此值的配置可能导致系统不稳定。
7. **更新事件对 CCR 的影响**：`set_ccr` 写入 CCR 后，新值在下次 UEV 时才加载到影子寄存器。写入时序应是：先写 CCR，再改 PSC/ARR。

---

## 十五、已知限制与待办

- **实例定义位置**：`pwm_tim5_ch4` 当前定义在 `pwm.c` 中，导致 `pwm.c` 依赖 `tim.h`，丧失了跨平台编译能力。**建议**：移至独立 `pwm_instance.c`。
- **缺少参数校验**：占空比（>1000）和频率（=0）无输入限幅保护。
- **缺少死区时间/刹车/极性配置 API**：当前抽象层未覆盖 TIM1 的高级功能。
- **缺少中断支持**：无中断回调注册接口。
- **缺少错误状态查询**：`Ch_State` 运行时不自动更新。
