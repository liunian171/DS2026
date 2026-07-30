# Motor 驱动架构设计

> 直流有刷电机 + TB6612FNG，对标 Servo 三层架构
> 
> **当前状态**：开环速度控制已完成（P1~P2）。编码器位置闭环和 PID 为二期。

---

## 一、硬件分析

### 1.1 控制对象

- **电机类型**：直流有刷减速电机（MG513XP28 等）
- **驱动芯片**：TB6612FNG（双路 H 桥）
- **反馈**：二期接入增量编码器（位置闭环）

### 1.2 控制信号需求（单路）

| 信号 | 类型 | 用途 |
|---|---|---|
| PWM | 定时器 PWM 输出 | 速度控制（0~100% 占空比） |
| IN1 | GPIO 推挽输出 | 方向控制 |
| IN2 | GPIO 推挽输出 | 方向控制 |
| STBY | 硬接 3.3V | 始终使能（不占用 GPIO） |

### 1.3 TB6612FNG 控制真值表

| IN1 | IN2 | PWM | 状态 |
|---|---|---|---|
| 0 | 0 | X | 惯性停机（H 桥关断） |
| 1 | 0 | 0~100% | 正转 |
| 0 | 1 | 0~100% | 反转 |
| 1 | 1 | X | 刹车（短接制动） |

### 1.4 PWM 资源

| 定时器 | 通道 | 引脚 | 用途 | 频率 |
|---|---|---|---|---|
| TIM5 | CH4 | PI0 | 电机 PWM（当前） | 20kHz |
| TIM1 | CH1 | PA8 | 电机 PWM（备用，分离频率时） | 20kHz |

---

## 二、对标 Servo 的分层架构

### 2.1 四层层级图

```
┌──────────────────────────────────────────────────────────────┐
│  main.c（应用层）                                              │
│    motor_bridge_init(0, &pwm, &in1, &in2, &stby, 200, 33.1) │
│    motor_bridge_set_speed_rpm(0, 100)  // 50% RPM            │
│    motor_bridge_set_speed_mps(0, 0.5)  // 0.5 m/s            │
│    motor_bridge_brake(0)               // 急停                │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  motor_bridge.h / motor_bridge.cpp（C 桥接层）                 │
│    对标 servo_bridge                                          │
│    · static TB6612MotorProtocol* tb6612_proto[MAX_MOTORS]     │
│    · static Motor* motor[MAX_MOTORS]                          │
│    · extern "C" 统一入口                                      │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  motor.h / motor.cpp（功能层 — C++ Motor 类）                  │
│    对标 Servo 类                                               │
│    职责：RPM/线速度 → 千分比换算，死区判断，限速钳位            │
│    · set_speed_rpm(rpm)         — RPM 模式                    │
│    · set_speed_mps(mps)         — 线速度模式（需轮半径）       │
│    · stop() / brake()/ set_dead_zone()                        │
│    内部：IMotorProtocol &protocol（接口注入）                  │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  IMotorProtocol（纯虚接口）                                    │
│    对标 IServoProtocol                                        │
│    · set_speed_rate_0E3(±1000) / stop() / brake()            │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  TB6612MotorProtocol（协议适配层）                               │
│    对标 PWMServoProtocol                                      │
│    · PWM_Handle *hpwm                                         │
│    · UserGPIO_Handle *hain1_gpio / *hain2_gpio               │
│    · 查真值表 → usergpio_write() + pwm_set_duty_0E3()          │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  pwm.h / pwm.c（PWM 策略层 — 已有，复用）                      │
│  usergpio.h / usergpio.c（GPIO 策略层 — 新增，跨平台）         │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 Servo vs Motor 层级对照表

| 层 | Servo | Motor | 差异 |
|---|---|---|---|
| **桥接层** | `servo_bridge` | `motor_bridge` | 多 IN1/IN2 GPIO 句柄参数 |
| **功能层** | `Servo` | `Motor` | RPM/线速度 vs 角度 |
| **协议接口** | `IServoProtocol` | `IMotorProtocol` | set_speed(±1000) vs set_position(0~1000) |
| **协议实现** | `PWMServoProtocol` | `TB6612MotorProtocol` | 真值表 + GPIO vs 脉宽映射 |
| **PWM 层** | `pwm.h/.c` | `pwm.h/.c` | **复用** |
| **GPIO 层** | 无 | `usergpio.h/.c` | Motor 专用 |

> 二期新增：`IPositionSensor` → `EncoderSensor`（编码器）、`PID`（闭环控制），Motor 类扩展 position 相关方法。

---

## 三、核心接口与类设计

### 3.1 协议层的角色：翻译官，不是操作工

参见 `Design_Philosophy.md`。

### 3.2 IMotorProtocol（协议抽象接口）

> 对标 `IServoProtocol`。Servo 的输入是千分比位置（0~1000），Motor 的输入是**千分比速率（-1000~1000）**，正负表示方向。

纯虚接口，两个方法（`stop()` 已在接口层移除，`set_speed_rate_0E3(0)` 即是惯性停）：

- `set_speed_rate_0E3(rate_0E3)` — 千分比速率，-1000~1000，正=正转，负=反转，0=停止
- `brake()` — 刹车制动（IN1=IN2=1，短接制动）

### 3.3 TB6612MotorProtocol（协议适配层）

> 对标 `PWMServoProtocol`。核心差异：Servo 把千分比映射为脉宽 → 写 CCR；TB6612 把千分比按正负查真值表 → 写 GPIO 方向 + 写 PWM 占空比。

内部持有：

- `PWM_Handle* hpwm` — PWM 通道句柄
- `UserGPIO_Handle* hain1_gpio` — IN1 方向引脚
- `UserGPIO_Handle* hain2_gpio` — IN2 方向引脚
- `UserGPIO_Handle* hstby_gpio` — STBY 引脚（可选，硬接高电平时可传任意 handle）

类名 `TB6612MotorProtocol`（非 `TB6612Protocol`）是为了预留未来 `TB6612StepProtocol`（步进电机）等扩展。

**`set_speed_rate_0E3` 逻辑**：按 speed 正负查真值表 — 正转（IN1=1,IN2=0）、反转（IN1=0,IN2=1）、零（调 stop）— 然后取绝对值写 PWM 占空比。

**`brake` 逻辑**：IN1=IN2=1（短接制动），PWM=0。

**`stop` 逻辑**：STBY=0（芯片待机，H 桥全关），IN1=IN2=0。

### 3.4 Motor 类（功能层 — 开环）

> 对标 `Servo` 类。接收 RPM 或 m/s 输入，内部换算千分比后交给协议层。

内部持有：

- `IMotorProtocol &protocol` — 协议接口引用
- `float max_rpm` — 额定转速（100% 占空比下的空载 RPM）
- `float wheel_radius_mm` — 轮子半径（mm），0=不启用线速度模式
- `float dead_zone_rpm` — 死区（RPM），默认 max_rpm × 5%

**函数实现**：

| 方法 | 说明 |
|---|---|
| `set_speed_rpm(rpm)` | RPM → `abs(rpm) < dead_zone` 则 stop → `map(rpm, ±max_rpm, ±1000)` → 调协议层 |
| `set_speed_mps(mps)` | m/s → RPM = (mps×60×1000)/(2π×r_mm) → 调 `set_speed_rpm` |
| `stop()` | 透传 `protocol.stop()` |
| `brake()` | 透传 `protocol.brake()` |
| `set_dead_zone(rpm)` | 运行时调整死区阈值 |

**物理公式**：`RPM = v(m/s) × 60 / (2π × r(mm) / 1000)`

### 3.5 桥接层（C 接口）

> 对标 `servo_bridge`，结构和行为完全一致。

```c
// 工厂初始化（C 接口）
void motor_bridge_init(uint8_t id,
                       PWM_Handle *pwm,
                       UserGPIO_Handle *ain1_gpio,
                       UserGPIO_Handle *ain2_gpio,
                       UserGPIO_Handle *stby_gpio,
                       float max_rpm,
                       float wheel_radius_mm);

// 运行控制
void motor_bridge_set_speed_rpm(uint8_t id, float rpm);
void motor_bridge_set_speed_mps(uint8_t id, float mps);
void motor_bridge_stop(uint8_t id);
void motor_bridge_brake(uint8_t id);
void motor_bridge_set_dead_zone(uint8_t id, float rpm);
```

内部维护两个静态指针数组（`TB6612MotorProtocol*[]` + `Motor*[]`），与 servo_bridge 完全相同的模式。

---

## 四、文件清单

| 文件 | 路径 | 说明 |
|---|---|---|
| `motor.h` | `Core/Inc/driver/` | IMotorProtocol 接口 + Motor 类声明 |
| `motor.cpp` | `Core/Src/driver/` | Motor 类实现 |
| `motor_protocol.h` | `Core/Inc/driver/` | TB6612MotorProtocol 类声明 |
| `motor_protocol.cpp` | `Core/Src/driver/` | TB6612MotorProtocol 实现 |
| `motor_bridge.h` | `Core/Inc/driver/` | C 桥接层声明 |
| `motor_bridge.cpp` | `Core/Src/driver/` | C 桥接层实现 |
| `usergpio.h` | `Core/Inc/driver/` | GPIO 跨平台抽象层 |
| `usergpio.c` | `Core/Src/driver/` | GPIO 策略层 |
| `usergpio_platform.h` | `Core/Inc/driver/` | STM32 GPIO ops 声明 |
| `usergpio_platform.c` | `Core/Src/driver/` | STM32 GPIO 平台实现 |
| `pwm.h` | `Core/Inc/driver/` | PWM 抽象层（复用） |
| `pwm.c` | `Core/Src/driver/` | PWM 策略层（复用） |

---

## 五、使用示例

```c
#include "driver/pwm.h"
#include "driver/motor_bridge.h"
#include "driver/usergpio.h"
#include "driver/usergpio_platform.h"

// 定义 GPIO 句柄
UserGPIO_Handle motor_in1 = { GPIOF, 1, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_in2 = { GPIOF, 0, &usergpio_platform_ops_stm32 };
UserGPIO_Handle motor_stby = { GPIOF, 0, &usergpio_platform_ops_stm32 };

void main(void) {
    HAL_Init(); SystemClock_Config();
    MX_GPIO_Init(); MX_TIM5_Init();

    pwm_set_freq(&pwm_tim5_ch4, 20000);
    pwm_start(&pwm_tim5_ch4);

    motor_bridge_init(0, &pwm_tim5_ch4,
                      &motor_in1, &motor_in2, &motor_stby,
                      200.0f, 33.1f);

    motor_bridge_set_speed_rpm(0, 100);   // 50% RPM
    // motor_bridge_set_speed_mps(0, 0.5);   // 0.5 m/s
    // motor_bridge_brake(0);                 // 急停
    // motor_bridge_stop(0);                  // 惯性停

    while (1);
}
```

---

## 六、设计原则

| 原则 | 体现 |
|---|---|
| **依赖倒置** | Motor 持有 `IMotorProtocol&`，不依赖 TB6612；换 L298N 只需新 Protocol 派生类 |
| **单一职责** | Protocol=真值表+信号，Motor=限速+死区+换算，PWM=寄存器 |
| **开闭原则** | 新增协议类型只需新派生类，Motor 和 Bridge 零修改 |
| **0E3 定点数** | 速度/PWM 占空比统一千分比，与 Servo 一致 |
| **C/C++ 桥接** | extern "C" bridge，与 servo_bridge 模式完全相同 |

### 与 Servo 的核心差异

| 对比维度 | Servo | Motor |
|---|---|---|
| **控制量** | 角度 0~180°，单向 | 速度/线速度，双向 |
| **频率** | 固定 50Hz | ≥20kHz |
| **协议复杂度** | 脉宽 → 占空比 | 真值表 + 2×GPIO + PWM |
| **停止方式** | PWM=0 | `set_speed(0)` = 惯性停 / `brake()` = 急停 |

> Motor 无需独立的 `stop()` 方法：`set_speed(0)` 对 TB6612 等于惯性停。

---

## 七、用到的硬件资源

| 资源 | 引脚 | 用途 | 已配置 |
|---|---|---|---|
| TIM5_CH4 | PI0 | 电机 PWM（20kHz） | ✅ |
| TIM1_CH1 | PA8 | 电机 PWM（备选） | ✅ CubeMX 已配 |
| GPIOF_PIN1 | PF1 | TB6612 IN1 | ✅ MX_GPIO_Init |
| GPIOF_PIN0 | PF0 | TB6612 IN2 | ✅ MX_GPIO_Init |
| STBY | 3.3V | 硬接使能 | ✅ 不占引脚 |

---

## 八、实现计划

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P1** | `motor_protocol.h/.cpp` — TB6612 协议实现 | ✅ |
| **P2** | `motor.h/.cpp` — 开环 RPM/线速度控制 | ✅ |
| **P3** | `motor_bridge.h/.cpp` — C 桥接层 | ✅ |
| **P4** | `usergpio` 驱动层 — GPIO 跨平台抽象 | ✅ |
| **P5（二期）** | `encoder_sensor.h/.cpp` — TIM 编码器模式 | ✅ |
| **P6（二期）** | `pid.h/.cpp` — PID 控制器 | ✅ |
| **P7（二期）** | Motor 类扩展闭环 — set_position / update | 待开始 |
