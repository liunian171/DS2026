# Servo 驱动设计

---

## 一、设计目标

- **易扩展**：支持 PWM 舵机、UART 总线舵机（Dynamixel / S.Bus）、I²C 舵机驱动板（PCA9685）等多种类型，新增协议不改已有代码
- **易移植**：硬件相关代码集中在底层，换个 MCU 只改底层实现层
- **易使用**：应用层只调 `setAngle()`，不关心底层走的是什么协议
- **非阻塞**：斜坡平滑等耗时操作不阻塞主循环

---

## 二、整体架构（三层分离）

```
┌─────────────────────────────────────────────────────────────────┐
│  应用层  main.c  （通过 C 桥接层访问 C++ 对象）                   │
│  职责：组装对象、编排动作                                          │
│  动作：servo_bridge_setAngle(0, 90);                              │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  servo_bridge.h / servo_bridge.cpp（C 桥接层）                    │
│  职责：C → C++ 适配，工厂创建 + 索引路由                           │
│  特点：extern "C" 接口，main.c 可直接 include                     │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  功能层  Servo 类                                                │
│  职责：角度换算、限位钳位、斜坡平滑、耗时控制                      │
│  特点：                                                         │
│    - 不对着某个具体协议编程，只对着 IServoProtocol* 接口调用       │
│    - 不知道底层是 PWM 还是 UART，只管算角度 → 脉宽                │
│    - 一个 Servo 对象控制一个舵机                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  适配层   IServoProtocol 虚基类 + 派生类                         │
│                                                                  │
│  IServoProtocol（接口契约）                                       │
│    ├─ set_position(rate_0E3)     ← 设置位置（千分比）             │
│    ├─ get_position()            ← 读反馈（可选）                  │
│    └─ ~IServoProtocol() = default                                │
│                                                                  │
│  派生类（各实现一种通信方式）：                                    │
│    ├─ PWMServoProtocol    → PWM 驱动 → pwm.h                   │
│    └─ ServoI2CProtocol    → I²C 总线 → HAL_I2C                  │
│    ├─ ServoUARTProtocol   → 串口总线 → HAL_UART                 │
│                                                                  │
│  特点：                                                          │
│    - 适配层是功能层和硬件之间的"翻译官"                             │
│    - PWM 适配器：pulse_us → 占空比 → 写 CCR                      │
│    - UART 适配器：pulse_us → 协议帧 → 串口发送                   │
│    - 新增一种舵机 = 新增一个派生类，功能层零修改                    │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────┐
│  驱动底层  硬件操作层（C 实现）                                    │
│                                                                  │
│  PWM 驱动：                                                      │
│    ├─ 策略层（pwm.c）：跨平台通用，只做计算和组合逻辑               │
│    └─ 平台层（pwm_platform_ops.c）：调 HAL 读写寄存器              │
│                                                                  │
│  UART / I²C 驱动：STM32 HAL 库                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 三层间如何连接（依赖注入）

```
组装顺序（在 main.c / bridge 中完成）：

  第1步：创建底层驱动的句柄实例
         → 比如拿到 pwm_tim5_ch4 这个 PWM_Handle

  第2步：用底层句柄构造适配层
         → PWMServoProtocol proto(&pwm_tim5_ch4)
         → 此时适配层内部持有底层句柄

  第3步：把适配层作为接口注入到功能层
         → Servo servo(&proto)
         → Servo 只拿到 IServoProtocol*，不知道具体类型,到时候注入的IServoProtocol下的子具体类

  第4步：应用层只管调 servo.setAngle()
```

每层都通过**构造函数**拿到下层的接口/句柄，不自己创建下层对象。

> **注**：`IServoProtocol&` 是抽象类的引用。引用只是别名，实际绑定的是 bridge 层创建的具体子类对象（如 `PWMServoProtocol`）。Servo 调 `protocol.set_position()` 时，C++ 虚函数机制自动派发到子类实现。Servo 只认识接口声明的方法，子类特有属性不可见——这恰好是依赖倒置的约束力。

---

## 三、桥接层（servo_bridge）

### 3.1 为什么需要桥

main.c 是 **C 文件**（STM32CubeMX 生成），不能直接创建 C++ 对象。桥把 C++ 的 class 封装成 C 函数给 main.c 调。

### 3.2 桥的两个角色

| 角色 | 职责 |
|---|---|
| **工厂** | 初始化时根据协议类型创建对应的 Protocol 子类和 Servo 对象 |
| **路由器** | 运行时根据索引找到对应的 Servo 对象，转发调用 |

### 3.3 索引作手柄

```c
servo_bridge_setAngle(0, 90);   // 0 号舵机转 90°
servo_bridge_setAngle(1, 45);   // 1 号舵机转 45°
```

`0` 和 `1` 是**手柄**，桥内部通过数组 `Servo*[]` 映射到具体的对象。调用方不需要知道对象指针、协议类型等细节。

### 3.4 内部结构

桥内部维护两组全局指针数组：

```
main.c
  │
  ▼
servo_bridge.cpp           ← C 函数接口
  │
  ├── PWMServoProtocol* pwm_protos[MAX_SERVOS]   ← PWM 协议对象池
  ├── Servo* servos[MAX_SERVOS]                  ← Servo 对象池
  │
  └── 映射关系：id → 指针数组下标
       id=0 → servos[0]  → Servo(pwm_protos[0](&pwm_tim5_ch4))
       id=1 → servos[1]  → Servo(pwm_protos[1](&pwm_tim1_ch2))
```

### 3.5 初始化流程

```cpp
#define MAX_SERVOS 8
static PWMServoProtocol* pwm_protos[MAX_SERVOS] = {nullptr};
static Servo* servos[MAX_SERVOS] = {nullptr};

void servo_bridge_init(uint8_t id, ServoProtocol_t proto, void *handle)
{
    switch (proto)
    {
    case SERVO_PROTOCOL_PWM:
        pwm_protos[id] = new PWMServoProtocol(*(PWM_Handle*)handle);
        servos[id] = new Servo(*pwm_protos[id]);
        break;
    }
}
```

```
servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim5_ch4)
  ① new PWMServoProtocol(&pwm_tim5_ch4)          ← 堆上创建协议对象
  ② pwm_protos[0] = 对象地址                       ← 存入协议池
  ③ new Servo(*pwm_protos[0])                      ← 堆上创建 Servo 对象
  ④ servos[0] = 对象地址                            ← 存入 Servo 池

servo_bridge_init(1, SERVO_PROTOCOL_PWM, &pwm_tim1_ch2)
  ① new PWMServoProtocol(&pwm_tim1_ch2)          ← 独立创建，互不干扰
  ② pwm_protos[1] = 对象地址
  ③ new Servo(*pwm_protos[1])
  ④ servos[1] = 对象地址
```

### 3.6 转发流程

```
servo_bridge_setAngle(0, 90)
  ① g_servos[0]->setAngle(90)        ← 数组查找，找到 Servo 对象
      ② protocol.set_position(rate)   ← 虚函数派发，自动落到 PWMServoProtocol
          ③ pwm_set_duty_0E3(...)     ← 操作硬件
```

### 3.7 设计要点

**对象生命周期**：用全局指针数组 + `new` 在堆上分配，每个 id 独立创建，互不干扰。不推荐 `static` 局部变量——同一 case 的 static 变量只构造一次，导致不同 id 拿到同一个对象。

**协议扩展**：加新协议只需在 init 函数中增加一个 case，并新增对应的指针数组：

```cpp
// 新增全局数组
static ServoUARTProtocol* uart_protos[MAX_SERVOS] = {nullptr};

case SERVO_PROTOCOL_UART:
    uart_protos[id] = new ServoUARTProtocol(*(UART_HandleTypeDef*)handle);
    servos[id] = new Servo(*uart_protos[id]);
    break;
```

Servo 类零修改，main.c 的调用方式不变。虚函数机制在 Servo 内部，桥完全不涉及。

### 3.8 C 接口定义

```c
typedef enum {
    SERVO_PROTOCOL_PWM,
    SERVO_PROTOCOL_UART,
    SERVO_PROTOCOL_I2C
} ServoProtocol_t;

void servo_bridge_init(int index, ServoProtocol_t proto_type, void *handle);
void servo_bridge_setAngle(int index, float angle);
void servo_bridge_start(int index);
void servo_bridge_stop(int index);
```

### 3.9 完整调用链路

```
┌─ main.c ──────────────────────────────────────────────────────┐
│  servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim5_ch4);     │
│  servo_bridge_setAngle(0, 90);                                │
└──────────────────┬───────────────────────────────────────────┘
                   │ C 函数调用
                   ▼
┌─ servo_bridge.cpp ─────────────────────────────────────────────┐
│  static PWMServoProtocol* pwm_protos[MAX_SERVOS] = {};         │
│  static Servo* servos[MAX_SERVOS] = {};                        │
│                                                                │
│  pwm_protos[0] = new PWMServoProtocol(*(PWM_Handle*)h);       │
│  servos[0] = new Servo(*pwm_protos[0]);                       │
│  servos[index]->setAngle(angle);                               │
└──────────────────┬───────────────────────────────────────────┘
                   │ C++ 虚函数派发
                   ▼
┌─ Servo.setAngle(90) ───────────────────────────────────────────┐
│  rate_0E3 = 90 * 1000 / 180 = 500                              │
│  protocol.set_position(500)     ← 多态，自动落子类              │
└──────────────────┬───────────────────────────────────────────┘
                   ▼
┌─ PWMServoProtocol.set_position(500) ───────────────────────────┐
│  pulse = map(500, 0, 1000, 500, 2500) = 1500µs                 │
│  duty = 1500 * 1000 / period = 75‰                             │
│  pwm_set_duty_0E3(&hpwm, 75)                                   │
└──────────────────┬───────────────────────────────────────────┘
                   ▼
┌─ pwm.c ───────────────────────────────────────────────────────┐
│  CCR = 75 * ARR / 1000 → 写寄存器 → 硬件输出 → 舵机转动        │
└────────────────────────────────────────────────────────────────┘
```

---

## 四、适配层（IServoProtocol）

### 协议层的角色：翻译官，不是操作工

适配层的分层依据不是"操作什么硬件"，而是**"翻译谁的指令"**：

```
每层的输入输出：

功能层（Servo）
  输入：角度（人的语言）
  输出：千分比 rate（协议层的语言）
  ↕
协议层（PWMServoProtocol / UART / I²C）    ← 翻译官
  输入：千分比（功能层的语言）
  输出：占空比 / 协议帧 / I²C 命令（硬件的语言）
  ↕
驱动层（PWM / UART / I²C）
  输入：占空比 / 数据帧（协议层的语言）
  输出：寄存器（芯片的语言）
```

协议层对上承诺"你只管给千分比，硬件的事我搞定"，对下只需要"你能写寄存器 / 发数据就行"。PWMServoProtocol 写 CCR，ServoUARTProtocol 发串口帧——操作完全不同，但角色一致：都是把抽象意图翻译成硬件信号。

### 核心接口

| 方法 | 说明 |
|---|---|
| `set_position(rate_0E3)` | 设置位置（千分比 0~1000），协议层唯一入口 |
| `get_position()` | 读取当前位置（总线舵机支持反馈，PWM 舵机返回 0） |

### 各派生类的"翻译"过程

| 派生类 | 输入 | 翻译过程 | 输出 |
|---|---|---|---|
| `PWMServoProtocol` | rate_0E3 | `duty = pulse_us × 1000 / period_us` | `pwm_set_duty_0E3()` |
| `ServoUARTProtocol` | rate_0E3 | 打包成协议帧（如 Dynamixel 指令包） | `HAL_UART_Transmit()` |
| `ServoI2CProtocol` | rate_0E3 | 计算 PCA9685 的 LEDn_ON/OFF 寄存器值 | `HAL_I2C_Mem_Write()` |

PWM 适配器是一对一的（一个适配器控制一个通道），UART/I²C 适配器是多对一的（一个适配器管理总线上多个舵机，靠 ID 区分）。

---

## 五、功能层（Servo 类）

### 核心 API

```
P0 — 基础控制
├── setAngle(deg)
│   思路：角度 → 千分比线性映射
│         rate = deg * 1000 / 180
│         内部做角度钳位（如 0~180°）
│
├── getAngle()
│   思路：返回内部缓存的角度值
│
└── setAngleLimit(min, max)
     思路：设置软件限位，setAngle 输出前先钳位

P1 — 平滑控制
└── setAngleWithDuration(deg, duration_ms)
     思路：将 movement 拆分成多个小步，每步调一次 setAngle
           - 计算总步数 = duration_ms / 步进间隔
           - 每步角度增量 = (目标角度 - 当前角度) / 总步数
           - 通过定时 tick 驱动步进，非阻塞执行
           - 步进到目标后自动停止

P2 — 进阶功能（可选）
├── setAngleSync(deg, duration_ms, trigger_time)
│   思路：多舵机同步——先缓存目标，主循环统一时间戳触发
│
├── setDeadBand(deg)
│   思路：死区补偿，小角度变化在阈值内忽略，减少抖动
│
└── powerOff()
     思路：紧急停止，通过协议层停止输出
```

### Servo 内部状态

```
当前角度缓存         ← getAngle() 直接返回它
目标角度缓存         ← 斜坡控制时记录终点
限位范围 (min/max)   ← 默认 0°~180°
死区阈值             ← 默认 0
错误状态标志          ← 限位触发等异常时置位
斜坡进度              ← 步进控制用的计数/定时变量
```

### 斜坡执行的触发方式

不使用 `HAL_Delay()` 在循环内死等。推荐方案：

| 方案 | 做法 | 优缺点 |
|---|---|---|
| 主循环 tick | Servo 暴露 `tick()` 函数，主循环每轮调用一次 | 最灵活，不依赖中断，适合裸机 |
| SysTick 中断 | 每 1ms 标记一次，Servo 内部轮询 | 简单但耦合了中断 |
| 硬件定时器中断 | 单独 TIM 中断里步进 | 精确但占用硬件资源 |

建议先用**主循环 tick 方案**。

---

## 六、多舵机管理

### 多个 Servo 对象的组织方式

```cpp
// 单个舵机
Servo servo(&proto);

// 多个舵机
Servo servos[] = {
    Servo(&proto1),   // 通道1
    Servo(&proto2),   // 通道2
    Servo(&proto3),   // 通道3
};
```

每个 Servo 对象独立控制一个舵机。斜坡控制的 `tick()` 需要在主循环中对每个 Servo 分别调用。

### 可选：ServoManager

如果项目中有十几个舵机，可以考虑加一层 `ServoManager`：

- 持有一个 `Servo*` 数组
- 批量调用所有 Servo 的 `tick()`
- 提供 `syncMove(angle_array, duration_ms)` 接口，统一触发多个舵机同时开始斜坡
- 小规模用数组遍历就够了

---

## 七、错误处理策略

| 错误场景 | 处理方式 |
|---|---|
| 目标角度超出限位 | 钳位到边界，不报错 |
| 舵机不响应（总线舵机） | 超时重试 n 次，超过次数置错误标志 |
| PWM 通道未初始化 | 适配层构造时检查状态，失败返回错误码 |
| 紧急情况 | 调 `powerOff()` 停止 PWM 输出 |

错误状态通过 `getErrorState()` 返回，应用层可轮询检查。

---

## 八、移植 / 扩展指南

### 换 MCU（如 STM32 → GD32）

只需改写驱动底层：
- 新增 `pwm_gd32_ops.c`，实现 `PWM_PlatformOps_t` 中的所有函数指针
- 如果舵机类型不变，适配层和功能层不需要任何修改

### 换舵机类型（如 PWM 舵机 → UART 总线舵机）

只需新增一个适配层派生类：
- 新建 `ServoUARTProtocol`，在 `set_position()` 里实现总线协议打包和发送
- 功能层的 `Servo` 类完全不感知也不修改

### 新增舵机通道

只需在 bridge 中新增一个 `PWM_Handle` 实例 + 调用 `servo_bridge_init()`，无需改动框架代码。

---

## 九、设计原则总结

| 设计原则 | 体现 |
|---|---|
| **开闭原则** | 新增协议类型只需新派生类，不修改已有代码 |
| **单一职责** | 功能层算角度，适配层翻译协议，驱动层操作寄存器 |
| **依赖倒置** | Servo 依赖 IServoProtocol 接口，不依赖具体硬件 |
| **接口隔离** | IServoProtocol 只有 2 个方法 |
| **0E3 定点数** | 内部统一使用千分比（0~1000）替代浮点 |

---

## 十、实现顺序建议

```
阶段1（基础可用）：
  ① IServoProtocol 虚基类 —— 定义接口
  ② PWMServoProtocol 派生类 —— PWM 适配层实现
  ③ Servo 类的 setAngle / getAngle —— 核心功能
  ④ servo_bridge 桥接层 —— C 接口
  ⑤ main.c 接入单舵机验证

阶段2（安全防护）：
  ⑥ 限位保护 setAngleLimit
  ⑦ 死区补偿 setDeadBand

阶段3（平滑控制）：
  ⑧ 斜坡平滑 setAngleWithDuration + tick() 驱动
  ⑨ 主循环接入 tick 验证

阶段4（扩展）：
  ⑩ 如有需要，扩展其他协议适配器
  ⑪ 如有需要，ServoManager 多舵机同步
```
---
main.c
  │  servo_bridge_set_angle(0, 90)          ← C 函数
  ▼
servo_bridge.cpp                            ← 路由层
  │  servo[0]->set_angle(90)
  ▼
Servo（功能层）                               ← 处理"角度"
  │  IServoProtocol &protocol              ← 只有一个引用成员
  │  set_angle(angle):
  │    rate = (angle - 0) * 1000 / 180      ← 角度 → 千分比
  │    protocol.set_position(rate)           ← 调接口，不管是谁
  ▼
PWMServoProtocol（协议层）                    ← 处理"脉宽"
  │  PWM_Handle &hpwm                      ← 只有一个引用成员
  │  set_position(rate_0E3):
  │    pulse_us = map(rate, 0,1000, 500,2500)  ← 千分比 → 微秒
  │    duty = pulse_us * 1000 / period_us       ← 微秒 → 占空比
  │    pwm_set_duty_0E3(&hpwm, duty)
  ▼
PWM_Handle → 硬件寄存器                       ← 处理"寄存器"
