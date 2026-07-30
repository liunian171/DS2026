# 嵌入式驱动框架设计思想

> 适用：PWM、Servo、Motor 及未来所有遵循此模式的驱动模块。

---

## 一、核心原则

**每层只对一种"语言"负责，层与层之间通过固定格式的接口传递意图。**

---

## 二、四层架构

```
┌──────────────────────────────────────────────────────────────┐
│  应用层  main.c                                              │
│  语言：人的意图                                                │
│  "舵机转到 90°"  "电机 50% 速度正转"                          │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  桥接层  xxx_bridge.h / xxx_bridge.cpp                        │
│  语言：C 函数声明                                              │
│  职责：C → C++ 适配                                           │
│     · 工厂：创建 C++ 对象                                      │
│     · 路由：按 id 找到对象，转发调用                            │
│     · 生命周期：静态数组管理堆上对象                             │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  功能层  Xxx 类（C++）                                        │
│  语言：业务抽象（千分比 / 角度 / 速度）                          │
│  职责：业务逻辑，不碰任何硬件细节                                │
│    · Servo：角度 → 千分比，限位，斜坡                          │
│    · Motor：速度限幅，死区，方向控制，PID 闭环                   │
│  特征：持有下层接口引用，不持有下层具体类                         │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  协议层  IXxxProtocol 虚基类 + 派生类（C++）                    │
│  语言：硬件信号（占空比 / GPIO / 串口帧 / I²C 命令）             │
│  职责：翻译官 —— 把功能层的抽象意图翻译成具体硬件操作信号         │
│    · PWMServoProtocol：千分比 → 脉宽 → 占空比                  │
│    · TB6612Protocol：千分比 → 真值表 → GPIO + PWM              │
│  特征：持有下层硬件句柄，实现上层接口                             │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  驱动层  pwm.c / gpio / adc / uart / ...（C）                 │
│  语言：寄存器                                                  │
│  职责：物理操作，不关心上层是谁                                  │
│    · pwm.c：策略层（跨平台算法）+ 平台层（HAL 寄存器）           │
└──────────────────────────────────────────────────────────────┘
```

---

## 三、层的定义不是按硬件，是按"翻译谁的指令"

PWMServoProtocol 和 TB6612Protocol 操作的对象完全不同（一个写 TIM CCR，一个写 GPIO + CCR），但都在协议层——因为它们的职责相同：**把功能层的千分比翻译成硬件能执行的具体信号**。

```
对照：

  PWMServoProtocol                TB6612Protocol
  ────────────────                ───────────────
  上位：Servo                      上位：Motor
  上位说：rate（0~1000）           上位说：speed（±1000）
  翻译：千分比 → 脉宽 → 占空比      翻译：千分比 → 真值表 → GPIO + 占空比
  下位：PWM_Handle                 下位：PWM_Handle + GPIO
```

**层定义的是接口方向，不是硬件相似性。**

---

## 四、层与层之间的连接：依赖注入

```
功能层持有协议层的接口引用      →  XxxClass(IXxxProtocol &proto)
协议层持有驱动层的硬件句柄      →  XxxProtocol(PWM_Handle *pwm)
```

每层通过构造函数接收下层的接口/句柄，**自己不创建下层对象**。上层只依赖接口声明，不依赖具体实现。

### 为什么用引用而不是值

XxxProtocol 是抽象类（含纯虚函数），不能创建对象。但引用可以——它只是别名，实际绑定的是 Bridge 层在堆上创建的子类对象：

```
Bridge:  pwm_protocol = new PWMServoProtocol(pwm_handle)   ← 创建子类
         servo = new Servo(*pwm_protocol)                  ← 子类引用注入 Servo

Servo:   IServoProtocol &protocol  ← 声明为父类引用
         protocol.set_position(500) ← 虚函数派发到 PWMServoProtocol::set_position()
```

### 父类引用视角的约束

父类引用只能看到接口声明的方法。子类特有属性和方法不可见——这恰好是接口隔离的约束力：

- 需要所有子类都有的能力 → 提升到虚接口
- 初始化时一次性配置 → Bridge 层直接操作子类对象（Bridge 认识具体类）
- 与具体实现无关的参数 → 留在功能层

### 抽象接口只放方法，不放数据

协议基类（IXxxProtocol）是"行为契约"（能做什么），不是"数据容器"（有什么属性）。不同子类的硬件资源完全不同：

```
PWMServoProtocol：  PWM_Handle&（1 个引用）
UARTServoProtocol： UART_Handle + 波特率 + 协议帧格式
TB6612Protocol：    PWM_Handle* + GPIO 引脚 ×3
```

如果往基类放属性，要么放不全，要么全放浪费内存，要么子类属性升级到接口后全部子类被迫改。**数据是每个具体实现的私有细节，不在接口层暴露。**

---

## 五、C/C++ 桥接机制

### 问题

`main.c` 是 C 文件（CubeMX 生成），不能直接 `new` C++ 对象、不能调成员函数。

### 方案

Bridge 层用 C++ 实现，但函数入口用 `extern "C"` 修饰，生成 C 风格的符号名。头文件通过 `#ifdef __cplusplus` 自适应：

```c
#ifdef __cplusplus
extern "C" {
#endif

void servo_bridge_set_angle(uint8_t id, float angle);

#ifdef __cplusplus
}
#endif
```

### 编译链接流程

```
main.c ──C 编译器──→ main.o
        看到：void servo_bridge_set_angle(int id, float angle)
        需要符号：servo_bridge_set_angle

servo_bridge.cpp ──C++ 编译器──→ servo_bridge.o
        看到：extern "C" void servo_bridge_set_angle(...)
        提供符号：servo_bridge_set_angle   ← C 命名，不是 _Z25...

链接：名字对上 ✅
```

### 黑盒效应

main.c 只知道"调了函数，硬件有反应"。幕后 `new C++ 对象`、虚函数派发、堆管理，它一概不知。

### 指针与引用的使用边界

```
C 文件（pwm.h、usergpio.h）                      → 只能用指针（PWM_Handle*）
C++ 文件（motor_protocol.h、servo.h）内部成员     → 引用更安全（PWM_Handle&）

调用 C 层 API 时（motor_protocol.cpp）：
  C 层是 C 风格接口，只认指针
  C++ 层持有的是引用 → 调 C API 时加 & 取地址
  pwm_set_duty_0E3(&hpwm, duty);    // & 转引用→指针
  usergpio_write(&hgpio, 1);         // & 转引用→指针
```

---

## 六、实例化模式

不采用类内 `static` 局部变量（不同 id 会共用同一个对象），而是由 Bridge 用全局指针数组 + `new` 独立分配：

```
static PWMServoProtocol* pwm_protos[MAX_SERVOS] = {nullptr};
static Servo* servos[MAX_SERVOS] = {nullptr};

void servo_bridge_init(uint8_t id, ...):
    pwm_protos[id] = new PWMServoProtocol(...)
    servos[id] = new Servo(*pwm_protos[id])
```

每个 id 拥有独立的对象链：id → `Servo` → `PWMServoProtocol` → `PWM_Handle`。

---

## 七、数据约定

- **0E3 定点数**：所有速率/占空比统一用千分比（0~1000 = 0%~100%），避免浮点
- **千分比是上下层通用语言**：功能层输出千分比，协议层接收千分比
- **驱动层不关心千分比**：它只收最终的占空比/脉冲数

---

## 八、扩展方式

### 新增一个驱动模块（如 Motor）

```
对照已有 Servo 文件，按模板创建：
  motor.h             → IMotorProtocol + Motor 类
  motor.cpp           → Motor 实现
  motor_protocol.h    → TB6612Protocol（派生类）
  motor_protocol.cpp  → TB6612Protocol 实现
  motor_bridge.h      → C 接口
  motor_bridge.cpp    → 桥实现
```

### 新增一种协议（如换驱动芯片）

新增一个 `IXxxProtocol` 派生类，功能层、桥接层、main.c 零修改。

### 新增一种传感器/控制算法（如编码器/PID）

新增独立的 `.h/.cpp` 文件，通过接口注入到功能层。功能层已有代码不动，只加新构造函数和新方法。

---

## 九、Servo vs Motor 对照速查

| 层 | Servo | Motor |
|---|---|---|
| **桥梁** | `servo_bridge` | `motor_bridge` |
| **功能** | Servo 类：角度 → 千分比 | Motor 类：速度 → 千分比（±） |
| **协议接口** | `IServoProtocol::set_position()` | `IMotorProtocol::set_speed()` |
| **协议实现** | `PWMServoProtocol`：千分比 → 脉宽 → CCR | `TB6612Protocol`：千分比 → 真值表 → GPIO + CCR |
| **硬件** | TIM5_CH4 | TIM1_CH1 + 2×GPIO |

> 结构完全相同，差异只在千分比的语义（角度 vs 速度）和协议层的翻译方式（脉宽 vs 真值表）。
