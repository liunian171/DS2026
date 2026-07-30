# C/C++ 桥接机制详解

> 以 Servo 驱动为例，解释 main.c 如何调用 C++ 对象的方法。

---

## 一、核心问题

```
main.c（C 文件）   →   需要调  servo_bridge_set_angle(0, 90)
servo_bridge.cpp（C++ 文件） →   内部操作 C++ 对象，提供这个函数
```

C 和 C++ 编译器对函数名的"翻译"不同（Name Mangling），直接链接会失败。Bridge 的作用就是解决这个问题。

---

## 二、三个参与者

| 文件 | 身份 | 职责 |
|---|---|---|
| `main.c` | 调用方（C） | `#include "servo_bridge.h"`，只调 C 函数 |
| `servo_bridge.h` | 头文件 | 被双方 include，只有声明，通过 `#ifdef __cplusplus` 自适应 |
| `servo_bridge.cpp` | 实现方（C++） | `new` C++ 对象，函数用 `extern "C"` 修饰 |

---

## 三、`servo_bridge.h` 的结构

```
┌─────────────────────────────────────────────────────┐
│  #ifndef __SERVO_BRIDGE_H__                         │
│                                                     │
│  #include <stdint.h>      ← C 类型                   │
│  typedef enum { ... } ServoProtocol_t;  ← 纯 C      │
│                                                     │
│  #ifdef __cplusplus    ← 开关：C++ 编译时定义        │
│  extern "C" {          ← 告诉 C++ 编译器用 C 命名    │
│  #endif                                             │
│                                                     │
│  void servo_bridge_init(...);     ← 4 个函数声明     │
│  void servo_bridge_set_angle(...);                  │
│  void servo_bridge_start(...);                      │
│  void servo_bridge_stop(...);                       │
│                                                     │
│  #ifdef __cplusplus                                 │
│  }                    ← extern "C" 结束              │
│  #endif                                             │
│                                                     │
│  #endif                                             │
└─────────────────────────────────────────────────────┘
```

---

## 四、编译流程（分步走）

### 第1步：main.c 编译

`#include "servo_bridge.h"` → **C 编译器**处理，`__cplusplus` 未定义：

```c
// main.c 实际看到的内容（#ifdef 跳过 extern "C"）：
typedef enum { SERVO_PROTOCOL_PWM = 0, ... } ServoProtocol_t;

void servo_bridge_init(uint8_t id, ServoProtocol_t protocol, void *handle);
void servo_bridge_set_angle(uint8_t id, float angle);
void servo_bridge_start(uint8_t id);
void servo_bridge_stop(uint8_t id);
```

**没有 `class`、没有 `new`、没有 C++ 痕迹。**

生成 `main.o`：

```
main.o 符号表（需要）：
  需要符号: servo_bridge_init
  需要符号: servo_bridge_set_angle
  需要符号: servo_bridge_start
  需要符号: servo_bridge_stop
```

### 第2步：servo_bridge.cpp 编译

C++ 编译器处理同一个 `servo_bridge.h`，`__cplusplus` 已定义：

```cpp
// servo_bridge.cpp 实际看到的内容（extern "C" 生效）：
extern "C" {
void servo_bridge_init(uint8_t id, ...);     // C 风格命名！
void servo_bridge_set_angle(uint8_t id, ...);// C 风格命名！
void servo_bridge_start(uint8_t id);         // C 风格命名！
void servo_bridge_stop(uint8_t id);          // C 风格命名！
}

// 然后 servo_protocol.h 展开，拿到 C++ 类定义
```

内部实现：

```cpp
static PWMServoProtocol *pwm_servo_protoc[8] = {nullptr};  // C++ 对象池
static Servo* servo[8] = {nullptr};                          // C++ 对象池

void servo_bridge_init(uint8_t id, ...) {
    pwm_servo_protoc[id] = new PWMServoProtocol(...);  // C++ 操作
    servo[id] = new Servo(...);
}

void servo_bridge_set_angle(uint8_t id, float angle) {
    servo[id]->set_angle(angle);  // C++ 成员函数调用
}
```

生成 `servo_bridge.o`：

```
servo_bridge.o 符号表（提供）：
  提供符号: servo_bridge_init          ← 不是 _Z19servo_bridge_initxxx
  提供符号: servo_bridge_set_angle     ← C 风格，没有类名/参数类型后缀
  提供符号: servo_bridge_start
  提供符号: servo_bridge_stop
```

### 第3步：链接

```
链接器合体：

 main.o 需要 servo_bridge_init ────────┐
                                      │ 名字对上了 ✅
 servo_bridge.o 提供 servo_bridge_init ┘

 main.o 需要 servo_bridge_set_angle ───┐
                                      │ 名字对上了 ✅
 servo_bridge.o 提供 servo_bridge_set_angle ┘

 // start / stop 同理
```

---

## 五、关键对比：有无 `extern "C"` 的区别

| | 有 `extern "C"` | 无 `extern "C"` |
|---|---|---|
| servo_bridge.cpp 编译出的符号名 | `servo_bridge_set_angle` | `_Z25servo_bridge_set_angleif` |
| main.o 需要的符号名 | `servo_bridge_set_angle` | `servo_bridge_set_angle` |
| 链接结果 | ✅ 成功 | ❌ 符号找不到 |

---

## 六、`#ifdef __cplusplus` 的作用

```
同一段代码，不同编译器看到不同结果：

C 编译器：   __cplusplus 未定义 → ifdef 不成立 → extern "C" 被跳过
            → 看到纯净的 C 函数声明

C++ 编译器： __cplusplus 已定义 → ifdef 成立 → extern "C" 生效
            → 函数用 C 命名规则
```

---

## 七、物件关系

```
main.c ───────── C 编译器 ───────── main.o
  │ #include                             │
  ▼                                      │ 编译阶段：各自独立
servo_bridge.h                           │
  │ #include                             │
  ▼                                      ▼
servo_bridge.cpp ── C++ 编译器 ── servo_bridge.o
  │ #include                             │
  ▼                                      │
servo_protocol.h                         │
（包含 PWMServoProtocol、Servo 类定义）    │
                                         │
                          链接阶段       │
            main.o  ←→  servo_bridge.o   │
             符号匹配，合体 ✅            │
```

---

## 八、main.c 眼中的黑盒

```c
// main.c 中：
servo_bridge_init(0, SERVO_PROTOCOL_PWM, &pwm_tim5_ch4);
servo_bridge_set_angle(0, 90);
```

这一行代码背后实际发生了什么，main.c **一概不知**：

- 不知道 `new PWMServoProtocol` 创建了 C++ 对象
- 不知道 `new Servo` 创建了 C++ 对象
- 不知道 `set_angle` 内部走了虚函数派发
- 不知道底层是 PWM 寄存器操作

main.c 只知道：调用函数 → 舵机转了。**这就是桥的设计目的。**
