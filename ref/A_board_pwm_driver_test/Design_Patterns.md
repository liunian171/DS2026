# 嵌入式 C 通用设计模式

> 记录项目开发中遇到的典型设计问题和通用解决方案。
> 从具体问题中提炼，避免重新造轮子。

---

## 一、从回调参数找回自定义结构体（反向映射）

### 问题

HAL 库的回调函数只传 `UART_HandleTypeDef *huart`，但你需要访问自定义的 `UART_Handle`（里面包含 `rx_byte`、`ringbuf` 等）。

### 方案对比

| 方案 | 原理 | 通用性 | 适用场景 |
|---|---|---|---|
| **全局指针** | 初始化时存一个 `g_uart`，回调直接用 | ❌ 单例 | 确定只有一个实例 |
| **全局 `if-else`** | `if (huart == &huart7) ... else if ...` | ❌ 硬编码 | 实例极少、不改代码 |
| **映射表** | 静态数组存 `{hal_ptr, custom_ptr}`，回调遍历匹配 | ✅ 通用 | 任意数量实例，不同模块间互相查找 |
| **地址差单射** | `(addr - base) / sizeof(type)` 算序号，直接索引数组 | ⚠️ 特定 | 同类型全局变量连续排列（如 UART 句柄） |
| **container_of** | `(type*)((char*)ptr - offsetof(type, member))` | ✅ 通用 | 已知成员在结构体内的偏移，类型固定 |

### 使用建议

```
通用场景 → 映射表（灵活）或 container_of（高性能，需要固定类型）
特殊场景 → 地址差单射（UART/USART 句柄天然连续，O(1)）
单例 → 全局指针（最简单）
```

### container_of 详解

**原理：** 已知结构体成员指针和成员名，通过 `offsetof` 算出成员在结构体中的偏移，反推出结构体的起始地址。

```c
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
```

**在本项目 UART 中的用法：**

```c
// uart.h —— 用宏定义 HAL 类型，便于换平台
#define UART_HAL_TYPE  UART_HandleTypeDef

typedef struct UART_Handle {
    UART_HAL_TYPE *huart;                    // 不用 void*，直接用具体类型
    // ...
} UART_Handle;

// 回调里一行 container_of 代替映射表/地址差/if-else
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Handle *hUART = container_of(huart, UART_Handle, huart);
    ringbuf_write(g_ringbuf, hUART->rx_byte);
    hUART->ops->receive_IT(hUART->huart, &hUART->rx_byte, 1);
}
```

**优势与代价：**

| 优势 | 代价 |
|---|---|
| O(1) 定位，不遍历 | `huart` 成员必须是具体类型，不能用 `void *` |
| 不依赖地址连续性（任意内存位置都能反查） | `huart` 必须是结构体**第一个成员**（或至少偏移固定），否则类型不匹配会出问题 |
| 不需要映射表数组 | 宏定义的类型名跨平台时仍要改 |
| 零内存开销 | 依赖 `stddef.h` 的 `offsetof`（标准库，所有编译器都有） |

**本项目当前选型：** 保持 `void *huart` + 地址差单射。因为平台层 ops 函数已经在各处做强转，`void *` 的跨平台性未实际用上，但换方案需要同步修改 `uart_platform_ops.c`、`uart.c`、`uart.h` 三个文件，当前 UART 模块已稳定，不破坏现有代码。

---

## 二、回调机制（Callback）

### 本质

回调是独立于分层体系的"穿层通道"——代码写在上层文件里，执行时机在下层中断上下文中，同时访问上层和下层资源。

```
硬件中断 → HAL_UART_IRQHandler → [你的回调]
                                      │
                      应用层 ← ringbuf_write() ←┘
                      平台层 ← receive_IT()
```

### 特征

| 维度 | 说明 |
|---|---|
| **代码归属** | 写在上层文件（如 `uart_cmd_parser.c`） |
| **执行时机** | 在下层中断上下文 |
| **调用的对象** | 既调上层逻辑（ringbuf），也调下层函数（receive_IT） |
| **与分层的关系** | 不冲突：分层管"正常路径从上往下调"；回调管"硬件事件从下往上通知" |

### 使用原则

- 回调里**不做重操作**（不解析、不调 bridge 函数、不 printf）
- 回调里**只存数据、重新使能**（ringbuf_write + receive_IT 加起来 < 2μs）
- 回调不是"下层调上层"，是"框架留了空位，你填空"

---

## 三、分层模式（驱动三层分离）

### 架构

```
┌──────────────────────┐
│  策略层（跨平台通用）   │ ← pwm.c, uart.c, usergpio.c
│  组合逻辑 + 数学计算   │   内部调 hpwm->ops->xxx()
└──────────┬───────────┘
           │ ops 操作表
┌──────────▼───────────┐
│  平台实现层（每 MCU 一套）│ ← pwm_platform_ops.c
│  原子操作，直接读写寄存器 │   封装 HAL：start/stop/set_ccr
└──────────┬───────────┘
           │ HAL 库
┌──────────▼───────────┐
│  CubeMX 配置层          │ ← tim.c, usart.c
│  外设时钟/GPIO/NVIC     │   生成代码，不手动改
└────────────────────────┘
```

### 分离原则

| 层 | 做什么 | 不做什么 |
|---|---|---|
| 策略层 | 计算、组合、调用 ops | 不碰寄存器、不包含平台宏 |
| 平台层 | 读/写硬件寄存器 | 不掺业务逻辑 |
| 配置层 | 初始化外设时钟/引脚/中断 | 不被手动修改（CubeMX 重新生成会覆盖） |

### ops 函数指针的约定

ops 函数的第一个参数名与句柄结构体中存储平台句柄的成员名保持一致：

```c
// 约定：handle->ops->xxx(handle->yyy)
// hpwm->ops->start(hpwm->htim, ...)   → 参数 void *htim = hpwm->htim
// hUART->ops->send(hUART->huart, ...)  → 参数 void *huart = hUART->huart
// hGPIO->ops->write(hGPIO->hgpio_port, ...) → 参数 void *hgpio_port = hGPIO->hgpio_port
```

---

## 四、形参命名约定

### 规则

所有驱动句柄的函数参数统一为 `h` + 首字母大写的缩写：

| 驱动 | 句柄类型 | 形参名 |
|---|---|---|
| UART | `UART_Handle` | `hUART` |
| PWM | `PWM_Handle` | `hPWM` |
| UserGPIO | `UserGPIO_Handle` | `hGPIO` |

### 目的

- 一眼区分"参数指针"和"成员名"（`hUART->huart` 不会混淆）
- 所有驱动参数命名风格统一
- ops 函数指针参数的命名与句柄成员名保持一致

---

## 五、环形缓冲区（Ring Buffer）

### 适用场景

中断（生产者）与主循环（消费者）之间的数据通道。解决"不知道数据何时来"的事件驱动问题和"主循环不能阻塞等待"的矛盾。

### 为什么用数组而不是链表

| 维度 | 环形数组 | 链表 |
|---|---|---|
| **内存碎片** | 编译时定好 buf[128] = 132 字节 | malloc 有碎片 |
| **中断安全** | 只写一个数组位置 | 调 malloc 是大忌 |
| **开销** | 多 2 个 uint8_t 索引（volatile head + tail） | 每节点多 2 个指针 |

### 无锁前提

- 单生产者（中断）单消费者（主循环）
- head 只归中断写（volatile uint8_t）
- tail 只归主循环读
- uint8_t 索引 — 单字节原子操作，天然无锁（RINGBUF_SIZE ≤ 255）
- 不需要关中断、不需要 spinlock

### 空/满判断

```
空：head == tail
满：(head + 1) % SIZE == tail  ← 牺牲一个元素区分
满时策略：丢弃新数据（不丢已有的）

索引溢出：head/tail 在 0~127 之间自动回绕，SIZE=128, uint8_t 最大 255 够用
```

### 归属

环形缓冲区是**纯软件数据结构**，不是驱动。放在 `common/` 目录，不与操作寄存器的 `driver/` 混放。

---

## 六、二进制枚举协议

### 设计思路

```
数据天生是二进制的，人不需要看的时候不转为文字。
人要看的时候，在 PC 侧查映射表显示。
```

### 帧结构

```
PC → STM32（命令帧）：
  0xAA | cmd | [id] | [参数...] | 0xFF | 0xFF（两字节帧尾）

STM32 → PC（响应帧）：
  0xAA | cmd | id | status | 0xFF | 0xFF（固定 6 字节）

帧头帧尾统一：0xAA / 0xFF 0xFF
使用两字节帧尾减少参数中偶然出现 0xFF 误触发帧尾的概率。
所有字段都是枚举/二进制值，无 ASCII 字符串，无 sscanf/sprintf。
```

### 对比 ASCII 字符串

| 维度 | ASCII 字符串 | 二进制枚举 |
|---|---|---|
| **指令密度** | "M0S100\n" = 7 字节 | AA 01 00 00 42 FF FF = 7 字节（含帧头尾） |
| **MCU 解析** | sscanf 解析字符串 | switch 跳枚举值 |
| **人可读性** | 串口助手直接显示 | 需上位机查表显示 |

### 适用场景

MCU 与 PC/上位机之间的协议通信。不适合通用串口助手直接打字的调试场景——调试阶段可以先用 ASCII，产品化再切到二进制，上层 bridge 接口完全不变。

---

## 七、中断最小化原则

### 原则

中断里只做"绝对必要且最快"的事，任何非紧急逻辑延迟到主循环处理。

### 中断里做的事

| 操作 | 时间 | 原因 |
|---|---|---|
| ringbuf_write | < 1μs | 必须马上存，否则下一个字节会覆盖 |
| receive_IT 重新使能 | < 1μs | 必须在下一个字节来之前准备好 |
| 以上两行加起来 | < 2μs | 系统无影响 |

### 中断里不做的事

| 操作 | 原因 |
|---|---|
| 帧解析、命令分发 | 耗时不确定，可能卡住其他中断 |
| 调 bridge 函数 | 可能触发嵌套中断导致死锁 |
| printf / 日志输出 | 同步等待 TX 完成，阻塞整个中断 |
| malloc / free | 不可重入，堆碎片 |

---

## 八、"单例先行，扩展后抽"原则

### 原则

```
先硬编码（单例），后抽象（多实例）。
不要为"可能"发生的未来搭结构。
```

### 当前项目体现

| 模块 | 当前（单例） | 未来（多实例） |
|---|---|---|
| UART | `g_uart` / `g_ringbuf` | `UART_Instance[]` 结构体数组 |
| Motor | `motor_bridge_init(id, ...)` | 已用 id 索引数组，已是多实例 |
| Servo | `servo_bridge_init(id, ...)` | 已用 id 索引数组 |
| UART 映射 | `uart_by_id[]` / `ringbuf_by_id[]` | 扩展数组大小即可 |

### 何时抽象

当**第二条实际需求出现**时才抽象，而不是"将来可能要用"。三条标准有一条满足就可以考虑：
1. 同一个 .c 文件里出现了第二个实例的声明
2. 出现了 `if (实例A) ... else if (实例B)` 的硬编码分支
3. 代码中出现复制粘贴并修改的迹象

---

## 九、三件套：结构体指针 + ops 表 + 对象池

### 总览

```
┌──────────────────────────────────────────────────────────────────┐
│  三件套是当前工程中所有驱动（PWM/UART/GPIO/Motor/Servo）共享的骨架 │
│                                                                  │
│  结构体指针 = C 层的手搓对象      （句柄 = 成员 + 操作表）        │
│  ops 表     = C 层的手搓虚表      （函数指针 = 多态）             │
│  对象池     = 全局指针数组 + id   （工厂 + 索引管理）             │
└──────────────────────────────────────────────────────────────────┘
```

### 9.1 结构体指针 — C 层的手搓对象

每个驱动定义一个句柄结构体，内含"硬件句柄 + 操作表指针 + 状态字段"：

```c
typedef struct PWM_Handle {
    void *htim;                     // 硬件句柄（C 层的"数据成员"）
    PWM_PlatformOps_t *ops;         // 操作表（C 层的"虚表"）
    uint32_t Channel;               // 状态字段
    PWM_Ch_State Ch_State;          // 状态字段
    // ...
} PWM_Handle;
```

所有 API 函数的第一个参数都是 `XXX_Handle *`——等价于 C++ 中的 `this` 指针：

```c
// C++:  hpwm->set_duty(500)     ← this 隐式传递
// C:    pwm_set_duty_0E3(hpwm, 500)  ← this 显式传递
```

`hPWM->ops->xxx(hPWM->htim, ...)` 的结构就是"对象.虚表.方法(数据成员)"在 C 中的等价写法。

### 9.2 ops 表 — C 层的手搓虚表

ops 表是一组函数指针，声明在 `xxx.h`，实例在 `xxx_platform_ops.c`：

```c
// 抽象层声明接口
typedef struct PWM_PlatformOps_t {
    void  (*start)(void *htim, uint32_t ch);
    void  (*stop)(void *htim, uint32_t ch);
    uint32_t (*get_arr)(void *htim);
    void  (*set_ccr)(void *htim, uint32_t ch, uint32_t ccr);
} PWM_PlatformOps_t;

// 平台层提供实现
const PWM_PlatformOps_t pwm_platform_ops_stm32 = {
    .start   = pwm_stm32_start,
    .stop    = pwm_stm32_stop,
    .get_arr = pwm_stm32_get_arr,
    .set_ccr = pwm_stm32_set_ccr,
};
```

**对比 C++ 虚函数：**

```
C++:  class PWM { virtual void start(); };  → vtable 由编译器自动构建
C:    struct PWM_Handle { ops->start };      → vtable 由开发者手动构建
```

两者本质一致：**通过函数指针表实现"同一份接口、多套实现"的多态**。换平台只需更换 `xxx_platform_ops_yyy` 实例，策略层代码零修改。

### 9.3 对象池 — 同类型结构体的统一管理

用 `static TYPE *pool[MAX] + id 索引` 来管理多个同类型实例：

```
┌───────────┐
│  pool[0]  │──→ XXXProtocol 对象 A
├───────────┤
│  pool[1]  │──→ XXXProtocol 对象 B
├───────────┤
│  pool[2]  │──→ nullptr（未初始化）
├───────────┤
│  ...      │
└───────────┘
        ↑
    id = 索引
```

**本项目中的三个实例：**

```c
// Motor 桥接层
static TB6612MotorProtocol *tb6612_proto[MAX_MOTORS] = {nullptr};
static Motor *motor[MAX_MOTORS] = {nullptr};

// Servo 桥接层
static PWMServoProtocol *pwm_servo_protoc[MAX_SERVOS] = {nullptr};
static Servo* servo[MAX_SERVOS] = {nullptr};

// UART 实例映射
static UART_Instance uart_instances[UART_INSTANCE_MAX];
typedef struct {
    UART_Handle *handle;
    RingBuffer  *ringbuf;
} UART_Instance;
```

**通用模板：**

```c
#define MAX_INSTANCES  8

/* 对象池声明 */
static XxxProtocol *g_protos[MAX_INSTANCES] = {nullptr};

/* 工厂函数（初始化） */
void xxx_bridge_init(uint8_t id, ...)
{
    g_protos[id] = new XxxProtocol(...);
}

/* 使用方法函数 */
void xxx_bridge_do_something(uint8_t id, ...)
{
    if (g_protos[id] == nullptr) return;  // 空安全保护
    g_protos[id]->do_something(...);
}
```

对象池非桥接层独有，UART 的 `uart_instances[]` 同样是"利用 id 索引的路由表"——只是存的是结构体而非指针。模式一致。

### 9.4 三件套的关系

```
             策略层 API                        桥接层
      ┌──────────────────────┐        ┌──────────────────────┐
      │  pwm_set_duty_0E3()  │        │  motor_bridge_xxx()  │
      │  uart_send()         │        │  servo_bridge_yyy()  │
      │  usergpio_write()    │        └──────┬───────────────┘
      └────────┬─────────────┘               │ id 索引
               │ ops 表 (虚表)                ▼
               │                             对象池
     ┌─────────▼────────────┐        ┌────────┴────────┐
     │  平台实现层          │        │  Protos[8]      │
     │  stm32/  gd32/  ch32 │        │  Motors[8]      │
     └──────────────────────┘        └─────────────────┘
            ↑                              ↑
       结构体指针驱动层              结构体指针功能层
       (PWM_Handle* → 寄存器)      (Motor* → 逻辑计算)
```

- 结构体指针是 C 层模拟对象的基本手段
- ops 表让同一套策略代码适配不同硬件
- 对象池让多个同类型实例通过 id 统一管理

**新加一个驱动时，三件套一起上。**
