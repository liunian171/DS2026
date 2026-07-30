# 串口通信设计

> 有线串口（UART）驱动抽象层 + 基于枚举的二进制命令协议
> 对标准 PWM 驱动层的三层分离模式，融入中断机制，为闭环控制铺路

---

## 目录

- [一、核心思路](#一核心思路)
  - [1.1 串口在系统中的角色](#11-串口在系统中的角色)
  - [1.2 为什么选择二进制枚举协议而不是 ASCII 字符串](#12-为什么选择二进制枚举协议而不是-ascii-字符串)
  - [1.3 协议约定：枚举定义](#13-协议约定枚举定义)
  - [1.4 帧结构](#14-帧结构)
  - [1.5 指令密度示意](#15-指令密度示意)
- [二、整体架构](#二整体架构)
  - [2.1 四层分离](#21-四层分离)
  - [2.2 核心原则：数据和文字分离](#22-核心原则数据和文字分离)
- [三、UART 驱动抽象层](#三uart-驱动抽象层新建设计)
  - [3.1 与已有需求的区别：为什么需要中断](#31-与已有需求的区别为什么需要中断)
  - [3.2 数据结构（对标 PWM_Handle 模式）](#32-数据结构对标-pwm_handle-模式)
  - [3.3 策略层 API](#33-策略层-api)
- [四、环形缓冲区](#四环形缓冲区新建设计)
  - [4.1 为什么要加这一层](#41-为什么要加这一层)
  - [4.2 为什么是数组而不是链表](#42-为什么是数组而不是链表)
  - [4.3 读写指针分离 = 无锁设计](#43-读写指针分离--无锁设计)
  - [4.4 缓冲区满的处理策略](#44-缓冲区满的处理策略)
- [五、命令解析层](#五命令解析层cmd_parser)
  - [5.1 作用](#51-作用)
  - [5.2 内部状态机](#52-内部状态机基于帧头帧尾定界不是换行符)
  - [5.3 核心函数](#53-核心函数)
  - [5.4 cmd_parser_tick 内部逻辑](#54-cmd_parser_tick-内部逻辑)
  - [5.5 命令分发](#55-命令分发)
  - [5.6 新增命令只需 2 步](#56-新增命令只需-2-步)
- [六、中断回调挂接](#六中断回调挂接)
  - [6.1 回调做了什么](#61-回调做了什么)
  - [6.2 回调的本质——独立于分层的穿层通道](#62-回调的本质独立于分层的穿层通道)
  - [6.3 重新使能 receive_IT 是否在中断里](#63-重新使能-receive_it-是否在中断里)
- [七、有线与无线串口](#七有线与无线串口对-stm32-透明)
- [八、与现有驱动的关系](#八与现有驱动的关系)
- [九、文件清单](#九文件清单)
- [十、使用示例](#十使用示例)
  - [10.1 STM32 侧 main.c](#101-stm32-侧-mainc)
  - [10.2 PC 侧 Python 上位机](#102-pc-侧-python-上位机示意)
- [十一、设计原则总结](#十一设计原则总结)
- [十二、扩展性](#十二扩展性)
- [十三、关键设计决策速查](#十三关键设计决策速查)
- [十四、实现计划](#十四实现计划)

---

## 一、核心思路

### 1.1 串口在系统中的角色

```
PC ──串口线──→ STM32 ──→ 电机转 / 舵机动

串口 = 耳朵 + 嘴巴
cmd_parser = 翻译官（把字节翻译成动作）
motor_bridge / servo_bridge = 执行者（调驱动操作寄存器）
```

串口本身只是管道的角色——它把一个 byte 数组从 A 运到 B，不在乎里面装的内容是 ASCII 字符还是二进制枚举值。换成蓝牙模块（HC-05/JDY-31），STM32 侧代码零修改，因为它看到的仍然是 UART 外设的那两个引脚。

### 1.2 为什么选择二进制枚举协议而不是 ASCII 字符串

两种方案本质都是在传 byte，区别在于**指令密度**：

```
字符串方案（7 字节）：
  'M'  '0'  'S'  '1'  '0'  '0'  '\n'
   一个 byte 只表达一个字符 → 需要多个字符拼出语义

枚举方案（4 字节）：
  0xAA  0x01  0x00  0x64
   一个 byte 就能表达一条完整指令

指令密度差 2 倍以上。
```

ASCII 和枚举在 MCU 眼里没有本质区别——`'M'` 就是 `0x4D`，和 `CMD_MOTOR = 0x01` 一样都是 switch 一个 byte。区别在于：

| | 字符串 | 枚举 |
|---|---|---|
| **表达一条指令** | 多个字符拼语义，需要 sscanf 还原数值 | 一个枚举值直接定位到处理函数 |
| **参数还原** | `"100"` 三个字节 → sscanf 计算后得到 100 | `0x00 0x64` 直接是 100 |
| **密集度** | 128 个可打印字符，表达能力受限于字符集 | 256 个值全可用于命令编址 |
| **人可读性** | 串口助手直接看到文字 | 需要上位机把 0x01 映射为 "设置电机速度" |

结论：数据天生是二进制的，人不看的时候不应该转成字符串。人要看的时候，在 PC 侧查映射表显示即可。

### 1.3 协议约定：枚举定义

PC 和 STM32 共享同一份枚举定义。发送用命令枚举，收到用状态枚举：

```c
// === 命令枚举（PC→STM32）===
typedef enum {
    CMD_NOP               = 0x00,

    // Motor 命令组（0x01~0x0F）
    CMD_MOTOR_SET_RPM     = 0x01,   // 参数：id + rpm(float,4字节)
    CMD_MOTOR_SET_MPS     = 0x02,   // 参数：id + mps(float,4字节)
    CMD_MOTOR_BRAKE       = 0x03,   // 参数：id
    CMD_MOTOR_SET_DEADZ   = 0x04,   // 参数：id + rpm(float,4字节)

    // Servo 命令组（0x10~0x1F）
    CMD_SERVO_SET_ANGLE   = 0x10,   // 参数：id + angle(float,4字节)
    CMD_SERVO_START       = 0x11,   // 参数：id
    CMD_SERVO_STOP        = 0x12,   // 参数：id

    // PWM 命令组（0x20~0x2F）
    CMD_PWM_SET_DUTY      = 0x20,   // 参数：id + duty(uint16)

    // 系统命令
    CMD_PING              = 0xF0,   // 心跳检测，无参数
    CMD_GET_STATUS        = 0xF1,   // 获取状态，无参数
} CmdCode_t;

// === 状态枚举（STM32→PC）===
typedef enum {
    STATUS_OK             = 0xA0,
    STATUS_ERROR          = 0xA1,
    STATUS_INVALID_CMD    = 0xA2,
    STATUS_INVALID_ID     = 0xA3,
    STATUS_INVALID_PARAM  = 0xA4,
    STATUS_BUSY           = 0xA5,
} StatusCode_t;
```

### 1.4 帧结构

```
PC → STM32（命令帧）：
┌──────┬──────┬────────┬──────────────┬────────────┐
│ 0xAA │ cmd  │  id    │  参数(0~4字节)│ 0xFF 0xFF  │
│ 帧头  │ 命令  │ 模块ID │ 长度由cmd决定  │ 双字节帧尾  │
└──────┴──────┴────────┴──────────────┴────────────┘

示例：
  电机0 转速100：0xAA 0x01 0x00 0x00 0x00 0xC8 0x42 0xFF 0xFF  ← float 100.0 (8字节)
  电机0 急停：    0xAA 0x03 0x00 0xFF 0xFF                     ← 4字节
  舵机1 角度90：  0xAA 0x10 0x01 0x00 0x00 0xB4 0x42 0xFF 0xFF  ← float 90.0 (8字节)
  心跳检测(PING)： 0xAA 0xF0 0xFF 0xFF                         ← 4字节(无id)

STM32 → PC（响应帧，与命令帧同格式、第2字节=原cmd）：
┌──────┬──────┬────────┬────────┬────────────┐
│ 0xAA │ cmd  │  id    │ status │ 0xFF 0xFF  │
│ 帧头  │ 命令  │ 模块ID │ 状态码  │ 双字节帧尾  │
└──────┴──────┴────────┴────────┴────────────┘

响应：
  成功(PING): 0xAA 0xF0 0xFF 0xA0 0xFF 0xFF         ← 6字节
  非法命令:    0xAA 0x02 0xFF 0xA2 0xFF 0xFF
```

### 1.5 指令密度示意

```
同一条指令的字节数：
  字符串 "M0S100\n"    = 7 字节（ID+子命令占了 2 字节，字符串数字 "100" 占 3 字节）
  枚举   0x01 0x00 + float(100.0) = 6 字节（cmd 就表达了"电机+设置速度"的完整语义，参数是 float 值）

字节利用率：
  字符串：7 字节 / 1 条指令 = 7
  枚举：  6 字节 / 1 条指令 = 6（帧头尾另算）
```

---

## 二、整体架构

### 2.1 四层分离

```
┌──────────────────────────────────────────────────────┐
│  主循环（main.c）                                      │
│    cmd_parser_tick() → 查二进制帧 → switch(cmd)       │
│                       → motor_bridge_set_speed_rpm()  │
└────────────────────┬─────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────┐
│  cmd_parser.h/.c（命令解析层，调用 bridge 接口）        │
│  职责：从环形缓冲区拿字节、拼二进制帧、switch 分发       │
└────────────────────┬─────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────┐
│  uart.h / uart_platform_ops.h/.c（UART 驱动抽象层）    │
│  职责：封装 HAL，提供 send/receive_IT，跨平台隔离       │
└────────────────────┬─────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────┐
│  硬件层：USART 外设 + HAL 库                           │
│  职责：自动串并转换，波特率生成，收到字节时触发中断       │
└──────────────────────────────────────────────────────┘
```

### 2.2 核心原则：数据和文字分离

```
┌── STM32 ──────────────── 串口线 ────────────── PC ──┐
│                                                      │
│  0xAA 0x01 0x00              ──── 只传二进制枚举 ──→  │
│  + float(100.0) 0xFF 0xFF         |                 │
│  0xAA 0xF0 0xFF 0xA0 0xFF 0xFF ←── 也是枚举 ─────── │
│                                                      │
│  STM32 永远不知道 "M" 和 "S" 是什么意义               │
│  PC 在需要时查映射表转成文字显示给用户                  │
└──────────────────────────────────────────────────────┘
```

---

## 三、UART 驱动抽象层（新建设计）

### 3.1 与已有需求的区别：为什么需要中断

之前的 PWM 和 GPIO 驱动全是**配置型**——写完寄存器，硬件自动运行，CPU 不需要再管。但串口是**事件驱动型**——不知道 PC 什么时候发数据。

```
配置型（PWM）：
  写 CCR → 寄存器硬件自动输出 → CPU 可以睡觉 → 不需要中断 ✓

事件驱动型（UART）：
  不知道数据何时到 → 不可能轮询着等（阻塞主循环）→ 必须中断 ✓

主循环阻塞在 HAL_UART_Receive(..., HAL_MAX_DELAY) 上
的后果：电机转一半卡住了，等字节来了再继续转 → 无法接受
```

### 3.2 数据结构（对标 PWM_Handle 模式）

```c
typedef struct UART_Handle {
    void *huart;                     // 平台句柄（UART_HandleTypeDef*）
    const UART_PlatformOps_t *ops;   // 平台操作表
    uint8_t  rx_byte;               // 中断接收临时落脚点（回调中写到此处）
} UART_Handle;

typedef struct {
    int8_t (*send)(void *huart, const uint8_t *data, uint16_t len);
    int8_t (*receive)(void *huart, uint8_t *data, uint16_t len);
    int8_t (*receive_IT)(void *huart, uint8_t *p_byte, uint16_t len);
} UART_PlatformOps_t;
```

### 3.3 策略层 API

```c
int8_t uart_send(UART_Handle *huart, const uint8_t *data, uint16_t len);
int8_t uart_receive(UART_Handle *huart, uint8_t *data, uint16_t len);
int8_t uart_receive_IT(UART_Handle *huart, uint8_t *p_byte);
int8_t uart_flush_rx(UART_Handle *huart);
```

---

## 四、环形缓冲区（新建设计）

### 4.1 为什么要加这一层

接收字节这件事必须是中断干的（不知道何时来），但解析和执行必须放在主循环里（中断不能花时间解析）。双方速度不匹配，需要一个桥梁来解耦：

```
           中断上下文（生产者）                    主循环上下文（消费者）
     ┌──────────────────────┐          ┌──────────────────────────────┐
     │ USART1_IRQHandler()  │          │ cmd_parser_tick()            │
     │   RxCpltCallback()   │          │   ringbuf_read() → 逐字节    │
     │     ringbuf_write()  │          │   组帧 → switch(cmd)         │
     └──────────┬───────────┘          │   motor_bridge_set_speed_rpm │
                │ 写入                 │   uart_send(STATUS_OK)       │
                ▼                      └──────────────┬───────────────┘
           ┌───────────┐                              │ 读取
           │ 环形缓冲区  │←─────────────────────────────┘
           │ (数组实现) │
           └───────────┘
```

没有这一层：中断收到的字节直接丢进主循环的局部变量里，主循环跑得慢，字节就被覆盖了。

### 4.2 为什么是数组而不是链表

链表的优势在"频繁在中间增删"（如 FreeRTOS 按优先级插拔任务），而这里的操作只有：

```
只在头部写入（中断）→ 只在尾部读取（主循环）→ FIFO

这就不是链表的用武之地。
```

| | 链表 | 环形数组 |
|---|---|---|
| **内存碎片** | malloc/free 有 | 编译时定好 `buf[128]`，零碎片 |
| **中断安全** | 中断里调 malloc 是大忌 | 中断里只写一个数组位置 |
| **指针开销** | 每节点多 2 个指针（数据本身才 1 byte） | 只多 2 个整数索引 |

### 4.3 读写指针分离 = 无锁设计

```
写指针 head  ← 只属于中断（写入时更新，volatile 修饰防编译器优化）
读指针 tail  ← 只属于主循环（读取时更新）

空：head == tail
满：(head + 1) % SIZE == tail（牺牲一个元素区分）

head 和 tail 各属于不同上下文，没有竞态，不需要关中断、不需要锁。
```

### 4.4 缓冲区满的处理策略

```c
void ringbuf_write(RingBuffer *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1) % RINGBUF_SIZE;
    if (next == rb->tail) {
        return;  // 满了，丢弃新数据
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
}
```

`SIZE = 128` 时，主循环只要不被阻塞超过 128 字节的接收时间（115200 波特率下 ≈ 11ms），缓冲区就不会满。实际场景中一条命令帧通常不超过 20 字节，11ms 完全足够主循环处理完剩下的任务。

---

## 五、命令解析层（cmd_parser）

### 5.1 作用

把环形缓冲区里连续的字节流恢复成二进制帧，识别出 cmd 值，switch 到位执行：

```
收到：0xAA 0x01 0x00 [float=100.0] 0xFF 0xFF

  ① 识别帧头 0xAA → 开始收帧
  ② 收到连续 0xFF 0xFF → 帧结束，提取 cmd = 0x01
  ③ switch (cmd)：
       case 0x01 → motor_bridge_set_speed_rpm(0, 100.0f)
       case 0x10 → servo_bridge_set_angle(id, angle)
       default   → uart_cmd_send_status(cmd, id, STATUS_INVALID_CMD)
  ④ uart_cmd_send_status(cmd, id, STATUS_OK)
```

### 5.2 内部状态机（基于帧头帧尾定界，不是换行符）

```
                    ┌──────────┐
          ringbuf空 │  等待帧头 │ ← 读到 0xAA → 进入收帧状态
         ──────────→│ WAIT_SOF │
                    └────┬─────┘
                         │ 0xAA
                         ▼
                    ┌──────────┐
            idx=0   │  收命令字节│ ← 逐字节填入帧缓冲区
                    │直到 0xFF  │
                    │  0xFF    │
                    └────┬─────┘
                         │ 0xFF 0xFF
                         ▼
                    ┌──────────────┐
                    │ switch(cmd)  │ → 执行
                    │ uart_send()  │ → 回状态码
                    └──────────────┘
```

### 5.3 核心函数

```c
void cmd_parser_init(UART_Handle *huart, RingBuffer *ringbuf);
void cmd_parser_tick(void);      // 主循环中每轮调用一次
```

### 5.4 cmd_parser_tick 内部逻辑

```c
void cmd_parser_tick(void)
{
    uint8_t byte;

    // 每 tick 取 1 字节（非 while），180MHz 下多轮间隔极短
    if (ringbuf_read(&g_rx_ringbuf, &byte) == 0)
    {
        if (byte == 0xAA && !g_in_frame)    // 帧头 → 开始新帧
        {
            g_rx_index = 0;
            g_in_frame = 1;
        }
        // 注意：fall through 到下方 if (g_in_frame) 存入 0xAA

        if (g_in_frame)                     // 收帧中
        {
            if (g_rx_index >= FRAME_RX_BUF_SIZE)  // 溢出保护
            {
                g_in_frame = 0;
                g_rx_index = 0;
                return;
            }

            g_rx_frame[g_rx_index++] = byte;

            if (byte == 0xFF && g_rx_index >= 2
                && g_rx_frame[g_rx_index - 2] == 0xFF)  // 双字节帧尾
            {
                cmd_dispatch(g_rx_frame, g_rx_index - 1);  // 末尾 FF 截掉
                g_in_frame = 0;
                g_rx_index = 0;
            }
        }
        // 非帧头也非帧内 → 丢弃
    }
}
```

### 5.5 命令分发

```c
static void cmd_dispatch(const uint8_t *frame, uint16_t len)
{
    uint8_t cmd  = frame[1];
    uint8_t id   = frame[2];

    float   val_f32;                                  // float 参数

    switch (cmd)
    {
    case CMD_MOTOR_SET_RPM:
        memcpy(&val_f32, &frame[3], 4);
        motor_bridge_set_speed_rpm(id, val_f32);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;

    case CMD_MOTOR_BRAKE:
        motor_bridge_brake(id);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;

    case CMD_SERVO_SET_ANGLE:
        memcpy(&val_f32, &frame[3], 4);
        servo_bridge_set_angle(id, val_f32);
        uart_cmd_send_status(cmd, id, STATUS_OK);
        break;

    case CMD_PING:
        uart_cmd_send_status(cmd, 0xFF, STATUS_OK);
        break;

    default:
        uart_cmd_send_status(cmd, 0xFF, STATUS_INVALID_CMD);
        break;
    }
}

// 响应封装（带 cmd + id，方便 PC 关联请求）
static void uart_cmd_send_status(CmdCode_t cmd, int8_t id, StatusCode_t status)
{
    uint8_t resp[] = {0xAA, cmd, id, status, 0xFF, 0xFF};
    uart_send(uart_instances[0].handle, resp, 6);
}
```

### 5.6 新增命令只需 2 步

```c
// 1. 在枚举中添加一个值
CMD_MOTOR_STOP  = 0x05,

// 2. 在 switch 中加一个 case
case CMD_MOTOR_STOP:
    motor_bridge_stop(id);
    uart_cmd_send_status(cmd, id, STATUS_OK);
    break;
```

switch-case 架构本身不变。这符合项目已有的开闭原则。

---

## 六、中断回调挂接

### 6.1 回调做了什么

中断做的事情极其简单——只往环形缓冲区写一个字节：

```c
// 利用 HAL 句柄地址差 O(1) 定位到对应 UART_Instance
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hal_huart)
{
    uint8_t       id    = handle_to_id(&UART_BASE_HANDLE, sizeof(UART_HAL_TYPE), hal_huart);
    UART_Instance *inst = &uart_instances[id];

    ringbuf_write(inst->ringbuf, inst->handle->rx_byte);  // < 1μs

    // 马上重新使能接收，保证连续收字节
    inst->handle->ops->receive_IT(inst->handle->huart,
                                  &inst->handle->rx_byte, 1);
}
```

中断不解析、不执行、不调 bridge。因为：

1. **中断时间不确定**——执行命令可能要几百微秒，中断堆栈撑不住
2. **调 bridge 触发的操作可能递归触发其他中断**——嵌套中断优先级不一致导致死锁
3. **ringbuf_write + receive_IT 各自 < 1μs**——加起来不超过 2μs，对系统无影响

### 6.2 回调的本质——独立于分层的"穿层通道"

正常函数调用是严格的分层路线：`应用层 → 策略层 → 平台层 → HAL → 硬件`。但回调走的是另一条路——硬件中断触发，HAL 框架在中断上下文中执行一个你填空的函数：

```
硬件 RXNE 中断 ──→ USART7_IRQHandler ──→ HAL_UART_IRQHandler ──→ [你的回调]
                                                                        │
                                                   应用层 ←── ringbuf_write() ←┘
                                                   平台层 ←── receive_IT()
```

回调**代码写在上层文件里（`uart_cmd_parser.c`），执行时机在下层（中断上下文），属于两端。**它既不是"下层调上层"也不是"上层调用下层"，而是一个双向的穿层接头：

| 方向 | 操作 | 归属 |
|---|---|---|
| 接收触发 | HAL 在中断里进入回调 | 执行时机 = 下层 |
| 写入数据 | `ringbuf_write()` | 访问的资源 = 上层 |
| 重新使能 | `g_uart->ops->receive_IT()` | 调用的对象 = 下层 |

**这是独立的挂钩机制，不属于分层体系的任何一层。** 分层管"正常路径怎么从上往下调"，回调管"硬件事件如何从下往上通知"。结合使用，各司其职。

**为什么 HAL 不把回调实现也放在下层？** HAL 声明了一个 `__weak` 的默认空函数——它在编译链接时被你的实现替换掉。HAL 不知道你填了 ringbuf 还是直接处理还是唤醒 RTOS 任务，它只管在自己约定的时机执行这个函数名。它不是"下层主动依赖上层"，而是"你填空，中断触发后自然就执行到了你的逻辑"。

### 6.3 重新使能 receive_IT 是否在中断里

```c
void HAL_UART_RxCpltCallback(...)
{
    ringbuf_write(...);                            // 运行时上下文：中断
    g_uart->ops->receive_IT(...);                  // 运行时上下文：中断
}
```

**是，receive_IT 在中断上下文里被重新调用。** 但这是安全的——`receive_IT` 只做一件事情：调用 `HAL_UART_Receive_IT(huart, p_byte, 1)` 配置寄存器使能下一次 RXNE 中断，耗时 < 1μs。如果在主循环中重新使能，就会产生"上一字节已到、下一字节中断还没使能"的窗口期，有可能丢字节。

---

## 七、有线与无线串口——对 STM32 透明

```
有线 CH340：  STM32 TX/RX ──铜线── CH340 ──USB── PC
无线 HC-05：  STM32 TX/RX ──铜线── HC-05 ──蓝牙── PC/手机

STM32 看到的始终是 UART 外设的两根引脚，
软 件 的 所 有 层（uart → ringbuf → cmd_parser）代码一 个 字 不 改。
```

---

## 八、与现有驱动的关系

```
串口模块不修改任何已有驱动，只是调用已有的 bridge 接口：

串口发来的 0x01  → cmd_parser → motor_bridge_set_speed_rpm()
                                       ↓
       已有的 TB6612MotorProtocol.set_speed_rate_0E3()
                                       ↓
       已有的 pwm_set_duty_0E3() + usergpio_write()
```

没有耦合，串口只是另一个调用入口，和 main.c 直接写 `motor_bridge_set_speed_rpm()` 完全等效。

---

## 九、文件清单

| 文件 | 路径 | 说明 | 新增 |
|---|---|---|---|
| `uart.h` | `Core/Inc/driver/` | UART 抽象层声明 | ✅ |
| `uart.c` | `Core/Src/driver/` | UART 策略层（转发到 ops） | ✅ |
| `uart_platform_ops.h` | `Core/Inc/driver/` | STM32 UART ops 声明 | ✅ |
| `uart_platform_ops.c` | `Core/Src/driver/` | STM32 UART 平台实现 | ✅ |
| `uart_cmd_parser.h` | `Core/Inc/driver/` | 命令解析层 + 共享枚举 | ✅ |
| `uart_cmd_parser.c` | `Core/Src/driver/` | 命令解析实现 + 中断回调 | ✅ |
| `ringbuf.h` | `Core/Inc/common/` | 环形缓冲区（纯软件，非驱动） | ✅ |
| `ringbuf.c` | `Core/Src/common/` | 环形缓冲区实现 | ✅ |
| `usart.c` / `usart.h` | `Core/Src/` / `Core/Inc/` | CubeMX 生成的 HAL 配置层（UART7） | ✅ |

> **文件归属说明**：`driver/` 目录 = 操作硬件寄存器的驱动（uart/pwm/gpio/协议适配）；`common/` 目录 = 纯软件组件（环形缓冲区/tool 工具函数）。环形缓冲区是数据结构，不是驱动，因此放在 `common/` 中。

---

## 十、使用示例

### 10.1 STM32 侧 main.c

```c
#include "driver/uart.h"
#include "driver/uart_cmd_parser.h"
#include "driver/motor_bridge.h"
#include "driver/servo_bridge.h"
#include "usart.h"                       // CubeMX 生成，提供 huart7

static UART_Handle uart_debug = {
    .huart = &huart7,                    // UART7, PE7(RX)/PE8(TX)
    .ops   = &uart_platform_ops_stm32,
};

static RingBuffer g_ringbuf_debug;

void main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM5_Init();
    MX_UART7_Init();

    // 启动串口
    ringbuf_init(&g_ringbuf_debug);
    uart_cmd_parser_init(&uart_debug, &g_ringbuf_debug);
    uart_receive_IT(&uart_debug, &uart_debug.rx_byte);

    // 可选: 初始化电机/舵机桥接实例
    // motor_bridge_init(0, &pwm_tim5_ch4, &in1, &in2, &stby, 200.0f, 33.1f);

    while (1)
    {
        uart_cmd_parser_tick();   // ← 每轮处理串口命令
    }
}
```

### 10.2 PC 侧 Python 上位机（示意）

```python
import serial
import struct

ser = serial.Serial('COM3', 115200)

# PING 心跳检测（验证链路通畅）
ser.write(bytes([0xAA, 0xF0, 0xFF, 0xFF]))
resp = ser.read(6)
# 响应: 0xAA 0xF0 0xFF 0xA0 0xFF 0xFF
print("链路正常" if resp[1] == 0xF0 and resp[3] == 0xA0 else "无响应")

# 电机 0 转 100 RPM
rpm = struct.pack('<f', 100.0)
frame = bytes([0xAA, 0x01, 0x00]) + rpm + bytes([0xFF, 0xFF])
ser.write(frame)
resp = ser.read(6)
# 响应: 0xAA 0x01 0x00 0xA0 0xFF 0xFF
print("OK" if resp[3] == 0xA0 else f"ERR: {resp[3]}")

# 舵机 1 转到 90°
angle = struct.pack('<f', 90.0)
frame = bytes([0xAA, 0x10, 0x01]) + angle + bytes([0xFF, 0xFF])
ser.write(frame)
resp = ser.read(6)
# 响应: 0xAA 0x10 0x01 0xA0 0xFF 0xFF
print("OK" if resp[3] == 0xA0 else f"ERR: {resp[3]}")
```

---

## 十一、设计原则总结

| 原则 | 体现 |
|---|---|
| **数据与文字分离** | 收发全部用枚举，数据天生二进制，人不需要看就不转为文字。PC 侧在需要时查映射表显示 |
| **中断最小化** | 中断只往 ringbuf 写一个字节 + 重新使能接收就返回，不解析不执行。两行代码 < 2μs |
| **回调 = 穿层接头** | 回调不属于分层体系的任何一层——代码在上层文件里，执行时机在下层中断中。分层管"正常路径从上往下调"，回调管"硬件事件从下往上通知"，各司其职 |
| **无锁解耦** | 环形缓冲区的 head/tail 各属一侧，自然消除竞态 |
| **平台隔离** | UART 驱动层仿 pwm.c 的 ops 表模式，换平台只换 ops |
| **命名一致性** | 所有驱动句柄形参统一为 h 前缀 + 大写缩写：`UART_Handle *hUART`、`PWM_Handle *hPWM`、`UserGPIO_Handle *hGPIO` |
| **模块独立** | 串口模块只调 bridge 接口，不侵入任何已有驱动 |
| **传输层无关** | 有线/无线对软件完全透明，更换模块时代码零修改 |

---

## 十二、扩展性

| 扩展方向 | 变化点 | 不变点 |
|---|---|---|
| 无线模块（蓝牙/WiFi） | 硬件接线：CH340 换成 HC-05 | STM32 全部代码 |
| DMA 发送（高波特率大批量数据回传） | uart_platform_ops 加 send_DMA | 上层接口不变 |
| 闭环控制（编码器+PID） | 新增 CMD_MOTOR_PID 枚举值 | switch-case 架构 |
| 二进制协议升级（Modbus） | cmd_parser 内部实现替换 | 外部接口不变 |
| 命令缓冲队列（指令速率 > 处理速率） | cmd_parser 前加一层队列 | 桥接层以下不变 |

---

## 十三、关键设计决策速查

| 决策点 | 选择 | 理由 |
|---|---|---|
| 二进制帧 vs ASCII 字符串 | 二进制枚举帧 | 指令密度高，MCU 解析只需一次 switch |
| 帧定界方式 | 0xAA / 0xFF 0xFF 头尾 | 双字节帧尾减少参数值误触发 |
| 响应帧格式 | 6 字节含 cmd+id | PC 可关联请求，方便异步匹配 |
| 环形缓冲区实现 | 数组，非链表 | 无 malloc，中断安全，无锁 |
| 环形缓冲区归属 | common/ 非 driver/ | 纯软件数据结构，不操作寄存器 |
| DMA | 当前不用 | 帧短、频率低，中断开销可忽略 |
| 回调 = 穿层接头 | 代码在上层文件，执行在中断里 | 分层管路径，回调管通知，各司其职 |
| 多 UART 实例映射 | 地址差 O(1) 查表 | HAL 句柄 .bss 连续排列，无需哈希 |

---

## 十四、实现计划

| 阶段 | 内容 | 前置 |
|---|---|---|
| **P0** | CubeMX 配置 UART7 — PE7(RX)/PE8(TX)，115200 8N1 | 无 |
| **P1** | `ringbuf.h/.c` — 环形缓冲区（`Core/common/`） | 无 |
| **P2** | `uart.h` / `uart_platform_ops.h/.c` — UART 驱动抽象层 | P0 |
| **P3** | `cmd_parser.h` — 共享枚举定义 + 解析层接口声明 | 无 |
| **P4** | `cmd_parser.c` — 解析实现 + 中断回调挂接 | P1+P2+P3 |
| **P5** | main.c 接入 + Python 上位机验证 | P4 |
