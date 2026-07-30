# 软件 I2C 驱动设计

> 基于 GPIO 模拟 I2C 协议，遵循项目"策略层 + ops 平台层 + HAL 配置层"三层分离模式。
>
> 本文档侧重设计原理和时序分析，代码仅作示意。

---

## 命名约定与函数速查

| 名字 | 类别 | 含义 |
|------|------|------|
| `delay_us(N)` | 阻塞延时 | CPU 空循环 N 微秒，阻塞版的核心问题所在 |
| `scl_set()` / `gpio_write()` | GPIO 操作 | 设置 SCL/SDA 电平（概念函数，实际用 usergpio 的 ops 表） |
| `i2c_write_reg_async()` | 策略层 API | 异步写 I2C 设备寄存器，填入参数后立即返回（非阻塞） |
| `i2c_read_reg_async()` | 策略层 API | 异步读 I2C 设备寄存器 |
| `ops->start_xfer()` | 平台层 ops | 启动一次完整 I2C 传输的底层入口，填入参数后开定时器 ISR |
| `busy()` | 状态查询 | 返回当前句柄是否正在传输，防止同一句柄被并发启动两次 |
| `done_cb()` | 完成回调 | 传输完成后 ISR 自动调用的通知函数，参数为句柄和结果码 |
| `i2c_software_timer_isr()` | ISR 入口 | 定时器 ISR 分派函数，读 macro_state 决定走 fsm_edge 还是 fsm_bit |
| `i2c_sw_fsm_edge()` | 边沿子 FSM | 处理 I2C START/STOP 条件（SCL=1 时 SDA 跳变），3 个 tick |
| `i2c_sw_fsm_bit()` | 逐 bit FSM | 每 tick 执行一次 PREPARE/SAMPLE 交替，翻 GPIO |
| `i2c_sw_fsm_byte()` | 逐字节 FSM | 由 fsm_bit 在 SAMPLE 末尾调用，负责字节调度和事务推进 |
| `cmd_parser_tick()` | 应用示例 | UART 命令解析器的逐字节消费函数（作为 ISR 频率对比） |
| `HAL_I2C_Mem_xxx()` | HAL 硬件 I2C | STM32 HAL 库的硬件 I2C 内存读写系列函数（`Mem_Write` / `Mem_Read` 等），作为软件方案的对比 |

---

## 目录

- [一、核心问题：阻塞 → 非阻塞](#一核心问题阻塞--非阻塞)
- [二、整体架构](#二整体架构)
- [二点五、函数调用关系一览](#二点五函数调用关系一览)
  - [2.5A 分层与调用链](#25a-分层与调用链)
  - [2.5B 一次写事务的完整调用时序](#25b-一次写事务的完整调用时序)
  - [2.5C 函数间数据流](#25c-函数间数据流)
  - [2.5D 函数关系速查表](#25d-函数关系速查表)
  - [2.5E 一句话记忆](#25e-一句话记忆)
- [三、数据结构](#三数据结构)
- [四、时序核心：2-tick 模型](#四时序核心2-tick-模型)
  - [4.1 什么是 2-tick](#41-什么是-2-tick)
  - [4.2 发送 1 bit 的过程](#42-发送-1-bit-的过程)
  - [4.3 接收 1 bit 的过程](#43-接收-1-bit-的过程)
  - [4.4 完整发送 1 字节（8 bit + ACK）](#44-完整发送-1-字节8-bit--ack)
  - [4.5 完整接收 1 字节（8 bit + ACK）](#45-完整接收-1-字节8-bit--ack)
  - [4.6 START 和 STOP —— 为什么不能走 2-tick](#46-start-和-stop--为什么不能走-2-tick)
- [五、平台层（ops）设计](#五平台层ops设计)
  - [5.1 抽象 ops 与软/硬件 ops 的关系](#51-抽象-ops-与软硬件-ops-的关系★接口契约)
- [六、策略层函数](#六策略层函数)
- [七、非阻塞实现：定时器 + 双层状态机](#七非阻塞实现定时器--双层状态机)
  - [7.1 定时器如何替代 delay_us](#71-定时器如何替代-delay_us)
  - [7.2 双层状态机的分工](#72-双层状态机的分工)
  - [7.3 两层如何通信](#73-两层如何通信)
  - [7.4 微状态机 —— 每个 ISR 只做 1 步](#74-微状态机--每个-isr-只做-1-步)
  - [7.4 START 和 STOP 的独立子 FSM](#74-start-和-stop-的独立子-fsm)
  - [7.5 宏状态机 —— 跨字节调度](#75-宏状态机--跨字节调度)
  - [7.6 CPU 占用分析](#76-cpu-占用分析)
- [八、与 UART 非阻塞的对照](#八与-uart-非阻塞的对照)
- [九、文件清单](#九文件清单)
- [十、已知限制与待办](#十已知限制与待办)

---

## 一、核心问题：阻塞 → 非阻塞

### 1.1 阻塞版的根本问题

软件 I2C 的本质是用 CPU 逐 bit 翻转两根 GPIO（SCL 和 SDA）。阻塞版每翻一次就 `delay_us()` 死等，CPU 全程陪跑：

```
scl_set(0) → delay_us(5) → scl_set(1) → delay_us(5) → ...
   ↑                           ↑
  GPIO 翻转只需 0.1μs       CPU 白白空等 5μs（占 99% 的时间）
```

以 100kHz I2C 发 2 字节为例：全程 ~380μs，CPU 在此期间**完全不能执行其他任务**。主循环被冻结，PID 控制、UART 解析、姿态计算全停。

### 1.2 非阻塞的思路

用硬件定时器替代 `delay_us`。定时器设成 200kHz（5μs 周期），每次中断干 **1 个 GPIO 原子动作 + 1~2 次状态更新**，耗时 <2μs，然后 CPU 立即退出 ISR 回到主循环。定时器下次自动触发，不需要 CPU 干预。

```
阻塞版：CPU 干完 GPIO → 空等 5μs → 干下一步 GPIO → 空等 5μs → ...
非阻塞：TIM_ISR 干 1 步 GPIO（<2μs） → CPU 回主循环跑 3μs → TIM 再触发 → ...
```

**代价：** 100kHz 下 CPU 负载上升到 ~40%（200000 次 ISR/秒 × 2μs/次）。频率越低负载越小（10kHz→4%），如需零负载必须用硬件 I2C 外设。

---

## 二、整体架构

```
                 ┌──────────────────────────────────────┐
                 │  策略层  useri2c.c                    │
                 │  职责：组装事务 → 启动 → 等回调       │
                 └──────────────┬───────────────────────┘
                                │ start_transfer(dev, reg, buf, len, cb)
                    ┌───────────┴───────────┐
                    │                       │
         ┌──────────▼──────────┐  ┌─────────▼─────────┐
         │  软件 I2C 平台实现   │  │  硬件 I2C 平台实现  │
         │  (双层状态机 + ISR)  │  │  (HAL_I2C_Mem_xxx) │
         └─────────────────────┘  └─────────────────────┘
                    │
         ┌──────────▼──────────┐
         │  usergpio_platform.c │  ← 换平台只换此处
         └─────────────────────┘
                    │
         ┌──────────▼──────────┐
         │  配置层（CubeMX）     │
         │  gpio.c + tim.c      │
         └─────────────────────┘
```

**核心解耦手段：**

| 机制 | 作用 |
|------|------|
| `UserGPIO_Handle` | GPIO 操作通过 usergpio 转发，换平台只换 usergpio_ops |
| `void *i2c_context` | 平台资源指针，SW 指向 `I2C_SoftwareContext*`，HW 指向 `I2C_HandleTypeDef*` |
| `I2C_PlatformOps_t *ops` | 协议级操作表，软件/硬件实现互换 |
| **硬件定时器 ISR** | 替代 `delay_us()` 死等，CPU 不阻塞 |

---

## 二点五、函数调用关系一览

### 2.5A 分层与调用链

```
应用层（main.c / 其他模块）
  │
  │  调用公开 API：i2c_write_reg_async() / i2c_read_reg_async()
  ▼
┌─────────────────────────────────────────────────────────────┐
│  useri2c.c（策略层）                                         │
│                                                             │
│  i2c_write_reg_async(handle, dev, reg, data, len, cb)       │
│  i2c_read_reg_async(handle, dev, reg, buf, len, cb)         │
│  i2c_is_busy(handle)                                        │
│                                                             │
│  职责：参数校验 → 委托 ops 表 → 返回                         │
│       不碰 GPIO、不碰定时器、不碰 FSM                        │
└──────────────┬──────────────────────────────────────────────┘
               │
               │  handle->ops->start_transfer(handle->i2c_context, ...)
               │  handle->ops->is_busy(handle->i2c_context)
               │
               ▼
┌─────────────────────────────────────────────────────────────┐
│  I2C_PlatformOps_t  ops 表（抽象接口层）                     │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  int8_t  (*start_transfer)(void *ctx, ...)          │    │
│  │  uint8_t (*is_busy)(void *ctx)                      │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  同一个 ops 表可指向不同实现：                                │
│    软件 I2C → i2c_software_platform_ops_stm32                │
│    硬件 I2C → i2c_hardware_platform_ops_stm32（未来）        │
│                                                             │
│  核心技巧：void *ctx 多态 — 软 I2C 传 I2C_SoftwareContext*   │
│                           硬 I2C 传 I2C_HandleTypeDef*       │
└──────────────┬──────────────────────────────────────────────┘
               │  ops 实例指向具体实现
               │
               ▼
┌─────────────────────────────────────────────────────────────┐
│  useri2c_ops.c（平台层 — 软件 I2C 实现）                      │
│                                                             │
│  ★ 填入 ops 表的函数：                                       │
│    i2c_software_start_transfer()  ← 填入参数 + 开定时器      │
│    i2c_software_is_busy()         ← 查 macro_state           │
│                                                             │
│  ★ ISR 入口（由硬件中断调用，不经过 ops 表）：                │
│    i2c_software_timer_isr()       ← 每 4μs 被 TIM6 调用      │
│      ├─ macro_state==START/STOP → i2c_sw_fsm_edge()         │
│      └─ macro_state==SEND/RECV  → i2c_sw_fsm_bit()          │
│           └─ SAMPLE 末尾 → i2c_sw_fsm_byte()                │
│                                                             │
│  ★ 对内（static helper）：                                   │
│    i2c_sw_scl_set() / i2c_sw_sda_set() / i2c_sw_sda_read() │
│    i2c_sw_timer_start() / i2c_sw_timer_stop()              │
│    i2c_sw_fsm_edge()              ← 边沿信号（START/STOP）   │
│    i2c_sw_fsm_bit()               ← 逐 bit 微状态机          │
│    i2c_sw_fsm_byte()              ← 逐字节宏状态机           │
│                                                             │
│  ★ ops 表实例（编译期绑定）：                                 │
│    const I2C_PlatformOps_t i2c_software_platform_ops_stm32   │
│      = { start_transfer, is_busy }                          │
└──────────────────────────────────────────────────────────────┘
```

**关键区分**：

| 调用方式 | 走 ops 表？ | 说明 |
|----------|------------|------|
| `start_transfer` / `is_busy` | ✅ 走 ops | 策略层→平台层的唯一通道，软/硬件实现共用接口 |
| `i2c_software_timer_isr` | ❌ 不走 ops | ISR 由硬件中断直调平台层，不经过策略层和 ops 表 |

```
[硬件] ─→ TIM6_DAC_IRQHandler ─→ i2c_software_timer_isr()  ← 直接调用
[应用] ─→ i2c_write_reg_async  ─→ ops → start_transfer()    ← 通过 ops
```

三个 FSM 分层关系：
```
timer_isr（入口）
  ├─ fsm_edge   ← 边沿级（START/STOP），3 tick
  └─ fsm_bit    ← 位级   (PREPARE/SAMPLE),  2 tick/bit
       └─ fsm_byte ← 字节级  (地址→寄存器→数据→STOP)
```

### 2.5B 一次写事务的完整调用时序

```
时间线 →

[主循环]                              [ISR 4μs 周期]

i2c_write_reg_async(dev=0x50, reg=0x00, data, 2, cb)
  │
  ├─→ ops->is_busy(ctx)              ← 同步调用，立即返回
  │     └─ 返回 0 (空闲)
  │
  ├─→ ops->start_transfer(ctx, ...)  ← 同步调用，立即返回
  │     ├─ 填入 device_address=0x50
  │     ├─ 填入 register_address=0x00
  │     ├─ 填入 data_buffer, data_length=2
  │     ├─ 填入 callback
  │     ├─ timer_start()  ← 开 TIM6
  │     └─ macro_state = START
  │
  └─ return 0  ← CPU 回到主循环       ─→ TIM6_IRQHandler
  [主循环继续干活]                            ├─ i2c_software_timer_isr()
                                             │   macro_state==START
                                             │   └─ i2c_sw_fsm_edge()
                                             │       START step0: SCL=1,SDA=1
                                             │       ── 4μs later ──
                                             │       START step1: SCL=1,SDA=0 ★
                                             │       ── 4μs later ──
                                             │       START step2: SCL=0,SDA=0
                                             │       → macro_state=SEND
                                             │       → send_byte=0x50<<1|0
                                             │       → bit_counter=7
                                             │       ── 4μs later ──
                                             │   macro_state==SEND
                                             │   └─ i2c_sw_fsm_bit()
                                             │       PREPARE: SCL=0,SDA=bit7
                                             │                 → SAMPLE
                                             │       ── 4μs later ──
                                             │       SAMPLE:  SCL=1
                                             │                bit_counter--→6
                                             │                → PREPARE
                                             │       ... (8bit + ACK, 18次 ISR) ...
                                             │       ── 4μs later ──
                                             │       i2c_sw_fsm_byte()
                                             │         → 装入寄存器地址 0x00
                                             │       ... (8bit+ACK, 18次 ISR) ...
                                             │       ── 4μs later ──
                                             │       i2c_sw_fsm_byte()
                                             │         → 装入 data[0]
                                             │       ... (8bit+ACK) ...
                                             │       ── 4μs later ──
                                             │       i2c_sw_fsm_byte()
                                             │         → 装入 data[1]
                                             │       ... (8bit+ACK) ...
                                             │       ── 4μs later ──
                                             │       i2c_sw_fsm_byte()
                                             │         → macro_state=STOP
                                             │       ── 4μs later ──
                                             │   macro_state==STOP
                                             │   └─ i2c_sw_fsm_edge()
                                             │       STOP step0: SCL=0,SDA=0
                                             │       ── 4μs later ──
                                             │       STOP step1: SCL=1,SDA=0
                                             │       ── 4μs later ──
                                             │       STOP step2: SCL=1,SDA=1 ★
                                             │       → timer_stop()
                                             │       → complete_callback(ctx, 0)
  [主循环继续干活]                    ←─────────┘ 回调通知！
```

### 2.5C 函数间数据流

```
i2c_write_reg_async(dev, reg, data, len, cb)
  │  打包参数
  ▼
start_transfer(ctx, dev, reg, is_read=0, data, len, cb)
  │  写入 I2C_SoftwareContext 各字段
  │  device_address ← dev
  │  register_address ← reg
  │  data_buffer ← data
  │  data_length ← len
  │  complete_callback ← cb
  │  macro_state = START
  ▼
[共享内存: I2C_SoftwareContext]  ← ISR 和主循环的唯一通信通道
  │
  │  ★ ISR 每 4μs 读/写这些字段 ★
  │
  ├─→ timer_isr() 读 macro_state → 分派到 fsm_edge 或 fsm_bit
  │     │
  │     ├─→ i2c_sw_fsm_edge() 读 edge_step → 翻 GPIO
  │     │     └─ 推进 macro_state: START→SEND, STOP→DONE
  │     │
  │     ├─→ i2c_sw_fsm_bit() 读 micro_state → PREPARE/SAMPLE 交替
  │     │     读 send_byte, bit_counter → 翻 GPIO
  │     │     写 receive_byte(收时)
  │     │     └─ 调 i2c_sw_fsm_byte()
  │     │
  │     └─→ i2c_sw_fsm_byte() 管理字节级调度
  │           读 bit_counter → 判断字节边界
  │           写 send_byte ← 下一字节数据
  │           写 bit_counter = 7
  │           写 byte_index++ → 推进字节序号
  │           写 macro_state → STOP（发完）或 RECV（读到切换）
  │
  └─→ ISR 末尾: complete_callback(ctx, result)  ← 回到调用者
```

### 2.5D 函数关系速查表

| 函数 | 所在层 | 调用者 | 被调用 | 同步/异步 |
|------|--------|--------|--------|-----------|
| `i2c_write_reg_async` | 策略层 | 应用代码 | `is_busy` → `start_transfer` | 同步（立即返回） |
| `i2c_read_reg_async` | 策略层 | 应用代码 | 同上 | 同步 |
| `i2c_is_busy` | 策略层 | 应用代码 / `write/read_async` | `ops->is_busy` | 同步 |
| `i2c_software_start_transfer` | 平台层 | ops 表分发 | `timer_start` | 同步（开定时器后返回） |
| `i2c_software_is_busy` | 平台层 | ops 表分发 | 无 | 同步 |
| `i2c_software_timer_isr` | 平台层 | `TIM6_DAC_IRQHandler`（硬件） | `fsm_edge` / `fsm_bit` | **异步**（中断上下文） |
| `i2c_sw_fsm_edge` | 平台层 | `timer_isr` | GPIO helper | 异步（START/STOP 边沿） |
| `i2c_sw_fsm_bit` | 平台层 | `timer_isr` | GPIO helper → `i2c_sw_fsm_byte` | 异步（PREPARE/SAMPLE 逐 bit） |
| `i2c_sw_fsm_byte` | 平台层 | `i2c_sw_fsm_bit` | 无（纯状态更新） | 异步（逐字节调度） |
| `complete_callback` | 应用层 | `timer_isr` / `fsm_edge` | 由应用定义 | **异步通知** |

### 2.5E 一句话记忆

| 层 | 职责 | 核心就一句话 |
|----|------|-------------|
| **策略层** | "能不能发？" → 能，帮你打包 | `is_busy` 做门卫，`write/read_async` 做快递打包 |
| **平台层 start_transfer** | "参数收到了，开搞" | 填 context → 开定时器 → 立刻返回 |
| **平台层 timer_isr** | "该我干活了" × 每 4μs | 读 macro_state → `fsm_edge` 管边沿 / `fsm_bit` 管逐 bit → `fsm_byte` 管调度 |
| **平台层 i2c_sw_fsm_byte** | "这个字节完了，下一个是啥？" | 按 地址→寄存器→数据→STOP 的顺序自动推进 |
| **callback** | "搞定了，结果给你" | ISR 末尾通知调用者 |

整个驱动的"心脏"在 `timer_isr`，"大脑"在 `i2c_sw_fsm_byte`，其他都是手脚。

---

## 三、数据结构

### 3.1 I2C_Handle（句柄）

只包含两个字段，对标项目的 `PWM_Handle` / `UART_Handle`：

```c
typedef struct I2C_Handle {
    const I2C_PlatformOps_t *ops;          // 操作表
    void                     *i2c_context; // 平台资源指针
} I2C_Handle;
```

### 3.2 I2C_SoftwareContext（软件 I2C 资源 + FSM 上下文）

这个结构体同时承载**硬件配置**（两个 GPIO 引脚 + 定时器句柄）和**FSM 运行时状态**（当前在哪个状态、传到第几个字节了）：

```
I2C_SoftwareContext
├─ 硬件资源
│   ├─ scl_pin: UserGPIO_Handle   ← SCL 引脚
│   ├─ sda_pin: UserGPIO_Handle   ← SDA 引脚
│   └─ timer_handle: void*        ← 定时器（管 SCL 时钟）
│
├─ FSM 状态（双层）
│   ├─ micro_state: PREPARE / SAMPLE / IDLE   ← 微状态
│   ├─ macro_state: START / SEND / RECV / STOP / DONE  ← 宏状态
│   └─ edge_step: 0~3                         ← 边沿信号子步骤
│
├─ 事务内容
│   ├─ device_address, register_address   ← 发给谁、写哪个寄存器
│   ├─ data_buffer, data_length           ← 数据在哪、多少字节
│   └─ byte_index                         ← 当前事务的第几个字节
│
├─ 当前字节上下文
│   ├─ send_byte / receive_byte           ← 当前在发/在收的字节
│   └─ bit_counter                        ← 7→0:数据位, -1:ACK 位
│
└─ complete_callback                       ← 完成通知函数
```

**两种微状态的语义：**

- `I2C_MICRO_PREPARE`（微状态 0）：SCL 应当为低。此状态下设 SDA = 下一个要传的值。这是**SDA 唯一允许变化的时机**。
- `I2C_MICRO_SAMPLE`（微状态 1）：SCL 应当为高。此状态下不允许碰 SDA——如果是在接收模式，此状态下读 SDA 电平；如果是在发送模式，从机在此状态下采样 SDA。

---

## 四、时序核心：2-tick 模型

### 4.1 什么是 2-tick

I2C 的铁律是：SCL 为高时 SDA 必须稳（从机采样），SCL 为低时 SDA 才能变。因此一个数据位天然需要一个"SCL 低段"＋一个"SCL 高段"。定时器 ISR 每 5μs 触发一次（100kHz I2C），每个数据位需要 2 次 ISR，即 **2 个 tick**。

**tick 0**（`I2C_MICRO_PREPARE`）负责准备数据，**tick 1**（`I2C_MICRO_SAMPLE`）负责让从机采样或让主机读取。

---

### 4.2 发送 1 bit 的过程

```
      ISR#0(tick0)       ISR#1(tick1)       ISR#2(tick0)
      ──────────          ──────────          ──────────
      ① SCL = 0           ④ SCL = 1           ① SCL = 0
      ② SDA = bit_n       ⑤ SDA 不变          ② SDA = bit_n+1
      ③ → tick1                                  → tick1
         
SCL ────┐    ┌────────────┐    ┌──────────
        │    │            │    │
        └────┘            └────┘
         ←5μs→             ←5μs→

SDA ─────────┐  ┌─────────────────
             │  │
             └──┘
         ② SDA 变化点
```

| 步骤 | 发生时机 | 动作 | 原因 |
|------|---------|------|------|
| ① | tick0 开头 | SCL 拉低 | 进入"准备数据"阶段 |
| ② | tick0 中间 | SDA = bit_n | 铁律允许（SCL 低时 SDA 可变化） |
| ③ | tick0 末尾 | 微状态切换到 tick1 | 下次 ISR 进入 tick1 |
| ④ | tick1 开头 | SCL 拉高 | 进入"数据有效"阶段 |
| ⑤ | tick1 期间 | SDA 保持 | 铁律禁止变化——从机在此时采样 |

**CPU 实际干的事：** tick0 ISR 写两次 GPIO ODR 寄存器（SCL=0, SDA=bit），tick1 ISR 写一次（SCL=1）。每次 ISR 耗时不到 2μs。

---

### 4.3 接收 1 bit 的过程

接收和发送的区别只在 tick0——此时主机不设 SDA 的具体值，而是设 SDA=1 来**释放总线**。在开漏输出模式下，SDA=1 意味着 GPIO 输出高阻，由外部上拉电阻把电平拉到高，此时从机可以拉低 SDA。主机在 tick1 SCL 拉高后读取 SDA 的实际电平，这就是从机发出的数据位。

```
     ISR#0(tick0)         ISR#1(tick1)
     ──────────            ──────────
     ① SCL = 0             ④ SCL = 1
     ② SDA = 1（释放）      ⑤ 读 SDA → 存入 receive_byte
     ③ → tick1

SCL ────┐    ┌─────────┐
        │    │         │
        └────┘         └──

SDA ---- [从机驱动] ----  ← 主机在 ② 释放，从机接管
             ↑
        主机在 ⑤ 读 SDA
```

---

### 4.4 完整发送 1 字节（8 bit + ACK）

```
   ┌──── 8 bit 数据（16 次 ISR）────┐  ┌─ ACK 位（2 次 ISR）─┐
   bit7  bit6  bit5  bit4  bit3  bit2  bit1  bit0     ACK
SCL ──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──
      │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │
      ┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──
SDA  X  X  X  X  X  X  X  X                        ?
                                     ↑               ↑
                                8 bit 全部发送     主机读 SDA 判断 ACK/NACK
```

**ACK 位细节**（也就是第 9 位）：

发完 bit0（8 位完成，bit_counter 从 0 变为 -1）后，进入 ACK 阶段。下一个 tick0 仍然执行 "SCL 低 → 设 SDA"，但此时 SDA=1（释放总线），把控制权交给从机。再下一个 tick1 执行 "SCL 高 → 读 SDA"——此时如果从机存在且地址匹配，它会拉低 SDA 作为 ACK 回应（读到 0=ACK，读到 1=NACK）。

---

### 4.5 完整接收 1 字节（8 bit + ACK）

接收时的 ACK 是主机发回给从机的。8 bit 收完后，tick0 阶段主机设 SDA = 0（通知从机"我收到了，继续发"），或 SDA = 1（通知从机"最后一字节，停止"）。这和发送模式下的 ACK 方向相反。

**完整的 I2C 写事务（START + 地址 + 寄存器 + 数据 + STOP）：**

```
START  地址+W  ACK  寄存器  ACK  数据    ACK  STOP
[3 ISR][16+2] [2]  [16+2] [2]  [16+2] [2]  [3 ISR]
  
总计约 66 次 ISR，每次 2μs，CPU 总开销 ~130μs（分布在 330μs 时间中）。
主循环在此期间持续运行，不会被冻结。
```

---

### 4.6 START 和 STOP —— 为什么不能走 2-tick

**问题：** START 和 STOP 的定义恰恰要求"**SCL 高时 SDA 变化**"——这直接违反了 2-tick 模型的铁律（SCL 高时 SDA 必须稳定）。

**START**：SCL=1 时，SDA 从 1 跳到 0（下降沿）。

```
SCL ────────────────────┐
                         │
                         └────────────
SDA ──────────────┐
                  │
                  └──────────────────
                  ↑ SDA 下降发生在 SCL 高时 → START
```

**STOP**：SCL=1 时，SDA 从 0 跳到 1（上升沿）。

```
SCL ──────────────┐   ┌────────────────
                  │   │
                  └───┘
SDA ──────────┐       ┌────────────
              │       │
              └───────┘
              ↑ SDA 上升发生在 SCL 高时 → STOP
```

如果在 2-tick 模型里做 START，tick0 会先把 SCL 拉到低，这就不是 START 了——START 要求**先有** SCL 高，再有 SDA 下降。所以 2-tick 的 `I2C_MICRO_PREPARE` 分支对 START/STOP 无效。设计中的解决方法是：在 `i2c_software_timer_isr` 入口处检查 `macro_state` 是否为 START/STOP，如果是，直接分派到独立的子 FSM 处理，不走常规数据分支。

---

## 五、平台层（ops）设计

ops 表只有两个操作——启动一次完整的事务，和查询忙状态：

| 操作 | 做什么 | 为什么这样设计 |
|------|--------|--------------|
| `start_transfer(hI2C, dev, reg, is_read, buf, len, cb)` | 填入事务参数 + 启动定时器 ISR | 启动即返回，不阻塞 |
| `is_busy(hI2C)` → `uint8_t` | 返回 0（空闲）或 1（传输中） | 防止同一句柄被重复启动 |

### 5.1 抽象 ops 与软/硬件 ops 的关系（★接口契约）

```
                     I2C_PlatformOps_t  ← 1 个类型，定义在 useri2c.h
                    ┌──────────────────────────┐
                    │ start_transfer: 函数指针   │  ← 契约：6 个参数，返回 int8_t
                    │ is_busy:        函数指针   │  ← 契约：1 个参数，返回 uint8_t
                    └──────────┬───────────────┘
                               │ 函数签名必须完全一致（返回值 + 参数个数 + 参数类型）
                               │ 参数名可以不同，void* 多态解耦平台资源类型
              ┌────────────────┼────────────────┐
              ▼                                 ▼
   软件 I2C ops 实例                  硬件 I2C ops 实例（未来）
   i2c_software_platform_ops_stm32   i2c_hardware_platform_ops_stm32
   ┌──────────────────────────┐      ┌──────────────────────────┐
   │ .start_transfer =        │      │ .start_transfer =        │
   │   i2c_software_start_    │      │   i2c_hardware_start_    │
   │   transfer               │      │   transfer               │
   │ .is_busy =               │      │ .is_busy =               │
   │   i2c_software_is_busy   │      │   i2c_hardware_is_busy   │
   └──────────────────────────┘      └──────────────────────────┘
          │                                    │
          │ ctx → I2C_SoftwareContext*         │ ctx → I2C_HandleTypeDef*
          │ 查 macro_state 判忙                 │ 调 HAL_I2C_GetState 判忙
          │ 开 TIM6 周期 ISR                   │ 调 HAL_I2C_Mem_Write_IT
```

**核心规则：**

| 规则 | 说明 |
|------|------|
| 类型 1 个 | `I2C_PlatformOps_t` 定义一份，软/硬共享 |
| 实例 N 个 | 每个平台、每种实现各一个 `const` 实例 |
| 函数签名必须一致 | 任何实现必须和 `I2C_PlatformOps_t` 里的函数指针类型逐字对齐 |
| `void*` 实现多态 | `i2c_context` 用 `void*`，实现时各自强转为自己需要的类型 |
| 策略层不感知差异 | `i2c_write_reg_async` 只看 `handle->ops`，不关心下面是谁 |

**违反契约的后果：**

```c
// ❌ 签名不一致 → 编译失败
uint8_t i2c_software_is_busy(I2C_SoftwareContext *ctx)  // 参数类型不对
// → error: incompatible function pointer types

// ✅ 正确写法
uint8_t i2c_software_is_busy(void *i2c_context)         // 参数类型一致
```

**与项目其他驱动的对比：**

| 驱动 | 非阻塞机制 |
|------|-----------|
| PWM | 不需要——写完寄存器硬件自己跑 |
| UART | 外设 ISR 数据来才触发（不定频） |
| **软件 I2C** | **定时器 ISR 周期触发（每 5μs）** |
| 硬件 I2C | 外设 ISR 传输完成才触发 1 次（CPU 零负载） |

---

## 六、策略层函数

策略层只做两件事：检查忙状态 + 把参数打包传给 ops 层的 `start_transfer`。

```c
int8_t i2c_write_reg_async(I2C_Handle *handle, uint8_t device_address,
    uint8_t register_addr, const uint8_t *data, uint16_t length,
    I2C_TransferCallback complete_callback);
```

调用流程：
1. 检查 `is_busy()`——如果上一次传输还没结束，返回 -1
2. 调用 `ops->start_transfer(...)`——填入所有参数，启动定时器
3. 函数立即返回
4. 主循环在此期间自由执行其他任务
5. 传输完成后，ISR 自动调用 `complete_callback(hI2C, result)` 通知

---

## 七、非阻塞实现：定时器 + 双层状态机

> **术语 —— ISR（Interrupt Service Routine，中断服务程序）**：当硬件事件（如定时器溢出）发生时，CPU 自动暂停当前代码，跳去执行的一段特殊函数。ISR 执行完，CPU 回到之前被中断的地方继续跑。本章中 `ISR` 均指定时器中断服务程序。

### 7.1 定时器如何替代 delay_us

**阻塞版：** CPU 写 GPIO，然后跑一个空循环等 5μs，再写 GPIO，再等 5μs。CPU 在这段期间 100% 被占用。

**非阻塞版：** CPU 写 GPIO 后用 2μs 更新状态机，然后退出 ISR。硬件定时器在 5μs 后自动触发下一次中断。ISR 退出到下次进入的 3μs 间隙，CPU 回到主循环执行业务代码。

```
         ISR(2μs)  主循环(3μs)  ISR(2μs)  主循环(3μs)  ISR(2μs)
         ┌────┐    ┌─────────┐  ┌────┐    ┌─────────┐  ┌────┐
TIM_IRQ ─┘    └────┘         └──┘    └────┘         └──┘    └──
              ←──── 5μs ────→
```

---

### 7.2 三层 FSM 的分工

严格来说是**三层**：`fsm_edge`（边沿）→ `fsm_bit`（位）→ `fsm_byte`（字节），但核心"双层"指 `fsm_bit` + `fsm_byte`。

**fsm_bit（位时序层）**——只管"这一拍 GPIO 怎么翻"：
- tick0（PREPARE）：SCL 拉低 + 设 SDA 值（发送）或释放 SDA（接收）
- tick1（SAMPLE）：SCL 拉高 + 读 SDA（接收）或从机自行采样（发送）
- 每 bit = tick0 + tick1，往复循环
- 不知道"现在在发第几个字节"——那是 fsm_byte 的事

**fsm_byte（字节调度层）**——只管"传到哪里了，下一步该干啥"：
- START 完成 → 装入设备地址，进入 SEND 状态
- 设备地址发完且 ACK → 装入寄存器地址
- 寄存器地址发完且 ACK → 根据需要装入数据或切 RECV
- 数据发完 → 进入 STOP 状态
- STOP 完成 → 关定时器 + 回调

**fsm_edge（边沿层）**——处理 START/STOP 这两个不走 2-tick 的特殊时序。

**分层的好处：** fsm_bit 不关心协议语义，对任何 "8 bit + ACK" 的传输都通用。fsm_byte 不关心 GPIO 怎么翻，只关心数据怎么排列。换一个协议（比如软件 SPI）只需换 fsm_bit，fsm_byte 的结构可以复用。

#### 两层状态机如何通信

两层共享同一份 `I2C_SoftwareContext`，通过其中的字段交流，**不靠函数参数传递**。

| 字段 | 谁写 | 谁读 | 含义 |
|------|------|------|------|
| `micro_state` | **fsm_bit**（每次 ISR 末尾 `^=1`） | fsm_bit 自己 | "下次 ISR 走 PREPARE 还是 SAMPLE" |
| `macro_state` | **fsm_byte** / **fsm_edge** | timer_isr 入口 | "整个事务进行到哪了" |
| `send_byte` | **fsm_byte**（装入下一字节） | **fsm_bit**（取 bit 推到 SDA） | "当前要发的字节" |
| `receive_byte` | **fsm_bit**（逐 bit 左移存入） | **fsm_byte**（写回 data_buffer） | "当前正在收的字节" |
| `bit_counter` | **fsm_byte**（重置 7 / `--`） | fsm_bit + fsm_byte 都读 | "当前发/收到第几位了" |
| `byte_index` | **fsm_byte** | fsm_byte 自己 | "当前是事务的第几个字节"（0=设备地址,1=寄存器地址,2+=数据） |
| `edge_step` | **fsm_edge** | fsm_edge 自己 | "START/STOP 走了几步了" |

**两层之间的交互只在一个方向触发：fsm_bit 主动通知 fsm_byte。** 每次 `I2C_MICRO_SAMPLE` 末尾，`fsm_bit` 调用 `i2c_sw_fsm_byte()`，宏状态机才工作。宏状态机不主动抢占——只在被调用时才干活，然后通过写入 `send_byte`、`bit_counter`、`macro_state` 告诉微状态机"接下来照这样执行"。

**一个 bit 的完整交互过程：**

```
ISR 进入（当前 micro_state = PREPARE）

fsm_bit:
  读 micro_state → PREPARE → 好，设 SCL=0
  "SDA 该设什么？" → 读 send_byte, bit_counter
  SDA = (send_byte >> bit_counter) & 1
  micro_state ^= 1 → 变成 SAMPLE

──────────── 定时器 4μs 后触发下一次 ISR ────────────

ISR 进入（当前 micro_state = SAMPLE）

fsm_bit:
  读 micro_state → SAMPLE → 好，拉高 SCL
  "这个 bit 完成了，通知宏"
  → 调用 i2c_sw_fsm_byte()   ← ★ 唯一入口，微主动通知宏

fsm_byte（在调用中）：
  bit_counter--   → 8 位发完了？→ 进 ACK(bit_counter = -1)
  ACK OK → 下一个是寄存器地址？→ send_byte = register_addr, bit_counter = 7
  或 数据全发完了？→ macro_state = STOP, edge_step = 0

回到 fsm_bit:
  micro_state ^= 1 → 变成 PREPARE
```

---

### 7.4 微状态机（fsm_bit）—— 每个 ISR 只做 1 步

`i2c_sw_fsm_bit()` 实现 PREPARE ↔ SAMPLE 交替的 2-tick 模型。每次 ISR 调用它，读当前 `micro_state` 走对应分支：

**完整代码逻辑：**

```c
static void i2c_sw_fsm_bit(I2C_SoftwareContext *ctx)
{
    if (ctx->micro_state == I2C_MICRO_PREPARE)
    {
        i2c_sw_scl_set(ctx, 0);                       // ① 拉低 SCL（允许改变 SDA）
        if (ctx->macro_state == I2C_MACRO_SEND)
            i2c_sw_sda_set(ctx, (ctx->send_byte >> ctx->bit_counter) & 1);
        else                                          // RECV
            i2c_sw_sda_set(ctx, 1);                   // ② 释放 SDA，让从机接管
    }
    else  // I2C_MICRO_SAMPLE
    {
        i2c_sw_scl_set(ctx, 1);                       // ③ 拉高 SCL（从机采样 / 主机读取）
        if (ctx->macro_state == I2C_MACRO_RECV)
            ctx->receive_byte = (ctx->receive_byte << 1)
                              | i2c_sw_sda_read(ctx); // ④ 接收：左移 + 存入 bit

        i2c_sw_fsm_byte(ctx);                         // ⑤ ★ 通知宏状态机：1 bit 完成
    }
    ctx->micro_state ^= 1;                            // ⑥ 0↔1 切换
}
```

**PREPARE 分支核心细节：**
- SEND 模式：从 `send_byte` 取第 `bit_counter` 位推到 SDA
- RECV 模式：SDA 设 1（开漏 = 高阻），从机可以拉低 SDA 来传数据位
- PREPARE 阶段绝不在 SAMPLE 操作 SDA——违反 I2C 铁律

**SAMPLE 分支核心细节：**
- RECV 模式：用左移法 `(receive_byte << 1) | bit` 逐 bit 组装字节，不依赖 bit_counter 当前值
- SEND 模式：从机自动在 SCL 上升沿采样 SDA，主机无需操作；但 ACK 位时 `fsm_byte` 会检查
- 末尾调 `i2c_sw_fsm_byte(ctx)`——这是两个 FSM 的唯一触发点

**`micro_state` 切换机制：**

```c
ctx->micro_state ^= 1;  // 异或翻转 bit0: 0→1→0→1...
```

用 `^= 1` 而非 `++; %=2`，原因：IDLE(2) 被误触时 `2^1=3` 异常值容易被发现，而 `2%2=0` 会悄无声息进入 PREPARE 导致第一个 bit 没有拉低 SCL。

**SEND 模式下 1 字节的完整执行序列（18 次 ISR）：**

| ISR | micro_state | 做的事 |
|-----|------------|--------|
| #0 | PREPARE | SCL=0, SDA=bit7, 切到 SAMPLE |
| #1 | SAMPLE | SCL=1, bit_counter 7→6, 切回 PREPARE |
| #2 | PREPARE | SCL=0, SDA=bit6 |
| ... | ... | ... |
| #14 | PREPARE | SCL=0, SDA=bit0 |
| #15 | SAMPLE | SCL=1, bit_counter 1→0, 切回 PREPARE |
| #16 | PREPARE | SCL=0, **SDA=1**（释放，等从机 ACK） |
| #17 | SAMPLE | SCL=1, **读 SDA → ACK(0) / NACK(1)** |

SDA 变化**只在 PREPARE 发生**（#0, #2, #4...）。SAMPLE 只拉高 SCL + 通知 `fsm_byte`，不碰 SDA。

### 7.4B bit_counter 专用说明

`bit_counter` 是 `fsm_bit` 和 `fsm_byte` 之间最重要的通信纽带。

| 值 | 含义 | 此时 SDA 上的内容 |
|----|------|------------------|
| 7 | 第 1 个数据 bit（MSB） | `send_byte` 的 bit7 |
| 6 | 第 2 个数据 bit | `send_byte` 的 bit6 |
| ... | ... | ... |
| 0 | 第 8 个数据 bit（LSB） | `send_byte` 的 bit0 |
| -1 | ACK/NACK 位 | 从机回 ACK(0) / NACK(1) |

**生命周期：**
```
fsm_byte 装入新字节 → bit_counter = 7
  ↓
每 bit 完成 → fsm_byte: bit_counter--
  ↓
bit_counter = 0 → fsm_byte: bit_counter = -1（进入 ACK）
  ↓
ACK 完成 → fsm_byte: bit_counter = 7（装下一个字节）或 切 STOP
```

**三个用途：**

| 用途 | 代码 | 所在函数 |
|------|------|---------|
| 发送时取数据位 | `(send_byte >> bit_counter) & 1` | fsm_bit (PREPARE) |
| 判断字节完成 | `if (bit_counter == 0)` → 进 ACK | fsm_byte |
| 判断 ACK 阶段 | `if (bit_counter == -1)` → ACK 完成 | fsm_byte |

---

### 7.5 START 和 STOP 的独立子 FSM（fsm_edge）

因为 START/STOP 要求 SCL 高时 SDA 变，不能复用 2-tick 的 tick 交替逻辑。每个用 `edge_step` 字段追踪自己在多 tick 序列中的位置。

**START 子 FSM（3 个 tick）：**

| tick | edge_step | SCL | SDA | 说明 |
|------|----------|-----|-----|------|
| 0 | 0 | 1 | 1 | 初始化：确保 SCL 高，SDA 高（总线空闲） |
| 1 | 1 | 1 | **0** | **核心：SCL 高时 SDA 下降 → START** |
| 2 | 2 | 0 | 0 | 恢复：拉低 SCL，准备传第一个数据位 |

**STOP 子 FSM（3 个 tick）：**

| tick | edge_step | SCL | SDA | 说明 |
|------|----------|-----|-----|------|
| 0 | 0 | 0 | 0 | 初始化：确保 SCL 低，SDA 低 |
| 1 | 1 | 1 | 0 | 先拉高 SCL |
| 2 | 2 | 1 | **1** | **核心：SCL 高时 SDA 上升 → STOP** |

---

### 7.6 宏状态机（fsm_byte）—— 跨字节调度

`i2c_sw_fsm_byte()` 在 `fsm_bit` 的 SAMPLE 末尾被调用，负责**纯逻辑决策**——不翻 GPIO、不改电平，只做字节级调度。

**`byte_index` 生命周期（★ 关键设计）：**

```
start_transfer:  byte_index = 0                           ← 初始化
fsm_edge END:    send_byte = 设备地址                      ← 装首发球（不碰 byte_index）

fsm_bit → fsm_byte: 设备地址发完 (byte_index==0)
  fsm_byte:      send_byte = 寄存器地址                     ← 装下一字节
                 byte_index = 1                            ← ★ fsm_byte 自增

fsm_bit → fsm_byte: 寄存器地址发完 (byte_index==1)
  fsm_byte:      send_byte = data_buffer[0]               ← 装下一字节  
                 byte_index = 2                            ← ★ fsm_byte 自增
                 
fsm_bit → fsm_byte: data[0] 发完 (byte_index>=2)
  fsm_byte:      byte_index++                             ← ★ 继续自增
                 send_byte = data_buffer[byte_index - 2]   ← -2 偏移访问 data_buffer
```

**核心原则：`byte_index` 始终指向"刚发完的那个索引"，下一字节由 fsm_byte 在判断 byte_index 后被装入 send_byte。`byte_index` 只由 `start_transfer`（初始化 0）和 `fsm_byte`（自增）修改，`fsm_edge` 不碰它。**

**通过 bit_counter 判断字节边界：**

```
fsm_byte 被调用
  │
  ├─ bit_counter > 0  → bit_counter--; return;
  │    当前字节还没发完，什么都不做
  │
  ├─ bit_counter == 0 → bit_counter = -1; return;
  │    8 bit 全部完成，进入 ACK 位（下一个 PREPARE 自动释放 SDA）
  │
  └─ bit_counter == -1：ACK 位已完成，开始决策
  
      ├─ SEND 模式：
      │    └─ 检查 ACK（读 SDA，低=ACK，高=NACK）
      │         ├─ NACK → macro_state = STOP（错误路径，补发 STOP）
      │         └─ ACK → 当前字节是什么？
      │               ├─ 第 1 字节是 [设备地址+W]
      │               │    → send_byte = register_addr
      │               │    → bit_counter = 7
      │               ├─ 第 2 字节是 [寄存器地址]
      │               │    → 判断 is_read
      │               │         ├─ 读事务 → 重装设备地址+R（切到 RECV）
      │               │         └─ 写事务 → send_byte = data_buffer[0]
      │               │    → bit_counter = 7
      │               └─ 第 3+ 字节是 [数据]
      │                    → byte_index++
      │                    → 还有数据？→ send_byte = data_buffer[byte_index - 2]
      │                    → 发完了    → macro_state = STOP, edge_step = 0
      │
      └─ RECV 模式：
           ├─ 不是最后一字节 → 主机发 ACK（SDA=0）
           │    → data_buffer[byte_index - 2] = receive_byte; byte_index++
           │    → bit_counter = 7
           └─ 最后一字节     → 主机发 NACK（SDA=1）
                → data_buffer[byte_index - 2] = receive_byte; byte_index++
                → macro_state = STOP, edge_step = 0
```

**读事务的特殊性：**
读事务需要先发送设备地址+W + 寄存器地址（SEND），然后重新 START 并切换为设备地址+R（RECV）。`fsm_byte` 在寄存器地址发完且 `is_read=1` 时，自动重新装入 `(device_address << 1) | 1` 并重走 START 流程，策略层对此无感知。

**一个完整读事务的状态流转（同时使用 SEND 和 RECV）：**

```
IDLE
  → START（fsm_edge: 3 tick）
  → SEND: 设备地址+W  （fsm_bit: 16+2 tick）   ← SEND 模式
  → SEND: 寄存器地址   （fsm_bit: 16+2 tick）   ← SEND 模式，byte_index=1
        │
        │ is_read=1 触发 RESTART
        ▼
  → START（fsm_edge: 3 tick，重 START）
  → SEND: 设备地址+R  （fsm_bit: 16+2 tick）   ← SEND 模式，byte_index=2
        │
        │ fsm_byte: macro_state = RECV  ★ 状态切换
        ▼
  → RECV: data[0]     （fsm_bit: 16+2 tick）   ← RECV 模式，主机读 SDA
  → RECV: data[1]     （fsm_bit: 16+2 tick）
  → ...重复...
  → STOP（fsm_edge: 3 tick）
  → DONE → 关定时器 → complete_callback(ctx, 0)
```

**一此读事务中 `macro_state` 的变化轨迹：IDLE → START → SEND → START → SEND → RECV → STOP → DONE → IDLE。** SEND 和 RECV 都出现，前两字节 SEND，后面的数据 RECV。

**一个完整写事务的状态流转：**

```
IDLE
  → START（fsm_edge: 3 tick）
  → SEND: 设备地址+W（fsm_bit: 16+2 tick）
  → SEND: 寄存器地址（16+2 tick）
  → SEND: data[0]（16+2 tick）
  → ...重复...
  → STOP（fsm_edge: 3 tick）
  → DONE → 关定时器 → complete_callback(ctx, 0)
```

**fsm_byte 的核心职责：它不执行，只决策。** 通过改写 `send_byte`、`bit_counter`、`macro_state` 来指挥 `fsm_bit` 下一步发什么。

---

### 7.7 CPU 占用分析

CPU 占用 = ISR 执行时间 / 定时器中断周期。

| I2C 速率 | 定时器中断周期 | ISR 耗时 | CPU 占用 |
|---------|--------------|---------|---------|
| 10kHz | 50μs | <2μs | ~4% |
| 50kHz | 10μs | <2μs | ~20% |
| 100kHz | 5μs | <2μs | ~40% |
| 400kHz | 1.25μs | <2μs | **不可实现**（ISR 退不出来） |

**结论：** 非阻塞软件 I2C 适合 <=100kHz 的场景，且频率越低 CPU 越轻松。如需 400kHz 或零 CPU 占用，必须用硬件 I2C 外设。

---

## 八、与 UART 非阻塞的对照

| 维度 | 软件 I2C | UART（当前项目） |
|------|---------|---------------|
| **ISR 触发源** | 定时器主动周期触发 | 硬件收到字节才触发 |
| **ISR 频率** | 固定（每 5μs@100kHz） | 不固定（没数据时完全不触发） |
| **ISR 做的事** | 翻转 GPIO + 更新状态机 | 写 ringbuf + 重新使能接收 |
| **主循环消费** | complete_callback() 一次性通知 | cmd_parser_tick() 逐字节消费 |
| **CPU 占用** | 高（持续中断） | 低（按需中断） |

---

## 九、文件清单

| 文件 | 路径 | 说明 |
|------|------|------|
| `useri2c.h` | `Core/Inc/driver/` | 句柄定义、ops 表声明、策略层函数声明 |
| `useri2c.c` | `Core/Src/driver/` | 策略层实现（组装参数 + 启动传输） |
| `useri2c_ops.h` | `Core/Inc/driver/` | 软件 I2C 平台 ops extern 声明 |
| `useri2c_ops.c` | `Core/Src/driver/` | 双层 FSM + 定时器 ISR 实现 |

> 配置层：`gpio.c`（SCL/SDA 开漏输出 + 上拉）、`tim.c`（定时器 2× SCL 频率中断）

---

## 十、已知限制与待办

- **CPU 占用**：100kHz 时 ~40%。降频可显著减轻，如需 400kHz 或零负载必须用硬件 I2C。
- **无时钟拉伸**：当前 SCL 高阶段不检测从机拉低 SCL。从机如果主动拉低 SCL 表示"我还没准备好"，主机会无视并继续发时钟，可能导致数据错误。
- **无多主机仲裁**：当前为单主设计。
- **同一句柄不可并发启动两次传输**：通过 `is_busy()` 检测保证。
- **NACK 后缺少显式 STOP**：当前 NACK 后直接 `return -1`，未补发 STOP。实际实现中应在错误路径加 STOP 后再返回。
