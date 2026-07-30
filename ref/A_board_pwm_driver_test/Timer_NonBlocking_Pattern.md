# 定时器中断驱动的非阻塞实现

> 适用于所有需要软件 bit-bang GPIO 的串行协议（软件 I2C、软件 SPI、OneWire、DHTxx 等）。

---

## 命名约定与函数速查

> 本文出现的大部分函数名都是**概念名**，不一定是最终代码中的真实函数名。它们的存在是为了在通用层面讲清楚"谁干什么事"。以下是本文主要用名一览。

| 名字 | 类别 | 含义 |
|------|------|------|
| `delay_us(N)` | 阻塞延时 | CPU 空循环 N 微秒，阻塞版的核心问题所在 |
| `send_byte()` | 阻塞示例 | 阻塞版逐字节发送的示意函数 |
| `set_gpio()` / `gpio_write()` / `gpio_read()` | GPIO 操作 | 设置/读取一根 GPIO 引脚的电平（概念函数，实际用 HAL_GPIO_WritePin 等） |
| `toggle_clock()` | GPIO 操作 | 翻转时钟线电平（概念函数） |
| `get_next_bit()` | 位提取 | 从当前发送字节中取下一个要发的 bit |
| `start_timer()` / `stop_timer()` | 定时器控制 | 概念上的启停定时器，实际对应 STM32 的 `HAL_TIM_Base_Start_IT` / `HAL_TIM_Base_Stop_IT` |
| `TIM_IRQHandler()` | ISR 入口 | 定时器中断服务程序，硬件在定时器溢出时自动跳入 |
| `HAL_TIM_IRQHandler()` | HAL 标准入口 | STM32 HAL 库的定时器中断通用处理函数，内部清标志并调用用户回调 |
| `HAL_TIM_Base_Start_IT()` | HAL 定时器 API | 启动定时器并使能中断（传输开始） |
| `HAL_TIM_Base_Stop_IT()` | HAL 定时器 API | 停止定时器并关闭中断（传输结束） |
| `__HAL_TIM_GET_FLAG()` | HAL 标志宏 | 检查定时器 Update 中断标志是否置位 |
| `__HAL_TIM_CLEAR_FLAG()` | HAL 标志宏 | 清除定时器 Update 中断标志，防止重复进入 ISR |
| `micro_state_machine()` | 微状态机 | 每个 tick 执行一次，只管 1 根 GPIO 的翻转 |
| `macro_fsm_advance()` | 宏状态机 | 由微状态机在时钟边沿调用，负责字节/事务级调度 |
| `handle_response()` | 协议响应 | 处理协议特定的应答（如 I2C 的 ACK/NACK） |
| `sub_fsm_start()` | 协议特例子 FSM | 处理不满足常规 tick 交替的起始条件（如 I2C START、SPI 片选） |
| `sub_fsm_stop()` | 协议特例子 FSM | 处理不满足常规 tick 交替的终止条件（如 I2C STOP、SPI 片选释放） |
| `start_xfer()` | 传输 API | 填入事务参数并启动定时器，调用后 CPU 立即返回（非阻塞） |
| `xfer_write_async()` | 上层 API | 对策略层的异步写接口 |
| `xfer_read_async()` | 上层 API | 对策略层的异步读接口 |
| `xfer_busy()` | 状态查询 | 返回是否正在传输，防止同一句柄被重复启动 |
| `done_cb()` | 完成回调 | 传输完成时 ISR 自动调用的通知函数 |
| `HAL_UART_Transmit_DMA()` | HAL UART API | STM32 的 DMA 后台发送，作为本文方案的对比对象 |
| `main_loop()` | 应用示例 | 主循环，展示非阻塞期间 CPU 可以并行干哪些事 |
| `parse_uart_commands()` | 应用示例 | 处理 UART 命令 |
| `update_pid_loop()` | 应用示例 | 运行 PID 控制回路 |
| `read_imu_data()` | 应用示例 | 从 SPI 传感器读取姿态数据 |
| `process_rx_data()` | 应用示例 | 消费收到的数据 |

---

## 一、解决的问题

**阻塞版本做的事：**

```c
void send_byte(uint8_t data) {
    for (int bit = 7; bit >= 0; bit--) {
        set_gpio(data >> bit);      // 写 GPIO（<1μs）
        delay_us(5);                // CPU 死等 5μs  ← 问题在这里
        toggle_clock();             // 写 GPIO
        delay_us(5);                // 又死等 5μs
        toggle_clock();
    }
}
// 发 2 字节 ≈ 380μs，主循环在此期间完全卡死
```

**问题**：`delay_us()` 是 `for(i=0; i<N; i++)` 空循环，CPU 被 100% 占用，任何中断外的逻辑都暂停。

**目标**：让 CPU 不卡在等 GPIO 状态切换上。

---

## 二、原理

> **术语 —— ISR（Interrupt Service Routine，中断服务程序）**：当硬件事件（如定时器溢出）发生时，CPU 自动暂停当前代码，跳去执行的一段特殊函数。ISR 执行完，CPU 回到之前被中断的地方继续跑。本文中所有 `TIM_ISR`、`ISR` 均指定时器中断服务程序。

### 2.1 核心思路

**用硬件定时器代替 `delay_us`。** 定时器周期 = 协议的最短动作间隔（比如 I2C 的半周期 = 5μs）。每次中断完成 1 个"原子动作"，其余时间 CPU 自由。

```
阻塞版：
  set_gpio → delay_us(5) → set_gpio → delay_us(5) → ...
    ↑                    ↑
    全部在一个函数调用里，CPU 全程封锁

定时器版：
  start_timer() → CPU 立即返回（允许主循环运行）
  
  TIM_ISR #0: set_gpio(bit7)     CPU 退出 ISR → 主循环跑
  TIM_ISR #1: toggle_clock
  TIM_ISR #2: set_gpio(bit6)
  TIM_ISR #3: toggle_clock
  ...
  TIM_ISR #17: toggle_clock      stop_timer() → done_cb()
```

### 2.2 什么是"原子动作"

每个 ISR 做的事被设计成**不可再分的最小单元**——通常只包含：清中断标志 + 翻转 1 根 GPIO + 1 次状态机步进。这种粒度保证：
- 每次 ISR 耗时**固定且极短**（<2μs），不会因数据长度变化而波动
- ISR 退出后，剩余时间（定时器周期减去 2μs）**原封不动还给主循环**
- 没有 "ISR 嵌套复杂逻辑" 的风险，所有决策已经在结构体字段里提前算好

**反面例子：** 如果一次 ISR 连续翻多根 GPIO 或做多层判断，ISR 变长，主循环被抢占太久，非阻塞就没意义了。

### 2.3 CPU 时间账

每次 ISR 耗时 <2μs。假设定时器周期为 T，则：

```
         ISR(<2μs)  主循环(T-2μs)  ISR(<2μs)  主循环(T-2μs)
         ┌────┐    ┌─────────────┐ ┌────┐    ┌─────────────┐
TIM_IRQ ─┘    └────┘             └─┘    └────┘             └──
              ←──────  T  ──────→
```

**CPU 占用 = ISR 耗时 / 定时器周期。** T 越小（协议速率越高），占用越高。例如 T=5μs 时约 40%，T=50μs 时仅 ~4%。具体数字见第八章。

### 2.4 这个模式适合什么协议

| 特征 | 适用 | 不适用 |
|------|------|--------|
| 软件 bit-bang 控制 GPIO | ✅ | — |
| 有时钟线的同步协议（I2C、SPI） | ✅ | — |
| 无时钟线的异步协议（OneWire、DHTxx 等） | ✅（需调整 tick 模型） | — |
| 速率导致中断周期 < ISR 耗时 | — | ❌ ISR 来不及退出 |
| 已有硬件外设且性能可接受 | ⚠️ 能用但不推荐 | 专用外设 ISR 开销更低 |

核心不限 I2C——任何"需要 CPU 手动控制 GPIO 时序、又不能阻塞主循环"的场景都适用。不同协议的微状态机不同，但宏状态机的调度结构可以复用。

---

## 三、架构：双层状态机

### 3.1 为什么要分层

如果所有逻辑——GPIO 翻转、位计数、字节切换、事务终止——都塞进一个平铺的 switch-case 里，结果是：

- **微动作和宏逻辑纠缠在一起**，改一个 GPIO 电平可能影响整个事务流程
- **协议换不动**——比如 I2C 变 SPI，GPIO 翻转逻辑全变了，但字节调度逻辑其实一样

解决方案是把状态机拆成两层，每层只面对一个层级的决策：

```
     定时器 ISR
        │
        ▼
┌──────────────────────────────┐
│  微状态机（ISR 内执行）         │
│  每个中断 = 1 tick            │
│  负责：翻转 1 根 GPIO + 延时  │
│  状态只有 2~4 个              │
└──────────┬───────────────────┘
           │ 通知宏
┌──────────▼───────────────────┐
│  宏状态机（ISR 内执行）         │
│  负责：跨 tick 调度            │
│  例：bit_cnt-- → 此字节发完？  │
│      → 装下一个字节            │
│      → 全部发完？→ stop_timer  │
│      → 回调 done_cb()         │
└──────────────────────────────┘
```

**微状态机（时序层）** — 不关心协议语义。只管"这一拍 GPIO 怎么翻"：
- 状态数量极少（通常 2~4 个），对应一个时钟周期内的几个相位
- 在发送模式下：准备数据 → 发出时钟 → 回到准备
- 在接收模式下：释放总线 → 发出时钟读回数据 → 回到释放
- **不知道**"现在在发第几个字节"——那是宏状态机的事

**宏状态机（事务层）** — 不碰 GPIO。只管"传到哪里了，下一步该干啥"：
- 跟踪当前字节的位进度（bit_cnt 递减）
- 一个字节完成后判断：装下一字节还是收尾
- 事务结束时关定时器 + 回调通知上层

### 3.2 两层如何通信

两层共享同一份上下文结构体（`ctx`），通过其中的字段交流，**不靠函数参数传递**。关键字段包括：

| 字段 | 谁写 | 谁读 | 含义 |
|------|------|------|------|
| `tick` | **微状态机** | 微状态机自己 | "下次 ISR 该走哪个分支" |
| `phase` | **宏状态机** | 微状态机 / 宏状态机自己 | "整个事务进行到哪个阶段"（起始/数据/收尾） |
| `tx_byte` / `rx_byte` | **宏状态机**（装入） | 微状态机（取 bit 写出 / 移位存入） | "当前处理中的字节" |
| `bit_cnt` | **宏状态机**（递减 + 重置） | 微状态机（取第几位）+ 宏状态机自己（判断是否发完） | "当前字节还剩几位" |
| `buf` / `len` / `idx` | **启动时填入** | 宏状态机 | "数据在哪、多长、发到第几个了" |

**两层交互的关键约束：只有一个方向触发——微状态机主动通知宏状态机。**

每次"时钟有效边沿"（即采样/锁存发生的那个 tick 末尾），微状态机调用 `macro_fsm_advance()`，宏状态机才干活。宏状态机不主动抢占——只在被调用时检查进度，然后通过写入 `tx_byte`、`bit_cnt`、`phase` 告诉微状态机"接下来照这样执行"。

### 3.3 一个完整位的执行流程

以同步串行协议的发模式为例，1 bit = 2 ticks：

```
ISR 进入（当前 tick = PREPARE）

微状态机：
  读 tick → PREPARE → 好，先把时钟拉低
  "信号线该设什么？" → 读 tx_byte 和 bit_cnt → 取出对应 bit 写到信号线
  然后 tick = SAMPLE   ← 微告诉"下次走另一个分支"

──────────── 定时器到期，触发下一次 ISR ────────────

ISR 进入（当前 tick = SAMPLE）

微状态机：
  读 tick → SAMPLE → 好，拉高时钟
  "这一位完成了，通知宏"
  → 调用 macro_fsm_advance()   ← ★ 唯一入口，微主动通知宏

宏状态机（在 macro_fsm_advance 内）：
  bit_cnt--                          ← 位计数 -1
  if bit_cnt >= 0: 没事，继续         ← 当前字节还有位
  if bit_cnt < 0:  8 位全发完了      ← 字节边界
    → 如果有响应（如 ACK）：读响应
    → 如果还有数据：装下一字节，bit_cnt 重置
    → 如果没有数据了：phase = TERMINATE

回到微状态机：
  tick = PREPARE   ← 微准备下一轮
```

**关键设计：**
- 微状态机永远按 `tick` 字段走，不自己判断"该不该发下一字节"
- 宏状态机永远等微状态机来调它，不自作主张翻 GPIO
- 这就做到了"换协议只换微状态机，宏状态机结构不动"

### 3.4 处理协议例外

有些协议的起始条件或终止条件**不满足常规的 tick 交替规律**。比如 I2C 的 START 条件要求在时钟高时信号线下降——这与正常数据位"时钟高时信号线必须稳定"的铁律冲突，常规 2-tick 模型吃不消。

**通用处理方式：** 在 ISR 入口处判断 `phase`，如果当前处于特殊阶段，走一个独立的**子 FSM**，不走常规的微/宏状态机路径。

子 FSM 的特点：
- 有自己的步骤计数器（如 `cond_step`），每一步仍然只做 1 个原子动作
- 步骤数固定（通常 2~4 tick），不依赖数据长度
- 执行完后自动把 `phase` 切换回常规数据阶段，后续走正常流程

```
ISR 入口：
  if phase == CONDITION_A:           // 协议自定义的特殊阶段
    → 走子 FSM（步骤 0→1→2→done）
    → phase 切回 DATA
    return
  if phase == CONDITION_B:           // 另一个特殊阶段（如终止条件）
    → 走子 FSM
    → 关定时器 + 回调 + return

  // 常规路径
  micro_state_machine(ctx);          // 正常 2-tick
```

**好处：** 子 FSM 与主流程隔离，不影响微状态机的简洁性。添加新协议的特殊阶段只需新增子 FSM 并注册 phase 枚举值，不需要改微/宏状态机核心逻辑。

### 3.5 完整事务的典型状态流转

```
IDLE
  → COND_A（子 FSM，2~4 tick）    ← 起始条件（如帧头、地址、起始信号）
  → DATA（微+宏 FSM 循环）        ← 逐字节收发（每字节 N tick）
      ↓  宏状态机判断数据发完
  → COND_B（子 FSM，2~4 tick）    ← 终止条件（如帧尾、停止信号）
  → DONE
  → 关定时器
  → done_cb(handle, result)        ← 通知上层
  → IDLE
```

### 3.6 分层的好处总结

| 换什么 | 改哪层 | 另一层要不要动 |
|--------|--------|--------------|
| 换协议（如 I2C → SPI） | 微状态机 | 宏状态机不动 |
| 改变事务流程（如"读前先写地址"） | 宏状态机 | 微状态机不动 |
| 换 GPIO 引脚 | 配置文件/初始化 | 两层都不动 |
| 换平台（如 STM32 → 其他 MCU） | HAL 层 + 微状态机的 GPIO 写函数 | 宏状态机不动 |
| 新增协议特殊阶段 | 新增子 FSM + 注册 phase | 核心微/宏不动 |

---

## 四、定时器配置

### 4.1 周期公式

定时器中断周期 = 1 个 tick 的时间。对于大部分同步串行协议，1 个时钟周期 = 2 个 tick，所以：

| 参数 | 通用公式 | 说明 |
|------|---------|------|
| 中断周期 | 1 / (2 × 协议频率) | I2C@100kHz → 5μs；SPI@1MHz → 0.5μs（注意，0.5μs 可能不够） |
| 定时器时钟 | 源时钟 / PSC | 让计数单位是整数 μs 或 ns，方便计算 ARR |
| ARR（自动重装） | 中断周期 × 定时器时钟频率 - 1 | 即计数器从 0 数到 ARR 的时间 = 中断周期 |

**I2C@100kHz 示例：**

| 参数 | 公式 | 数值 |
|------|------|------|
| 中断周期 | 1 / (2 × 100kHz) | **5μs** |
| 定时器时钟 | APB1/PSC | 90MHz / 90 = **1MHz**（即 1 tick = 1μs） |
| ARR | 5μs × 1MHz - 1 | **4** |

### 4.2 硬件选择

CubeMX 操作：选一个空闲的基本定时器（如 TIM6、TIM7），只开 Update 中断，不用输出通道。设好 PSC 和 ARR 即可。

### 4.3 通用注意事项

- **周期下限：** 中断周期必须 > ISR 耗时（~2μs + 余量），否则 CPU 永远在 ISR 里出不来
- **周期上限：** 太慢会导致协议速率不达标，外设可能超时
- **位宽 ≠ 时钟周期：** 部分协议（如 OneWire）没有独立的时钟线，但仍有严格的时序窗口。此时定时器周期 = 最短时序窗口，通过 tick 计数来获得不同长度的延时
- **开定时器 = 开始消耗 CPU：** 即使没有数据传输，定时器中断也在跑。设计时要保证传输开始才 `HAL_TIM_Base_Start_IT`，传输结束立刻 `HAL_TIM_Base_Stop_IT`

---

## 五、伪代码

### 5.1 微状态机（通用模板）

微状态机的核心结构——tick 交替——在大多数同步协议中都是相同的：

```c
// 通用微状态枚举
enum { TICK_PREPARE, TICK_SAMPLE, TICK_IDLE };

void micro_state_machine(struct ctx *io)
{
    switch (io->tick) {

    case TICK_PREPARE:               // tick 0: 时钟低 / 准备阶段
        gpio_write(CLK, 0);          // 确保时钟低
        if (io->dir == SEND)
            gpio_write(DATA, get_next_bit(io)); // 发送: 放数据到信号线
        else
            gpio_write(DATA, 1);     // 接收: 释放信号线
        io->tick = TICK_SAMPLE;
        break;

    case TICK_SAMPLE:                // tick 1: 时钟高 / 采样阶段
        gpio_write(CLK, 1);          // 发出时钟边沿
        if (io->dir == RECV)
            io->rx_byte = (io->rx_byte << 1) | gpio_read(DATA); // 读回数据
        macro_fsm_advance(io);        // 通知宏状态机: 1 位完成
        io->tick = TICK_PREPARE;
        break;
    }
}
```

> **注意：** 不同协议的微状态机差异就在于 TICK_PREPARE 和 TICK_SAMPLE 这两个 case 里的 GPIO 操作。比如 SPI 的 MOSI/MISO 方向转换、OneWire 的单线读写切换，都只需改动这两个 case 的内部代码，整体结构不变。

### 5.2 宏状态机（通用模板）

宏状态机不碰 GPIO，纯做数据调度：

```c
void macro_fsm_advance(struct ctx *io)
{
    io->bit_cnt--;
    if (io->bit_cnt < 0) {                    // 当前字节结束
        handle_response(io);                   // 协议特有的响应处理（如 ACK）

        if (io->data_idx < io->data_len) {     // 还有数据
            io->tx_byte = io->buf[io->data_idx++];
            io->bit_cnt = 7;
        } else {
            io->phase = PHASE_TERMINATE;        // 全部发完, 走终止子 FSM
        }
    }
}
```

### 5.3 协议特例子 FSM

当协议有阶段不遵循常规 TICK_PREPARE / TICK_SAMPLE 交替时，用子 FSM：

```c
void sub_fsm_start(struct ctx *io)
{
    switch (io->cond_step) {
    case 0:                             // 初始化: 保证总线空闲态
        gpio_write(CLK, 1);
        gpio_write(DATA, 1);
        io->cond_step = 1; break;

    case 1:                             // 核心动作: 发特殊信号
        gpio_write(DATA, 0);            // 例: 时钟高时数据下降 = START
        io->cond_step = 2; break;

    case 2:                             // 恢复: 拉回常规状态
        gpio_write(CLK, 0);
        io->cond_step = 0;
        io->phase = PHASE_DATA;         // 切回常规数据路径
        break;
    }
}
```

### 5.4 ISR 入口（含特例分派）

```c
void TIM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim);
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim, TIM_FLAG_UPDATE);

        struct ctx *io = &g_ctx;

        // 先判断是否处于协议特例阶段
        if (io->phase == PHASE_COND_A) {
            sub_fsm_start(io);           // 走子 FSM
            return;
        }
        if (io->phase == PHASE_COND_B) {
            sub_fsm_stop(io);            // 终止子 FSM
            HAL_TIM_Base_Stop_IT(&htim); // 关定时器
            io->done_cb(io->h, 0);       // 通知上层
            return;
        }

        // 常规路径
        micro_state_machine(io);         // 1 tick
    }
}
```

### 5.5 启动传输

```c
int8_t start_xfer(struct ctx *io, const uint8_t *buf, uint16_t len,
                  done_cb_t cb)
{
    if (io->busy) return -1;

    io->buf       = buf;     io->data_len = len;
    io->data_idx  = 0;       io->bit_cnt  = 7;
    io->busy      = 1;       io->done_cb  = cb;
    io->phase     = PHASE_COND_A;       // 先从起始子 FSM 开始
    io->tick      = TICK_PREPARE;

    HAL_TIM_Base_Start_IT(&htim);
    return 0;                            // CPU 立即返回
}
```

---

## 六、ISR 执行要点补充

### 6.1 每次 ISR 的固定开销

```
清中断标志 → 判断 phase（是否特例阶段）→ 走对应分支 → 退出
   ~0.1μs           1 次对比                       ~2μs
                                            ← 全程 <2μs
```

### 6.2 为什么 ISR 入口要先判断特例

有些协议阶段（如 I2C 的 START/STOP）不能用微状态机的 TICK_PREPARE/TICK_SAMPLE 交替处理——因为微状态机的铁律是"时钟高时信号线不变"，而 START/STOP 恰恰要求在时钟高时改变信号线。

通用方案就是：**ISR 入口先 check phase，命中特例直接走子 FSM 并 return**，没命中才走常规微状态机。这样特例逻辑完全隔离在子 FSM 里，不影响微状态机的简洁性。

### 6.3 关定时器的时机

传输过程中定时器**一直开着**，即使在 IDLE 状态下也要关掉——否则每个 tick 都会触发一次无用的 ISR，白白消耗 CPU。关定时器的唯一时机：

1. 事务正常结束（终止子 FSM 最后一步）
2. 事务异常中断（超时、NACK、总线错误）

关定时器后调用 `done_cb(handle, result)`，result 取 0 表示成功，负数表示失败码。

---

## 七、给上层提供的 API

```c
typedef void (*done_callback_t)(void *handle, int8_t result);

// 启动非阻塞写
int8_t xfer_write_async(handle_t *h, uint8_t reg, uint8_t *buf,
                        uint16_t len, done_callback_t cb);

// 启动非阻塞读
int8_t xfer_read_async(handle_t *h, uint8_t reg, uint8_t *buf,
                       uint16_t len, done_callback_t cb);

// 查询是否正在传输
uint8_t xfer_busy(handle_t *h);
```

**调用示例：**

```c
// 主循环中：
static volatile int i2c_done = 0;

void i2c_done_callback(void *h, int8_t result) {
    i2c_done = 1;
}

void main_loop() {
    // 启动一次 I2C 读（CPU 立即返回）
    i2c_read_reg_async(&hi2c, 0x50, 0x10, rx_buf, 2, i2c_done_callback);

    // CPU 期间可以干这些：
    while (!i2c_done) {
        parse_uart_commands();           // 处理串口
        update_pid_loop();               // 跑 PID
        read_imu_data();                 // 读 SPI 传感器
    }

    // 数据已就绪，消费 rx_buf
    process_rx_data(rx_buf);
}
```

---

## 八、代价

### 8.1 CPU 占用公式

```
CPU 占用 = ISR 耗时 / 定时器中断周期
```

ISR 耗时固定（~2μs），可变的是定时器周期。协议速率越高 → 中断周期越短 → CPU 占用越高。

### 8.2 数值分析

| 协议 | 速率 | 中断频率 | 中断周期 | CPU 占用 | 可行性 |
|------|------|---------|---------|---------|--------|
| I2C | 10kHz | 20kHz | 50μs | ~4% | ✅ 极低 |
| I2C | 50kHz | 100kHz | 10μs | ~20% | ✅ 可接受 |
| **I2C** | **100kHz** | **200kHz** | **5μs** | **~40%** | **⚠️ 勉强（上限）** |
| I2C | 400kHz | 800kHz | 1.25μs | **>100%** | ❌ ISR 来不及退出 |
| SPI | 1MHz | 2MHz | 0.5μs | **>100%** | ❌ 同不可行 |
| SPI | 100kHz | 200kHz | 5μs | ~40% | ⚠️ 勉强 |
| OneWire | 标准模式 | 最短时隙 ~1μs | 不固定 | 可变 | 需按最坏情况评估 |

### 8.3 核心结论

**软件 bit-bang 的非阻塞方案只在低速率下可行。** 实用上限约为中断周期 ≥ 5μs（对应 I2C 100kHz、SPI 100kHz）。更高速率**必须用硬件外设**，因为硬件 ISR 只在传输完成后触发一次，CPU 负载几乎为零。

**速率越低，非阻塞的优势越明显：**
- 10kHz 时 CPU 仅 4%，主循环几乎感觉不到中断的存在
- 100kHz 时 40% 的 CPU 被打走，但对于总线上只有偶尔传输的场景仍可接受
- 连续大量传输 + 高频率时，建议降频或改用硬件外设

### 8.4 降低占用的手段

| 手段 | 效果 | 代价 |
|------|------|------|
| 降低协议速率 | 直接缩放占用（100kHz→50kHz，占用减半） | 传输变慢 |
| 使用硬件外设 | CPU 占用降到零 | 占用硬件引脚 + 外设，不支持任意 GPIO |
| 使用 DMA + 定时器联动 | 部分自动化，减少 ISR 频率 | 配置复杂，并非所有 MCU 支持 |

---

## 九、与项目已有非阻塞手段的对比

| 驱动 | 非阻塞实现 |
|------|-----------|
| PWM | 不需要 — 写寄存器后硬件自己跑 |
| UART 接收 | 外设 ISR**数据来才触发**（不定频） |
| UART 发送 | `HAL_UART_Transmit_DMA` — DMA 后台搬运 |
| **软件 I2C / SPI / OneWire** | **定时器 ISR 主动周期触发** — 本文档描述的模式 |

---

## 十、平台移植清单

> 双层状态机的逻辑（微/宏 FSM、子 FSM、结构体字段）是纯 C，不依赖任何 MCU。需要换的只有**定时器配置 + ISR 入口的 4 行胶水代码**。

### 10.1 ISR 入口逐行对照

以第五章 5.4 的 ISR 代码为例，逐行标注哪些是 STM32 专有、换平台怎么改：

| 行 | STM32 代码 | 作用 | 平台相关性 | 换成其他平台 |
|----|----------|------|-----------|------------|
| ① | `void TIM6_IRQHandler(void)` | ISR 入口函数名 | **专有**——每个 MCU 的中断向量名不同 | ESP32: `void IRAM_ATTR on_timer_isr(void*)`；nRF52: `void TIMER0_IRQHandler(void)`；GD32: 同名直接用；AVR: `ISR(TIMER1_COMPA_vect)` |
| ② | `HAL_TIM_IRQHandler(&htim6)` | HAL 层通用中断分发，内部判断中断源 | **STM32 HAL 专有** | ESP32: 不需要这行，直接用 `timer_get_counter_time_sec()` 判断；nRF52: 不需要，用 `nrfx_timer_event_check()`；其他 ARM MCU 用各自 SDK 的 HAL |
| ③ | `__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)` | 读取定时器 Update 中断标志 | **STM32 HAL 宏**——本质是读 SR 寄存器的某一位 | 所有 MCU 都有类似的状态寄存器，只是宏名不同。ESP32: `timer_isr_callback_add()` 自动清；AVR: `if (TIFR1 & (1 << OCF1A))` |
| ④ | `__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE)` | 清除中断标志，防止重复进入 ISR | **STM32 HAL 宏**——本质是写 SR 寄存器 | ESP32: 框架自动清；AVR: `TIFR1 |= (1 << OCF1A)`；ARM: `TIMER->INTCLR = TIMER_INTCLR_COMPARE0` |
| ⑤ | `struct ctx *io = &g_ctx` | 取全局上下文指针 | **通用** ← 只依赖 C 语言 | **完全不用改** |
| ⑥ | `io->phase == PHASE_COND_A / _B` | 分派子 FSM | **通用** ← 纯 C 逻辑 | **完全不用改** |
| ⑦ | `micro_state_machine(io)` | 走常规微状态机 | **通用** ← 纯 C 逻辑 | 函数体里的 GPIO 操作需换成各平台的 GPIO 写函数，但调用入口不变 |
| ⑧ | `HAL_TIM_Base_Stop_IT(&htim6)` | 停止定时器 | **STM32 HAL API** | ESP32: `esp_timer_stop()`；nRF52: `nrfx_timer_disable()`；AVR: `TIMSK1 &= ~(1 << OCIE1A)` |
| ⑨ | `HAL_TIM_Base_Start_IT(&htim6)` | 启动定时器 | **STM32 HAL API** | ESP32: `esp_timer_start_periodic()`；nRF52: `nrfx_timer_enable()`；AVR: `TIMSK1 |= (1 << OCIE1A)` |

### 10.2 定时器初始化的差异

定时器配置不在 ISR 里，而在 `start_xfer` 调用前的一次性初始化：

| 配置项 | STM32（CubeMX 生成） | ESP32 | nRF52 | AVR |
|--------|---------------------|-------|-------|-----|
| 选定时器 | CubeMX 图形界面配 TIM6/TIM7 | 调用 `esp_timer_create()` API | 调用 `nrfx_timer_init()` | 直接写寄存器 `TCCR1A/B` |
| 周期换算 | `PSC` + `ARR`（分频器 + 自动重装值） | 直接填微秒值，框架自动算 | `CC[0]` 寄存器 = 周期 × 时钟频率 | `OCR1A` = 周期 × 时钟频率 / 预分频 |
| 使能中断 | CubeMX 勾选 NVIC 使能 | `timer_set_alarm()` 自动注册 | `nrfx_timer_compare_int_enable()` | `TIMSK1 |= (1 << OCIE1A)` |
| 注册中断回调 | CubeMX 生成 `stm32f4xx_it.c` 里的 `TIMx_IRQHandler` | `timer_isr_callback_add()` | `nrfx_timer_event_handler_t` | `ISR(TIMER1_COMPA_vect)`（宏自动注册） |

### 10.3 `UserI2C_Design.md` 里的中断部分同理

`UserI2C_Design.md` 第七章的 `i2c_sw_timer_tick()` 就是上述 ISR 入口的真实代码版本，里面的 `HAL_TIM_xxx` 宏和 `__HAL_TIM_GET_FLAG` 替换方式完全一致。核心逻辑（判断 `xfer` → 分派子 FSM / 微状态机）一行不动。

### 10.4 迁移步骤（实操顺序）

1. **把 `struct ctx` 和所有 FSM 函数复制过去**——这些是纯 C，零依赖
2. **在新平台上配一个定时器，设好周期**（按第四章公式，不限于 CubeMX）
3. **把 `HAL_TIM_Base_Start_IT` / `Stop_IT` 替换为新平台 API**（共 2 处：`start_xfer` 里 1 处 + ISR 里 `PHASE_COND_B` 分支 1 处）
4. **把 ISR 入口的前 4 行 STM32 胶水代码替换为新平台的等价形式**（函数签名 + 标志位宏，对照 10.1 表）
5. **把 GPIO 写函数替换为新平台 API**（`gpio_write(CLK, 0)` → 新平台的设置引脚电平函数）

5 个文件要改，但总共改动 <30 行，核心状态机逻辑不变。
