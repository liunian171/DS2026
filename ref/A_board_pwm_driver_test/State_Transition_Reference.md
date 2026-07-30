# I2C FSM 状态转换参考

> 所有字段的变更条件、变更位置、完整写事务轨迹。代码实现时对照此文档。

---

## 一、状态变更速查

### macro_state（事务阶段）

| 从 | 到 | 条件 | 变更位置 |
|----|-----|------|---------|
| IDLE | START | `start_transfer` 被调用 | `start_transfer` |
| START | SEND | fsm_edge START step2 完成 | `fsm_edge` |
| SEND | START | `byte_index==1 && is_read==1`（读 RESTART） | `fsm_byte` |
| SEND | RECV | `byte_index==2 && is_read==1`（地址+R发完） | `fsm_byte` |
| SEND | STOP | NACK 或 `byte_index >= data_length+2` | `fsm_byte` |
| RECV | STOP | `byte_index >= data_length+2` | `fsm_byte` |
| STOP | DONE | fsm_edge STOP step2 完成 | `fsm_edge` |
| DONE | IDLE | `complete_callback` 执行完毕 | `fsm_edge`（回调后） |

### micro_state（逐 bit 节奏）

| 从 | 到 | 条件 | 变更位置 |
|----|-----|------|---------|
| PREPARE(0) | SAMPLE(1) | PREPARE 分支执行完毕 | `fsm_bit` 末尾 |
| SAMPLE(1) | PREPARE(0) | SAMPLE 分支执行完毕 | `fsm_bit` 末尾 |

无条件交替，每 4μs 一次。

### bit_counter（bit 进度，`int8_t`）

| 当前值 | 变化 | 条件 | 变更位置 |
|--------|------|------|---------|
| 7→1 | `bit_counter--` | `bit_counter > 0` | `fsm_byte` |
| 0 | `bit_counter = -1` | 8 bit 完，进 ACK | `fsm_byte` |
| -1 | `bit_counter = 7` | ACK 完，装下一字节 | `fsm_byte` |

### edge_step（边沿步骤）

| 当前值 | 变化 | 条件 | 变更位置 |
|--------|------|------|---------|
| 0 | →1 | tick0 完成 | `fsm_edge` |
| 1 | →2 | tick1 完成 | `fsm_edge` |
| 2 | 结束 | START/STOP 完成 | `fsm_edge` |

### byte_index（字节序号，`uint16_t`）

| 当前值 | 变化 | 条件 | 变更位置 |
|--------|------|------|---------|
| — | 0 | 事务初始化 | `start_transfer` |
| 0 | 1 | 设备地址发完（ACK OK） | `fsm_byte` |
| 1 | 2 | 寄存器地址发完（ACK OK） | `fsm_byte` |
| ≥2 | +1 | 每字节数据发/收完（ACK OK） | `fsm_byte` |

`byte_index` 含义：0=设备地址，1=寄存器地址，≥2=数据字节。访问 data_buffer 用 `byte_index - 2`。

---

## 二、写事务完整字段变化轨迹

```
触发者          macro_state  micro_state  bit_counter  edge_step  byte_index
────────────────────────────────────────────────────────────────────────────
start_transfer  IDLE→START   PREPARE      未初始化      0          0
fsm_edge(#0)    START        —            —            0→1        —
fsm_edge(#1)    START        —            —            1→2        —
fsm_edge(#2)    START→SEND   —            7            —          —
fsm_bit#PREPARE —            PREPARE→SAMPLE —          —          —
fsm_bit#SAMPLE  —            —            7→6          —          —
  └fsm_byte     —            —            7→6          —          —
...（7→0, 16次 ISR）...
fsm_bit#PREPARE —            —            0            —          —
fsm_bit#SAMPLE  —            —            0→-1         —          —
  └fsm_byte     —            —            0→-1         —          —
fsm_bit#PREPARE —            PREPARE→SAMPLE —          —          —（ACK PREPARE，SDA=1释放）
fsm_bit#SAMPLE  —            —            —           —          —（ACK SAMPLE，SCL=1）
  └fsm_byte     —            —            -1→7        —          0→1 ★ 设备→寄存器
  └fsm_byte     —            —            7           —          —

（寄存器地址 18次 ISR 同上）
  └fsm_byte     —            —            -1→7        —          1→2 ★ 寄存器→data[0]
  └fsm_byte     —            —            7           —          —

（data[0] 18次 ISR）
  └fsm_byte     —            —            -1→7        —          2→3

（data[1] 18次 ISR，发完）
  └fsm_byte     SEND→STOP    —            —           0          —

fsm_edge(#0)    STOP         —            —            0→1        —
fsm_edge(#1)    STOP         —            —            1→2        —
fsm_edge(#2)    STOP→DONE    —            —            —          —
  └callback     DONE→IDLE    —            —            —          —
```

---

## 三、FSM 分派逻辑（timer_isr 入口）

```
timer_isr 读 macro_state:
  │
  ├─ START  → fsm_edge  处理边沿（START step0~2）
  ├─ STOP   → fsm_edge  处理边沿（STOP step0~2）
  │
  ├─ SEND   → fsm_bit   逐 bit 微状态机（PREPARE↔SAMPLE）
  │             └─ SAMPLE 末尾 → fsm_byte 推进
  │
  ├─ RECV   → fsm_bit   同上
  │
  └─ DONE/IDLE → 兜底（不应到达）
```

---

## 四、ACK 处理

| 模式 | 谁发 ACK | fsm_bit PREPARE 做什么 | fsm_byte 做什么 |
|------|---------|----------------------|---------------|
| SEND | 从机 | SDA=1 释放，让从机拉低 | 读 SDA：0=ACK ✅，1=NACK→STOP |
| RECV | 主机 | SDA=0(ACK)/1(NACK)，判断是否最后一字节 | 数据收完存 buffer，切 STOP 或继续 |

### RECV ACK 决议公式

```c
// fsm_bit PREPARE 分支，bit_counter==-1 时
uint16_t data_idx = ctx->byte_index - 2;
if (data_idx < ctx->data_length - 1)
    SDA = 0;   // ACK：还有字节要收
else
    SDA = 1;   // NACK：最后一字节
```

---

## 五、各函数职责矩阵

| 函数 | 写 macro | 写 micro | 写 bit | 写 edge | 写 byte | 翻 GPIO |
|------|---------|---------|--------|--------|--------|---------|
| `start_transfer` | ✅ | ✅ | — | ✅ | ✅ | — |
| `timer_isr` | — | — | — | — | — | — |
| `fsm_edge` | ✅ (STOP→DONE) | — | ✅ (7) | ✅ | — | ✅ |
| `fsm_bit` | — | ✅ (^=1) | — | — | — | ✅ |
| `fsm_byte` | ✅ | — | ✅ | ✅ (STOP时) | ✅ | — |

---

## 六、read 事务的特殊路径

```
IDLE
 → START（fsm_edge）
 → SEND: device_addr+W  （byte_index=0）
 → SEND: register_addr  （byte_index=1）
        ↓ is_read=1 → macro_state=START, send_byte=device_addr+R
 → START（fsm_edge 重开）
 → SEND: device_addr+R  （byte_index=2）
        ↓ fsm_byte 判断 → macro_state=RECV
 → RECV: data[0]        （byte_index=2，读入 data_buffer[0]）
 → RECV: data[1]
 → ...直到 byte_index >= data_length+2
 → STOP → DONE → IDLE
```

macro_state 轨迹: `IDLE → START → SEND → START → SEND → RECV → STOP → DONE → IDLE`
