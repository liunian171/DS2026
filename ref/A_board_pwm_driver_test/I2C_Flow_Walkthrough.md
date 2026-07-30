# I2C 收发流程详解

> 以 TIM6 4μs 定时器中断驱动，全程非阻塞。以下均以实际 tick 为单位。

---

## 一、动态状态量（7 个）

| 字段 | 范围 | 变化频率 | 管理者 |
|------|------|---------|--------|
| `micro_state` | 0↔1 | 每 tick | fsm_bit |
| `macro_state` | IDLE→START→SEND→(RECV)→STOP→DONE→IDLE | 每阶段 | start_transfer / fsm_edge / fsm_byte |
| `bit_counter` | 7→0→-1→7 | 每 tick | fsm_byte |
| `edge_step` | 0→1→2 | START/STOP 每 tick | fsm_edge |
| `byte_index` | 0→1→2→... | 每字节完成 | fsm_byte |
| `send_byte` | 设备地址 / 寄存器地址 / data[i] | 每字节完成 | fsm_edge(首发) / fsm_byte |
| `receive_byte` | 逐 bit 左移 | 每 tick(RECV) | fsm_bit |

---

## 二、写事务流程（例：发 2 字节数据）

```
器件地址=0x50, 寄存器=0x00, data[2]={0xAA, 0x55}
起始: macro_state=IDLE, byte_index=0, bit_counter 未初始化
```

### Phase 0: 启动

```
应用层调用 i2c_write_reg_async(...)
  → start_transfer
      ├─ 填入 device_address=0x50, register_address=0x00, data_buffer={0xAA,0x55}, data_length=2
      ├─ byte_index=0
      ├─ micro_state=PREPARE, edge_step=0
      ├─ macro_state=START
      └─ timer_start() → TIM6 开始每 4μs 中断

──────────────── 4μs 后开始 ────────────────
```

### Phase 1: START（fsm_edge，3 tick）

```
ISR#0  macro_state=START → fsm_edge
       edge_step=0: SCL=1, SDA=1  → edge_step=1

ISR#1  edge_step=1: SCL=1, SDA=0 ★下降沿★ → edge_step=2

ISR#2  edge_step=2: SCL=0, SDA=0
       → macro_state=SEND
       → send_byte = (0x50<<1|0) = 0xA0   ← 首发球：设备地址+W
       → bit_counter = 7
```

### Phase 2: 发送设备地址（fsm_bit + fsm_byte，18 tick）

```
ISR#3  macro_state=SEND → fsm_bit
       micro_state=PREPARE: SCL=0, SDA=1 (0xA0的bit7) → micro_state=SAMPLE

ISR#4  micro_state=SAMPLE: SCL=1 → fsm_byte: bit_counter 7→6 → micro_state=PREPARE

...bit6~bit1, 12 tick...

ISR#17 PREPARE: SCL=0, SDA=0 (bit0)

ISR#18 SAMPLE: SCL=1 → fsm_byte: bit_counter 0→-1 → micro_state=PREPARE

ISR#19 PREPARE: SCL=0, SDA=1 ★释放SDA★(ACK位，从机拉低表示ACK)

ISR#20 SAMPLE: SCL=1 → fsm_byte: bit_counter==-1
        → 读SDA=0(ACK!)
        → byte_index==0 → send_byte=0x00(寄存器地址)
        → byte_index=1, bit_counter=7
```

### Phase 3: 发送寄存器地址（18 tick，同上模式）

```
...18 tick...
ISR#38 fsm_byte: bit_counter==-1, ACK OK
        → byte_index==1 → is_read=0(写事务) → send_byte=data_buffer[0]=0xAA
        → byte_index=2, bit_counter=7
```

### Phase 4: 发送 data[0]（18 tick）

```
...18 tick...
ISR#56 fsm_byte: bit_counter==-1, ACK OK
        → byte_index>=2 → 下一数据下标=byte_index-1=2, 但 data_length=2
        → 2 >= 2 → 没数据了
        → send_byte=data_buffer[2-1]=data_buffer[1]=0x55
        → byte_index=3, bit_counter=7
```

### Phase 5: 发送 data[1]（18 tick）

```
...18 tick...
ISR#74 fsm_byte: bit_counter==-1, ACK OK
        → byte_index≥2 → 下一数据下标=byte_index-1=3
        → 3 >= data_length(2) → 全部发完！
        → macro_state=STOP, edge_step=0
```

### Phase 6: STOP（fsm_edge，3 tick）

```
ISR#75 macro_state=STOP → fsm_edge
       edge_step=0: SCL=0, SDA=0 → edge_step=1

ISR#76 edge_step=1: SCL=1, SDA=0 → edge_step=2

ISR#77 edge_step=2: SCL=1, SDA=1 ★上升沿★
       → timer_stop() ★定时器关闭
       → macro_state=DONE
       → complete_callback(ctx, 0) ★通知调用者
       → macro_state=IDLE
```

### 总结

```
发 2 字节数据 = START(3) + 设备地址(18) + 寄存器地址(18) + data[0](18) + data[1](18) + STOP(3)
             = 78 次 ISR × 4μs = 312μs
CPU 占用 = 78 × 2μs = 156μs (分布在 312μs 中，约占 50%)
主循环在此期间永不冻结。
```

---

## 三、读事务流程（例：读 2 字节数据）

```
器件地址=0x50, 寄存器=0x00, 读 2 字节
```

### Phase 1~2: 同写事务（START → SEND 设备地址+W → SEND 寄存器地址）

```
...38 tick...
byte_index=1, is_read=1(读事务!)
```

### Phase 3: RESTART（fsm_byte 触发）

```
ISR#38 fsm_byte: bit_counter==-1, ACK OK
        → byte_index==1, is_read==1 → RESTART!
        → send_byte = (0x50<<1|1) = 0xA1(设备地址+R)
        → macro_state = START
        → edge_step = 0
        → byte_index=2, bit_counter=7
        → return (跳过 byte_index++)

ISR#39~41 fsm_edge: START 3 tick

ISR#42 macro_state=SEND
        send_byte=0xA1(设备地址+R), bit_counter=7
```

### Phase 4: 发送设备地址+R（18 tick）

```
...18 tick...
ISR#60 fsm_byte: bit_counter==-1, ACK OK
        → byte_index==2, 但 macro_state==SEND
        → 这是 RESTART 后的地址字节
        → 需要切 RECV

★ 注意：当前代码 SEND 分支 byte_index>=2 时只处理数据字节。
   需要在 fsm_byte SEND 分支判断：byte_index==2 && is_read==1 → 切 macro_state=RECV
   或者：RESTART 回来的 macro_state=SEND 发完地址+R 后 fsm_byte 无特殊处理，
   下一个 SAMPLE 调 fsm_byte 时 byte_index 仍然是 2，会走数据分支。

   实际上：RESTART 时 byte_index 已经是 2（在 fsm_byte 的 byte_index==1 分支里 +1 了），
   然后 RESTART SEND 发地址+R，发完后 fsm_byte 被调，byte_index==2。
   
   此时需要：
   - macro_state 切到 RECV
   - byte_index 保持在 2（不要 +1，因为数据从 byte_index-2 开始）
   - bit_counter=7
```

### Phase 5: RECV 数据（每字节 18 tick）

```
RECV, byte_index=2:
  fsm_bit PREPARE: SCL=0, SDA=1(释放)
  fsm_bit SAMPLE:  SCL=1, receive_byte <<=1 | 读SDA
  ...16 tick...
  fsm_bit #ACK PREPARE: SCL=0, SDA=数据位(NACK判断)
  fsm_bit #ACK SAMPLE:  SCL=1

  fsm_byte: bit_counter==-1
    → 存 ctx->data_buffer[2-2]=data_buffer[0]=receive_byte
    → 还有字节 → byte_index=3, bit_counter=7

RECV, byte_index=3:
  ...收 data[1]...
  fsm_byte: bit_counter==-1
    → 存 ctx->data_buffer[3-2]=data_buffer[1]=receive_byte
    → (byte_index-2)+1 = 2 >= data_length(2) → 全部收完
    → macro_state=STOP, edge_step=0

→ STOP 3 tick → timer_stop → callback
```

---

## 四、函数调用链总结

```
timer_isr（每 4μs）
  ├─ macro_state==START/STOP → fsm_edge
  │    3 tick START: SCL/SDA 翻 3 次 → macro_state=SEND, 装首发球
  │    3 tick STOP:  SCL/SDA 翻 3 次 → timer_stop + callback
  │
  └─ macro_state==SEND/RECV → fsm_bit
       ├─ PREPARE: SCL=0, 设 SDA
       ├─ SAMPLE:  SCL=1, 读 SDA(RECV)
       └─ fsm_byte: bit_counter管理 + 字节调度
```

---

## 五、关键数字

| 事务 | tick 数 | 时间(4μs/tick) |
|------|---------|---------------|
| START | 3 | 12μs |
| STOP | 3 | 12μs |
| 1 字节(8bit+ACK) | 18 | 72μs |
| 写 2 字节 | 3+18+18+18+18+3=78 | 312μs |
| 读 2 字节 | 3+18+18+3+18+18+18+3=99 | 396μs |
