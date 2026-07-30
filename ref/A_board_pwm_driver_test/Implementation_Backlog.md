# 可实现功能点清单

> 记录所有驱动模块中可落地的功能点，按紧急度排序。
> 设计理念参考 `Design_Patterns.md` / `Design_Philosophy.md`。

---

## 🔴 P0 — 阻塞功能（必须优先）

### 1. I2C 驱动核心实现
| 子任务 | 位置 | 状态 |
|--------|------|------|
| ① `i2c_software_is_busy()` | `useri2c_ops.c` | ✅ |
| ② `i2c_software_platform_ops_stm32` 实例 | `useri2c_ops.c` | ✅ |
| ③ `i2c_software_start_transfer()` | `useri2c_ops.c` | ✅ |
| ④ `i2c_software_timer_isr()` — 双层 FSM | `useri2c_ops.c` | ✅ |
| ⑤ `i2c_is_busy()` | `useri2c.c` | ✅ |
| ⑥ `i2c_write_reg_async()` | `useri2c.c` | ✅ |
| ⑦ `i2c_read_reg_async()` | `useri2c.c` | ✅ |

### 2. I2C ISR 集成（硬件对接）
| 子任务 | 位置 | 状态 |
|--------|------|------|
| TIM6_DAC_IRQHandler 中调用 `i2c_software_timer_isr()` | `stm32f4xx_it.c` | ✅ |
| main.c 中 `HAL_TIM_Base_Start_IT(&htim6)` | `main.c` | ✅ |

### 3. motor_bridge.cpp 语法错误
| 问题 | 位置 | 说明 |
|------|------|------|
| 第 4 行缺少分号 | `motor_bridge.cpp` | `= {nullptr}` → `= {nullptr};` |

---

## 🟡 P1 — 功能缺口（影响使用）

### 4. uart_flush_rx 实现
| 说明 | 位置 |
|------|------|
| 当前是空壳 `return 0;`，遇到 RX 噪声无法恢复 | `uart.c` |
| 实现方式：`HAL_UART_AbortReceive()` 或循环读 DR 直到 RXNE=0 | — |

### 5. CMD_PWM_SET_DUTY 解除注释
| 说明 | 位置 |
|------|------|
| 整个 case 分支被注释，串口无法控制 PWM 占空比 | `uart_cmd_parser.c:267-273` |
| 前提：需先定义 PWM 实例数组 `pwm_ch[]` | — |

### 6. 硬件自检（health check）
| 子任务 | 所属模块 | 说明 |
|--------|---------|------|
| `i2c_sw_self_check()` | `useri2c_ops.c` | 检查引脚指针、上拉电平（用 `usergpio_read` 读回 SCK/SDA 高电平） |
| 综合检查函数 `health_check()` | 新文件或 `main.c` | 统一调度各驱动自检 + 串口打印结果 |
| 可扩展：`pwm_self_check()`、`gpio_self_check()` | 各模块 | 按需追加 |

### 7. 调试指示灯
| 子任务 | 说明 |
|--------|------|
| 心跳灯 | GPIO 500ms 翻转（Systick 或主循环定时），证明主循环没死 |
| 通信灯 | UART 收/发时翻转 GPIO，证明串口通路正常 |

### 8. 启用电机初始化
| 说明 | 位置 |
|------|------|
| main.c 中整个电机初始化代码被 `#if 0` 禁用 | `main.c:165` |
| 改为 `#if 1` 或直接去掉条件编译 | — |

### 9. 里程计上报
| 说明 | 位置 |
|------|------|
| 编码器脉冲 → 差速模型 → ΔX/ΔY/Δθ → 串口上报 | 待新建 `encoder_odom.c/h` |
| 缺轮距参数 | — |

### 10. 硬件 I2C ops 实现
| 说明 | 位置 |
|------|------|
| 目前 Bluepill 版 OLED 直接调 `HAL_I2C_Mem_Write`，破坏了 ops 抽象 | 待新建 `i2c_hardware_ops.c/h` |
| 原工程不受影响 | — |

---

## 🟢 P2 — 架构改进（低优先级）

### 11. PWM 实例分离
| 说明 | 位置 |
|------|------|
| `pwm_tim5_ch4` 实例从 `pwm.c` 移至 `pwm_instance.c` | `pwm.c` → 新文件 |
| 使 `pwm.c` 不再依赖 `tim.h`，恢复跨平台编译能力 | — |

### 12. Ch_State 自动维护
| 说明 | 位置 |
|------|------|
| `PWM_Handle.Ch_State` 初始化后永不更新 | `pwm.h` / `pwm.c` |
| 在 `set_duty_0E3` / `set_freq` 中更新状态 | — |

### 13. UART 多实例支持
| 说明 | 位置 |
|------|------|
| 帧解析状态机 (`g_rx_frame` / `g_rx_index` / `g_in_frame`) 当前是全局 static | `uart_cmd_parser.c` |
| 迁移到 `UART_Handle` 或 `UART_Instance` 结构体内 | — |

### 14. Fault Handler 诊断输出
| 说明 | 位置 |
|------|------|
| HardFault/MemManage/BusFault/UsageFault 当前只死循环 | `stm32f4xx_it.c` |
| 加寄存器 dump / 错误码打印到串口 | — |

---

## 🔵 P3 — 预留拓展（未来功能）

### 15. 速度闭环
| 说明 |
|------|
| PID 控制器已实现（`common/pid.c`），需在 main.c 中加 1ms 定时调用 |
| 编码器读数 → 换算 RPM → PID 计算 → 设电机 RPM |

### 16. 位置闭环
| 说明 |
|------|
| 里程计给出当前位置 → 外层 PID → 目标速度给速度环 |
| 依赖里程计上报模块完成 |

### 17. I2C 硬件外设 ops
| 说明 |
|------|
| 利用 `I2C_PlatformOps_t` 的 `void* i2c_context` → `I2C_HandleTypeDef*` 能力 |
| 新增 `i2c_hardware_ops.c`，start_transfer 调 `HAL_I2C_Mem_Write/Read_IT` |
| 上层策略层 `useri2c.c` 一行不改，换 ops 表即可切换软/硬 I2C |

### 18. DMA 驱动
| 参考文档 |
|------|
| `DMA_Driver_Analysis.md` 已分析可行性，未实现 |

### 19. Servo 拓展（设计文档中列出）
| 功能 | 优先级(原文档) |
|------|---------------|
| `getAngle()` | P0 |
| `setAngleLimit()` | P0 |
| `setAngleWithDuration()` — 平滑转动 | P1 |
| `setAngleSync()` — 多舵机同步 | P2 |
| `setDeadBand()` | P2 |
| `powerOff()` | P2 |
| ServoUART / ServoI2C 派生类 | 预留 |

### 20. 多协议扩展（枚举已定义）
| 枚举 | 所属模块 | 实现类 |
|------|---------|--------|
| `MOTOR_PROTOCOL_L298N = 1` | motor_bridge.h | 未实现 |
| `SERVO_PROTOCOL_UART = 1` | servo_bridge.h | 未实现 |

### 21. 死代码清理
| 项目 | 说明 |
|------|------|
| `TIM_PWM_g_Param` 结构体 | 全工程无引用 |
| `PWM_Handle.PWM_CCR` 字段 | 预留缓存，从未使用 |
| `TB6612CH` 枚举（motor_protocol.cpp） | 被注释掉的残留代码 |
| `Encoder_Handle.CH_A` / `CH_B` | 预留字段，当前未在驱动逻辑中使用 |

### 22. usertimer 定时器抽象层
| 说明 |
|------|
| 对标 `usergpio.h/c`，封装 `HAL_TIM_Base_Start_IT` / `Stop_IT` 等基本操作 |
| I2C 当前用 2 个 static helper 临时包裹 HAL，可行但不通用 |
| 等 Servo `setAngleWithDuration` 或其他模块也需要定时器时，抽成 `usertimer.h/c` |
| 原则：**只有 1 个使用者时不做抽象，≥2 个时再抽** |

---

## 📋 变更摘要（按时间倒序）

| 日期 | 内容 |
|------|------|
| 2026-07-20 | 新增 PID 控制器（`common/pid.c/h`），位置式+增量式+级联 |
| 2026-07-20 | 新增 `Porting_Guide.md` 移植注意事项文档 |
| 2026-07-20 | 新增 `Document_Index.md` 文档索引 |
| 2026-07-19 | Bluepill 移植完成（`Bluepill_Porting_Plan.md` + `Bluepill_Porting_Pitfalls.md`） |
| 2026-07-19 | 编码器驱动完成（`encoder.c/h` + `encoder_platform_ops.c`） |
| 2026-06-16 | 新增 19. usertimer 定时器抽象层（P3） |
| 2026-06-16 | 初始创建，从全模块审计中提取所有可实现点 |
