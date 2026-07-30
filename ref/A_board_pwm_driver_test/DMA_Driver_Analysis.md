# DMA 驱动——是否需要、何时需要

> 基于项目"策略层 + ops + HAL"分离模式，分析 DMA 驱动的定位。

---

## 一、DMA 的角色：基础设施，不是功能外设

```
功能型外设（项目已有驱动）：
  PWM  → 产生波形
  UART → 收发串行数据     这些是"终端"—做的事有语义
  I2C  → 读写寄存器
  GPIO → 翻引脚

基础设施型外设：
  DMA  → 不产生数据，不消费数据
         只负责把 A 处的数据搬到 B 处
         必须绑定到另一个外设上
```

**你没有独立的 `dma_send(data, len)` ——它总是作为加速器依附于其他外设：**

```c
HAL_UART_Transmit_DMA(&huart7, buf, len);     // DMA 跟随 UART
HAL_SPI_Transmit_DMA(&hspi1, buf, len);       // DMA 跟随 SPI
HAL_I2C_Mem_Write_DMA(&hi2c1, ...);           // DMA 跟随 I2C
```

---

## 二、DMA 在你的项目里如何存在

DMA 不是驱动模块，是**被注入到外设平台实现里的硬件资源**：

```
CubeMX 生成的 dma.c:
  hdma_uart7_tx  (DMA1, Stream1, Channel4)
  hdma_uart7_rx  (DMA1, Stream3, Channel4)
         │ 注入（编译时静态绑定）
         ▼
usart.c:
  huart7.hdmatx = &hdma_uart7_tx    ← HAL 句柄内部已经持有 DMA 指针
         │
         ▼
uart_platform_ops.c:
  HAL_UART_Transmit_DMA(huart, ...)  ← 句柄里自带 DMA 信息
```

**你写的驱动层完全不需要感知 DMA。**

| 层 | 看到 DMA 了吗 | 代码改动 |
|----|-------------|---------|
| 策略层 (`uart.c`) | ❌ 不知道 | 不变 |
| 平台实现 (`uart_platform_ops.c`) | ✅ 换一行后缀 | `HAL_UART_Transmit` → `HAL_UART_Transmit_DMA` |
| 配置层 (`dma.c`) | ✅ CubeMX 生成 | 不变 |

不需要独立的 `dma.h` / `dma.c` / `dma_platform_ops.c`。现有的 `hdma_xxx` 全局变量已经在被使用的外设驱动里消化掉了。

---

## 三、什么情况下需要 DMA 驱动

**唯一场景：DMA 流不够用，需要运行时动态分配。**

### 举例说明

```
STM32F427 有 2 个 DMA 控制器 × 8 个流 = 16 个流。

假设你的项目需要 DMA 的外设：
  UART7_TX, UART7_RX            → 2 个流
  SPI1_TX, SPI1_RX              → 2 个流
  SPI2_TX, SPI2_RX              → 2 个流
  ADC1                          → 1 个流
  I2C1_TX, I2C1_RX              → 2 个流
  TIM1_CH1 (给电机驱动)          → 1 个流
  共 10 个，还有 6 个空闲        → 静态分配完全够用
```

**但如果项目有 20 个外设需要 DMA（比如跑 RTOS，任务动态创建/销毁外设），16 个流装不下，就需要"借还"机制：**

```c
// DMA 驱动提供的能力：
DMA_HandleTypeDef *hdma = dma_stream_alloc(DMA_CHANNEL_4, &huart7, DMA_MEMORY_TO_PERIPH);
if (hdma == NULL) return -1;  // 没有空闲流了

HAL_UART_Transmit_DMA(&huart7, buf, len);  // 用分配的流
dma_stream_free(hdma);                     // 用完了归还
```

### DMA 驱动的职责

| 职责 | 说明 |
|------|------|
| **流分配（alloc）** | 遍历 16 个流，找到空闲的 → 配置通道号/方向 → 标记占用 → 返回句柄 |
| **流释放（free）** | 取消标记，允许其他外设使用 |
| **冲突检测** | 同一流被两个外设同时申请 → 报错或排队 |
| **当前状态查询** | 哪些流空闲、哪些被谁占用 |

**它不管数据怎么传——`HAL_DMA_Start` 传输本身是 HAL 提供的，DMA 驱动只管"借出去、收回来"。**

---

## 四、硬件限制：不是任意流配任意外设

每个外设的 DMA 请求信号在芯片设计时已经硬连到特定的流和通道：

| 外设 | 允许的 DMA 绑定 |
|------|----------------|
| UART7_TX | **只能** DMA1_Stream1_Channel4 |
| UART7_RX | **只能** DMA1_Stream3_Channel4 |
| SPI1_TX | **只能** DMA1_Stream3_Channel3 |
| ADC1 | **只能** DMA2_Stream0_Channel0 |

流编号和通道号是硬件写死的，CubeMX 下拉框只显示合法的组合。**不存在"UART7 TX 借给 SPI1 用"**——不同的通道号本来就分在不同流上，不存在竞争。

这也进一步降低了动态分配的必要性——硬件天然隔离了不同外设的 DMA 请求。

---

## 五、静态 vs 动态对比

| 维度 | 静态分配（CubeMX） | 动态分配（DMA 驱动） |
|------|--------------------|---------------------|
| 复杂度 | 零——CubeMX 点选 | 需要完整的管理器（追踪 16 个流的状态） |
| 运行时开销 | 零 | 每次 alloc 遍历流、改 Init、HAL_DMA_Init |
| 确定性 | 编译时就确定，不会失败 | 运行时可能 `alloc` 返回 NULL |
| 适用场景 | 99% 的嵌入式项目 | 外设数 > DMA 流数，或外设动态创建 |
| 代码量 | 0 行 | ~200 行 + 测试 |

---

## 六、结论

✅ **不需要 DMA 驱动**——CubeMX 静态分配对绝大多数项目完全足够。

DMA 驱动的价值不在"让 DMA 跑起来"，而在**"16 个流不够分时的动态调度"**。按项目"单例先行，扩展后抽"原则，等到确实有一个外设因为没 DMA 流可用而跑不了，那时再写管理器——且那时的改动也只增加一个 `dma_stream_alloc/free` 模块，不影响现有任何外设驱动代码。

---

## 七、与项目驱动的位阶对照

```
驱动层（核心驱动）：
  PWM_Handle    → 句柄 + ops
  UART_Handle   → 句柄 + ops
  I2C_Handle    → 句柄 + ops

非驱动（硬件资源）：
  htim5         → 被 PWM 注入使用
  huart7        → 被 UART 注入使用
  hdma_xxx      → 被 UART 注入使用（同样的注入模式）

不存在：
  DMA_Handle + DMA_PlatformOps_t  ← 不需要
```
