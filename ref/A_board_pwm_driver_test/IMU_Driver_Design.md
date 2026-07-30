# IMU 驱动设计

> 遵循项目四层架构。核心创新：用**参数表驱动 + ImuBase 默认实现**替代每芯片各自覆写，MPU 系列加新型号只需填一张表。

---

## 目录

- [一、为什么跟 Motor 层数不同](#一为什么跟-motor-层数不同)
- [二、整体架构](#二整体架构)
- [三、类继承体系](#三类继承体系)
  - [3.1 三层类结构](#31-三层类结构)
  - [3.2 定制行为：子类如何特化](#32-定制行为子类如何特化)
  - [3.3 父类没有的方法子类能否定义](#33-父类没有的方法子类能否定义)
- [四、文件清单](#四文件清单)
- [五、接口设计](#五接口设计)
  - [5.1 IIMU（芯片能力接口）](#51-iimu芯片能力接口)
  - [5.2 ImuChipDesc（参数表）](#52-imuchipdesc参数表)
  - [5.3 ImuBase（MPU 系列默认实现）](#53-imubasempu-系列默认实现)
  - [5.4 MPU6050（芯片类）](#54-mpu6050芯片类)
  - [5.5 ImuFilter（通用数据处理）](#55-imufilter通用数据处理)
  - [5.6 Bridge（C 接口）](#56-bridgec-接口)
- [六、数据流](#六数据流)
- [七、扩展方式](#七扩展方式)
  - [7.1 加新芯片（MPU 系列）](#71-加新芯片mpu-系列)
  - [7.2 加新芯片（非 MPU 系列）](#72-加新芯片非-mpu-系列)
  - [7.3 加 SPI 传输](#73-加-spi-传输)
  - [7.4 加 IBusTransport 后的改动范围](#74-加-ibustransport-后的改动范围)
- [八、与项目现有模块对照](#八与项目现有模块对照)
- [九、设计决策汇总](#九设计决策汇总)
- [十、MPU6050 寄存器速查](#十mpu6050-寄存器速查)

---

## 一、为什么跟 Motor 层数不同

```
┌── Motor 四层 ──────────────────────┐  ┌── IMU 结构 ───────────────────────┐
│                                    │  │                                  │
│  motor_bridge (C 桥)               │  │  imu_bridge (C 桥)               │
│       │                            │  │       │                          │
│       ▼                            │  │       ▼                          │
│  Motor 类（功能层）                 │  │  IIMU（芯片能力接口）              │
│  职责：RPM/死区/轮子                │  │  职责：read_accel/gyro/temp       │
│  持有：IMotorProtocol&              │  │        ↑ 顶层接口                 │
│       │                            │  │       │                          │
│       ▼                            │  │  ImuBase（MPU 系列默认实现）       │
│  IMotorProtocol（协议层）           │  │  职责：参数表驱动的通用逻辑         │
│  职责：千分比→硬件信号翻译           │  │        init/read/scale 只写一次    │
│  TB6612 / L298N                    │  │        ↑ 中层通用                 │
│       │                            │  │       │                          │
│       ▼                            │  │  MPU6050 / MPU9250               │
│  PWM + GPIO（驱动层）               │  │  只填 ImuChipDesc 表              │
│                                    │  │  或覆写特定方法实现特殊功能         │
│                                    │  │        ↑ 中层芯片                 │
│                                    │  │       │                          │
│                                    │  │  I2C / SPI（驱动层）              │
└────────────────────────────────────┘  └──────────────────────────────────┘
```

### 差异根源

| | Motor/Servo | IMU |
|---|---|---|
| **中层体为什么拆** | 真值表翻译 和 RPM 换算是两个独立维度 | 寄存器地址 和 量程换算绑定在同一颗芯片上 |
| **为什么有 Protocol 层** | 驱动芯片可换（TB6612→L298N），通信信号不同 | 都走 I2C/SPI，通信差异在 IBusTransport |
| **数据处理** | Motor 类自身完成（简单线性换算） | 独立 ImuFilter（姿态角/互补滤波/卡尔曼） |
| **加新型号成本** | 新 Protocol 类 + 新功能类（~200行/芯片） | MPU 系列只填表（~30行/芯片） |

---

## 二、整体架构

```
┌──────────────────────────────────────────────────────────────────┐
│  main.c                                                          │
│  imu_bridge_init() / imu_bridge_read_accel()                     │
└──────────────────────────┬───────────────────────────────────────┘
                           │ extern "C"
┌──────────────────────────▼───────────────────────────────────────┐
│  imu_bridge.h / imu_bridge.cpp                                    │
│  static IIMU* devices[MAX];                                       │
│  static ImuType_t types[MAX];   ← 记录芯片型号，分发特殊功能       │
│  通用功能 → dev->read_accel_raw()                                 │
│  特殊功能 → if(type==MPU9250) static_cast<MPU9250*>(dev)->xxx()   │
└──────┬───────────────────────────────┬────────────────────────────┘
       │                               │
┌──────▼──────────────┐   ┌────────────▼───────────────────────────┐
│  芯片层（driver/）   │   │  通用算法（common/）                    │
│                     │   │                                        │
│  IIMU (虚接口)      │   │  ImuFilter                             │
│  ├─ read_accel_raw  │   │  ├─ raw_to_accel()                     │
│  ├─ read_gyro_raw   │   │  ├─ calc_roll_pitch()                  │
│  └─ read_temp_raw   │   │  ├─ complementary_filter()             │
│       ▲              │   │  └─ calibrate_bias()                  │
│  ┌────┴───────────┐ │   │  零硬件依赖，跨芯片复用                  │
│  │  ImuBase        │ │   └────────────────────────────────────────┘
│  │  持有 desc 表   │ │
│  │  init/read/scale│ │
│  │  默认实现        │ │
│  ├────────────────┤ │
│  │ MPU6050        │ │
│  │ 只填表          │ │
│  ├────────────────┤ │
│  │ MPU9250        │ │
│  │ 填表 + 磁力计   │ │
│  ├────────────────┤ │
│  │ BMI160         │ │
│  │ 全部覆写        │ │
│  └───────┬─────────┘ │
│          │            │
│     IBusTransport    │  (将来：I2C/SPI 统一接口)
│          │            │
│  I2C / SPI 驱动层    │
└──────────────────────┘
```

---

## 三、类继承体系

### 3.1 三层类结构

```
IIMU（纯虚接口，定义所有 IMU 的公共能力）
  ↑ 虚继承
ImuBase（MPU 系列默认实现，参数表驱动）
  ↑ 虚继承
MPU6050 / MPU9250 / BMI160（具体芯片）
```

| 层级 | 职责 | 是否可实例化 |
|------|------|:---:|
| `IIMU` | 定义契约：所有 IMU 必须实现的方法 | ❌ 纯虚 |
| `ImuBase` | 参数表驱动的默认实现，MPU 系列复用 | ✅ 可实例化 |
| `MPU6050` | 填一张 ImuChipDesc 表 | ✅ |
| `MPU9250` | 填表 + 新增磁力计 + 覆写 `init` | ✅ |
| `BMI160` | 不填表，全部覆写 | ✅ |

### 3.2 定制行为：子类如何特化

子类通过 `override` 覆写父类虚函数实现特殊逻辑，有三种策略：

```cpp
class ImuBase : public IIMU {
public:
    virtual int8_t init() override {
        // 默认实现：按 desc.init_seq 循环写寄存器
        for (int i = 0; desc->init_seq[i].reg; i++)
            bus_->write_reg(desc->init_seq[i].reg, desc->init_seq[i].val);
        return 0;
    }
};
```

**策略一：完全继承（MPU6050）**——什么都不覆写

```cpp
class MPU6050 : public ImuBase { /* 所有方法走 ImuBase 默认实现 */ };
```

**策略二：复用 + 追加（MPU9250）**——调父类再加自己的

```cpp
class MPU9250 : public ImuBase {
    int8_t init() override {
        ImuBase::init();           // ← 先做父类的通用初始化
        return init_ak8963();      // ← 再加磁力计初始化
    }
    int8_t read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz); // 纯新增
};
```

**策略三：完全替换（BMI160）**——不调父类，自己实现

```cpp
class BMI160 : public ImuBase {
    int8_t init() override {
        // 完全不调 ImuBase::init()
        bus_->write_reg(0x7E, 0xB6);   // 自己的寄存器序列
        bus_->write_reg(0x40, 0x28);
        ...
    }
    int8_t read_accel_raw(...) override {
        bus_->read_regs(0x12, buf, 6);   // 自己的寄存器地址
        *ax = (int16_t)(buf[0] | (buf[1]<<8)); // 字节序可能不同
        ...
    }
};
```

**覆写决策表**：

| 方法 | MPU6050 | MPU9250 | BMI160 |
|------|:---:|:---:|:---:|
| `init()` | 用父类 | 调父类 + 追加 | 自己写 |
| `read_accel_raw()` | 用父类 | 用父类 | 自己写 |
| `read_gyro_raw()` | 用父类 | 用父类 | 自己写 |
| `read_temp_raw()` | 用父类 | 用父类 | 覆写 |
| `accel_scale()` | 用父类 | 用父类 | 覆写 |

### 3.3 父类没有的方法子类能否定义

**可以**，但通过父类指针/引用调不到，需要通过子类指针访问。

```cpp
class MPU9250 : public ImuBase {
    // 父类没有，IIMU 也没有的方法
    int8_t read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz);
};

// 错误用法：
IIMU *dev = imu_devices[0];
dev->read_mag_raw(...);           // ❌ 编译错误，IIMU 没有此方法

// 正确用法：Bridge 层做类型判断后 static_cast
int8_t imu_bridge_read_mag(uint8_t id, int16_t *mx, ...) {
    if (types[id] == IMU_MPU9250) {
        MPU9250 *mpu = static_cast<MPU9250*>(devices[id]);
        return mpu->read_mag_raw(mx, my, mz);  // ✅
    }
    return -1;  // 不支持磁力计的芯片
}
```

**设计原则**：IIMU 放所有芯片都有的能力，子类特殊能力通过 Bridge 按型号分发。

---

## 四、文件清单

### 当前

| 文件 | 目录 | 职责 |
|------|------|------|
| `imu.h` | `Core/Inc/driver/` | `IIMU` 纯虚接口 + 量程枚举 |
| `imu_base.h` | `Core/Inc/driver/` | `ImuChipDesc` 参数表 + `ImuBase` 类声明 |
| `imu_base.cpp` | `Core/Src/driver/` | `ImuBase` 默认实现 + I2C 同步回调 |
| `mpu6050.h` | `Core/Inc/driver/` | `class MPU6050 : public ImuBase` |
| `mpu6050.cpp` | `Core/Src/driver/` | 填表 + 构造 |
| `imu_bridge.h` | `Core/Inc/driver/` | `extern "C"` C 桥接声明 |
| `imu_bridge.cpp` | `Core/Src/driver/` | C++ 桥实现（对象池 + 克隆芯片兜底） |
| `imu_filter.h` | `Core/Inc/common/` | `ImuFilter` 类声明 |
| `imu_filter.cpp` | `Core/Src/common/` | 互补滤波 / 姿态 / 校准实现 |
| `imu_uart_handler.h` | `Core/Inc/driver/` | C 接口 IMU UART 命令处理器 |
| `imu_uart_handler.cpp` | `Core/Src/driver/` | 响应帧打包 / 滤波更新 / 校准 |
| `oled_driver.h` | `Core/Inc/driver/` | `OledDriver` 类声明 |
| `oled_driver.cpp` | `Core/Src/driver/` | SSD1315 初始化 / 8×16 ASCII / 汉字 |
| `oled_font.h` | `Core/Inc/driver/` | 字库声明（8×16 ASCII + 16×16 汉字） |
| `oled_font_data.c` | `Core/Src/driver/` | 字库数据（95 ASCII 字符 + 流年） |
| `oled_c_wrapper.h` | `Core/Inc/driver/` | `extern "C"` OLED 测试接口 |
| `oled_c_wrapper.cpp` | `Core/Src/driver/` | C++ → C 包装 |

### 将来扩展

| 文件 | 何时加 |
|------|--------|
| `mpu9250.h/.cpp` | 加新 MPU 芯片（填表 ~30 行） |
| `bmi160.h/.cpp` | 加非 MPU 芯片（全覆写） |
| `ibus_transport.h` | SPI 需求出现时 |
| `imu_bus_i2c.cpp` | 同上（把 I2CSync 封装搬到这里） |
| `imu_bus_spi.cpp` | 同上 |

---

## 五、接口设计

### 5.1 IIMU（芯片能力接口）

```cpp
class IIMU {
public:
    virtual int8_t  init() = 0;
    virtual int8_t  get_device_id() = 0;
    virtual int8_t  read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) = 0;
    virtual int8_t  read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) = 0;
    virtual int8_t  read_temp_raw(int16_t *temp) = 0;
    virtual float   accel_scale() const = 0;
    virtual float   gyro_scale() const = 0;
    virtual ~IIMU() = default;
};
```

### 5.2 ImuChipDesc（参数表）

```cpp
struct ImuChipDesc {
    const char *name;                  ///< 芯片名（调试用）

    uint8_t    who_am_i_reg;           ///< WHO_AM_I 寄存器地址
    uint8_t    who_am_i_val;           ///< 期望返回值
    uint8_t    accel_reg;              ///< 加速度起始寄存器
    uint8_t    temp_reg;               ///< 温度起始寄存器
    uint8_t    gyro_reg;               ///< 角速度起始寄存器

    float      accel_scales[4];        ///< 按量程 0/1/2/3 的 LSB/g
    float      gyro_scales[4];         ///< 按量程 0/1/2/3 的 LSB/(°/s)

    struct {
        uint8_t reg;
        uint8_t val;
    } init_seq[16];                    ///< 初始化序列，以 {0,0} 结尾
};
```

### 5.3 ImuBase（MPU 系列默认实现）

```cpp
class ImuBase : public IIMU {
public:
    ImuBase(IBusTransport *bus, const ImuChipDesc *desc,
            ImuAccelRange_t accel_range, ImuGyroRange_t gyro_range);

    virtual int8_t  init() override;
    virtual int8_t  get_device_id() override;
    virtual int8_t  read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) override;
    virtual int8_t  read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) override;
    virtual int8_t  read_temp_raw(int16_t *temp) override;
    virtual float   accel_scale() const override;
    virtual float   gyro_scale() const override;

protected:
    IBusTransport    *bus_;
    const ImuChipDesc *desc_;
    uint8_t            dev_addr_;
    ImuAccelRange_t    accel_range_;
    ImuGyroRange_t     gyro_range_;
};
```

### 5.4 MPU6050（芯片类）

```cpp
static const ImuChipDesc kMpu6050Desc = {
    .name = "MPU6050",
    .who_am_i_reg = 0x75,   .who_am_i_val = 0x68,
    .accel_reg = 0x3B,      .temp_reg = 0x41,  .gyro_reg = 0x43,
    .accel_scales = {16384, 8192, 4096, 2048},
    .gyro_scales  = {131, 65.5, 32.8, 16.4},
    .init_seq = {{0x6B, 0x01}, {0x19, 9}, {0x1A, 0x05},
                 {0x1B, 0x00}, {0x1C, 0x00}, {0,0}},
};

class MPU6050 : public ImuBase {
public:
    MPU6050(IBusTransport *bus, ImuAccelRange_t ar = IMU_ACCEL_RANGE_2G,
            ImuGyroRange_t gr = IMU_GYRO_RANGE_250)
        : ImuBase(bus, &kMpu6050Desc, ar, gr) {}
    // 所有方法走 ImuBase 默认实现，不需覆写
};
```

### 5.5 ImuFilter（通用数据处理）

放在 `Core/Inc/common/imu_filter.h`，跨芯片复用：

```cpp
class ImuFilter {
    float accel_scale_factor, gyro_scale_factor;
    int16_t accel_bias[3], gyro_bias[3];
    float roll, pitch, yaw;

public:
    void set_accel_scale(float lsb_per_g);
    void set_gyro_scale(float lsb_per_dps);
    void raw_to_accel(int16_t rx, int16_t ry, int16_t rz,
                      float *ax, float *ay, float *az);
    void raw_to_gyro(int16_t rx, int16_t ry, int16_t rz,
                     float *gx, float *gy, float *gz);
    void calc_roll_pitch(float ax, float ay, float az,
                         float *roll, float *pitch);
    void complementary_filter(float gx, float gy, float gz, float dt);
    void calibrate_accel_bias(const int16_t *sx, const int16_t *sy,
                              const int16_t *sz, uint16_t count);
    void calibrate_gyro_bias(const int16_t *sx, const int16_t *sy,
                             const int16_t *sz, uint16_t count);
    float get_roll()  const;
    float get_pitch() const;
};
```

### 5.6 Bridge（C 接口）

```cpp
// imu_bridge.h
#define MAX_IMUS 4

typedef enum { IMU_MPU6050 = 0, /* IMU_MPU9250, IMU_BMI160 */ } ImuType_t;

#ifdef __cplusplus
extern "C" {
#endif

void   imu_bridge_init(uint8_t id, ImuType_t type, I2C_Handle *hi2c);
int8_t imu_bridge_get_device_id(uint8_t id);

// 原始值
int8_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, int16_t *ay, int16_t *az);
int8_t imu_bridge_read_gyro_raw(uint8_t id, int16_t *gx, int16_t *gy, int16_t *gz);

// 物理量
int8_t imu_bridge_read_accel(uint8_t id, float *ax, float *ay, float *az);
int8_t imu_bridge_read_gyro(uint8_t id, float *gx, float *gy, float *gz);
int8_t imu_bridge_get_roll_pitch(uint8_t id, float *roll, float *pitch);

// Filter 控制
void   imu_bridge_calibrate(uint8_t id, uint16_t sample_count);
void   imu_bridge_filter_update(uint8_t id, float dt_sec);

#ifdef __cplusplus
}
#endif
```

**Bridge 内部结构**：

```cpp
static IIMU      *imu_devices[MAX_IMUS];
static ImuType_t  imu_types[MAX_IMUS];    // 记录型号，分发特殊功能
static ImuFilter *imu_filters[MAX_IMUS];

void imu_bridge_init(uint8_t id, ImuType_t type, I2C_Handle *hi2c) {
    switch (type) {
    case IMU_MPU6050:
        imu_devices[id]  = new MPU6050(new I2CBusTransport(hi2c));
        imu_types[id]    = IMU_MPU6050;
        break;
    // case IMU_MPU9250:
    //     imu_devices[id]  = new MPU9250(new I2CBusTransport(hi2c));
    //     imu_types[id]    = IMU_MPU9250;
    //     break;
    }
    imu_filters[id] = new ImuFilter();
    imu_filters[id]->set_accel_scale(imu_devices[id]->accel_scale());
    imu_filters[id]->set_gyro_scale(imu_devices[id]->gyro_scale());
    imu_devices[id]->init();
}

// 通用功能：直接走 IIMU*
int8_t imu_bridge_read_accel_raw(uint8_t id, int16_t *ax, ...) {
    return imu_devices[id]->read_accel_raw(ax, ay, az);
}

// 特殊功能：按类型分发
int8_t imu_bridge_read_mag(uint8_t id, int16_t *mx, ...) {
    if (imu_types[id] == IMU_MPU9250) {
        return static_cast<MPU9250*>(imu_devices[id])->read_mag_raw(mx, my, mz);
    }
    return -1;
}
```

---

## 六、数据流

```
              ┌─ PC 串口助手 ────────────────────┐
              │  发送 AA 30 id FF FF               │
              │  ← 接收 AA 30 id roll pitch yaw   │
              │  (4B float × 3 = 12B)              │
              └──────────┬─────────────────────────┘
                         │ UART ISR
                         ▼
              uart_cmd_parser_tick()
              → uart_cmd_dispatch()
              → imu_uart_handler_dispatch()
                         │
                         ▼
              imu_send_data(id)
              → imu_bridge_read_accel/gyro_raw
              → ImuFilter::raw_to_accel/gyro
              → ImuFilter::complementary_filter
              → 打包 17B 帧发送
                         │
  ┌──────────────────────┼─────────────────────────┐
  │                      │                         │
  ▼                      ▼                         ▼
main.c 主循环       OLED 显示                  UART 响应帧
每 500ms             show_string(row,col,str)   AA cmd id ...
读原始值             show_chinese(row,col,idx)  roll(4B) pitch(4B) yaw(4B)
uart_send("A:...")                        0xFF 0xFF
                         │
                         ▼
              OledDriver::write(0x40, font_data, len)
              → i2c_write_reg_async()
              → TIM6 ISR FSM
                         │
          ┌──────────────┼────────────────┐
          ▼              ▼                ▼
    I2C SEND        I2C RECV          I2C RESTART
    (OLED/配置)      (MPU6050读)       (读寄存器)
          │              │                │
          └──────────────┴────────────────┘
                         │
                         ▼
              i2c_sw_fsm_edge/fsm_bit/fsm_byte
              → 物理 GPIO 操作 (PF0=SCL, PF1=SDA)
```

### 6.1 主循环流程

```c
while (1) {
    uart_cmd_parser_tick();       // 处理串口命令 → IMU 响应
    
    if (HAL_GetTick() - imu_tick >= 500) {
        // 调试输出：原始值
        imu_bridge_read_accel_raw(0, ...);
        imu_bridge_read_gyro_raw(0, ...);
        uart_send("A: xxx, G: xxx");
    }
}
```

### 6.2 UART 命令响应流程

```
PC → AA 30 00 FF FF
  → uart_cmd_parser_tick() 检测帧尾 FF FF
  → uart_cmd_dispatch()
  → case CMD_IMU_GET_DATA:
    imu_uart_handler_dispatch(cmd, frame, length)
    → imu_send_data(0)
      → 读 accel/gyro raw
      → ImuFilter::raw_to_accel/gyro (去零偏)
      → ImuFilter::complementary_filter (融合)
      → 发送 17B 帧 (float roll/pitch/yaw)
```
  │  i2c_read_reg_async(hi2c, dev, 0x3B, buf, 6, cb)
  │  ISR 后台翻 GPIO → cb 设 done=1 → 主循环解阻塞
  ▼
物理总线 → MPU6050 → 寄存器 0x3B~0x40
```

---

## 七、扩展方式

### 7.1 加新芯片（MPU 系列）

MPU9250 为例，只需 **~30 行**：

```cpp
// mpu9250.cpp
static const ImuChipDesc kMpu9250Desc = {
    .name = "MPU9250",
    .who_am_i_val = 0x71,          // ← 只改 ID
    .accel_reg = 0x3B,             // ← 跟 MPU6050 一样
    .gyro_reg  = 0x43,
    .init_seq  = {{0x6B, 0x01}, {0x19, 9}, ..., {0,0}},
};

class MPU9250 : public ImuBase {
    MPU9250(IBusTransport *bus) : ImuBase(bus, &kMpu9250Desc) {}
    int8_t init() override { ImuBase::init(); return init_ak8963(); }
    int8_t read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz);
    int8_t init_ak8963();
};
```

Bridge 加一个 case：

```cpp
case IMU_MPU9250:
    devices[id] = new MPU9250(new I2CBusTransport(hi2c));
    types[id]   = IMU_MPU9250;
    break;
```

以下文件零修改：`imu.h`、`imu_base.h/.cpp`、`imu_filter.h/.cpp`、`main.c`。

### 7.2 加新芯片（非 MPU 系列）

BMI160 为例，全部覆写（~200 行）：

```cpp
class BMI160 : public ImuBase {
    BMI160(IBusTransport *bus) : ImuBase(bus, nullptr) {} // desc 传空
    int8_t init() override { /* 自己的初始化序列 */ }
    int8_t read_accel_raw(...) override { /* 0x12 + 小端字节序 */ }
    // read_gyro_raw 等全部覆写
};
```

`ImuChipDesc` 表不可靠时，直接覆写比强行填表更清晰。

### 7.3 加 SPI 传输

```
1. 新增 ibus_transport.h    → IBusTransport 虚接口
2. 新增 imu_bus_i2c.cpp      → class I2CBusTransport（搬当前同步封装）
3. 新增 imu_bus_spi.cpp      → class SpiBusTransport
4. Bridge 构造时注入对应 Transport
5. IIMU / ImuBase / 芯片类 / ImuFilter / main.c → 零修改
```

IBusTransport 接口：

```cpp
class IBusTransport {
public:
    virtual int8_t write_reg(uint8_t dev, uint8_t reg, uint8_t data) = 0;
    virtual int8_t read_regs(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len) = 0;
    virtual ~IBusTransport() = default;
};
```

### 7.4 加 IBusTransport 后的改动范围

| 文件 | 改动 |
|------|------|
| `mpu6050.h` | `I2C_Handle*` → `IBusTransport*` |
| `mpu6050.cpp` | `write_reg`/`read_regs` 改为 `bus_->write_reg()/read_regs()` |
| `imus_base.h/.cpp` | 同理，`I2C_Handle*` → `IBusTransport*` |
| **新增** `ibus_transport.h` | 接口声明 |
| **新增** `imu_bus_i2c.cpp` | 把 `I2CSync` + `i2c_sync_cb` + `i2c_wait_done` 搬来 |
| 不变 | `imu.h`、`imu_filter`、`imu_bridge`、`main.c` |

---

## 八、与项目现有模块对照

| 模块 | 分层数 | 中层拆几块 | 通用模块 | 抽象维度 |
|------|:---:|------|------|------|
| **PWM** | 2 | — | — | 芯片平台 |
| **UART** | 3 | — | RingBuffer（common/） | 平台 + 帧协议 |
| **Servo** | 4 | Servo + PWMServoProtocol | — | 型号 + 信号 |
| **Motor** | 4 | Motor + TB6612Protocol | — | 控制 + 芯片 |
| **I2C** | 2 | — | — | 软件/硬件 |
| **IMU** | 3 + 通用 | ImuBase + 芯片类 | ImuFilter（common/） | 型号 + 总线 |

---

## 九、设计决策汇总

| 决策 | 选择 | 原因 |
|------|------|------|
| 类抽象 vs 参数表抽象 | 参数表驱动（ImuBase） | MPU 系列寄存器高度重合，填表比覆写省 80% 代码 |
| ImuBase 内方法是否 virtual | 是 | 子类按需覆写，MPU6050 全继承，BMI160 全覆写 |
| ImuFilter 放哪 | `common/` | 纯数学、零硬件依赖，跨芯片复用 |
| 特殊功能（磁力计） | 子类新增 + Bridge static_cast | 遵循项目 Motor 模式（Bridge 已知型号） |
| SPI 要不要现在就设计 | 预留，不实现 | MPU6050 没有 SPI，没有第二个实现来验证接口 |
| I2CSync 放哪 | 当前在 `mpu6050.cpp`，加 SPI 时搬到 `imu_bus_i2c.cpp` | 避免过度抽象 |
| IBusTransport 放哪 | `driver/` | 操作硬件句柄，属于驱动层 |

---

## 十、调试记录与已知问题

### 10.1 I2C 同步回调 Bug（已修复）

**问题**：`i2c_sync_cb` 回调被调用时，第一个参数 `ctx` 是 I2C FSM 传入的 `I2C_SoftwareContext*`，而非用户自定义的上下文指针。原始代码将其强转为栈上的 `I2CSync*`，导致 `done` 标志写入错误内存地址。

```cpp
// ❌ 错误：ctx 实际是 I2C_SoftwareContext*
static void i2c_sync_cb(void *ctx, int8_t result) {
    I2CSync *sync = (I2CSync *)ctx;  // 类型不匹配！
    sync->done = 1;                   // 写入错误地址
}

// ✅ 修复：用 static volatile 全局变量
static volatile uint8_t g_i2c_sync_done;
static void i2c_sync_cb(void *ctx, int8_t result) {
    (void)ctx;                        // ctx 是 I2C_SoftwareContext*，忽略
    g_i2c_sync_done = 1;              // 写入正确地址
}
```

**影响**：第一次 I2C 传输靠超时（~10ms）勉强通过，后续传输不稳定。

### 10.2 MPU6050 克隆芯片兼容性

部分 MPU6050 克隆/山寨芯片存在以下行为差异：

| 问题 | 现象 | 解决 |
|------|------|------|
| WHO_AM_I 返回 0xFF | `init()` 校验失败 | 跳过 WHO_AM_I 校验，直接配置寄存器 |
| 数据读回全 0xFF | ACK 正常但数据位不驱动 | 降速 I2C 或改用单字节读写 |
| RESTART 不可靠 | 读多字节失败 | 改为单字节分别读 |

**建议**：`ImuChipDesc` 增加 `skip_id_check` 标志位，允许关闭 WHO_AM_I 校验。

```cpp
typedef struct ImuChipDesc {
    ...
    uint8_t skip_id_check;  ///< 跳过 WHO_AM_I 校验（兼容克隆芯片）
};
```

### 10.3 I2C RESTART 时序 Bug（已修复）

**问题**：RESTART 时上一字节 ACK 结束后 SCL = 1，FSM 进入 MACRO_START step0 直接设 SDA = 1，此时 SCL 仍为高 → **SDA 在 SCL 高期间跳变，从机判为 STOP 条件**。读数据阶段全部返回 0xFF。

**修复**（`useri2c_ops.c` `i2c_sw_fsm_edge`）：
```c
// 首次 START (byte_index=0): 3 步
//   step 0: SCL=1,SDA=1 → step 1: SDA=0 → step 2: SCL=0
//
// RESTART (byte_index>0): 4 步（多一步拉低 SCL）
//   step 0: SCL=0 ← ★ 先确保 SCL 低
//   step 1: SCL=1,SDA=1
//   step 2: SDA=0 (START)
//   step 3: SCL=0 (切 SEND)
int step_offset = (ctx->byte_index > 0) ? 0 : 1;
```

**影响**：所有 I2C 读操作（MPU6050 WHO_AM_I、加速度、陀螺仪）返回全 0xFF。写操作（OLED）不受影响。

### 10.4 OLED 字库选择

**背景**：先后尝试了 16×16 和 8×16 两种 ASCII 字库。

| 对比 | 16×16 | 8×16（现用） |
|------|:---:|:---:|
| 每字字节 | 32 | 16 |
| 每行字符 | 8 | 16 |
| Flash 占用 | ~3KB | ~1.5KB |
| 来源 | gen_font.py (Pillow) | 参考工程 1-4 OLED 驱动 |
| 汉字支持 | 16×16 | 仍用 16×16（流年） |

**结论**：8×16 更实用、更省空间，与行业标准字库兼容。汉字保持 16×16 以保障可读性。

### 10.5 I2C 总线共享问题（已解决）

**现象**：OLED 和 MPU6050 接在同一组 I2C 引脚（PF0/PF1）后，MPU6050 持续 NAK（不响应），OLED 读写正常。

**根因**：内部 ~40kΩ 上拉电阻（`GPIO_PULLUP`）太弱，两个设备并联后总线电容增大，SDA/SCL 上升沿变缓，MPU6050 无法识别 I2C 时序。

**解决**：外部加 4.7kΩ 上拉电阻到 3.3V（SCL 和 SDA 各一个）。之后 MPU6050 和 OLED 和谐共存。

```
3.3V ─┬─[4.7kΩ]─ SCL (PF0)
      └─[4.7kΩ]─ SDA (PF1)
```

### 10.6 互补滤波参数调优

| 参数 | 初始值 | 优化值 | 效果 |
|------|:---:|:---:|------|
| alpha（加速度权重） | 0.02 | **0.10** | 收敛快 5 倍 |
| 陀螺死区 | 无 | **<1.4°/s 置零** | 静止时不漂 |
| 陀螺零偏 | 无校准 | **开机 30 次采样取均值** | yaw 漂移减小 |
| yaw 处理 | 纯积分 | **零偏补偿 + 死区** | yaw 不再无限增长 |

### 10.7 STM32CubeMX 重生成注意

CubeMX 只保留 `USER CODE` 标记内的代码。以下代码需写在 `USER CODE` 内：

```c
/* USER CODE BEGIN 2 */
htim6.Init.Period = 899;          // I2C 降速 10μs/tick
HAL_TIM_Base_Init(&htim6);

imu_bridge_init(0, IMU_MPU6050, &g_i2c_dev);  // IMU 初始化
oled_bridge_init();                              // OLED 初始化
oled_bridge_show_string(0, 0, "MPU6050 OK");    // 状态显示
/* USER CODE END 2 */
```

### 10.3 I2C 上拉电阻

STM32 内部上拉 ~40kΩ 对 I2C 标准要求偏弱。**强烈建议外接 4.7kΩ 上拉电阻**到 SCL 和 SDA：

```
3.3V ──[4.7kΩ]── SCL (PF0)
3.3V ──[4.7kΩ]── SDA (PF1)
```

### 10.4 I2C 速度调整

克隆芯片可能需要更慢的 I2C 速度。TIM6 周期可在运行时覆盖：

```c
// 初始化后修改（CubeMX 生成 ARR=359 → 4μs/tick）
MX_TIM6_Init();
htim6.Init.Period = 899;          // → 10μs/tick (~50kHz I2C)
HAL_TIM_Base_Init(&htim6);
```

### 10.6 I2C RESTART 时序 bug（已修复）

**问题**：播放 RESTART 时，从机的上一条 ACK 结束后 SCL 仍为高。FSM 进入 MACRO_START step0 直接设 SDA=1，此时 SCL=1 → **SDA 在 SCL 高期间跳变，从机判为 STOP**。读数据阶段全部返回 0xFF。

**修复**（`useri2c_ops.c` `fsm_edge` START 序列）：
```c
// step0: 先拉低 SCL（防止 SDA 跳变被误判 STOP，仅 RESTART 时触发）
if (byte_index > 0) { SCL=0; }   // byte_index>0  = RESTART
// step1: SCL=1, SDA=1 (总线空闲)
// step2: SDA=0 (START)
// step3: SCL=0 (切 SEND)
```

### 10.7 HAL_Delay 不能用

`HAL_Delay` 依赖 SysTick 中断，在初始化阶段可能未就绪。使用忙等循环替代：

```cpp
for (volatile int i = 0; i < 500000; i++) { }  // ~50ms @180MHz
```

---

## 十一、MPU6050 寄存器速查

| 寄存器 | 地址 | 说明 |
|--------|------|------|
| WHO_AM_I | 0x75 | 返回 0x68 |
| PWR_MGMT_1 | 0x6B | bit6=复位, bit5=唤醒, bit3=温度使能 |
| SMPLRT_DIV | 0x19 | 采样率 = 1kHz / (1 + div) |
| CONFIG | 0x1A | DLPF 配置 |
| GYRO_CONFIG | 0x1B | bit4-3: 量程（00=250, 01=500, 10=1000, 11=2000 °/s） |
| ACCEL_CONFIG | 0x1C | bit4-3: 量程（00=2, 01=4, 10=8, 11=16 g） |
| ACCEL_XOUT_H | 0x3B | 加速度 X 高字节（大端，共 6 字节） |
| TEMP_OUT_H | 0x41 | 温度高字节（共 2 字节） |
| GYRO_XOUT_H | 0x43 | 角速度 X 高字节（大端，共 6 字节） |

I2C 7 位地址：`0x68`（AD0 接地）。
