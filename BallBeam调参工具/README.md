# BallBeam_UNO

本工程当前目标：K230D持续测量钢珠相对中心的位置，电脑桥接将位置帧转发给Arduino UNO R3，UNO运行位置PID并以`PUL/STEP`脉冲列和独立`DIR`电平控制张大头X42S，使钢珠回到并保持在`0 mm`中心。X42S的Emm固件和内部编码器完成电机轴闭环；是否增加UART/RS485只读遥测另行确认。

## 当前边界

- 旧版 STEP/DIR 实现、`PUL_WID`、`JOG`、旧 PID 与旧视觉协议代码均已停止使用；当前选择脉冲方向接口不代表恢复旧代码。
- 用户在 2026-08-01 最新确认当前测试接线为 D2=`DIR`、D3=`PWM/STEP`；先前 D5/D6/D7 诊断映射已废止。
- 用户实机确认X42S菜单为`P_Pul=PUL_ENA`、`CtrlMode=CR_VFOC`、`En=HOLD`、`MStep=16`；厂商资料定义`HOLD`为一直使能，因此当前不接UNO的EN输出不会阻止脉冲运动。
- R10连续闭环设计把`SIGN=+1`、`RATE=200 pulse/s`、`ACCEL=400 pulse/s²`、`RETURNRATE=200 pulse/s`和`DEADBAND=5 mm`固化为已使用过的执行参数；用户只输入`KP/KI/KD`。收到三个增益且TRACK新鲜后自动进入闭环，丢失TRACK时回命令零位，TRACK恢复后自动继续。
- 实物`MStep=16`已由用户读取确认，因此当前3200脉冲/圈、0.1125°/脉冲的换算成立。程序允许的最高1000 pulse/s只是软件保护上限，不是X42S规格，也不是首次测试参数。
- 当前源码已通过UNO编译；用户明确由其自行编译、烧录和实机观察，本工程不代为烧录或占用COM9。
- 用户实机确认方向编号：`DIR=1`为顺时针，`DIR=0`为逆时针；观察视角尚未另行定义。
- 当前`C:\Users\Y_cheng\Desktop\DS_26\k230_try.py`使用UART2、38400、8N1并将同一严格`B,<frame_id>,<position_mm>,<state>\n`帧镜像到COM7；电脑桥接已实测把COM7帧转发到UNO COM9，`position_mm=0`为中心且控制只采用`state=2`（TRACK）。
- 用户给出的机械安全范围按手动调平点计数：顺时针最多 `+2/3` 圈、逆时针最多 `-1/3` 圈。按已确认的 3200 脉冲/圈，软件硬限位保守取顺时针 `+2133` 脉冲、逆时针 `-1066` 脉冲。
- 用户确认不使用物理零位按键。R10流程要求双击BAT前手动调平，BAT打开COM9使UNO复位后自动把启动姿态定义为命令零位；这仍不是X42S实际编码器或梁角测量。`DISARM`立即锁定输出；非TRACK或200 ms超时进入累计命令零位返回。
- Emm 自由协议和 MODBUS 资料只用于可选遥测及后续诊断，不能与脉冲方向运动接口混写，也不能相互混用帧格式。
- 每次可验收的工程操作都必须同步更新 `doc/操作日志.md`，验证后单独提交。

## 资料基线

- `C:\Users\Y_cheng\Desktop\DS2026\doc\H题设计\ZDT闭环步进电机MODBUS协议使用说明V1.0.1_260401_提取.md`
- `C:\Users\Y_cheng\Desktop\DS2026\doc\H题设计\ZDT_X42S&Y42固件更新使用说明_提取.md`
- `C:\Users\Y_cheng\Desktop\资料\STM32F407_串口通讯__切换开环和闭环模式\Src\Emm_V5.c`
- `C:\Users\Y_cheng\Desktop\资料\STM32F407_串口通讯__切换开环和闭环模式\Src\Emm_V5.h`
- DS2026 中由用户确认的项目事实。

## 文档入口

- `doc/当前联调总结.md`：当前能跑通、未跑通、故障范围、下一步和全部程序路径
- `doc/工作流.md`：开发、确认、验证和提交规则
- `doc/工作条目.md`：重新开发的工作分解与状态
- `doc/操作日志.md`：每次工程操作的追踪记录
- `doc/ZDT_Emm资料映射.md`：资料事实、协议边界和待实测项
- `doc/开发跟踪.md`：当前阶段与下一步
- `doc/调试记录.md`：问题、根因和处理结果
- `doc/引脚映射表.md`：仅记录已确认的物理连接
- `doc/平台映射表.md`：系统职责和实现映射

## 当前进度

R07/R08视觉桥接已跑通，UNO实测收到连续TRACK帧。R10三参数自动固件已经烧录，能够自动ARM并产生200 pulse/s命令计数；但在`MStep=16`下UNO累计1197个命令脉冲时，用户确认X42S电机轴本身几乎没有转动。当前阻塞点是区分X42S未收到脉冲，还是收到后因供电、校准、堵转或保护而未跟随。钢球中心稳定和“每次完成后角度归零、计数清零”均尚未实现。

## 电脑桥接启动

首次安装项目内隔离环境：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

K230D运行已加入USB镜像的`C:\Users\Y_cheng\Desktop\DS_26\k230_try.py`后，先关闭占用COM7/COM9的IDE串口窗口，再双击：

```text
start_ballbeam_bridge.bat
```

BAT依次要求输入`KP`、`KI`和`KD`，桥接启动后自动向UNO发送一条`PID <kp> <ki> <kd>`命令。运行中重新整定时也只需输入同一条PID命令；`STATUS`只读，`RETURN`停止自动闭环并回命令零位，`DISARM`用于紧急锁定。
