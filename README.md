# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 2026 赛季 RoboMaster 全向轮步兵电控工程：`Chassis/` 底盘板与 `Gimbal/` 云台板各一片 **STM32F407IGH6**（RoboMaster C 型板），`RobotComponents/` 为两板共享代码（接口契约、状态枚举、双板通信帧）。
> 
> 工程目标：用两板完成「DR16 遥控 → 底盘全向移动（O 型全向轮）/ 云台两轴增稳与瞄准」的完整闭环，具备三态安全状态机、IWDG、总线错误恢复与实车可调参/可观测能力。
> 
> **当前状态（2026-11-17）**：软件控制链与 L2 硬件链路已全部打通并通过实车联调（DBUS、双板 IMU、4×M3508、云台 DM 双轴、双板帧、三态、IWDG 复位验证），完成多轮实车参数整定；剩余工作为 T2/T4–T7 台架验收项、底盘整车运动测试与 L3/L4 整机联调。发射机构、裁判系统、视觉、超级电容不在本工程范围内。

---

## 1. 项目目标与范围

### 1.1 目标

1. 两板固件可独立构建、独立烧录，通过 **CAN1 双板帧（1 ms）** 协同工作：底盘板负责遥控解析与整车决策，云台板负责云台两轴世界系增稳。
2. 底盘：DR16 → 三态状态机 → 三模式（分离/跟随/小陀螺）→ δ 旋转变换 → O 型全向轮逆解 → 4×M3508 轮速环 → CAN 电流帧。
3. 云台：C2G 目标角 → 三态状态机 → yaw 世界坐标换算 / pitch 软限位 → 位置环 → 速度环 → DM-J4310 MIT 力矩。
4. 安全：急停、对板离线、遥控离线、IMU 未标定、电机反馈超时、总线 bus-off 自动恢复、IWDG 兜底。

### 1.2 范围

| 范围        | 内容                                                                                                                   |
| --------- | -------------------------------------------------------------------------------------------------------------------- |
| 已实现       | 1 ms 调度、PID 库、BMI088+Mahony IMU、M3508/DM-J4310 驱动、DBUS 解码、三态状态机、O 型全向轮逆解、底盘三模式、云台两轴控制、双板通信帧、IWDG、台架单轮调试、Ozone 调试镜像体系 |
| 未实现（本工程外） | 发射机构（拨弹/摩擦轮）、裁判系统、视觉/自瞄、超级电容管理、图传/UWB                                                                                |

### 1.3 里程碑

| 版本   | 内容                                            |
| ---- | --------------------------------------------- |
| P0   | 双板构建工程化、接口契约 v1.0、config 集中管理、IMU/DBUS 硬件链路   |
| P1   | PID/IMU 公共组件落地、CAN 电机收发、单轮 ControlTask 台架验证   |
| P2   | 两板完整整车控制链（三态 + 三模式 + 逆解 + 云台两轴 + 双板帧）         |
| v1.2 | 实车联调：CAN 拓扑纠正、云台零位/上电行为、前馈与重力补偿、拨轮 4 版解码、多轮调参 |

---

## 2. 主要工作

### 2.1 工程化阶段（2026-10）

- 建立两板 CMake + Ninja + `arm-none-eabi-gcc 14.3.1`（C++20）构建体系，统一 `Debug` 预设与 `build/omni_<board>.elf` 产物路径。
- 输出并迭代 **接口契约**（`RobotComponents` 共享层语义、CAN 帧格式、状态枚举、控制源、决策记录 D-1~D-12），所有跨板语义先改契约再改代码。
- 修复骨架缺陷：`tick` 宏未定义、DaMiao 解包错位、IWDG 重复初始化、RTT 未链接、CAN2 滤波组 `CAN2SB` 未配置、调试符号被 `--gc-sections` 删除等。
- 建立验收口径：L0 构建（0 error / 0 warning + size 基线）、L1 主机单测、L2 单板硬件、L3 台架、L4 整车。

### 2.2 公共组件阶段（2026-10 ~ 11）

- **PID**：两板同源位置式 PID（输出/积分限幅、条件积分抗饱和、微分对测量值 + 一阶低通、死区、NaN 防御），既有固定 `Config` 结构又支持运行期镜像；主机单测覆盖 11 类工况 / 87 项断言。
- **IMU**：BMI088（SPI1，软件片选 PA4/PB0）+ Mahony 姿态解算；上电静置 750 帧零漂标定（静止判定：均值 + 标准差阈值），标定后 100 帧置就绪；两板同源。
- **DBUS/DR16**：USART3 100 kbit/s、9 bit 字长 + 偶校验、DMA + IDLE 收 18 字节；解决帧起点对齐（见 §9.2）。
- **CAN/UART 硬件层**：M3508 电流帧与反馈解码、DM-J4310 使能/MIT 帧、CAN 错误快照与 bus-off 自动恢复、发送邮箱满时清理。

### 2.3 双板完整控制链（2026-11-07 ~ 11-08）

- 底盘：`Robot`（状态机 + 模式选择 + 遥测）+ `Chassis`（逆解 + 四轮独立速度环）+ `ControlTask`（台架单轮）+ `CommTask`（DBUS/CAN/双板帧/IWDG）。
- 云台：`Robot`（状态机 + 目标消费）+ `Gimbal`（yaw 世界坐标、pitch 限位、位置环/速度环/MIT 力矩）+ `CommTask`（DM 收发 + G2C 帧 + IWDG）。
- 主机集成自测：底盘 76/76 PASS（DT7 解码、三态机、三模式、δ 变换、逆解、帧编解码、集成序列）；云台 15 条 `static_assert` 编译期校验。
- 两板 L0：0 error / 0 warning，建立 size 基线。

### 2.4 实车联调与参数整定（2026-11-08 ~ 11-17）

- **CAN 拓扑纠正**：4×M3508 从 CAN1 移到 **底盘板 CAN2**；双板帧改走 **两板物理相连的 CAN1**（1 ms）；云台 DM：yaw 在 CAN1（id=2）、pitch 在 CAN2（id=1）。
- **云台零位**：以实车校准点确定（正对车头时 `g_dbg_dm_angle[0]≈0`，`kYawAngleOffset=-1.3340`），消除跟随模式 7° 残差。
- **上电行为**：对板未 Working 时冻结云台目标 + 进入 Working 时锁定当前姿态 → 上电**不自动回正**，并修复上电 pitch 低头/抖动。
- **控制增强**：yaw 底盘速率前馈、pitch 目标速率前馈、pitch 重力前馈（幅度 + 符号实车标定）、小陀螺专用 yaw 关节限速。
- **DBUS 专项**：拨杆位域、拨轮两段编码 4 版解码演进（详见 §9.3）、拨轮死区与方向。
- **参数整定**：轮速环/跟随环、云台位置环/速度环、力矩与速率限幅多轮实车调参。
- **IWDG 验证**：临时注入 10 s 卡死测试代码，验证两板均能按超时复位，随后回退。
- **可观测性**：建立 `g_dbg_*` / `robot_debug` / `gimbal_debug` 镜像体系与符号保留策略（`used, retain`、C 链接）。

---

## 3. 主要功能（现状总表）

| 功能        | 状态  | 实现位置                                                       | 要点                                                 |
| --------- | --- | ---------------------------------------------------------- | -------------------------------------------------- |
| 参数集中管理    | ✅   | 两板 `inc/config.hpp`                                        | 周期、CAN ID/总线、限幅、`TEST_MODE`/`DEBUG_NO_IWDG`、PID 初值 |
| 1 ms 调度   | ✅   | 两板 `Task/component/main_task.cpp`                          | TIM6 中断→`MainTask→ControlTask(底盘)→CommTask`        |
| PID 库     | ✅   | 两板 `inc/pid.hpp`、`src/pid.cpp`                             | 位置式、抗饱和、D 低通、死区、NaN 防御、两板同源                        |
| IMU 采集与标定 | ✅   | `inc/BMI088.hpp`、`inc/imu.hpp`、`src/imu.cpp`               | SPI1、750 帧零漂、静止判定、就绪判定、两板同源                        |
| 姿态解算      | ✅   | `inc/mahony.hpp`、`src/mahony.cpp`                          | Mahony（acc 修正 roll/pitch，gyro 积分 yaw）              |
| DBUS 遥控   | ✅   | `inc/DT7.hpp`、`src/DT7.cpp`、`Task/component/comm_task.cpp` | USART3+DMA、帧边界重启、拨杆/摇杆/拨轮解码                        |
| 4×M3508   | ✅   | `src/motor_M3508.cpp`、`src/HW_can.cpp`                     | 底盘板 CAN2：0x200 电流帧、0x201~0x204 反馈、bus-off 恢复       |
| DM-J4310  | ✅   | `Gimbal/src/motor_DaMiao.cpp`、`src/HW_can.cpp`             | yaw=CAN1(id2)、pitch=CAN2(id1)，MIT 纯力矩，自动使能         |
| 三态状态机     | ✅   | 两板 `RobotModules/component/robot.cpp`                      | Dead/Resurrection/Working，急停最外层，非 Working 零输出      |
| O 型全向轮逆解  | ✅   | `Chassis/inc/chassis_ik_solver.hpp`                        | δ 旋转 + 轮安装角/位置/符号可配，轮速限幅后进 PID                     |
| 底盘三模式     | ✅   | `Chassis/RobotModules/component/robot.hpp`                 | 分离（只平移）/跟随（δ 跟随 PD）/小陀螺（平移+自转）                     |
| 云台两轴控制    | ✅   | `Gimbal/RobotModules/component/gimbal.cpp`                 | 世界系 yaw、pitch 软限位、位置环→速度环→MIT                      |
| 双板通信      | ✅   | `RobotComponents/component/gimbal_chassis_comm.*`          | CAN1，0x210/0x211，8 字节，1 ms，离线 100 ms               |
| IWDG      | ✅   | 两板 `Core/Src/iwdg.c`、`CommTask`                            | 300 ms（底盘）/600 ms（云台），1 ms 链末喂狗                    |
| 台架单轮调试    | ✅   | `Chassis/Task/component/control_task.cpp`                  | `TEST_MODE=1` 单轮三角波扫掠（±40 rad/s、2 A、15 s）          |
| 调试镜像      | ✅   | 两板 `robot.*`/`gimbal.*`/`comm_task.cpp`                    | Ozone 可直接观测，C 链接符号 + `used/retain` 防裁剪             |

---

## 4. 硬件与通信

### 4.1 板卡与外设

| 项       | 底盘板                                                        | 云台板                              |
| ------- | ---------------------------------------------------------- | -------------------------------- |
| MCU     | STM32F407IGH6                                              | STM32F407IGH6                    |
| 板型      | RoboMaster C 型开发板                                          | RoboMaster C 型开发板                |
| 1 ms 时基 | TIM6                                                       | TIM6                             |
| DBUS    | USART3：100000 baud、9 bit、偶校验、1 停止位，DMA1_Stream1 CH4，18 B/帧 | 不直接接接收机（遥控数据经 C2G 转发）            |
| IMU     | BMI088（SPI1：PB3/PB4/PA7，CS PA4（acc）/PB0（gyro））             | 同左                               |
| 测速/调试串口 | USART1/USART6：115200 8N1（预留）                               | 同左                               |
| CAN1    | 1 Mbit：双板帧                                                 | 1 Mbit：双板帧 + DM yaw（id=2）        |
| CAN2    | 1 Mbit：4×M3508                                             | 1 Mbit：DM pitch（id=1）+ 发射机构（未实现） |
| 看门狗     | IWDG，PSC=32、Reload=300（≈300 ms）                            | IWDG，PSC=32、Reload=600（≈600 ms）  |
| 调试      | SWD + SEGGER RTT                                           | SWD + SEGGER RTT                 |

### 4.2 CAN 总线分配

| 总线                       | 挂载设备                              | 帧                                               |
| ------------------------ | --------------------------------- | ----------------------------------------------- |
| 底盘板 CAN1（与云台板 CAN1 物理相连） | 无电机                               | C2G `0x210`（底盘→云台，1 ms）、G2C `0x211`（云台→底盘，1 ms） |
| 底盘板 CAN2                 | 4×M3508（id 1~4）                   | `0x200` 电流下发；`0x201`~`0x204` 反馈                 |
| 云台板 CAN1                 | DM-J4310 yaw（id=2）                | MIT 命令 `0x02`；反馈 `0x12`                         |
| 云台板 CAN2                 | DM-J4310 pitch（id=1）+ 拨弹/摩擦轮（未实现） | MIT 命令 `0x01`；反馈 `0x11`                         |

### 4.3 双板帧（`RobotComponents/component/gimbal_chassis_comm.hpp`）

- 标准帧 8 字节，大端，1 ms 周期（`config::kInterboardPeriodMs = 1`），离线阈值 `100 ms`。
- **C2G 0x210**（底盘→云台）：
  - `b0`：flags（对板在线、Working 位、急停、IMU 已标定、遥控在线）；
  - `b1`：底盘模式（分离/跟随/小陀螺）+ 控制源 + 发送序号；
  - `b2-3`：云台期望**世界系 yaw**（int16，1e-3 rad）；
  - `b4-5`：云台期望 pitch（int16，1e-3 rad）；
  - `b6-7`：底盘 IMU yaw（int16，1e-3 rad）。
- **G2C 0x211**（云台→底盘）：
  - `b0`：flags（在线、Working、急停、IMU 已标定、遥控在线）；
  - `b1`：云台模式 + 控制源 + 发送序号；
  - `b2-3`：yaw 关节角（含零位偏置，int16，1e-3 rad）；
  - `b4-5`：pitch 关节角（含零位偏置，int16，1e-3 rad）；
  - `b6-7`：云台 IMU 世界 yaw（int16，1e-3 rad）。

> `tx_seq` 由 `encode()` 自动递增，接收端 `decode()` 刷新；`isOffline(now_ms)` 以 100 ms 判定。两板 `RobotComponents` 内容逐字节一致。

### 4.4 控制权与模式

- DR16 接收机接在**底盘板**，因此所有遥控输入（摇杆/拨杆/拨轮/急停）由底盘板统一解析，云台所需的遥控状态（在线/急停）经 C2G `b0` 转发。
- 左拨杆：上=跟随、中=分离、下=小陀螺；
- 右拨杆：下=急停（最外层，云台经 C2G estop 生效）。

---

## 5. 仓库结构

```text
omni-skeleton/                     Git 仓库顶层（远端 main 同一内容）
├── Chassis/                       底盘板固件（STM32F407IGH6）
│   ├── Core/                      CubeMX 生成：main.c/can.c/spi.c/usart.c/tim.c/iwdg.c/dma.c、中断
│   ├── Drivers/                   STM32F4 HAL + CMSIS
│   ├── inc/                       config.hpp、pid.hpp、imu.hpp、mahony.hpp、BMI088.hpp、
│   │                              HW_can.hpp、motor_base.hpp、motor_M3508.hpp、chassis_ik_solver.hpp
│   ├── src/                       上述实现 + DT7.cpp（DBUS 解码）
│   ├── Instance/component/        ins_* 单例（motor/pid/imu/comm/robot/chassis_gimbal_comm）
│   ├── Task/component/            main_task（1 ms 回调）、control_task（台架单轮）、comm_task（CAN/DBUS/帧/IWDG）
│   ├── RobotModules/component/    robot.*（三态/模式/遥测）、chassis.*（逆解+轮速环）
│   ├── third_party/               CMSIS-DSP、Eigen3、SEGGER RTT
│   └── cmake/                     arm-none-eabi 工具链文件、CMakePresets.json
├── Gimbal/                        云台板固件（目录同构）
│   ├── inc/ gimbal_math.hpp       pitch 限位/角度归一化纯函数（static_assert 校验）
│   ├── RobotModules/component/    robot.*（三态/目标消费）、gimbal.*（两轴控制/前馈/重力补偿）
│   └── Task/component/            main_task、comm_task（DM 收发/G2C/IWDG）
├── RobotComponents/component/     双板共享：module_state.hpp、gimbal_chassis_comm.*（0x210/0x211）
├── scripts/                       本地工具（不入库）：gen_ide_db（IntelliSense DB）、push（普通推送）
├── .gitattributes / .gitignore    行尾与忽略规则（入库）
└── README.md
```

---

## 6. 运行时架构

### 6.1 1 ms 主循环

```text
TIM6 1 ms → HAL_TIM_PeriodElapsedCallback（各板 Task/component/main_task.cpp）
  底盘：
    MainTask()    IMU update + Robot::update（状态机/模式/跟随/C2G 目标/逆解/轮速环）
    ControlTask() TEST_MODE==1 单轮扫掠；==0 不参与
    CommTask()    DBUS 校验 → 0x200 电流帧（CAN2）→ C2G 0x210（CAN1，1 ms）→ IWDG 喂狗
  云台：
    MainTask()    IMU update + Robot::update（状态机/目标冻结/云台控制）
    CommTask()    DM yaw MIT（CAN1）+ DM pitch MIT（CAN2）+ G2C 0x211（CAN1，1 ms）→ IWDG 喂狗
```

### 6.2 三态状态机（两板同构，条件不同）

| 迁移                     | 底盘板                                                                  | 云台板                                                                                  |
| ---------------------- | -------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| 任一时段                   | `safe_ok = 遥控在线 && !急停`                                              | `SafetyCheck = !急停 && C2G 在线 && 遥控在线 && IMU ready && DM 反馈新鲜且已使能`                    |
| Dead → Resurrection    | `safe_ok` 持续 100 ms                                                  | `SafetyCheck` 持续 100 ms                                                              |
| Resurrection → Working | `safe_ok && ready_ok && 摇杆回中` 持续 200 ms（`ready_ok = IMU 就绪 && 对板在线`） | `WorkingReady`（`SafetyCheck && IMU 已标定 && pitch 未越限(含 0.10 rad 裕度)`）持续 200 ms；无需摇杆回中 |
| Working → Dead         | `ready_ok` 丢失立即回 Dead                                                | `WorkingReady` 丢失立即回 Dead                                                            |
| 非 Working 输出           | 轮电流 0、蜂鸣器/状态安全                                                       | MIT 力矩 0（DM 在 kp=kd=0 下自由下垂，见 §9.9）                                                  |

> 底盘进入 Working 时 `onStateTransit` 会把云台目标初始化为当前反馈（避免目标突变）；云台进入 Working 时锁定当前姿态（不自动回正）；对板未 Working 时云台目标冻结。

### 6.3 底盘三模式

| 模式           | 触发    | 平移                   | 自转                                                | 备注        |
| ------------ | ----- | -------------------- | ------------------------------------------------- | --------- |
| 分离 kSeparate | 左拨杆中位 | 左摇杆 vx/vy（限 1.5 m/s） | **恒 0**（拨轮不参与），2026-11-16 用户要求                    | 云台独立瞄准    |
| 跟随 kFollow   | 左拨杆上  | 同上                   | 跟随环 PD 输出（δ→0，限 6 rad/s，死区 0.005 rad）             | 云台朝向后底盘跟随 |
| 小陀螺 kSpin    | 左拨杆下  | 同上（方向随云台 δ 旋转到车体系）   | 拨轮 × `kSpinWMax` 4.5 rad/s × `kSpinWheelSign`(-1) | 边旋转边平移    |

> 模式选择 30 ms 去抖；模式切换不重置状态机；急停在最外层。左/右摇杆水平方向已按实车极性修正（`vy = -rc_lh·kMaxVxVy`，云台 yaw 目标 `-= rc_rh·kGimbalYawRateMax·dt`）。

### 6.4 云台控制链

1. 世界系目标：底盘每 1 ms 根据右摇杆积分并限速（`kGimbalYawRateMax=5.0 rad/s`）生成 `gimbal_yaw_target`，pitch 由右摇杆竖直积分并 clamp 到 `[-0.35, +0.55] rad`。
2. 关节目标：`yaw_joint_target = WrapPi(world_target − chassis_imu_yaw)`；pitch 目标经 `ClampPitchTarget`（软限位 + 0.10 rad 裕度判定）。
3. 位置环：PID（yaw Kp=13/Ki=0/Kd=0；pitch Kp=30/Ki=0/Kd=0.5）输出速度目标，限幅 `kYawRateMax=8`（小陀螺模式 `kYawRateMaxSpin=12`）、`kPitchRateMax=4.5`。
4. 前馈：yaw 底盘速率前馈（差分 C2G `chassis_imu_yaw`，LPF α=0.06，0.08 rad/s 死区）+ pitch 目标速率前馈（差分 pitch_target，同滤波）。
5. 速度环：`Kp=0.8` 输出力矩，叠加 pitch 重力前馈 `τ = sign · A · cos(pitch)`（A=0.65 N·m、sign=−1），限幅 `kYawTorqueMax/kPitchTorqueMax = 6.5 N·m`。
6. DM MIT：`setMitInput(0, 0, 0, 0, τ)`，kp=kd=0 纯力矩。

### 6.5 安全链

- **急停**（右拨杆下）→ 底盘立即零输出并回 Dead，云台经 C2G estop 同理；
- **对板离线**（C2G/G2C >100 ms）→ 丢失 Working；
- **遥控离线**（DBUS >100 ms）→ 底盘回 Dead，云台经 flags 获知；
- **IMU 未标定** → 不进入 Working；
- **DM 反馈超时/未使能/状态码异常** → 云台回 Dead；
- **CAN bus-off** → 自动恢复（ABOM）+ 错误快照计数；
- **IWDG** → 1 ms 链末喂狗，任务链卡死则复位。

---

## 7. 关键参数速查

### 7.1 底盘（`Chassis/inc/config.hpp`）

| 参数                                          | 值                                                     | 说明                               |
| ------------------------------------------- | ----------------------------------------------------- | -------------------------------- |
| `kTs`                                       | 0.001 s                                               | 控制周期                             |
| `kMaxVxVy`                                  | 1.5 m/s                                               | 平移限幅                             |
| `kMaxW`                                     | 3.0 rad/s                                             | 历史常量（分离模式已不自转，保留兼容）              |
| `kSpinWMax`                                 | 4.5 rad/s                                             | 小陀螺自转限幅                          |
| `kSpinWheelSign`                            | -1.0                                                  | 小陀螺拨轮方向（实车标定）                    |
| `kWheelDeadZone`                            | 0.20                                                  | 拨轮死区（吃弹簧回中过冲），死区外线性重标定           |
| `kMaxWheelRadS` / `kMaxWheelCurrent`        | 800 rad/s / 10 A                                      | 轮速/电流限幅                          |
| `kPidWheel`                                 | Kp=1.2, Ki=0.8, Kd=0                                  | 轮速环，积分限幅=电流限幅                    |
| `kPidFollow`                                | Kp=5.0, Ki=0, Kd=0.55, α=0.4                          | 跟随环（δ→w），死区 0.005 rad，限幅 6 rad/s |
| `kGimbalYawRateMax` / `kGimbalPitchRateMax` | 5.0 / 1.5 rad/s                                       | 遥控目标角速率上限                        |
| `kGimbalPitchMin/Max`                       | -0.35 / +0.55 rad                                     | 软限位（与云台板一致）                      |
| `kStateDebounceMs` / `kResurrectionHoldMs`  | 100 / 200 ms                                          | 状态迁移去抖/保持                        |
| `kInterboardPeriodMs`                       | 1 ms                                                  | 双板帧周期                            |
| CAN                                         | C2G 0x210、G2C 0x211（CAN1）；电机帧 0x200/0x201~0x204（CAN2） | —                                |

### 7.2 云台（`Gimbal/inc/config.hpp`）

| 参数                                    | 值                                     | 说明                                          |
| ------------------------------------- | ------------------------------------- | ------------------------------------------- |
| `kYawAngleOffset`                     | -1.3340 rad                           | yaw 机械零位（正对车头时 `angle()≈0`，实车标定）            |
| `kPitchAngleOffset`                   | 0 rad                                 | pitch 零位（待硬件标定）                             |
| `kPitchMin/Max` + `kPitchLimitMargin` | -0.35 / +0.55 rad，0.10                | 软限位与越限裕度                                    |
| `kYawRateMax` / `kYawRateMaxSpin`     | 8.0 / 12.0 rad/s                      | 常规/小陀螺 yaw 关节限速                             |
| `kPitchRateMax`                       | 4.5 rad/s                             | pitch 关节限速                                  |
| `kYawTorqueMax` / `kPitchTorqueMax`   | 6.5 / 6.5 N·m                         | 实车调参值（注意电机温升，必要时回退）                         |
| `kPitchGravityTorqueAmp/Sign`         | 0.65 N·m / -1.0                       | pitch 重力前馈（符号实车标定：取 −1 才抵消下垂）               |
| 位置环 PID                               | yaw 13/0/0；pitch 30/0/0.5             | 实车调参值                                       |
| 速度环 PID                               | yaw 0.8/0/0；pitch 0.8/0/0             | 输出力矩                                        |
| DM                                    | yaw CAN1 id=2、pitch CAN2 id=1；减速比 4.3 | `config::kYawMotorCanBus/kPitchMotorCanBus` |

### 7.3 固件版本与构建基线（2026-11-17，`TEST_MODE=0`、`DEBUG_NO_IWDG=0`）

| 板       | 编译单元 | 结果                  | text     | data  | bss     | ELF         | `g_dbg_fw_tag` |
| ------- | ---- | ------------------- | -------- | ----- | ------- | ----------- | -------------- |
| Chassis | 677  | 0 error / 0 warning | 65,612 B | 304 B | 8,712 B | 2,415,556 B | `0x2026114C`   |
| Gimbal  | 678  | 0 error / 0 warning | 71,324 B | 196 B | 8,188 B | 2,371,964 B | `0x20261153`   |

> `g_dbg_fw_tag` 每次改动固件行为必须 +1，烧录后先在 Ozone 确认该值与源码一致，再看其它变量。

---

## 8. 调试变量（Ozone）

### 8.1 通用

- `g_dbg_fw_tag`（uint32）：固件版本号；
- `g_dbg_tick_cnt`（uint32）：1 ms 周期心跳（C 链接，两板各约 1000/s）；
- `g_dbg_robot_update_cnt`（底盘）：`Robot::update` 运行计数（判断 Robot 是否在跑）。

### 8.2 底盘（`g_dbg_*`，全局）

| 分组       | 变量                                                                                                                                                                     |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 状态机      | `g_dbg_life_state`、`g_dbg_safe_ok`、`g_dbg_remote_online`、`g_dbg_imu_ready`、`g_dbg_peer_online`、`g_dbg_estop`、`g_dbg_sticks_centered`                                   |
| 模式与目标    | `g_dbg_mode`、`g_dbg_source`、`g_dbg_delta_rad/deg`（C 链接镜像）、`g_dbg_follow_w`、`g_dbg_target_vx/vy/w`、`g_dbg_gimbal_yaw_target/pitch_target`                               |
| 遥控输入     | `g_dbg_rc_lv/lh/rv/rh/wheel/l_switch/r_switch`、`g_dbg_wheel_dz`、`g_dbg_rc_wheel_v16/raw/unwrapped`、`g_dbg_rc_wheel_raw_log[32]`/`unwrapped_log[32]`/`log_idx`          |
| 轮电机      | `g_dbg_wheel_target/vel/actual/out/curr/input/angle/temp[4]`（顺序 LF/LB/RB/RF）、`g_dbg_wheel_tx_cnt`                                                                      |
| CAN      | `g_can_wheel_rx_cnt[4]`、`g_can_wheel_unknown_cnt`、`g_can1_unknown_cnt`、`g_can_g2c_rx_cnt`、`g_dbg_c2g_tx_cnt`、`g_can_err_instance/code/esr/tsr/cnt`、`g_can_tx_fail_cnt` |
| PID/限幅镜像 | `g_dbg_pid_wheel_kp/ki/kd`、`g_dbg_pid_wheel_integ_max`、`g_dbg_max_wheel_current`                                                                                       |
| 参数镜像     | `g_dbg_pid_target`、`g_dbg_pid_fdb`、`g_dbg_pid_out`（TEST_MODE 扫掠用）                                                                                                      |

### 8.3 云台

- `robot::robot_debug::*`：`life_state`、`safety_ok`、`c2g_online`、`c2g_remote_online`、`c2g_estop`、`c2g_working`、`c2g_chassis_yaw`、`c2g_yaw_target`、`c2g_pitch_target`、`gimbal_yaw_target`、`gimbal_pitch_target`、`yaw_joint`、`pitch_joint`、`gimbal_imu_yaw`；
- `gimbal_debug::*`：`yaw_err`、`yaw_vel_target`、`yaw_torque`、`pitch_err`、`pitch_vel_target`、`pitch_torque`、`pitch_gravity_ff`、`chassis_yaw_rate`、`yaw_vel_ff`、`pitch_vel_ff`、`world_yaw_est`（= `WrapPi(chassis_yaw + yaw_joint)`）、`cycle_count`；
- DM/CAN：`g_dbg_dm_angle/vel/torq/curr/input/voltage/temp/enabled/status_code[2]`（[0]=yaw，[1]=pitch）、`g_dbg_can1_tx_cnt`/`g_dbg_can2_tx_cnt`/`g_dbg_g2c_tx_cnt`、`g_can_yaw_fdb_cnt`/`g_can_pitch_fdb_cnt`/`g_can_c2g_rx_cnt`、`g_can_yaw_motor_last_rx_tick`/`g_can_pitch_motor_last_rx_tick`、`g_can_err_*`。

### 8.4 使用注意

1. 必须先 `File → Open` 与烧录一致的 `.elf` 并 `Download & Reset`，否则变量地址/符号可能不匹配（观测恒 0 的常见原因）。
2. 匿名命名空间/静态局部变量的符号可能无法按名解析：本项目对关键量用 **`extern "C"` + `__attribute__((used, retain))`** 镜像为全局符号（如 `g_dbg_delta_rad`、`g_dbg_tick_cnt`）。
3. `TEST_MODE=1` 时 `Robot::update` 不运行，只有 `comm_task` 的轮速/电流/角度镜像有效；`wheel_target/actual/out` 来自 Robot，恒 0 属正常。
4. 曲线观察用 Ozone `Graph` 窗口（采样率 1 kHz 数据需缩小时基）；符号表搜索 `g_dbg_` 可列出全部底盘镜像。

---

## 9. 调试过程与典型问题记录

> 本节记录实车联调中定位并修复的主要问题，按主题归档，供后续维护/迭代参考。

### 9.1 DBUS 帧起点错位（2026-11-08）

- **现象**：拨杆恒 3、拨轮满量程反向、`rx_ok` 与 `bad` 同步增长。
- **根因**：USART3 循环 DMA 的 18 字节窗口从半帧开始 → 字段整体旋转；且 `HAL_UARTEx_RxEventCallback` 每帧会先收到 HT(9) 事件再收到 TC(18)/IDLE(0)，原代码把 HT 计为 bad；后因字节截断另加 13 位处理（详见 §9.3）。
- **修复**：DMA 改 **NORMAL** 模式，回调只在 TC/IDLE 帧边界解码并 `HAL_UART_AbortReceive + HAL_UARTEx_ReceiveToIdle_DMA` 重启（≤2 帧自动对齐）；诊断镜像 `g_dbg_dbus_raw[18]`/`g_dbg_dbus_chg[18]`（按位累计变化掩码）+ `center_err`（四通道对 1024 偏差和）验证。
- **验证**：静止帧 `00 04 20 00 01 F8 ...`、`center_err=0`；拨杆/摇杆/拨轮全部正确。

### 9.2 拨轮解码 4 版演进（2026-11-15 ~ 11-16）

1. `& 0x07FF` 截断 → 顺时针满量程反向（超出 11 位）；
2. 13 位连续展开（模 8192、阈值 4096）→ 逆时针到底 +660（`0→2047` 被误判 +2047）；
3. 混合分段 + v16 分段展开 → `v16==2048` 丢约 2048 counts，快速回拨残留正位移；
4. **第 4 版（当前）**：重叠带速度预测 + 单帧整程兜底（§拨轮规格见原实现记录：`Chassis/src/DT7.cpp`，两板同源）。Python 模型 `wheel_sim_v8.py` 验证缓慢/快速回拨、顺/逆/回中均正确。

### 9.3 CAN 拓扑与总线错误（2026-11-08）

- **现象**：`hcan1.ErrorCode = 0x00200000`（HAL_CAN_ERROR_PARAM，邮箱满）、电机不转、`ESR` 显示 bus-off/ACK 错误。
- **根因**：①固件假设与实物拓扑不符（3508 实际在 CAN2、双板帧在 CAN1、pitch 在 CAN2）；②无 ACK 导致持续重发 → 邮箱满、TEC 升高至 bus-off。
- **修复**：按实物重排总线（config 化）+ 打开 bus-off 自动恢复（ABOM）+ 发送失败清理邮箱 + 错误回调快照（`g_can_err_*`）+ CAN2SB 滤波组缺陷修复（`CAN_FilterTypeDef can_filter = {0}; SlaveStartFilterBank = 14`）。
- **验证**：轮反馈计数 `g_can_wheel_rx_cnt[4]` 各≈1000/s、错误计数不再增长。

### 9.4 上电 pitch 低头/抖动（2026-11-16）

- **现象**：云台上电先抬头后突然低头约 5°、偶发上下抖动；用遥控器动一下 pitch 后恢复。
- **根因**：底盘 Dead 期间 C2G 目标角为复位值 0，云台只判 estop/online 不消费 Working 位 → 云台先 Working 时目标被拉向 0；Dead 态 MIT 零力矩无重力补偿加剧。
- **修复**：云台消费 C2G `b0` Working 位：对板未 Working 时目标**冻结到当前反馈**；进入 Working 时锁定当前姿态（上电不回正）。
- **验证**：`c2g_working` 0→1 时 `pitch_target` 不再被拉 0。

### 9.5 跟随模式 7° 偏左 / 上电回正（2026-11-16）

- **现象**：跟随模式云台停在正前方偏左约 7°；上电后 yaw 自动转回正前方。
- **根因**：yaw 零位按 90° 假设偏了约 7°（实际 97°）；“回正”是底盘目标初值 0 的副产物而非显式功能。
- **修复**：按实车校准点设 `kYawAngleOffset = -1.3340`（正对车头时 `angle()≈0`）；用户选择“上电不回正”→ 云台进入 Working 时锁定当前姿态。

### 9.6 跟随模式响应慢/震荡（2026-11-15）

- **现象**：跟随响应慢、有较大超调、目标附近震荡较久。
- **根因**：跟随环为纯比例控制（斜坡稳态滞后 ≈ 目标速率/Kp）、阻尼不足、量化噪声导致极限环。
- **修复**：PD + 前馈：Kp=5、Kd=0.55、d 低通 α=0.4、死区 0.005 rad；云台侧 yaw 底盘速率前馈（LPF α=0.06 + 0.08 rad/s 死区）与 pitch 目标速率前馈；`kGimbalYawRateMax` 2.5→5.0；新增 `world_yaw_est` 验证世界固定。

### 9.7 小陀螺云台跟不上（2026-11-15 ~ 16）

- **现象**：小陀螺时云台被底盘拖着转、逆向补偿不足。
- **根因**：关节需求 = 底盘自转 + 摇杆补偿，原 `kYawRateMax=8` 被截断；力矩限幅 4.5 在叠加时不够。
- **修复**：新增 `kYawRateMaxSpin=12`（仅 kSpin 生效）；自转限幅降到 4.5；力矩限幅提到 6.5（用户调参，注意温升）；yaw 底盘速率前馈补斜坡滞后。
- **验证**：`yaw_vel_target` 允许 ±12、`yaw_torque` 不长期贴限幅。

### 9.8 pitch 下垂与重力前馈（2026-11-17）

- **现象**：pitch 静止时缓慢下垂、大力矩下静差大。
- **根因**：pitch 轴重心相对电机轴有机械偏置，重力力矩随俯仰角变化；MIT kp=kd=0 无重力补偿。
- **修复**：`τ_ff = kPitchGravityTorqueSign · kPitchGravityTorqueAmp · cos(pitch_joint)`（A=0.65 N·m），叠加在速度环输出后、力矩限幅前；符号实车标定为 −1（取 + 反而加剧下垂）。镜像 `pitch_gravity_ff`。
- **标定方法**：水平静止读 `pitch_torque` 稳态值≈A；方向反则改 sign；仍下垂加 A、上抬减 A。

### 9.9 IWDG 任务卡死验证（2026-11-16）

- **目的**：确认 IWDG 真能兜底任务链卡死。
- **方法**：临时在两板 `CommTask` 的喂狗前注入“上电 10 s 后 `__disable_irq(); while(true);`”，烧录观察：约 10 s + 超时后自动复位并循环。
- **结果**：两板均按预期复位（底盘 300 ms / 云台 600 ms），验证后代码已回退、未入库。

### 9.10 Ozone 观测恒 0（2026-11-16）

- **现象**：车轮在转，但 Ozone 里轮速等镜像恒 0。
- **根因**：①烧录 ELF 与 Ozone 打开的 ELF 不一致；②查看的变量只在 `Robot::update` 中刷新（`TEST_MODE=1` 时不运行）；③符号被链接器裁剪或为内部链接无法按名解析。
- **修复**：关键镜像加 `extern "C"` + `used, retain`（并加 `g_dbg_fw_tag`/`g_dbg_tick_cnt`/`g_dbg_robot_update_cnt` 心跳辅助定位）；操作流程：先确认 `fw_tag` → 再确认 `tick_cnt` 在涨 → 最后看业务变量。

### 9.11 拨轮弹簧回中“先顺后逆”（2026-11-15）

- **现象**：快速拨动后松手，拨轮先按拨动方向转一点、随后反向自转。
- **根因**：弹簧回中过冲越过中位（过冲量 >0.1），越过中位后解码方向翻转。
- **修复**：`kWheelDeadZone` 0.10→0.20（死区外线性重标定到满量程，保号）；中位硬复位 ±25 counts。

---

## 10. 构建、烧录与验证

### 10.1 构建

工具链：CMake + Ninja + `arm-none-eabi-gcc 14.3.1`（C++20），`arm-none-eabi-` 在 `PATH` 中。

```powershell
cd Chassis            # 或 Gimbal
cmake --preset Debug  # 生成 build/Debug
cmake --build --preset Debug -j 8
```

> 也可用 `tools/build_embedded.py`（计划仓库工具，自动选择 ninja 并校验 0 error/0 warning）。
> ELF 固定输出到 `<板>/build/omni_<board>.elf`，Ozone（SWD 4 MHz）加载。
> 路径含中文时如需 CMake 报错，可从 ASCII 路径（如 `D:\rm\proj_ascii` junction）构建，产物仍在原目录。

### 10.2 烧录后自检（每次）

1. Ozone `File → Open` 对应 ELF → `Download & Reset`；
2. 确认 `g_dbg_fw_tag` 与源码一致（底盘 `0x2026114C`、云台 `0x20261153`）；
3. 确认 `g_dbg_tick_cnt` 在增长（1 ms 任务链在跑）；
4. 底盘：`g_dbg_remote_online=1` → 拨杆/摇杆读数正常 → `g_dbg_life_state` 走到 2；
5. 云台：`c2g_online=1` → `life_state=2` → `yaw_joint/pitch_joint` 随遥控变化。

### 10.3 台架验证清单（L2，车轮/云台离地）

- T0：进入 Working（安全链全绿）；
- T1：遥杆/pitch 遥控方向、限速、死区；
- T2：世界系增稳（转底盘/转动云台，`world_yaw_est` 保持平直）；
- T3：pitch 软限位与越限保护；
- T4：模式切换（分离/跟随/小陀螺）、切换瞬态；
- T5：急停/恢复；
- T6：断联保护（拔 CAN1、关遥控）；
- T7：记录验收数据（阶跃响应、稳态误差、温升）。

> ⚠️ `TEST_MODE=1` 时上电自动三角波扫掠且无遥控入口，**必须车轮离地**；`DEBUG_NO_IWDG` 仅调试器场景使用；交付态二者必须为 0。

---

## 11. 已知限制与后续计划

### 11.1 已知限制

1. 
2. **发射机构未实现**：云台板 CAN2 上拨弹/摩擦轮未接入控制。
3. **力矩限幅 6.5 N·m 为实车调参值**：接近 DM-J4310-2EC 峰值区间，长时间大负载需监控温度，必要时回退。
4. **拨轮解码常量按当前接收机标定**：更换接收机/拨轮批次需重测三点（中位/顺到底/逆到底）更新 `kWheelSpanPos/kWheelSpanNeg/kWheelDeadZone`。
5. **上电不回正**：进入 Working 锁定当前姿态；如需要回正应作为显式功能（契约 v1.1.26 记录）。
6. **主机侧测试**：底盘 harness 76/76（`D:\rm\task8_selftest\test_logic.exe`，未入库）；云台为 15 条 `static_assert` 编译期校验。
7. **L3/L4 未完成**：整机联调、底盘整车运动（直线/8 字/跟随/小陀螺极速）、5 min 温升与噪声测试待做。

### 11.2 后续计划

- 底盘整车运动测试与 PID 精调（轮速环/跟随环）；
- 云台 pitch 零位与重力前馈复标定；
- 发射机构接入（云台板 CAN2）；
- 可选：速度环小积分/摩擦前馈、yaw 角加速度前馈、陷波滤波、ADRC。

---

## 12. 工程约定与文档索引

### 12.1 代码约定

- 所有可调参数集中在两板 `inc/config.hpp` 并注明标定状态（`TODO(硬件)`/`TODO(调参)`）；业务代码不写魔法数。
- 两板共享代码放 `RobotComponents/`，逐字节一致；`inc/pid.*`、`src/pid.cpp`、`inc/imu.hpp`、`src/imu.cpp`、`inc/mahony.*`、`src/mahony.cpp`、`inc/DT7.*`、`src/DT7.cpp` 两板同源（md5 校验）。
- 跨板语义变更必须先改接口契约再改代码；固件行为变更 `g_dbg_fw_tag` 必须 +1。
- 提交信息：`type(scope): 中文描述 [both boards]`（type ∈ feat/fix/test/docs/chore/tune）。
- 交付态：`TEST_MODE=0`、`DEBUG_NO_IWDG=0`。

### 12.2 文档索引（计划仓库，不在本仓库内）

- `D:\rm\RM步兵电控计划\01_任务分解与一个月计划.md`：WBS/里程碑；
- `D:\rm\RM步兵电控计划\02_接口契约_v1.0.md`：帧格式、状态语义、决策记录（D-1~D-12）、变更历史；
- `D:\rm\RM步兵电控计划\03_Agent任务卡.md`：任务卡与验收标准；
- `D:\rm\RM步兵电控计划\04_验收与调试清单.md`：L0–L4 验收清单；
- `D:\rm\RM步兵电控计划\reviews\`、`reports\`：评审与阶段报告。

### 12.3 参考

- RoboMaster 官方开发板 C 型例程、《RoboMaster 开发板 C 型嵌入式软件教程》；
- Hello World 战队双板骨架工程；

---

*本 README 描述当前实车联调状态；参数/幅度均在持续整定中，以 `config.hpp` 与固件 `g_dbg_fw_tag` 为准。*
