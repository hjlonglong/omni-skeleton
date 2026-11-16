# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 2026 赛季 RoboMaster 全向轮步兵电控工程，基于 Hello World 战队双板骨架二次开发：`Chassis/` 底盘板（RoboMaster C 型板）与 `Gimbal/` 云台板各一片 STM32F407IGH6，`RobotComponents/` 为两板共享代码。
> 当前里程碑 **v1.2 实车联调**（2026-11-15）：双板完整控制链已实现，L2 硬件链路验证完成——DBUS（摇杆/拨杆/**拨轮混合分段回绕解码**）、两板 IMU、CAN 分发（**4×M3508 接底盘板 CAN2**；云台 yaw=CAN1、pitch=CAN2）、**双板帧走 CAN1（1 ms）**、云台 yaw/pitch 遥控正常；2026-11-15 完成一轮实车闭环调参（小陀螺=分离同速、左右摇杆方向修正、yaw 底盘速率前馈 + pitch 目标速率前馈、调参镜像与跟随模式稳定加固）。待验证：T2 世界系增稳复测、T4/T5/T6/T7、pitch PID 整定、底盘整车运动，L3/L4 整机联调。

## 功能与实现文件

| 功能 | 状态 | 实现文件（入口） | 说明 |
| --- | --- | --- | --- |
| 参数集中管理 | ✅ | `Chassis/inc/config.hpp`、`Gimbal/inc/config.hpp` | 周期、CAN ID、限幅、调试开关（`TEST_MODE` / `DEBUG_NO_IWDG`）；业务代码不散落魔法数 |
| 1 ms 调度 | ✅ | `Chassis/Task/component/main_task.cpp`、`Gimbal/Task/component/main_task.cpp`、`Core/Src/tim.c` | TIM6 1 ms 中断 → 底盘 `MainTask → ControlTask → CommTask`，云台 `MainTask → CommTask` |
| PID 速度/位置环 | ✅ | `inc/pid.hpp`、`src/pid.cpp`（两板同源） | 闭环输出限幅、积分抗饱和、死区与 D 项低通可配 |
| IMU 采集（BMI088 / SPI1） | ✅ | `inc/BMI088.hpp`、`src/BMI088.cpp`、`inc/imu.hpp`、`src/imu.cpp`（两板同源） | 六轴读取 + 750 帧静止零漂标定 + 就绪判定；两板均接入 TIM6 1 ms 更新 |
| Mahony 姿态解算 | ✅ | `inc/mahony.hpp`、`src/mahony.cpp`（两板同源，经 `src/imu.cpp` 调用） | 姿态欧拉角（roll/pitch/yaw）与 `gyro_z` 输出；两板均接入 1 ms 更新 |
| CAN 电机收发（M3508×4） | ✅ | `src/motor_M3508.cpp`、`src/HW_can.cpp`、`Instance/component/ins_motor.cpp` | 轮电机接**底盘板 CAN2**：0x200 单帧下发 4 轮电流；0x201~0x204 反馈经 CAN2 FIFO1 解码；bus-off 自动恢复、错误快照计数 |
| DBUS 遥控（DR16） | ✅ | `inc/DT7.hpp`、`src/DT7.cpp`、`Task/component/comm_task.cpp`、`Instance/component/ins_comm.cpp` | USART3 空闲+DMA 收 18 B（`HAL_UARTEx_RxEventCallback`，帧边界重启对齐）；**拨轮 ch4 为 13 位段/11 位回绕段混合编码，用“分段选基 + 最小幅度回绕 + 中位磁滞锁存”解码（见 §拨轮编码）**；死区/限幅/通道映射见契约 §7.1；>100 ms 判离线 |
| 三态状态机 | ✅ | `Chassis/RobotModules/component/robot.cpp`、`Gimbal/RobotModules/component/robot.cpp` | Dead / Resurrection / Working；急停最外层；任一条安全条件丢失立即回 Dead 并零输出 |
| O 型全向轮运动学 | ✅ | `Chassis/inc/chassis_ik_solver.hpp`、`Chassis/RobotModules/component/chassis.cpp` | 云台系速度 → δ 旋转 → 车体系 → 4 轮逆解 → 轮速环；安装角/几何/符号为可配常量（默认值待实车标定） |
| 底盘三模式 | ✅ | `Chassis/RobotModules/component/robot.cpp` | 分离（自转取拨轮，限幅 `kMaxW=3.0 rad/s`）/ 跟随（云台 δ 跟随 PD）/ 小陀螺（**自转限幅 = `kSpinWMax = kMaxW = 3.0`**，平移方向仍随云台，支持边旋转边平移：左摇杆平移 + 拨轮自转同时生效）；左拨杆切换，30 ms 去抖；左右摇杆水平方向已按实车极性修正 |
| 云台两轴控制 | ✅ | `Gimbal/RobotModules/component/gimbal.cpp`、`Gimbal/RobotModules/component/robot.cpp` | yaw 世界目标 − 底盘 IMU yaw → 关节目标; pitch 软限位 [-0.35, +0.55] rad；位置环 → 速度环 → DM MIT 力矩；**yaw 底盘速率前馈**（差分 C2G `chassis_imu_yaw`，强低通 α=0.06 + 0.08 rad/s 死区）与 **pitch 目标速率前馈**（差分 `pitch_target`，同滤波）；`kYawRateMax=8` / `kPitchRateMax=3.0`；调参镜像见 §Ozone 变量 |
| DM-J4310 收发 | ✅ | `Gimbal/inc/motor_DaMiao.hpp`、`Gimbal/src/motor_DaMiao.cpp`、`Gimbal/src/HW_can.cpp` | yaw=CAN1、pitch=CAN2 反馈解码；MIT 纯力矩 `setMitInput(0,0,0,0,τ)`；已修 CAN2SB 滤波组缺陷 |
| 双板通讯帧 | ✅ | `RobotComponents/component/gimbal_chassis_comm.*`、两板 `Task/component/comm_task.cpp`、`src/HW_can.cpp` | C2G 0x210 / G2C 0x211，8 字节大端；**走两板物理相连的 CAN1，周期 1 ms（`kInterboardPeriodMs`）**；v1.1：C2G b2-3=云台期望世界 yaw、b4-5=期望 pitch；`isOffline()` 100 ms |
| 看门狗（IWDG） | ✅ | `Core/Src/iwdg.c`、两板 `Task/component/comm_task.cpp` | 初始化末尾喂一次；`CommTask` 每 1 ms 喂狗（1 ms 链末）；`DEBUG_NO_IWDG=1` 仅台架+调试器使用 |
| 单轮台架调试任务 | ✅ | `Task/component/control_task.hpp`、`Task/component/control_task.cpp` | `TEST_MODE=1` 时单轮三角波扫掠（±40 rad/s、2 A、15 s 归零），`g_dbg_*` 可 Ozone 观测；`0` 时完全不覆盖 Robot 输出 |

## 完成进度（P2 硬件联调，2026-11-15）

- ✅ **L2 硬件链路已验证（2026-11-09，实测）**：
  - DBUS 遥控链路正常（摇杆/拨杆/拨轮全量解码正确，帧对齐与通道映射已修正）；
  - 底盘/云台 IMU 初始化、零漂标定与姿态输出正常；
  - CAN 分发与电机配置：**4×M3508 接底盘板 CAN2**，反馈 0x201~0x204 与 0x200 电流下发正常；云台 DM yaw/pitch 使能与反馈正常（yaw=CAN1 id2、pitch=CAN2 id1）；
  - **云台 yaw/pitch 遥控控制正常**（右摇杆 → C2G 目标 → 位置环/速度环 → DM MIT 力矩环）；
  - 双板帧走两板 CAN1、**1 ms 收发稳定**（CAN1 仅双板帧+yaw，负载约 45%）。
- ✅ 底盘板完整控制链（软件）：DBUS → 三态 → 三模式 → δ 变换 → O 型逆解 → 4 轮速度环 → 0x200（CAN2）。
- ✅ 云台板完整控制链（软件）：C2G 目标消费 → 三态 → yaw 世界坐标换算 + pitch 软限位 → 位置环/速度环 → DM MIT 力矩；G2C 回发（CAN1，1 ms）。
- ✅ 双板协议：0x210/0x211、`isOffline()` 100 ms 阈值、v1.1 云台目标角语义；两板 `RobotComponents` 同源。
- ✅ 安全：急停/遥控离线/云台离线/IMU 未标定/电机反馈超时 → Dead + 全轴零输出；交付态 `TEST_MODE=0`、`DEBUG_NO_IWDG=0`。
- 🟡 **待硬件验证（下一步）**：T2 世界系增稳（转底盘云台保持指向）、T4 死区/限速、T5 急停与恢复、T6 断联保护、T7 验收数据；底盘整车运动（直线/8 字/跟随/小陀螺）；L3/L4 整机联调。
- 🟡 无硬件自测：底盘主机 harness 76/76 PASS（DT7 解码、三态机、三模式、δ、逆解、帧往返、集成序列）；云台 15 条编译期 `static_assert`；两板 L0 0 error / 0 warning。
- ⛔ 待标定（`TODO(硬件)`）：轮安装角/方向/几何（`kXxxMountAngleRad`/`kWheelSign`/`kWheelDir`）、`kDeltaSign` 实车确认、DM 方向/PMAX 核对、PID 实车整定。
- ✅ 云台 yaw 机械零位：以实车校准点为准——**云台正对车头时 `g_dbg_dm_angle[0] ≈ 0`**，已预置 `kYawAngleOffset = -1.3340f`（上电读数 ≈ +1.334，正对车头 ≈ 0，跟随模式 7° 残差随之消除）；若正对车头读数 R≠0，按 `kYawAngleOffset += R` 再校（契约 D-9）。
- ✅ 跟随/小陀螺实车修正（2026-11-11）：跟随环 Kp/Kd/LPF/死区已二次整定（提高阻尼、消除死区边界震荡）；`kYawRateMax` 2.5→8.0 rad/s，使小陀螺时云台仍能世界稳定、不被底盘拖着转。
- ✅ 实车控制修正（2026-11-15，最终版）：分离 `kMaxW` 6.0→**3.0 rad/s**；小陀螺 `kSpinWMax` = **`kMaxW` = 3.0**（两模式同速，用户要求）；小陀螺支持边旋转边平移（左摇杆平移 + 拨轮自转，方向随云台朝向 δ）；**左右摇杆水平方向修正**（`vy = -rc_lh·kMaxVxVy`、云台 yaw 目标 `-= rc_rh·kGimbalYawRateMax·dt`，此前左推反而右移/右转）；`kGimbalYawRateMax` 2.5→**5.0 rad/s**；云台 **yaw 底盘速率前馈**（消除小陀螺“被拖着转”的斜坡滞后）与 **pitch 目标速率前馈**（消除 pitch ramp 的 ~11° 稳态滞后）；`kPidYawAngleKp` **10**、`kPitchRateMax` **3.0**；新增调参镜像 `yaw_err/yaw_vel_target/pitch_err/pitch_vel_target/yaw_vel_ff/pitch_vel_ff/world_yaw_est/chassis_yaw_rate`；跟随模式实车反馈“云台不稳/跟底盘偏移”→ 前馈强低通（α=0.06）+0.08 rad/s 死区 + `world_yaw_est` 验证变量（T2 复测中）。
- ✅ 拨轮死区增大（2026-11-15）：`kWheelDeadZone` 0.10→**0.20**——拨轮快速拨动后弹簧回中过冲越过中位，过冲量 >0.1，表现为“先按拨动方向转一点、随后反向自转”；死区吃掉回弹（死区外仍线性重标定到满量程）。
- ✅ 拨轮编码最终解（2026-11-15，实测标定）：ch4 为 **13 位段（顺时针，1024→8051）+ 11 位回绕段（逆时针，1024→0→2047→1684，−1387）的混合编码**；当前用“分段选基 + 最小幅度回绕差分 + 中位磁滞锁存（仅 11 位段）”解码，顺时针输出按 v16 分段（1024→2048 线性、>2048 满速 1.0），逆时针按展开值/1387 归一；完整规格见下一节。拨轮死区 `kWheelDeadZone=0.20` 用于吃掉弹簧回中过冲。

## 拨轮（DR16 通道 4）编码方式与解码（2026-11-15 实测标定，当前实现）

**硬件表现**（实现：`Chassis/src/DT7.cpp`，两板 `DT7.*` 同源；中位带弹簧回中）
- 原始值 `v16 = raw[16] | (raw[17] << 8)`，**中位 `v16 = 1024`**（松手回中）。
- 该接收机拨轮**不是单一 11 位/13 位编码**，而是两段不同编码基：
  - **顺时针**：13 位段，`v16: 1024 → 8051`（**+7027 counts**）；
  - **逆时针**：11 位回绕段，`v16: 1024 → 0 → 2047 → 1684`（**−1387 counts**，跨 0 回绕）。

**解码算法（顺序固定，`DT7::decode`）**

1. **分段选基**：`raw = (v16 > 2048) ? (v16 & 0x1FFF) : (v16 & 0x07FF)`；
2. **最小幅度回绕差分**：`d = raw − prev`；若 `|d| > 1024`，在 `d`、`d±2048`、`d±8192` 中取 `|·|` 最小者为真实增量（自动兼容两种基）；
3. **中位磁滞锁存**：仅 `v16 ≤ 2048` 段执行——`|raw − 1024| ≤ 25` 进入中位锁（展开值清 0），`> 140` 才离开并重锚，避免弹簧回中振荡反复归零；
4. **展开限幅**：`wheel_unwrapped_ ∈ [−1387, +7027]` counts；
5. **归一化输出**：
   - 顺时针（`v16 > 2048` 或展开值 > 0）：`rc = clamp((v16 − 1024) / 1024, 0, 1)`——`1024→2048` 线性加快、`> 2048` 恒 **1.0**；
   - 逆时针：`rc = wheel_unwrapped_ / 1387`（0 → **−1**）；
6. **拨轮死区**：`kWheelDeadZone = 0.20` 死区 + 死区外线性重标定到满量程（`ApplyWheelDeadZone`，保号）——吃掉弹簧回中过冲造成的“先顺后逆”现象。

**标定常量**（`Chassis/src/DT7.cpp`；更换接收机/拨轮批次后必须重测三点更新）：`kWheelSpanPos = 7027`、`kWheelSpanNeg = 1387`、`kWheelDetentIn = 25`、`kWheelDetentOut = 140`。

**观测镜像**：`g_dbg_rc_wheel_v16`（16 位原值）、`g_dbg_rc_wheel_raw`（分段选值）、`g_dbg_rc_wheel_unwrapped`（展开 counts，顺 **+7027** / 逆 **−1387**）、`g_dbg_rc_wheel_raw_log[32]` / `g_dbg_rc_wheel_unwrapped_log[32]`（环形日志，`g_dbg_rc_wheel_log_idx` 为写指针）。

**历史与结论**：①`& 0x07FF` 截断 → 顺时针满量程反向；②13 位连续展开 → 逆时针 +660（`0→2047` 被误判为 +2047）；③**当前混合分段模型同时解决两者**——2026-11-15 实测拨轮顺时针/逆时针/回中均正常，小陀螺自转方向随之正常。

## 调试开关与已知限制

> ⚠️ `TEST_MODE`：交付态两板均为 `0`。台架单轮调试改为 `1`——上电即三角波扫掠且无遥控入口，**必须车轮离地、禁止落地**；测完改回 `0` 并重编。
> ⚠️ `DEBUG_NO_IWDG`：仅台架 + 调试器使用，交付态必须为 `0`。
> ⚠️ 硬件相关常量（轮安装角/几何/符号、DM 方向/零位、PMAX/VMAX/TMAX）均为**默认假设值**，已在 `config.hpp` 标 `TODO(硬件)`，实车 L3 必须逐项标定后才可落地运行。
> ⚠️ **拨轮解码常量按当前接收机实测标定**（规格见上一节）：更换接收机/遥控器批次后，需重测中位/顺到底/逆到底三点，更新 `kWheelSpanPos/kWheelSpanNeg` 与 `kWheelDeadZone`。
> ⚠️ **上电/复位窗口（2026-11-16 修复）**：底盘进入 Working 前，其 C2G 帧里的云台目标角是复位值（`gimbal_pitch_target_` 上电为 0）。云台板现在会检查 C2G 的 Working 位（b0 bit1），底盘未 Working 时把云台目标**冻结到当前反馈**、不跟踪复位目标；底盘 Working 后自动恢复跟踪。修复前表现为云台上电 pitch 先抬头再突然低头约 5°、甚至上下抖动（目标 0 甩动 + 重力下垂叠加）。

## 目录结构

```text
omni-skeleton/
├── Chassis/                    # 底盘板固件（STM32F407IGH6，C 型板）
│   ├── Core/                   # CubeMX 生成：main.c、can.c、spi.c、usart.c、tim.c、iwdg.c、dma.c 等
│   ├── Drivers/                # STM32F4 HAL + CMSIS
│   ├── inc/  src/              # config.hpp、pid.*、imu.*、mahony.*、BMI088.*、HW_can.*、motor_*、chassis_ik_solver.hpp
│   ├── Instance/component/     # ins_* 单例：motor/pid/imu/comm/robot/chassis_gimbal_comm
│   ├── Task/component/         # main_task（1 ms 回调）、control_task（台架单轮）、comm_task（CAN/DBUS/双板帧/喂狗）
│   ├── RobotModules/component/ # robot.*（状态机/模式/遥测）、chassis.*（逆解+轮速环）
│   ├── third_party/            # CMSIS-DSP、Eigen3、SEGGER RTT
│   └── cmake/                  # arm-none-eabi 工具链文件
├── Gimbal/                     # 云台板固件（目录同构；robot.* 状态机、gimbal.* 两轴控制、gimbal_math.hpp 纯函数）
├── RobotComponents/component/  # 双板共享：module_state.hpp、gimbal_chassis_comm.*（0x210/0x211 帧）
└── README.md
```

## 1 ms 主循环

```text
TIM6 1 ms → HAL_TIM_PeriodElapsedCallback（各板 Task/component/main_task.cpp）
  底盘：MainTask()   IMU 更新 +（TEST_MODE==0 时）Robot::update（状态机/模式/逆解/轮速环）
        ControlTask() TEST_MODE==1 单轮扫掠；==0 不覆盖输出
        CommTask()   0x200 电流帧（**CAN2**）+ C2G 0x210 双板帧（CAN1，每 1 ms）+ IWDG 喂狗
  云台：MainTask()   IMU 更新 + Robot::update（状态机/云台控制）
        CommTask()   yaw MIT 0x02（CAN1）+ pitch MIT 0x01（CAN2）+ G2C 0x211 双板帧（CAN1，每 1 ms）+ IWDG 喂狗
```

## 构建

工具链：CMake + Ninja + arm-none-eabi-gcc（14.3.1，C++20），`arm-none-eabi-` 需在 `PATH`。

```powershell
cd Chassis            # 或 Gimbal
cmake --preset Debug  # 生成 build/Debug
cmake --build --preset Debug -j 8
```

> 也可不用 preset，直接 `cmake -S . -B build -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug`；`-DCMAKE_TOOLCHAIN_FILE` 必须为绝对路径。
> ELF 固定输出到 `<板>/build/omni_<board>.elf`，可用 Ozone（SWD 4 MHz）加载。

基线（2026-11-15，yaw 校准点 + 跟随阻尼 + 小陀螺限幅同速 + 云台速率前馈 + 左右摇杆方向修正 + 拨轮混合分段回绕解码 + 顺时针 v16 分段编码版，`TEST_MODE=0` 交付态，Debug 全量构建）：

| 板 | 编译单元 | 结果 | text | data | bss | ELF |
| --- | --- | --- | --- | --- | --- | --- |
| Chassis | 677 | 0 error / 0 warning | 65,652 B | 304 B | 8,712 B | 2,415,872 B |
| Gimbal | 678 | 0 error / 0 warning | 66,836 B | 120 B | 8,256 B | 2,365,700 B |

固件版本（Ozone 中确认烧录）：底盘 `g_dbg_fw_tag = 0x20261144`（539365700）、云台 `g_dbg_fw_tag = 0x20261143`（539365699）。

无硬件自测：底盘主机 harness `D:\rm\task8_selftest\test_logic.exe`（76/76 PASS）；云台 `gimbal_math.hpp` 15 条 `static_assert` 随构建验证。

Ozone 可观测调试变量：
- 底盘全局：`g_dbg_life_state/mode/source/estop/remote_online/imu_ready/peer_online`、`g_dbg_delta_rad`/`g_dbg_delta_deg`（δ，C 链接镜像，Ozone 直接输名字即可；`g_dbg_delta` 为内部链接同名量）、`g_dbg_follow_w`、`g_dbg_target_vx/vy/w`、`g_dbg_wheel_dz`、`g_dbg_rc_wheel_raw`（按 v16 分段选值：`v16>2048` 取 `v16 & 0x1FFF`、否则取 `v16 & 0x07FF`；中位=1024）、`g_dbg_rc_wheel_v16`（未截断 16 位原值）、`g_dbg_rc_wheel_unwrapped`（分段展开位移 counts：顺时针到底 +7027、逆时针到底 −1387；顺时针输出按 v16 分段（1024→2048 线性、>2048 满速），逆时针按 span 1387 归一；中位锁存带 25/140 counts 回差，回中后锁 0、离开超过 140 才重新计，避免弹簧回中振荡反复归零/上升）、`g_dbg_rc_wheel_raw_log[32]`/`g_dbg_rc_wheel_unwrapped_log[32]`（每帧原始/展开值环形日志，`g_dbg_rc_wheel_log_idx` 为下一个写位置，用于回中异常诊断）、`g_dbg_yaw_target`/`g_dbg_pitch_target`、`g_dbg_rc_lv/lh/rv/rh/wheel/l_switch/r_switch`、`g_dbg_wheel_vel/curr/input/angle/temp[4]`、`g_dbg_wheel_target/actual/out[4]`、`g_dbg_wheel_tx_cnt`/`g_dbg_c2g_tx_cnt`、`g_can_wheel_rx_cnt[4]`、`g_can_wheel_unknown_cnt`、`g_can1_unknown_cnt`、`g_can_g2c_rx_cnt`、`g_dbg_dbus_*`；
- 云台：`robot::robot_debug::{life_state,safety_ok,c2g_online,c2g_remote_online,c2g_estop,c2g_yaw_target,c2g_pitch_target,c2g_chassis_imu_yaw}`、`gimbal_debug::{yaw_world_target,yaw_joint_target,yaw_joint,yaw_torque,pitch_target,pitch_joint,pitch_torque,cycle_count}`（前馈调试新增 `gimbal_debug::chassis_yaw_rate`、`gimbal_debug::yaw_vel_ff`，单位 rad/s；`gimbal_debug::world_yaw_est` 为世界系 yaw 估计 = WrapPi(chassis_yaw+yaw_joint)，用于验证跟随/小陀螺时云台是否真正世界固定；调 PID 用 `gimbal_debug::yaw_err`/`yaw_vel_target`、`pitch_err`/`pitch_vel_target`——后者即**速度环目标**，单位 rad/s；`gimbal_debug::pitch_vel_ff` 为 pitch 目标速率前馈（2026-11-15 新增，消除 ramp 滞后）、`g_dbg_dm_vel/curr/torq/angle/input/enabled/status_code[2]`、`g_dbg_can1_tx_cnt`/`g_dbg_can2_tx_cnt`/`g_dbg_g2c_tx_cnt`、`g_can_yaw_fdb_cnt`/`g_can_pitch_fdb_cnt`/`g_can_c2g_rx_cnt`。

> 提示：云台目标为**世界系 yaw**（v1.1 语义），转动底盘时云台会反向补偿保持指向（T2）。2026-11-15 已加 yaw/pitch 前馈与验证变量：`gimbal_debug::world_yaw_est` 应保持平直（底盘转动时云台世界指向不变）；可用“分离模式、车轮架空、手转底盘”测试符号链（云台应反向补偿）。若该量仍随底盘漂移，先查关节/世界公式符号链再调参。
