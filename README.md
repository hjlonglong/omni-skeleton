# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 2026 赛季 RoboMaster 全向轮步兵电控工程，基于 Hello World 战队双板骨架二次开发：`Chassis/` 底盘板（RoboMaster C 型板）与 `Gimbal/` 云台板各一片 STM32F407IGH6，`RobotComponents/` 为两板共享代码。
> 当前里程碑 **v1.1.x 硬件联调**（2026-11-08）：双板完整控制链已实现并完成 L2 硬件链路验证——DBUS 遥控、两板 IMU、CAN 分发（4×M3508 反馈/0x200）、云台 DM yaw+pitch 反馈、**双板帧走 CAN1（100 Hz）**、云台 yaw/pitch **遥控控制正常**；待测：世界系增稳（T2）、死区/限速（T4）、急停恢复（T5）、断联保护（T6）、验收数据（T7）与底盘整车运动，L3/L4 整机联调。

## 功能与实现文件

| 功能 | 状态 | 实现文件（入口） | 说明 |
| --- | --- | --- | --- |
| 参数集中管理 | ✅ | `Chassis/inc/config.hpp`、`Gimbal/inc/config.hpp` | 周期、CAN ID、限幅、调试开关（`TEST_MODE` / `DEBUG_NO_IWDG`）；业务代码不散落魔法数 |
| 1 ms 调度 | ✅ | `Chassis/Task/component/main_task.cpp`、`Gimbal/Task/component/main_task.cpp`、`Core/Src/tim.c` | TIM6 1 ms 中断 → 底盘 `MainTask → ControlTask → CommTask`，云台 `MainTask → CommTask` |
| PID 速度/位置环 | ✅ | `inc/pid.hpp`、`src/pid.cpp`（两板同源） | 闭环输出限幅、积分抗饱和、死区与 D 项低通可配 |
| IMU 采集（BMI088 / SPI1） | ✅ | `inc/BMI088.hpp`、`src/BMI088.cpp`、`inc/imu.hpp`、`src/imu.cpp`（两板同源） | 六轴读取 + 750 帧静止零漂标定 + 就绪判定；两板均接入 TIM6 1 ms 更新 |
| Mahony 姿态解算 | ✅ | `inc/mahony.hpp`、`src/mahony.cpp`（两板同源，经 `src/imu.cpp` 调用） | 姿态欧拉角（roll/pitch/yaw）与 `gyro_z` 输出；两板均接入 1 ms 更新 |
| CAN 电机收发（M3508×4） | ✅ | `src/motor_M3508.cpp`、`src/HW_can.cpp`、`Instance/component/ins_motor.cpp` | 0x200 单帧下发 4 轮电流；0x201~0x204 反馈解码；bus-off 自动恢复、错误快照计数 |
| DBUS 遥控（DR16） | ✅ | `inc/DT7.hpp`、`src/DT7.cpp`、`Task/component/comm_task.cpp`、`Instance/component/ins_comm.cpp` | USART3 空闲+DMA 收 18 B（`HAL_UARTEx_RxEventCallback`）；死区/限幅/通道映射见契约 §7.1；>100 ms 判离线 |
| 三态状态机 | ✅ | `Chassis/RobotModules/component/robot.cpp`、`Gimbal/RobotModules/component/robot.cpp` | Dead / Resurrection / Working；急停最外层；任一条安全条件丢失立即回 Dead 并零输出 |
| O 型全向轮运动学 | ✅ | `Chassis/inc/chassis_ik_solver.hpp`、`Chassis/RobotModules/component/chassis.cpp` | 云台系速度 → δ 旋转 → 车体系 → 4 轮逆解 → 轮速环；安装角/几何/符号为可配常量（默认值待实车标定） |
| 底盘三模式 | ✅ | `Chassis/RobotModules/component/robot.cpp` | 分离（自转取拨轮）/ 跟随（云台 δ 跟随 PID）/ 小陀螺（平移方向仍随云台）；左拨杆切换，30 ms 去抖 |
| 云台两轴控制 | ✅ | `Gimbal/RobotModules/component/gimbal.cpp`、`Gimbal/RobotModules/component/robot.cpp` | yaw 世界目标 − 底盘 IMU yaw → 关节目标；pitch 软限位 [-0.35, +0.55] rad；位置环 → 速度环 → DM MIT 力矩 |
| DM-J4310 收发 | ✅ | `Gimbal/inc/motor_DaMiao.hpp`、`Gimbal/src/motor_DaMiao.cpp`、`Gimbal/src/HW_can.cpp` | yaw=CAN1、pitch=CAN2 反馈解码；MIT 纯力矩 `setMitInput(0,0,0,0,τ)`；已修 CAN2SB 滤波组缺陷 |
| 双板通讯帧 | ✅ | `RobotComponents/component/gimbal_chassis_comm.*`、两板 `Task/component/comm_task.cpp`、`src/HW_can.cpp` | C2G 0x210 / G2C 0x211，8 字节大端；**走两板物理相连的 CAN1，周期 10 ms（100 Hz，`kInterboardPeriodMs`）**；v1.1：C2G b2-3=云台期望世界 yaw、b4-5=期望 pitch；`isOffline()` 100 ms |
| 看门狗（IWDG） | ✅ | `Core/Src/iwdg.c`、两板 `Task/component/comm_task.cpp` | 初始化末尾喂一次；`CommTask` 每 1 ms 喂狗（1 ms 链末）；`DEBUG_NO_IWDG=1` 仅台架+调试器使用 |
| 单轮台架调试任务 | ✅ | `Task/component/control_task.hpp`、`Task/component/control_task.cpp` | `TEST_MODE=1` 时单轮三角波扫掠（±40 rad/s、2 A、15 s 归零），`g_dbg_*` 可 Ozone 观测；`0` 时完全不覆盖 Robot 输出 |

## 完成进度（P2 硬件联调，2026-11-08）

- ✅ **L2 硬件链路已验证（2026-11-08，实测）**：
  - DBUS 遥控链路正常（摇杆/拨杆/拨轮全量解码正确，帧对齐与通道映射已修正）；
  - 底盘/云台 IMU 初始化、零漂标定与姿态输出正常；
  - CAN 分发与电机配置：4×M3508 反馈解码 + 0x200 电流下发正常；云台 DM yaw/pitch 使能与反馈正常（yaw=CAN1 id2、pitch=CAN2 id1）；
  - **云台 yaw/pitch 遥控控制正常**（右摇杆 → C2G 目标 → 位置环/速度环 → DM MIT 力矩环）；
  - 双板帧走两板 CAN1、100 Hz 收发稳定（CAN1 负载约 65%）。
- ✅ 底盘板完整控制链（软件）：DBUS → 三态 → 三模式 → δ 变换 → O 型逆解 → 4 轮速度环 → 0x200。
- ✅ 云台板完整控制链（软件）：C2G 目标消费 → 三态 → yaw 世界坐标换算 + pitch 软限位 → 位置环/速度环 → DM MIT 力矩；G2C 回发（CAN1，100 Hz）。
- ✅ 双板协议：0x210/0x211、`isOffline()` 100 ms 阈值、v1.1 云台目标角语义；两板 `RobotComponents` 同源。
- ✅ 安全：急停/遥控离线/云台离线/IMU 未标定/电机反馈超时 → Dead + 全轴零输出；交付态 `TEST_MODE=0`、`DEBUG_NO_IWDG=0`。
- 🟡 **待硬件验证（下一步）**：T2 世界系增稳（转底盘云台保持指向）、T4 死区/限速、T5 急停与恢复、T6 断联保护、T7 验收数据；底盘整车运动（直线/8 字/跟随/小陀螺）；L3/L4 整机联调。
- 🟡 无硬件自测：底盘主机 harness 76/76 PASS（DT7 解码、三态机、三模式、δ、逆解、帧往返、集成序列）；云台 15 条编译期 `static_assert`；两板 L0 0 error / 0 warning。
- ⛔ 待标定（`TODO(硬件)`）：轮安装角/方向/几何（`kXxxMountAngleRad`/`kWheelSign`/`kWheelDir`）、`kDeltaSign` 实车确认、DM 方向/零位/PMAX 核对、PID 实车整定。

## 调试开关与已知限制

> ⚠️ `TEST_MODE`：交付态两板均为 `0`。台架单轮调试改为 `1`——上电即三角波扫掠且无遥控入口，**必须车轮离地、禁止落地**；测完改回 `0` 并重编。
> ⚠️ `DEBUG_NO_IWDG`：仅台架 + 调试器使用，交付态必须为 `0`。
> ⚠️ 硬件相关常量（轮安装角/几何/符号、DM 方向/零位、PMAX/VMAX/TMAX）均为**默认假设值**，已在 `config.hpp` 标 `TODO(硬件)`，实车 L3 必须逐项标定后才可落地运行。

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
        CommTask()   0x200 电流帧（CAN1）+ C2G 0x210 双板帧（CAN1，每 10 ms）+ IWDG 喂狗
  云台：MainTask()   IMU 更新 + Robot::update（状态机/云台控制）
        CommTask()   yaw MIT 0x02（CAN1）+ pitch MIT 0x01（CAN2）+ G2C 0x211 双板帧（CAN1，每 10 ms）+ IWDG 喂狗
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

基线（2026-11-08，CAN 拓扑修正版 / 云台遥控已验证版，`TEST_MODE=0` 交付态，Debug 全量构建）：

| 板 | 编译单元 | 结果 | text | data | bss | ELF |
| --- | --- | --- | --- | --- | --- | --- |
| Chassis | 677 | 0 error / 0 warning | 64,744 B | 288 B | 8,440 B | 2,408,332 B |
| Gimbal | 678 | 0 error / 0 warning | 66,036 B | 120 B | 8,200 B | 2,363,444 B |

固件版本（Ozone 中确认烧录）：底盘 `g_dbg_fw_tag = 0x20261121`（539365665）、云台 `g_dbg_fw_tag = 0x20261122`（539365666）。

无硬件自测：底盘主机 harness `D:\rm\task8_selftest\test_logic.exe`（76/76 PASS）；云台 `gimbal_math.hpp` 15 条 `static_assert` 随构建验证。

Ozone 可观测调试变量：
- 底盘全局：`g_dbg_life_state/mode/source/estop/remote_online/imu_ready/peer_online`、`g_dbg_yaw_target`/`g_dbg_pitch_target`、`g_dbg_rc_lv/lh/rv/rh/wheel/l_switch/r_switch`、`g_dbg_wheel_vel/curr/input/angle/temp[4]`、`g_dbg_can1_tx_cnt`/`g_dbg_c2g_tx_cnt`、`g_can_wheel_rx_cnt[4]`、`g_can_g2c_rx_cnt`、`g_dbg_dbus_*`；
- 云台：`robot::robot_debug::{life_state,safety_ok,c2g_online,c2g_remote_online,c2g_estop,c2g_yaw_target,c2g_pitch_target,c2g_chassis_imu_yaw}`、`g_dbg_dm_vel/curr/torq/angle/input/enabled/status_code[2]`、`g_dbg_can1_tx_cnt`/`g_dbg_can2_tx_cnt`/`g_dbg_g2c_tx_cnt`、`g_can_yaw_fdb_cnt`/`g_can_pitch_fdb_cnt`/`g_can_c2g_rx_cnt`。

> 提示：云台目标为**世界系 yaw**（v1.1 语义），转动底盘时云台会反向补偿保持指向（T2 待验证项，属预期设计）。
