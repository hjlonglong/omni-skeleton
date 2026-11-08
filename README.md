# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 2026 赛季 RoboMaster 全向轮步兵电控工程，基于 Hello World 战队双板骨架二次开发：`Chassis/` 底盘板（RoboMaster C 型板）与 `Gimbal/` 云台板各一片 STM32F407IGH6，`RobotComponents/` 为两板共享代码。
> 当前里程碑 `v1.1`（2026-11-07）：**双板完整整车控制链**——底盘 4×M3508 速度环 + 三态状态机 + DBUS 遥控 + O 型全向轮逆解；云台 DM-J4310 两轴位置/力矩控制 + C2G/G2C 双板帧；两板 1 ms 调度与 IWDG 喂狗齐备。

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
| DM-J4310 收发 | ✅ | `Gimbal/inc/motor_DaMiao.hpp`、`Gimbal/src/motor_DaMiao.cpp`、`Gimbal/src/HW_can.cpp` | CAN1 反馈解码；MIT 纯力矩 `setMitInput(0,0,0,0,τ)`；已修 CAN2SB 滤波组缺陷 |
| 双板通讯帧 | ✅ | `RobotComponents/component/gimbal_chassis_comm.*`、`Task/component/comm_task.cpp`、`src/HW_can.cpp` | C2G 0x210 / G2C 0x211，8 字节大端，两板每 1 ms 收发；v1.1：C2G b2-3=云台期望世界 yaw、b4-5=期望 pitch |
| 看门狗（IWDG） | ✅ | `Core/Src/iwdg.c`、两板 `Task/component/comm_task.cpp` | 初始化末尾喂一次；`CommTask` 每 1 ms 喂狗（1 ms 链末）；`DEBUG_NO_IWDG=1` 仅台架+调试器使用 |
| 单轮台架调试任务 | ✅ | `Task/component/control_task.hpp`、`Task/component/control_task.cpp` | `TEST_MODE=1` 时单轮三角波扫掠（±40 rad/s、2 A、15 s 归零），`g_dbg_*` 可 Ozone 观测；`0` 时完全不覆盖 Robot 输出 |

## 完成进度（P2，2026-11-07）

- ✅ 底盘板完整控制链：DBUS → 三态状态机 → 三模式 → δ 变换 → O 型逆解 → 4 轮速度环 → 0x200；C2G 每周期发送；IWDG 1 ms 链末喂狗。
- ✅ 云台板完整控制链：C2G 目标消费 → 三态状态机 → yaw 世界坐标换算 + pitch 软限位 → 位置环/速度环 → DM MIT 力矩；G2C 每周期回发；CAN2SB 缺陷修复。
- ✅ 双板协议：0x210/0x211 每 1 ms 收发、`isOffline()` 100 ms 阈值、v1.1 云台目标角语义；两板 `RobotComponents` 头文件同源。
- ✅ 安全：急停/遥控离线/云台离线/IMU 未标定/电机反馈超时 → Dead + 全轴零输出；交付态 `TEST_MODE=0`、`DEBUG_NO_IWDG=0`。
- 🟡 无硬件自测：底盘主机 harness **76/76 PASS**（DT7 解码、三态机、三模式、δ、逆解、帧往返、集成序列）；云台 15 条编译期 `static_assert`；两板 L0 全量构建 0 error / 0 warning。
- ⛔ 待硬件/整机（L3/L4）：轮安装角与方向标定（`kXxxMountAngleRad`/`kWheelSign`/`kWheelDir`）、δ 符号（`kDeltaSign`）实车确认、DM 方向/零位/PMAX 核对、PID 实车整定、双板联调与 8 字/直线/跟随/小陀螺测试。

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
        CommTask()   0x200 电流帧 + 0x210 C2G 帧 + IWDG 喂狗
  云台：MainTask()   IMU 更新 + Robot::update（状态机/云台控制）
        CommTask()   0x211 G2C 帧 + IWDG 喂狗
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

基线（2026-11-07，`TEST_MODE=0` 交付态，Debug 全量构建，Lead 独立 `build\leadfull` 复验）：

| 板 | 编译单元 | 结果 | text | data | bss |
| --- | --- | --- | --- | --- | --- |
| Chassis | 677 | 0 error / 0 warning | 62,688 B | 280 B | 8,168 B |
| Gimbal | 678 | 0 error / 0 warning | 65,168 B | 112 B | 8,072 B |

无硬件自测：底盘主机 harness `D:\rm\task8_selftest\test_logic.exe`（76/76 PASS）；云台 `gimbal_math.hpp` 15 条 `static_assert` 随构建验证。Ozone 可观测 `robot::robot_debug`、`robot::gimbal_debug`、`g_dbg_*`、`g_can_*` 等调试变量。DBUS 链路自检变量（2026-11-08 起）：`g_dbg_dbus_rx_ok`（约 70 帧/s）、`g_dbg_dbus_rx_bad`、`g_dbg_rc_online`、`g_dbg_rc_l_switch`/`g_dbg_rc_r_switch`（1=上/2=下/3=中）、`g_dbg_rc_lv`/`lh`/`rv`/`rh`/`wheel`（[-1,1]）。
