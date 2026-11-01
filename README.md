# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 2026 赛季 RoboMaster 全向轮步兵电控工程，基于 Hello World 战队双板骨架二次开发：`Chassis/` 底盘板（RoboMaster C 型板）与 `Gimbal/` 云台板各一片 STM32F407IGH6，`RobotComponents/` 为两板共享代码。
> 当前里程碑 `v0.1.0`（2026-11-01）：底盘单轮速度环可台架运行，IMU 封装、1 ms 调度与看门狗就绪。

## 功能与实现文件

| 功能 | 状态 | 实现文件（入口） | 说明 |
| --- | --- | --- | --- |
| 参数集中管理 | ✅ | `Chassis/inc/config.hpp`、`Gimbal/inc/config.hpp` | 周期、CAN ID、限幅、调试开关（`TEST_MODE` / `DEBUG_NO_IWDG`）；业务代码不散落魔法数 |
| 1 ms 调度 | ✅ | `Chassis/Task/component/main_task.cpp`、`Chassis/Core/Src/tim.c` | TIM6 1 ms 中断 → `MainTask → ControlTask → CommTask` |
| PID 速度环 | ✅ | `inc/pid.hpp`、`src/pid.cpp`（两板同源） | 闭环输出限幅、积分抗饱和、死区与 D 项低通可配 |
| IMU 采集（BMI088 / SPI1） | 🟡 | `inc/BMI088.hpp`、`src/BMI088.cpp`、`inc/imu.hpp`、`src/imu.cpp` | 六轴读取 + 750 帧静止零漂标定；Gimbal 完整，Chassis 的姿态更新待补 |
| Mahony 姿态解算 | 🟡 | `inc/mahony.hpp`、`src/mahony.cpp`、`Gimbal/src/imu.cpp` | 姿态欧拉角与角速度输出；目前仅在 Gimbal 板接入 |
| CAN 电机收发（M3508×4） | ✅ | `inc/motor_M3508.hpp`、`src/motor_M3508.cpp`、`src/HW_can.cpp`、`Instance/component/ins_motor.cpp` | 0x200 单帧下发 4 轮电流；0x201~0x204 反馈解码；bus-off 自动恢复、错误快照计数 |
| 单轮台架调试任务 | ✅ | `Task/component/control_task.hpp`、`Task/component/control_task.cpp` | `TEST_MODE=1` 时启用：单轮三角波扫掠（±40 rad/s、2 A 限幅、15 s 归零），调试变量 `g_dbg_*`；`0` 时零电流兜底 |
| 双板通讯帧 | 🟡 | `RobotComponents/component/gimbal_chassis_comm.hpp`、`gimbal_chassis_comm.cpp`、`module_state.hpp` | 8 字节帧、大端 int16 角度（1e-3 rad）；编解码已完成，尚未接入两板调度 |
| DBUS 遥控 | ⛔ | `inc/DT7.hpp`、`src/DT7.cpp`、`Core/Src/usart.c` | USART3 + DMA 循环接收已配置，IDLE 帧解析待实现 |
| 状态机 / 全向轮运动学 | ⛔ | `Instance/component/ins_chassis_iksolver.cpp`、`RobotModules/component/chassis.cpp`、`robot.cpp` | 逆解与状态机尚未接入主循环 |
| 云台控制 | ⛔ | `Gimbal/Task/component/main_task.cpp`、`Gimbal/RobotModules/component/gimbal.cpp` | Gimbal 任务层为空壳；底层 PID/IMU/CAN 代码与底盘共用 |
| 看门狗（IWDG） | ✅ | `Core/Src/iwdg.c`、`Task/component/comm_task.cpp` | 约 300 ms；初始化后先喂一次、`CommTask` 每周期喂狗；`DEBUG_NO_IWDG` 可关 |

## 完成进度（P1，2026-11-01）

- ✅ 底盘侧闭环可跑：CAN 收发 → 反馈解码 → PID 单轮速度环 → 电流下发，1 ms 调度与看门狗齐备。
- 🟡 IMU：Gimbal 板已具备零漂标定 + Mahony 姿态 + 就绪判定；Chassis 板仅完成零漂标定（`Chassis/src/imu.cpp`）。
- 🟡 双板协议：帧结构、编解码与离线检测已实现（`RobotComponents/`），未接入两板调度。
- ⛔ 待开发：DBUS 遥控（WP-09）、全向轮运动学/状态机（WP-11/12）、云台控制（WP-08）、整机联调（WP-15）。

> ⚠️ 调试开关：两板 `TEST_MODE` 当前均为 `0`（交付/正常态）。台架调试单轮时改为 `1`：上电即执行三角波扫掠且无遥控入口，必须车轮离地、禁止落地测试。
>
> ⚠️ 已知限制：Chassis 板 `isReady()` 恒为 `false`（姿态更新未实现）；Gimbal 任务层未接入，云台暂不可控。

## 目录结构

```text
omni-skeleton/
├── Chassis/                    # 底盘板固件（STM32F407IGH6，C 型板）
│   ├── Core/                   # CubeMX 生成：main.c、can.c、spi.c、usart.c、tim.c、iwdg.c、dma.c 等
│   ├── Drivers/                # STM32F4 HAL + CMSIS
│   ├── inc/  src/              # 应用层：config.hpp、pid.*、imu.*、mahony.*、BMI088.*、HW_can.*、motor_*
│   ├── Instance/component/     # ins_* 单例：电机/PID/IMU 等实例获取（ins_motor、ins_pid、ins_imu…）
│   ├── Task/component/         # 任务层：main_task（1 ms 回调）、control_task（单轮速度环）、comm_task（CAN/喂狗）
│   ├── RobotModules/           # 板级组件：chassis.*、robot.*（整车/机器人级组合）
│   ├── third_party/            # CMSIS-DSP、Eigen3、SEGGER RTT
│   └── cmake/                  # arm-none-eabi 工具链文件
├── Gimbal/                     # 云台板固件（目录同构；Task 层待实现）
├── RobotComponents/component/  # 双板共享：module_state.hpp、gimbal_chassis_comm.*（0x210/0x211 帧）
└── README.md
```

## 1 ms 主循环（底盘）

```text
TIM6 1 ms → HAL_TIM_PeriodElapsedCallback（main_task.cpp）
  ├─ MainTask()     传感器更新 / 状态机（WP-12 待接入）
  ├─ ControlTask()  单轮速度环；后续接全车运动学（WP-11）
  └─ CommTask()     CAN 收发 + 看门狗 + 双板协议（WP-10 待接入）
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

基线（2026-11-01，`TEST_MODE=1`，Debug 全量构建）：

| 板 | 编译单元 | 结果 | text | data | bss |
| --- | --- | --- | --- | --- | --- |
| Chassis | 677 | 0 error / 0 warning | 46,988 B | 112 B | 7,960 B |
| Gimbal | 678 | 0 error / 0 warning | 38,840 B | 20 B | 6,900 B |
