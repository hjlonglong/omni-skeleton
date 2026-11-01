# omni-skeleton — RoboMaster 全向轮步兵电控（双板）

> 本仓库是 2026 赛季 RoboMaster 全向轮步兵的电控工程，基于 Hello World 战队提供的双板骨架二次开发。
> **本 README 随仓库推送**：远端快照 = `README.md` + `Chassis/` + `Gimbal/` + `RobotComponents/`；`scripts/`、`.vscode/`、`*.jdebug` 等本地工具不推送。

- 代码基线：本地 `main`，远端 `origin/main`（`https://github.com/hjlonglong/omni-skeleton.git`）为 `README + Chassis + Gimbal + RobotComponents` 快照。
- 计划 / 契约 / 验收文档：`D:\rm\RM步兵电控计划\`（`01_任务分解与一个月计划.md`、`02_接口契约_v1.0.md`、`03_Agent任务卡.md`、`04_验收与调试清单.md`）。
- 旧仓库（全量历史，归档只读）：`D:\rm\大作业\omni_skeleton`。

---

## 1. 当前状态（2026-11-01）

阶段 P1「底盘单轮闭环 + IMU 封装」已收口，底盘板具备：CAN 电机收发闭环、PID 类、单轮速度环调试任务、BMI088 + Mahony 姿态解算、1 ms 调度与 IWDG。

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| PID 类 | ✅ 完成 | 两板同源 `inc/pid.hpp` + `src/pid.cpp`（契约 §6.1，md5 一致；主机单测在旧仓库 `tests/host`） |
| IMU 封装 | ✅ 完成 | BMI088(SPI1) + Mahony；750 帧静止零漂标定、`isReady` 门控；两板同源 |
| CAN 电机收发 | ✅ 完成（底盘） | 4×M3508：0x201~0x204 反馈解码、0x200 单帧下发；CAN2 双板通讯仅初始化未接入 |
| ControlTask 单轮 | ✅ 完成 | `TEST_MODE=1` 台架三角波扫掠（±40 rad/s、2 A 限幅、15 s 归零）；`TEST_MODE=0` 零电流兜底 |
| 1 ms 调度 | ✅ 完成 | TIM6 → `HAL_TIM_PeriodElapsedCallback` → `MainTask → ControlTask → CommTask` |
| IWDG 看门狗 | ✅ 完成 | 约 300 ms；`MainTaskInit` 内先喂一次，`CommTask` 每周期喂狗；`DEBUG_NO_IWDG` 可临时关闭 |
| DBUS 遥控 | ⛔ 未开始 | WP-09 |
| 双板协议接入 | ⛔ 未开始 | `RobotComponents/gimbal_chassis_comm.*` 已实现 encode/decode，尚未接入两板调度（WP-10） |
| 状态机 / 单车运动学 | ⛔ 未开始 | WP-12 / WP-11 |
| 云台控制 | ⛔ 未开始 | WP-08；Gimbal 任务层仍为空壳 |
| Gimbal IMU | 🟡 代码已实现 | 与底盘同源，但 `Gimbal/Task/component/main_task.cpp` 仍为 TODO，未接入调度 |

> ⚠️ **当前提交处于台架调试态**：`Chassis/inc/config.hpp` 中 `TEST_MODE 1`。上台/交付前必须改回 `0` 并重新编译（见 §9.4）。

---

## 2. 目录结构

```text
omni-skeleton/                # 本地工作根（Git 仓库顶层 = README.md + 三个子目录）
├── README.md                 # 本文件（随仓库推送）
├── Chassis/                  # 底盘板（RoboMaster C 型开发板，STM32F407IGH6）
│   ├── Core/                 # CubeMX 生成：main.c / can.c / spi.c / tim.c / iwdg.c / gpio.c、中断
│   ├── Drivers/              # STM32F4 HAL + CMSIS
│   ├── inc/                  # 应用头文件：config.hpp、pid.hpp、imu.hpp、BMI088.hpp、mahony.hpp、
│   │                         #   motor_base.hpp、motor_M3508.hpp、motor_DaMiao.hpp、DT7.hpp 等
│   ├── src/                  # 应用实现：pid.cpp、imu.cpp、HW_can.cpp、motor_*.cpp、ahrs*、pid、RTT 配置等
│   ├── Instance/component/   # ins_* 单例层：ins_motor / ins_pid / ins_imu / ins_chassis_iksolver ...
│   ├── Task/component/       # 任务层：main_task / comm_task / control_task
│   ├── third_party/          # CMSIS-DSP、Eigen3、SEGGER RTT
│   ├── cmake/                # arm-none-eabi 工具链文件
│   ├── chassis.jdebug        # SEGGER Ozone 工程（加载 build/omni_chassis.elf）
│   └── build/                # 构建产物（Git 忽略）：ide/（IntelliSense DB）、omni_chassis.elf
├── Gimbal/                   # 云台板（同构板级目录，gimbal.jdebug）
├── RobotComponents/          # 双板共享代码：module_state.hpp、gimbal_chassis_comm.{hpp,cpp}
│   └── component/
├── scripts/                  # 本地工具（不推送）：gen_ide_db、push、clean_builds、README
├── omni.code-workspace       # 多根 VSCode 工作区
└── test_chassis.jdebug       # 临时调试工程（可删）
```

---

## 3. Git 与推送约定

- 远端仓库：`https://github.com/hjlonglong/omni-skeleton.git`，`main` 分支 = `README.md` + `Chassis/` + `Gimbal/` + `RobotComponents/` 的快照。
- **白名单推送**：`scripts/push.ps1` 只允许 `README.md`、`Chassis`、`Gimbal`、`RobotComponents` 四个顶层条目进入快照，其它顶层文件（scripts、计划文档、PDF、`*.code-workspace`）一律不推；推送前校验提交树顶层条目，违反白名单直接报错。
- README 已纳入 Git 版本管理：修改 README 后先 `git add README.md && git commit`，再运行 `push.cmd` 才会同步到远端。
- `scripts/push.ps1` 还会：`--force-with-lease` 强制覆盖 `origin/main`；推送成功后删除远端除 `main` 外的分支与全部 tag（`-NoCleanRefs` 关闭）。
- 本仓库历史为简化历史（旧仓库 `D:\rm\大作业\omni_skeleton` 保留完整开发历史），远端被强推改写属预期行为。
- **不要**在本地 `pull`/`merge` 远端 `main`：远端是无父快照，只用于分发；本地 `main` 才是开发主线。
- 提交信息规范：`type(scope): 中文简述`，多处改动加 `[both boards]`；正文写动机与验证证据（如 `L0: 677/677 OK, 0 warning`）。

```powershell
# 推送（在普通终端、能访问 GitHub 的环境执行）
D:\rm\omni-skeleton\scripts\push.cmd
```

---

## 4. 硬件与外设映射

| 项目 | 配置 | 备注 |
| --- | --- | --- |
| MCU | STM32F407IGH6（168 MHz） | 底盘为 RoboMaster C 型开发板 |
| CAN1 | PD0/PD1，AF9，1 Mbps | 接 C620 电调 ×4（M3508） |
| CAN2 | PB5/PB6，AF9，1 Mbps | 预留双板通讯（0x210/0x211） |
| USART3 | 100000 bps，9 bit（含校验）+ 偶校验 + 1 stop | DBUS 接收（DMA/IDLE） |
| USART1 / USART6 | 115200 bps | 调试/图传等预留 |
| SPI1 | PB3/SCK、PB4/MISO、PA7/MOSI，软件 CS：PA4(BMI088 acc)、PB0(BMI088 gyro) | 8 bit、MSB、预分频 16 |
| TIM6 | 1 ms 周期中断 | `tick::GetTickMs()` 时基、任务调度 |
| IWDG | 约 300 ms | 停喂复位；调试可 `DEBUG_NO_IWDG` 关闭 |
| 电机 | 4×M3508（C620），id 1~4，dir 全 +1（待标定） | 三麦轮/全向轮顺序见 `config.hpp` |

> 引脚/电机 ID/方向以 `Chassis/inc/config.hpp` 与 `docs/hardware_table.md`（旧仓库）为准；接线变更后务必同步。

---

## 5. 软件架构

### 5.1 分层

```text
Core (CubeMX)   →   Task 层 (main_task / comm_task / control_task)
                →   Instance 层 (ins_motor / ins_pid / ins_imu / ...，函数内 static 单例)
                →   组件库 (motor、BMI088、Mahony、Pid、DT7、offline_checker、tick)
                →   硬件抽象 (HAL / CAN / SPI / UART)
RobotComponents/component/  →  两板共享类型与帧协议（module_state、gimbal_chassis_comm）
```

- **参数唯一来源**：`Chassis/inc/config.hpp`、`Gimbal/inc/config.hpp`（公共段保持一致，板级差异单独分区）；业务代码禁止散落魔法数。
- **单例约定**：`GetXxx()` 内部函数内 `static` 构造，避免全局构造在 HAL/时钟初始化之前运行（契约 §6.3）。
- **构建口径**：组件库头文件两板同源（PID/IMU 已 md5 校验一致），改动需同步两板。

### 5.2 1 ms 主循环（底盘）

```text
TIM6 更新中断
  └─ HAL_TIM_PeriodElapsedCallback
       ├─ MainTask()      // 传感器/状态机（待 WP-12 接入）
       ├─ ControlTask()   // 单轮速度环 / 后续全车运动学
       └─ CommTask()      // CAN 收发 + 看门狗 + 双板协议（待 WP-10）
```

数据流：CAN RX 中断按 StdId 分发 `decode()` → `ControlTask` 读 `Motor::vel()` 做 PID → `setInput()` → `CommTask` 用同一 `uint8_t tx[8]` 依次 `encode()` 4 轮，再发一帧 0x200（每周期必发，电机 100 ms 离线保护）。

### 5.3 双板通讯协议（已实现，未接入）

`RobotComponents/component/gimbal_chassis_comm.hpp`：`GimbalChassisComm(CodePart, tx_id, rx_id)`，8 字节帧，大端 `int16` 角度（1e-3 rad），`b0` 在线/急停/IMU 标定/遥控标志，`b1` 模式 + 来源 + 序号，`b2~b7` 三组 int16；编码自动递增 `tx_seq`，解码刷新 `last_rx_tick_`，`isOffline()` 阈值 `config::kCommOfflineThresMs = 100`。接入见 WP-10。

---

## 6. 构建（L0）

### 6.1 工具链

| 工具 | 版本 / 位置（本机） |
| --- | --- |
| CMake | 4.4.3，`D:\CMake\bin\cmake.exe` |
| Ninja | `D:\ninja-win\ninja.exe` |
| arm-none-eabi-gcc | 14.3.1，`D:\gnu\arm-gnu-toolchain-14.3.rel1-mingw-w64-i686-arm-none-eabi\bin` |
| 烧录/调试 | Ozone（`chassis.jdebug`/`gimbal.jdebug`）、OpenOCD 0.12.0 |

> 工具链路径不要含中文；本仓库根 `D:\rm\omni-skeleton` 为纯 ASCII，可直接构建。

### 6.2 两种构建方式

方式 A（推荐，Python 驱动，自动配置 + 编译，产物复制到 `build/omni_<board>.elf`）：

```powershell
cd D:\rm\omni-skeleton\Chassis
python D:\rm\RM步兵电控计划\tools\build_embedded.py . --build-subdir build\dev -j 8
```

方式 B（原生命令）：

```powershell
cd D:\rm\omni-skeleton\Chassis
cmake -S . -B build\dev -G Ninja `
  -DCMAKE_MAKE_PROGRAM=D:/ninja-win/ninja.exe `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake `
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build\dev -j 8
```

产物在 `build\dev`（或 `build\omni_chassis.elf`），可直接用 Ozone 加载。

### 6.3 基线（2026-11-01，`TEST_MODE=1`）

| 板 | 编译单元 | 结果 | text | data | bss | ELF |
| --- | --- | --- | --- | --- | --- | --- |
| Chassis | 677 | ✅ 0 error / 0 warning | 46,988 B | 112 B | 7,960 B | `Chassis/build/omni_chassis.elf` |
| Gimbal | 678 | ✅ 0 error / 0 warning | 38,840 B | 20 B | 6,900 B | `Gimbal/build/omni_gimbal.elf` |

> 构建目录可随时删除（`build\` 已 Git 忽略）；保留 `build\ide\`（IntelliSense 数据库）即可继续代码补全。全量 Debug 构建约 165 MB/板，不要同时堆积多份。

---

## 7. VSCode / IntelliSense（F12 跳转）

- 打开 `omni.code-workspace`（多根）或直接打开 `Chassis`/`Gimbal` 板目录；单板目录打开时只加载该板 `.vscode` 配置。
- 生成/刷新编译数据库：

```powershell
D:\rm\omni-skeleton\scripts\gen_ide_db.cmd
```

  脚本会对两板执行 CMake configure（`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`），输出 `<板>/build/ide/compile_commands.json`（Chassis 约 676、Gimbal 约 677 条），不产生编译产物。
- 然后在 VSCode 执行 `C/C++: Reset IntelliSense Database` 并重载窗口。
- 仍无法跳转时：查看 `C/C++: Log Diagnostics`（确认 compilerPath/compileCommands 被采用）；检查是否装了会抢占 C++ 解析的扩展（如 STM32CubeIDE/clangd 插件），必要时禁用；确认路径不含中文（本仓库已满足）。

---

## 8. 调试（Ozone / RTT）

- 工程文件：`Chassis/chassis.jdebug`、`Gimbal/gimbal.jdebug`（STM32F407IG，SWD 4 MHz，自动加载 `$(ProjectDir)/build/omni_<board>.elf`）。
- SEGGER RTT 已链接（`segger_rtt` 库），可直接在 Ozone Terminal 加 RTT 通道；当前应用层尚未打日志，主要靠变量观测。
- 常用观测变量：

| 变量 | 位置 | 含义 |
| --- | --- | --- |
| `g_inited` / `g_start_tick_ms` | `control_task.cpp` | 单轮任务是否初始化 / 扫掠起点 |
| `g_dbg_target` / `g_dbg_vel` / `g_dbg_out` | `control_task.cpp` | 目标、实测轮速（rad/s）、PID 输出（A）；仅 `TEST_MODE=1` 编译 |
| `g_motor->vel_` / `curr_` / `angle_` / `temp_` | `Motor`（`ins_motor` 静态实例） | 单轮反馈 |
| `g_pid->cfg_.out_max` | `Pid` | 应等于 2.0（TEST_MODE=1 时为台架限幅，可验证调试模式） |
| `g_can_err_instance` / `code` / `esr` / `tsr` / `cnt` | `HW_can.cpp` | CAN 错误快照：`instance` 0=CAN1、1=CAN2、0xFF=其它；`code`=HAL ErrorCode、`esr`/`tsr`=寄存器 |
| `g_can_tx_fail_cnt` | `HW_can.cpp` | 发送失败累计（邮箱满/总线异常） |
| `hcan1.ErrorCode` / `Instance->ESR` / `TSR` | 外设 | HAL 当前错误码、错误状态寄存器、发送状态寄存器 |

---

## 9. 调试模式与安全须知

### 9.1 `TEST_MODE`

- `TEST_MODE 1`：台架/单项调试态。底盘 `ControlTask` 对 1 号轮做三角波速度扫掠（1 s 等待 → ±40 rad/s、80 rad/s²、15 s 后归零），电流限幅 2 A（`kTestWheelCurrentLimit`）。
- `TEST_MODE 0`：正常态。`ControlTask` 保持零电流兜底，等待 WP-09/WP-12 接入真实目标。
- **交付/上台前必须为 0**。改宏后必须重新编译并烧录，`TEST_MODE=1` 时 ELF 无遥控入口，上电即自转，严禁落地测试。
- 台架流程：车轮离地 → 烧录 → 1 s 等待 → 观察 `g_dbg_*` 曲线（Ozone Live Watch）→ 15 s 自动归零 → 断电。

### 9.2 IWDG

约 300 ms 超时；`MainTaskInit()` 内先喂一次，`CommTask()` 每周期喂狗（`DEBUG_NO_IWDG != 0` 时不喂）。调试时若长时间断点看到复位，属预期；需要时临时 `DEBUG_NO_IWDG 1`（勿提交）。

### 9.3 CAN 总线排查

- 典型故障：`hcan1.ErrorCode = 0x00200000`（PARAM = 3 个发送邮箱全满）→ 帧发不出去；`ESR` 中 `BOFF=1`、`LEC=5`（位显性错误）、`TEC` 高位 → 无节点应答（ACK 缺失），通常是硬件层：C620 未上电/未接、总线只接了 CAN_H/CAN_L 两根线而收发器无电、两端各 120 Ω 导致只剩 60 Ω 不匹配、或插错 CAN 口。
- 固件侧已开 `AutoBusOff = ENABLE`（bus-off 后自动恢复）、CAN 错误中断（`CAN_IT_ERROR`）与 `HAL_CAN_ErrorCallback` 错误快照；`CAN_Send_Msg` 失败会记录并释放邮箱，避免 3 个邮箱永久占满。
- 若怀疑收发器/MCU 侧，可临时把 CAN 设为 `LOOPBACK` 自检；正常模式 `ESR.BOFF=0`、`TXOK` 置位才算通。

### 9.4 其他安全红线

- 上电前确认 `TEST_MODE`、`DEBUG_NO_IWDG` 状态；
- 任何电机使能/输出改动须有零输出兜底；
- 云台 pitch 限幅、急停标志（`estop`）必须在控制链生效后再上台架。

---

## 10. 已知问题与 TODO

1. **CAN 无 ACK**（`0x200` 发不出去、电机不转）：固件已加自动 bus-off 恢复与错误快照，硬件接线仍需逐项确认（C620 供电、60 Ω 终端、CAN1 口、共地）。
2. **`TEST_MODE=1` 在最近提交中**（台架态）：交付前改回 0 并重新验证。
3. Gimbal 任务层为空（WP-08 未开始）；Gimbal IMU 代码已编译但未 `init/update`。
4. `RobotComponents` 协议帧未接入调度（WP-10），双板在线状态尚未建立。
5. `Chassis/inc/Pid.h` 为用户遗留旧版 PID 头文件，与 `pid.hpp` 重复，可删除或迁移。
6. `test_chassis.jdebug` 为临时调试文件，未跟踪，可删。
7. TODO 统计：Chassis 约 36 处、Gimbal 约 43 处（`grep TODO`）。
8. 主机单测（PID/IMU 纯函数）目前只在旧仓库 `tests/host`，待迁移到本仓库后纳入 CI 式验证。

### 后续路线（对应计划 01）

```text
WP-05 PID ✅ → WP-06 IMU ✅ → WP-07 底盘 CAN 电机 ✅
        → WP-08 云台电机/IMU
        → WP-09 DBUS 遥控
        → WP-10 双板协议接入
        → WP-11 全向轮运动学（单车）
        → WP-12 状态机 + 1 ms 调度收口
        → WP-13 跟随/小陀螺
        → WP-14 安全与异常（离线、急停、看门狗）
        → WP-15 整机联调
```

---

## 11. 参考

- 任务书文本：`D:\rm\RM步兵电控计划\00_任务书文本提取.txt`
- 任务分解与一个月计划：`D:\rm\RM步兵电控计划\01_任务分解与一个月计划.md`
- 接口契约 v1.0（含 PID/IMU/CAN/协议/调度约束）：`D:\rm\RM步兵电控计划\02_接口契约_v1.0.md`
- Agent 任务卡：`D:\rm\RM步兵电控计划\03_Agent任务卡.md`
- 验收与调试清单：`D:\rm\RM步兵电控计划\04_验收与调试清单.md`
- 开源参考工程：`D:\rm\大作业\开源-山海mas`（DBUS/电机驱动/麦轮逆解/PID/Mahony 参考实现）
