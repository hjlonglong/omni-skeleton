/**
 *******************************************************************************
 * @file      :config.hpp
 * @brief     :云台板硬件/控制参数唯一来源（WP-03 建立，契约 §5.2）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 初版：两板通用参数 + 云台板参数
 *******************************************************************************
 * @attention :
 *  - 所有 magic number 必须放这里；改参数只改本文件（禁止在业务代码里散落常量）。
 *  - 标 TODO(硬件) 的项为默认假设，须按 docs/hardware_table.md 实测后更新；
 *    标 TODO(调参) 的项在实车调试阶段整定。
 *  - 改动本文件任何参数：同步 docs/hardware_table.md 并通知全组（契约 §0.3）。
 *  - 单位约定：长度 m、角度 rad、速度 m/s、角速度 rad/s、电流 A、力矩 N·m、时间 ms。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONFIG_HPP_
#define CONFIG_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace config
{
/* ============================ 两板通用（与 Chassis/inc/config.hpp 必须完全一致）============================ */
inline constexpr float kTs = 0.001f;                 ///< s，控制周期（TIM6 1 ms）
inline constexpr uint32_t kCommOfflineThresMs = 100; ///< ms，双板帧离线阈值（契约 §3.2）
inline constexpr uint32_t kRemoteOfflineThresMs = 100; ///< ms，遥控器离线阈值
inline constexpr uint32_t kMotorOfflineThresMs = 100;  ///< ms，电机反馈离线阈值
inline constexpr float kStickDeadZone = 0.05f;         ///< 归一化摇杆死区
inline constexpr uint32_t kStateDebounceMs = 100;      ///< ms，状态迁移去抖
inline constexpr uint32_t kResurrectionHoldMs = 200;   ///< ms，Dead→Working 保持时间

// 双板帧 CAN ID（契约 §4.1：两板必须完全一致，改 ID = 改契约）
inline constexpr uint32_t kCanIdC2G = 0x210u;  ///< 底盘 → 云台
inline constexpr uint32_t kCanIdG2C = 0x211u;  ///< 云台 → 底盘
// 双板通讯总线（2026-11-08 实测：两板 CAN1 物理相连；原契约 §4.2 的 CAN2 假设作废）
inline constexpr uint8_t kCanBusInterboard = 1u;  ///< 1=CAN1 2=CAN2（两板必须一致）
/* 双板帧发送周期（ms）。两板 CAN1 连成一条总线，已挂 4×M3508 反馈(1 kHz)+0x200
 * (1 kHz)+云台 yaw 帧，约 7 帧/ms（~80% 负载）。若双板帧也按 1 ms 发，总数 9 帧/ms
 * 会超过 1 Mbps 容量；故降为 10 ms（100 Hz，状态机/跟随延迟可接受）。
 * TODO(硬件)：若把 yaw 电机改接到云台板 CAN2（与 pitch 同总线），可改回 1。
 * 两板必须一致（契约 §5.2 通用段）。 */
inline constexpr uint32_t kInterboardPeriodMs = 1;  ///< 双板帧周期：CAN1 仅双板帧+yaw 时 1 ms（2026-11-09 拓扑修正后负载低）；若同总线还有电机等节点可调 10

/* ============================ 云台板专用 ============================ */
/* ---- CAN 总线分配（2026-11-08 实物通讯拓扑核对）----
 *  - CAN1：yaw DM 电机 + 双板通讯帧（两板 CAN1 相连）；
 *  - CAN2：pitch DM 电机（与发射机构同总线）。 */
inline constexpr uint8_t kYawMotorCanBus = 1u;    ///< yaw 电机所在 CAN
inline constexpr uint8_t kPitchMotorCanBus = 2u;  ///< pitch 电机所在 CAN
inline constexpr uint32_t kCanIdMotorTxBase = 0x200u;  ///< 达妙 MIT 控制帧基址（见 contract §4.1）

/* ---- 云台电机（DM-J4310；yaw 为下，pitch 为上）----
 * 2026-11-08 通讯拓扑核对：yaw 电机 ID=2、pitch 电机 ID=1（以实物接线为准，
 * 勿再对调；反馈 ID = 本 ID + 0x10，即 yaw 0x12 / pitch 0x11）。 */
inline constexpr uint8_t kYawMotorId = 2u;
inline constexpr uint8_t kPitchMotorId = 1u;
inline constexpr int8_t kYawMotorDir = 1;     ///< TODO(硬件): 正方向标定
inline constexpr int8_t kPitchMotorDir = 1;   ///< TODO(硬件): 正方向标定
/* yaw 机械零位（2026-11-11 实测校准点）：**云台正对车头（正方向）时，
 * Ozone 读 `g_dbg_dm_angle[0] = 0.3590`**。DaMiao 的 angle_ = normAngle(实际角 - offset)，
 * 要让“正对车头”对应关节角 0，须把 offset 再增大 0.359：
 *   offset = -1.6930 + 0.3590 ≈ -1.3340 rad。
 * 上电（电机零点、云台仍朝左约 76.4°）读数约为 +1.334；正对车头时读数应为 0.000。
 * 若正对车头时读数仍为 R（非 0），按 `kYawAngleOffset += R` 再校一次。 */
inline constexpr float kYawAngleOffset = -1.3340f;  ///< rad，yaw 机械零位（云台正对车头时 angle()=0）
inline constexpr float kPitchAngleOffset = 0.0f;  ///< rad，机械零位对齐（TODO(硬件)）

/* ---- 云台机械限位/需求约束 ---- */
inline constexpr float kPitchMin = -0.35f;   ///< rad，TODO(硬件)：低头限位（保守占位，实车标定）
inline constexpr float kPitchMax = 0.55f;    ///< rad，TODO(硬件)：抬头限位（保守占位，实车标定）
/* yaw 关节速度限幅（位置环输出上限）。2026-11-11 由 2.5 提高到 8 rad/s：
 * 小陀螺模式底盘自转可达 kMaxW/kSpinWMax=6 rad/s，云台要世界稳定就必须以
 * 至少同等角速度反向补偿；原 2.5 rad/s 只能跟到 2.5 → 云台被底盘“拖着转”。
 * 手动遥控的摇杆速率上限仍由 kStickYawGain=2.5 决定，不受影响。 */
inline constexpr float kYawRateMax = 8.0f;   ///< rad/s，yaw 关节速度限幅（小陀螺稳定需 ≥ 底盘自转）
/* 小陀螺模式专用 yaw 关节速度限幅（2026-11-16 用户反馈：小陀螺 4.5 rad/s 时“云台逆向转跟不上”）。
 * 小陀螺稳定需求 = 底盘自转 4.5 + 摇杆世界目标速率 kGimbalYawRateMax(5.0) ≈ 9.5 rad/s，
 * 超过常规 8.0 → 逆向补偿被截断。仅在 C2G 报告 kSpin 模式时切换到此限幅，其它模式仍用 8.0。
 * 若仍跟不上，优先查速度环力矩是否饱和（gimbal_debug::yaw_torque 贴 ±kYawTorqueMax），
 * 再考虑降低底盘 kSpinWMax 或摇杆 kGimbalYawRateMax。 */
inline constexpr float kYawRateMaxSpin = 12.0f;  ///< rad/s，小陀螺模式 yaw 关节速度限幅（2026-11-16 新增）
inline constexpr float kPitchRateMax = 4.5f; ///< rad/s，pitch 关节速度限幅（2026-11-15 1.5→3.0；2026-11-17 3.0→4.5：用户要求提高 pitch 速度上限，位置环输出余量更大）
/* yaw 力矩限幅。2026-11-15 由 3.0 提高到 4.5：小陀螺（底盘 3 rad/s）时云台仍需
 * 反向补偿，3.0 N·m 在跟随/自转叠加时不够 → 云台被底盘“拖着转”。DM-J4310-2EC
 * 峰值力矩允许该值；若电机温度明显升高再回退。 */
inline constexpr float kYawTorqueMax = 4.5f;   ///< N·m，yaw 力矩限幅（2026-11-15 3.0→4.5）
inline constexpr float kPitchTorqueMax = 2.5f; ///< N·m，pitch 力矩限幅（安全）
inline constexpr float kPitchLimitMargin = 0.10f;  ///< rad，pitch 反馈越限裕度（状态机故障判定）
/* ---- pitch 重力前馈（2026-11-17 用户要求“加入重力前馈”）----
 * pitch 轴重心相对电机轴有机械偏置，重力力矩随俯仰角变化：
 *   τ_g(θ) ≈ A·cos(θ)，θ = pitch 关节角（0 = 水平，正 = 抬头），A = 等效 m·g·L。
 * 水平位置力矩最大、±90° 为 0；纯位置环需靠静差积累出力矩平衡 → 稳定时“偏一点才停”。
 * 直接按模型前馈到 MIT 力矩（叠加在速度环输出之后、力矩限幅之前），可减小静差、
 * 抑制缓慢低头/抬头，降低对位置环增益的依赖。
 * A 需实车标定：水平位置静止时，位置环静差力矩 ≈ A（可读 gimbal_debug::pitch_torque 估计）；
 * 若实车观察到前馈方向相反（偏得更远），把 A 取负值即可。 */
inline constexpr float kPitchGravityTorqueAmp = 0.40f;  ///< N·m，pitch 重力力矩幅值（实车标定，符号按机构定）

/* ---- 云台位置环 PID 初值（输入角度误差 rad，输出目标关节速度 rad/s）---- */
inline constexpr float kPidYawAngleKp = 13.0f;   ///< 2026-11-15 8→12→10；2026-11-17 用户实车调参 10→13
inline constexpr float kPidYawAngleKi = 0.0f;    ///< TODO(调参)
inline constexpr float kPidYawAngleKd = 0.0f;    ///< TODO(调参)
inline constexpr float kPidPitchAngleKp = 20.0f;  ///< 2026-11-17 用户实车调参 8→20（提高 pitch 刚度）
inline constexpr float kPidPitchAngleKi = 0.0f;  ///< TODO(调参)
inline constexpr float kPidPitchAngleKd = 0.0f;  ///< TODO(调参)

/* ---- 云台速度环 PID 初值（输入速度误差 rad/s，输出 MIT 力矩前馈/阻尼）---- */
inline constexpr float kPidYawVelKp = 0.8f;      ///< TODO(调参)
inline constexpr float kPidYawVelKi = 0.0f;      ///< TODO(调参)
inline constexpr float kPidPitchVelKp = 0.8f;    ///< TODO(调参)
inline constexpr float kPidPitchVelKi = 0.0f;    ///< TODO(调参)

/* ---- 遥控映射/IMU ---- */
inline constexpr float kStickYawGain = 2.5f;     ///< rad/s per 归一化摇杆量（手动模式）
inline constexpr float kStickPitchGain = 1.5f;   ///< rad/s per 归一化摇杆量（手动模式）
inline constexpr uint32_t kZeroDriftSamples = 1000;  ///< 上电静止零漂标定样本数（1 kHz 下 1 s）

/* ---- 调试开关（交付前必须为 0，见工具 README §4/§9.3）---- */
#ifndef DEBUG_NO_IWDG
#define DEBUG_NO_IWDG 0  ///< 1 = 调试期临时关闭喂狗/复位（仅限台架 + Ozone 调试）
#endif
#ifndef TEST_MODE
#define TEST_MODE 0  ///< 1 = 开环/单项调试模式（交付前必须为 0）
#endif

}  // namespace config

#endif /* CONFIG_HPP_ */
