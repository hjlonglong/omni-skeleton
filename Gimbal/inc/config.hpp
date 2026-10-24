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

/* ============================ 云台板专用 ============================ */
/* ---- CAN 总线分配（契约 §4.2：CAN1=电机，CAN2=双板帧）---- */
inline constexpr uint32_t kCanIdMotorTxBase = 0x200u;  ///< 达妙 MIT 控制帧基址（见 contract §4.1）

/* ---- 云台电机（DM-J4310；yaw 为下，pitch 为上）---- */
inline constexpr uint8_t kYawMotorId = 1u;    ///< TODO(硬件): 达妙 CAN ID 核对
inline constexpr uint8_t kPitchMotorId = 2u;  ///< TODO(硬件): 达妙 CAN ID 核对
inline constexpr int8_t kYawMotorDir = 1;     ///< TODO(硬件): 正方向标定
inline constexpr int8_t kPitchMotorDir = 1;   ///< TODO(硬件): 正方向标定
inline constexpr float kYawAngleOffset = 0.0f;    ///< rad，机械零位对齐（TODO(硬件)）
inline constexpr float kPitchAngleOffset = 0.0f;  ///< rad，机械零位对齐（TODO(硬件)）

/* ---- 云台机械限位/需求约束 ---- */
inline constexpr float kPitchMin = -0.35f;   ///< rad，TODO(硬件)：低头限位（保守占位，实车标定）
inline constexpr float kPitchMax = 0.55f;    ///< rad，TODO(硬件)：抬头限位（保守占位，实车标定）
inline constexpr float kYawRateMax = 2.5f;   ///< rad/s，yaw 关节速度限幅
inline constexpr float kPitchRateMax = 1.5f; ///< rad/s，pitch 关节速度限幅
inline constexpr float kYawTorqueMax = 3.0f;   ///< N·m，yaw 力矩限幅（安全）
inline constexpr float kPitchTorqueMax = 2.5f; ///< N·m，pitch 力矩限幅（安全）

/* ---- 云台位置环 PID 初值（输入角度误差 rad，输出目标关节速度 rad/s）---- */
inline constexpr float kPidYawAngleKp = 8.0f;    ///< TODO(调参)
inline constexpr float kPidYawAngleKi = 0.0f;    ///< TODO(调参)
inline constexpr float kPidYawAngleKd = 0.0f;    ///< TODO(调参)
inline constexpr float kPidPitchAngleKp = 8.0f;  ///< TODO(调参)
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
