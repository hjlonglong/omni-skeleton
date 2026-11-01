/*
 * @Author: hjlonglong 356794146@qq.com
 * @Date: 2026-10-24 19:05:02
 * @LastEditors: hjlonglong 356794146@qq.com
 * @LastEditTime: 2026-10-28 20:42:09
 * @FilePath: \Chassis\inc\config.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :config.hpp
 * @brief     :底盘板硬件/控制参数唯一来源（WP-03 建立，契约 §5.2）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 初版：两板通用参数 + 底盘板参数
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
/* ============================ 两板通用（与 Gimbal/inc/config.hpp 必须完全一致）============================ */
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

/* ============================ 底盘板专用 ============================ */
/* ---- CAN 总线分配（契约 §4.2：CAN1=电机，CAN2=双板帧）---- */
inline constexpr uint32_t kCanIdMotorTx = 0x200u;       ///< M3508 控制帧（含电调 1~4）
inline constexpr uint32_t kCanIdMotorFdbBase = 0x201u;  ///< 反馈帧 = base + 电调 ID

/* ---- 轮电机（M3508；索引顺序 LF, LB, RB, RF）---- */
inline constexpr uint8_t kWheelCanId[4] = {1u, 2u, 3u, 4u};  ///< TODO(硬件): 电调 ID 与接线核对
inline constexpr int8_t kWheelDir[4] = {1, 1, 1, 1};          ///< TODO(硬件): 正方向标定（前进全正）
inline constexpr float kMaxWheelCurrent = 10.0f;              ///< A，调试期电流限幅
inline constexpr float kMaxWheelRadS = 800.0f;                ///< rad/s，轮速限幅

/* ---- 麦轮几何（45° 滚子，标准 X 型布置）---- */
inline constexpr float kWheelRadius = 0.076f;        ///< m，RM 标准 76 mm 轮（⚠️ 默认假设）
inline constexpr float kHalfWheelbase = 0.19f;       ///< m，轴距一半（TODO(硬件): 卷尺实测）
inline constexpr float kHalfTrack = 0.19f;           ///< m，轮距一半（TODO(硬件): 卷尺实测）
inline constexpr float kMecanumK = kHalfWheelbase + kHalfTrack;  ///< L+W，逆解旋转项
inline constexpr int8_t kRotationSign = 1;           ///< TODO(硬件): 逆解自转项符号（实车验证）

/* ---- 速度限幅（云台系）---- */
inline constexpr float kMaxVxVy = 1.5f;         ///< m/s，平移速度限幅
inline constexpr float kMaxW = 6.0f;            ///< rad/s，分离模式拨轮自转限幅
inline constexpr float kSpinWMax = 6.0f;        ///< rad/s，小陀螺自转限幅
inline constexpr float kFollowWMax = 3.0f;      ///< rad/s，跟随环输出限幅
inline constexpr float kFollowDeadBandRad = 0.035f;  ///< rad，跟随死区（≈2°）

/* ---- 轮速环 PID 初值（位置式，输入 rad/s 误差，输出电流 A）---- */
inline constexpr float kPidWheelKp = 10.0f;        ///< TODO(调参)
inline constexpr float kPidWheelKi = 0.0f;        ///< TODO(调参)
inline constexpr float kPidWheelKd = 0.0f;        ///< TODO(调参)
inline constexpr float kPidWheelIntegMax = kMaxWheelCurrent;  ///< 积分限幅

/* ---- 跟随环 PID 初值（输入 δ 误差 rad，输出自转 w rad/s）---- */
inline constexpr float kPidFollowKp = 10.0f;       ///< TODO(调参)
inline constexpr float kPidFollowKi = 0.0f;       ///< TODO(调参)
inline constexpr float kPidFollowKd = 0.1f;       ///< TODO(调参)
inline constexpr float kPidFollowIntegMax = kFollowWMax;

/* ---- TEST_MODE 1：单轮台架扫掠（仅车轮离地调试；交付前 TEST_MODE 必须为 0）---- */
inline constexpr int kTestWheelIdx = 0;                  ///< 被测轮索引（0=LF,1=LB,2=RB,3=RF）
inline constexpr float kTestWheelCurrentLimit = 2.0f;    ///< A，台架测试电流限幅（≤2 A，安全）
inline constexpr float kTestWheelSweepMaxRadS = 40.0f;   ///< rad/s，扫掠目标幅值
inline constexpr float kTestWheelSweepRateRadS2 = 80.0f; ///< rad/s²，扫掠斜率（0.5 s 到峰值）
inline constexpr uint32_t kTestWheelStartDelayMs = 1000; ///< ms，上电后等待时间（便于观察）
inline constexpr uint32_t kTestWheelDurationMs = 15000;  ///< ms，扫掠总时长，到时目标归零

/* ---- 调试开关（交付前必须为 0，见工具 README §4/§9.3）---- */
#ifndef DEBUG_NO_IWDG
#define DEBUG_NO_IWDG 0  ///< 1 = 调试期临时关闭喂狗/复位（仅限台架 + Ozone 调试）
#endif
#ifndef TEST_MODE
#define TEST_MODE 1  ///< 1 = 开环/单项调试模式（交付前必须为 0）
#endif

}  // namespace config

#endif /* CONFIG_HPP_ */
