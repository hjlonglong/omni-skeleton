/**
 *******************************************************************************
 * @file      : gimbal_math.hpp
 * @brief     : 云台纯函数角度换算与软限位（无 HAL 依赖，可编译期自测）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-13：yaw 世界目标→关节目标换算、
 *                                                pitch 软限位，含 static_assert 自测
 *******************************************************************************
 * @attention :
 *  - 本文件只做浮点角度运算，不包含任何 HAL/STM32 头，属于云台板「纯函数」层，
 *    便于无硬件自测（编译期 static_assert 验证边界换算，0 error 构建即通过）。
 *  - yaw 关节目标 = wrap(期望世界 yaw − 底盘 IMU yaw)。C2G 帧 b2-3/b4-5 按契约
 *    v1.1 扩展后表示【期望世界 yaw】/【期望绝对 pitch】，不再是保留字段。
 *  - 角度 wrap 语义与 RobotComponents/component/module_state.hpp 的 wrapPi 一致
 *    （[-π, π)）；此处提供 constexpr 版本以便编译期自测，运行时结果相同。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOTMODULES_COMPONENT_GIMBAL_MATH_HPP_
#define ROBOTMODULES_COMPONENT_GIMBAL_MATH_HPP_

/* Includes ------------------------------------------------------------------*/
#include "config.hpp"

namespace robot
{
namespace gimbal_math
{

inline constexpr float kPi = 3.14159265358979323846f;  ///< π
inline constexpr float kTwoPi = 2.0f * kPi;            ///< 2π

/**
 * @brief       角度 wrap 到 [-π, π)
 * @param       angle: 输入角度（rad）
 * @retval      wrap 后的角度（rad）
 * @note        与 module_state::wrapPi 语义一致；constexpr 版本用于编译期自测。
 */
inline constexpr float WrapPi(float angle)
{
  while (angle >= kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

/**
 * @brief       期望世界 yaw + 底盘 IMU yaw → 云台 yaw 关节目标角
 * @param       yaw_world: C2G 帧 b2-3，期望世界 yaw（rad）
 * @param       chassis_imu_yaw: C2G 帧 b6-7，底盘 IMU yaw（rad）
 * @retval      关节目标角（rad，[-π, π)）
 * @note        底盘板由 Agent B 把「遥控/模式」换算成世界 yaw 目标，本板只消费。
 */
inline constexpr float YawJointTarget(float yaw_world, float chassis_imu_yaw)
{
  return WrapPi(yaw_world - chassis_imu_yaw);
}

/**
 * @brief       数值限幅
 * @param       value: 输入值
 * @param       min_value: 下限
 * @param       max_value: 上限
 * @retval      限幅后的值
 */
inline constexpr float ClampValue(float value, float min_value, float max_value)
{
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}

/**
 * @brief       pitch 目标软限位（clamp 到 [kPitchMin, kPitchMax]）
 * @param       pitch: 期望 pitch（rad）
 * @retval      限幅后的 pitch 目标（rad）
 */
inline constexpr float ClampPitchTarget(float pitch)
{
  return ClampValue(pitch, config::kPitchMin, config::kPitchMax);
}

/**
 * @brief       pitch 反馈是否越限（含 kPitchLimitMargin 裕度，供状态机判故障）
 * @param       pitch: pitch 关节反馈（rad）
 * @retval      true = 越限（状态机应进 Dead 并归零输出）
 */
inline constexpr bool IsPitchOutOfLimit(float pitch)
{
  return pitch < config::kPitchMin - config::kPitchLimitMargin ||
         pitch > config::kPitchMax + config::kPitchLimitMargin;
}

/* ------------------------------ 编译期自测 ------------------------------ */
/* 以下 static_assert 在编译期验证「yaw 世界目标→关节目标」「pitch 软限位」的
 * 边界数值：0 error 构建即证明纯函数换算正确（无硬件自测，见 04 验收清单 §3.2）。 */
inline constexpr bool NearEqual(float a, float b, float eps = 1e-5f)
{
  return (a - b) < eps && (b - a) < eps;
}

static_assert(NearEqual(WrapPi(0.0f), 0.0f), "wrapPi(0) 必须为 0");
static_assert(NearEqual(WrapPi(kPi), -kPi), "wrapPi(π) 必须为 -π（区间左闭）");
static_assert(NearEqual(WrapPi(3.5f), 3.5f - kTwoPi), "wrapPi(3.5) 必须减 2π");
static_assert(NearEqual(WrapPi(-3.5f), -3.5f + kTwoPi), "wrapPi(-3.5) 必须加 2π");
static_assert(WrapPi(20.0f) > -kPi && WrapPi(20.0f) < kPi,
              "wrapPi(20) 必须落在 [-π,π)（多圈输入）");

static_assert(NearEqual(YawJointTarget(0.5f, 0.2f), 0.3f),
              "世界 yaw 0.5、底盘 yaw 0.2 → 关节目标 0.3");
static_assert(NearEqual(YawJointTarget(-3.0f, 3.1f), 0.183185f),
              "跨 ±π 边界：世界 -3.0、底盘 3.1 → 关节目标 ≈ 0.183");
static_assert(NearEqual(YawJointTarget(3.0f, 0.0f), 3.0f),
              "期望世界 yaw 在限幅内时关节目标等于其本身");

static_assert(NearEqual(ClampPitchTarget(10.0f), config::kPitchMax),
              "pitch 目标上限必须被软限位 clamp");
static_assert(NearEqual(ClampPitchTarget(-10.0f), config::kPitchMin),
              "pitch 目标下限必须被软限位 clamp");
static_assert(NearEqual(ClampPitchTarget(0.1f), 0.1f), "pitch 目标在限位内不应被改写");

static_assert(!IsPitchOutOfLimit(config::kPitchMax), "pitch = 上限判据内不应报故障");
static_assert(!IsPitchOutOfLimit(config::kPitchMin), "pitch = 下限判据内不应报故障");
static_assert(IsPitchOutOfLimit(config::kPitchMax + config::kPitchLimitMargin + 0.01f),
              "pitch 超上限 + 裕度必须报故障");
static_assert(IsPitchOutOfLimit(-10.0f), "pitch 远低于下限必须报故障");

}  // namespace gimbal_math
}  // namespace robot

#endif /* ROBOTMODULES_COMPONENT_GIMBAL_MATH_HPP_ */
