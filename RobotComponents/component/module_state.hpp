/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     :底盘板与云台板共享的状态类型/枚举/内联函数（契约 §2，WP-04 冻结）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 按契约 §2 实现：生命状态/控制源/
 *                                                底盘模式/云台模式/运动状态/wrapPi
 *******************************************************************************
 * @attention :
 *  - 本文件会被底盘板、云台板同时编译，只能放「纯类型 / 枚举 / 内联函数」，
 *    严禁依赖任何具体外设（电机 / CAN / 板级驱动），否则另一块板编不过。
 *  - 字段命名 / 枚举值被 RobotModules 状态机、RobotComponents 双板通讯帧、
 *    Instance 接线三处共同使用，修改必须走契约变更流程（契约 §0.3）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace robot
{
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kTwoPi = 2.0f * kPi;

/* Exported types ------------------------------------------------------------*/

/* ---- 生命状态（三态状态机，契约 §2）---- */
enum class LifeState : uint8_t {
  kDead = 0,          ///< 上电/急停/故障：所有输出零
  kResurrection = 1,  ///< 自检/对齐/等待摇杆回中：所有输出零
  kWorking = 2,       ///< 正常控制
};

/* ---- 手动控制源（本版只有遥控器，预留键鼠）---- */
enum class ControlSource : uint8_t { kNone = 0, kRemote = 1 };

/* ---- 底盘工作模式 ---- */
enum class ChassisMode : uint8_t {
  kSeparate = 0,  ///< 分离：底盘与云台解耦
  kFollow = 1,    ///< 跟随：车头自动对准云台
  kSpin = 2,      ///< 小陀螺：绕中心自旋，平移方向仍随云台
};

/* ---- 云台工作模式：关节空间手动（本版唯一）---- */
enum class GimbalMode : uint8_t { kManual = 0 };

/* ---- 底盘运动状态（平移定义在云台系，契约 §1.2）---- */
struct ChassisVelocity {
  float vx = 0.0f;  ///< m/s，云台系前向
  float vy = 0.0f;  ///< m/s，云台系左向
  float w = 0.0f;   ///< rad/s，逆时针为正

  ChassisVelocity operator+(const ChassisVelocity& o) const {
    return {vx + o.vx, vy + o.vy, w + o.w};
  }
  ChassisVelocity operator-(const ChassisVelocity& o) const {
    return {vx - o.vx, vy - o.vy, w - o.w};
  }
  ChassisVelocity operator*(float k) const { return {vx * k, vy * k, w * k}; }
  ChassisVelocity& operator+=(const ChassisVelocity& o) {
    vx += o.vx;
    vy += o.vy;
    w += o.w;
    return *this;
  }
  void zero() { vx = vy = w = 0.0f; }
};

/* ---- 云台关节目标/反馈角（rad）---- */
struct GimbalAngle {
  float yaw = 0.0f;    ///< rad，连续角（世界系目标）或关节角（反馈）
  float pitch = 0.0f;  ///< rad
};

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       把角度 wrap 到 [-pi, pi)
 * @param[in]   a 任意角度 rad
 * @retval      wrap 后的角度 rad
 * @note        全工程唯一实现；双板帧打包（契约 §3.3）、δ 三角函数前必须调用
 */
inline float wrapPi(float a) {
  while (a >= kPi) a -= kTwoPi;
  while (a < -kPi) a += kTwoPi;
  return a;
}

/**
 * @brief       把角度 wrap 到 [0, 2*pi)
 * @param[in]   a 任意角度 rad
 * @retval      wrap 后的角度 rad
 */
inline float wrapZeroTo2Pi(float a) {
  while (a >= kTwoPi) a -= kTwoPi;
  while (a < 0.0f) a += kTwoPi;
  return a;
}

}  // namespace robot

/* Exported variables --------------------------------------------------------*/

#endif /* MODULE_STATE_HPP_ */
