/**
 *******************************************************************************
 * @file      : gimbal.hpp
 * @brief     : 云台两轴（yaw 连续旋转 / pitch 软限位）关节空间位置控制
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-13：位置环（角度误差→目标关节角速度）
 *                                                + 速度环（速度误差→MIT 力矩），DM MIT 下发
 *******************************************************************************
 * @attention :
 *  - 控制目标是【关节角】：yaw 由 Robot 传入期望世界 yaw 与底盘 IMU yaw，本类换算成
 *    关节目标（wrap 到 [-π,π)）；pitch 目标做软限位 clamp（config kPitchMin/Max）。
 *  - yaw 电机配置 angle_range = kNegInfToPosInf，反馈是多圈连续角；误差统一 wrap，
 *    保证连续旋转不跳变。pitch 电机 angle_range = kNegPiToPosPi（反馈 [-π,π)）。
 *  - 输出：DM MIT 模式纯力矩 setMitInput(0,0,0,0,torq)，力矩/速度上限取 config。
 *  - Dead/Resurrection 由 Robot 调 stop() 归零（同时 reset PID）；DM 仍需每周期发帧
 *    以保持反馈（一帧一反馈）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace hello_world
{
namespace motor
{
class Motor;
}  // namespace motor
namespace pid
{
class Pid;
}  // namespace pid
namespace imu
{
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot
{

/* 云台调试观测变量（Ozone Live Watch 用；无硬件自测时观察目标/反馈/力矩） */
namespace gimbal_debug
{
extern volatile float yaw_world_target;  ///< 期望世界 yaw（rad）
extern volatile float yaw_joint_target;  ///< 换算后的 yaw 关节目标（rad，[-π,π)）
extern volatile float yaw_joint;         ///< yaw 关节反馈（rad，多圈连续角）
extern volatile float yaw_torque;        ///< 本周期下发的 yaw MIT 力矩（N·m）
extern volatile float pitch_target;      ///< clamp 后的 pitch 目标（rad）
extern volatile float pitch_joint;       ///< pitch 关节反馈（rad）
extern volatile float pitch_torque;      ///< 本周期下发的 pitch MIT 力矩（N·m）
extern volatile uint32_t cycle_count;    ///< update() 调用计数
}  // namespace gimbal_debug

/**
 * @brief 云台两轴位置控制（契约 §6.3 / §7.3，WP-13）
 */
class Gimbal
{
 public:
  Gimbal() = default;

  /**
   * @brief       装配云台控制对象
   * @param       yaw / pitch: 两轴 DM-J4310（Instance 层静态实例）
   * @param       yaw_pid / pitch_pid: 位置环 PID（参数见 config，输出关节速度）
   * @param       imu: 云台板 IMU（保留接口；控制用 DM 角度反馈，IMU 就绪由 Robot 判）
   * @retval      None
   * @note        必须在 CAN 接收中断打开前调用（Ins 层保证静态对象先构造）。
   */
  void init(hello_world::motor::Motor *yaw, hello_world::motor::Motor *pitch,
            hello_world::pid::Pid *yaw_pid, hello_world::pid::Pid *pitch_pid,
            hello_world::imu::Imu *imu);

  /**
   * @brief       设置期望【世界 yaw】目标（C2G 帧 b2-3，契约 v1.1 语义）
   * @param       yaw_world: 期望世界 yaw（rad，可为连续角，内部按 2π 等价处理）
   */
  void setYawTarget(float yaw_world);

  /**
   * @brief       设置底盘 IMU yaw（C2G 帧 b6-7），用于世界 yaw→关节目标换算
   * @param       chassis_imu_yaw: 底盘 IMU 世界 yaw（rad，wrap 后）
   */
  void setChassisYaw(float chassis_imu_yaw);

  /**
   * @brief       设置期望 pitch 目标（C2G 帧 b4-5，绝对 pitch，内部软限位 clamp）
   * @param       pitch_joint: 期望 pitch（rad）
   */
  void setPitchTarget(float pitch_joint);

  /** @brief 1ms 位置控制：读关节反馈 → 位置环 → 速度环 → MIT 力矩下发 */
  void update();

  /** @brief 输出全零力矩并复位 PID（Dead/Resurrection 每周期调用，DM 仍需发帧） */
  void stop();

  /** @brief 最近一次 yaw 关节反馈（rad，多圈连续角） */
  float yawJoint() const;

  /** @brief 最近一次 pitch 关节反馈（rad） */
  float pitchJoint() const;

  /** @brief 本周期下发的 yaw 力矩（N·m，调试用） */
  float yawTorque() const { return yaw_torque_; }

  /** @brief 本周期下发的 pitch 力矩（N·m，调试用） */
  float pitchTorque() const { return pitch_torque_; }

 private:
  /** @brief 两轴对象是否已装配 */
  bool IsInited() const;

  hello_world::motor::Motor *yaw_ = nullptr;
  hello_world::motor::Motor *pitch_ = nullptr;
  hello_world::pid::Pid *yaw_pid_ = nullptr;
  hello_world::pid::Pid *pitch_pid_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;  ///< 保留接口，云台控制不使用

  float yaw_world_target_ = 0.0f;  ///< 期望世界 yaw（rad）
  float chassis_yaw_ = 0.0f;       ///< 底盘 IMU yaw（rad）
  float pitch_target_ = 0.0f;      ///< 期望 pitch（rad，未 clamp）
  float yaw_vel_integ_ = 0.0f;     ///< yaw 速度环积分状态
  float pitch_vel_integ_ = 0.0f;   ///< pitch 速度环积分状态
  float yaw_torque_ = 0.0f;        ///< 本周期 yaw 力矩（N·m）
  float pitch_torque_ = 0.0f;      ///< 本周期 pitch 力矩（N·m）
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
