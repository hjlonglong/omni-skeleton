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

#include "config.hpp"

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
extern volatile float chassis_yaw_rate;  ///< 底盘 yaw 角速度（rad/s，LPF 后，2026-11-15 前馈用）
extern volatile float yaw_vel_ff;        ///< yaw 关节速度前馈（rad/s，=−底盘 yaw 速率）
extern volatile float world_yaw_est;     ///< 世界 yaw 估计（rad）= chassis_yaw + yaw_joint，用于验证世界稳定
extern volatile float yaw_err;           ///< yaw 位置误差（rad，WrapPi 后）
extern volatile float yaw_vel_target;    ///< yaw 速度环目标（rad/s，位置环输出+前馈，已限幅）
extern volatile float pitch_err;         ///< pitch 位置误差（rad）
extern volatile float pitch_vel_target;  ///< pitch 速度环目标（rad/s，位置环输出+目标速率前馈，已限幅）
extern volatile float pitch_vel_ff;      ///< pitch 目标速率前馈（rad/s，2026-11-15）
extern volatile float yaw_hold_torque;   ///< 静止保持叠加的 yaw 力矩（N·m，2026-11-17）
extern volatile float pitch_hold_torque; ///< 静止保持叠加的 pitch 力矩（N·m，2026-11-17）
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

  /**
   * @brief       设置 yaw 关节速度限幅（rad/s）
   * @param       rate_max: 限幅值；小陀螺模式用 config::kYawRateMaxSpin，其它模式用 kYawRateMax
   * @note        仅影响位置环输出/前馈 LPF 的 clamp，不改变正常遥控手感
   */
  void setYawRateMax(float rate_max) { yaw_rate_max_ = rate_max; }

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
  float yaw_rate_max_ = config::kYawRateMax;  ///< yaw 关节速度限幅（模式相关，见 setYawRateMax）
  float chassis_yaw_ = 0.0f;       ///< 底盘 IMU yaw（rad）
  /* 底盘 yaw 速率前馈（2026-11-15）：小陀螺时底盘自转 3 rad/s，纯位置环跟踪
   * 斜坡会有稳态滞后（误差 = 角速度/Kp），前馈直接把所需关节反向速度加到
   * 速度环目标上，消除“云台被底盘拖着转”。差分来自 C2G 的 chassis_imu_yaw。 */
  float prev_chassis_yaw_ = 0.0f;    ///< 上一拍底盘 yaw（差分用）
  bool chassis_yaw_inited_ = false;  ///< 上一拍是否有效（进 Working 首拍不计速率）
  float chassis_yaw_rate_ = 0.0f;    ///< 底盘 yaw 速率 LPF 值（rad/s）
  float pitch_target_ = 0.0f;      ///< 期望 pitch（rad，未 clamp）
  /* pitch 目标角速率前馈（2026-11-15）：pitch 目标 ramp 时位置环有“速率/Kp”
   * 的稳态滞后（1.5 rad/s ÷ Kp 8 ≈ 0.19 rad ≈ 11°），差分目标角得到前馈后消除。 */
  float prev_pitch_target_ = 0.0f;    ///< 上一拍 pitch 目标（差分用）
  bool pitch_target_inited_ = false;  ///< 首拍不计速率
  float pitch_vel_ff_ = 0.0f;         ///< pitch 目标速率前馈 LPF 值（rad/s）
  float yaw_vel_integ_ = 0.0f;     ///< yaw 速度环积分状态
  float pitch_vel_integ_ = 0.0f;   ///< pitch 速度环积分状态
  float yaw_torque_ = 0.0f;        ///< 本周期 yaw 力矩（N·m）
  float pitch_torque_ = 0.0f;      ///< 本周期 pitch 力矩（N·m）
  /* 静止保持（2026-11-17）：两轴独立判定“已稳定”后叠加虚拟弹簧刚度，
   * 提高手推/外力下的静止刚度。timer=进入条件累计时间(s)，blend=渐入渐出系数(0~1)。 */
  float yaw_hold_timer_ = 0.0f;    ///< yaw 进入保持条件累计时间（s）
  float pitch_hold_timer_ = 0.0f;  ///< pitch 进入保持条件累计时间（s）
  float yaw_hold_blend_ = 0.0f;    ///< yaw 保持力矩渐变系数（0~1）
  float pitch_hold_blend_ = 0.0f;  ///< pitch 保持力矩渐变系数（0~1）
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
