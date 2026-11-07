/**
 *******************************************************************************
 * @file      : ins_pid.cpp
 * @brief     : 云台两轴位置环 PID 实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-08/13：yaw/pitch 位置环 PID 实例化
 *******************************************************************************
 * @attention :
 *  - 位置环输入为角度误差（rad），输出为目标关节角速度（rad/s）
 *    （见 config.hpp kPid?AngleKp 注释）；速度环在 gimbal.cpp 内完成
 *    （config kPid?VelKp/Ki，输出 MIT 力矩）。
 *  - PID 的 out/integ 限幅都取该轴最大角速度，实现输出限幅 + 抗积分饱和。
 *  - 参数全部来自 config.hpp，不在本文件写死数字，便于实车调参。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "config.hpp"
#include "pid.hpp"

namespace
{

/**
 * @brief       生成云台位置环 PID 配置
 * @param       kp / ki / kd: 角度环增益（config）
 * @param       rate_max: 该轴最大关节角速度（rad/s），用作输出/积分限幅
 * @retval      Pid::Config
 */
hello_world::pid::Pid::Config MakeAnglePidConfig(float kp, float ki, float kd,
                                                 float rate_max)
{
  hello_world::pid::Pid::Config cfg;
  cfg.kp = kp;
  cfg.ki = ki;
  cfg.kd = kd;
  cfg.out_min = -rate_max;
  cfg.out_max = rate_max;
  cfg.integ_min = -rate_max;
  cfg.integ_max = rate_max;
  cfg.dead_zone = 0.0f;
  cfg.d_lpf_alpha = 0.0f;  // Kd 当前为 0，暂不做微分低通
  return cfg;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::pid::Pid *GetPidMotorYaw()
{
  static hello_world::pid::Pid pid(MakeAnglePidConfig(
      config::kPidYawAngleKp, config::kPidYawAngleKi, config::kPidYawAngleKd,
      config::kYawRateMax));
  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch()
{
  static hello_world::pid::Pid pid(MakeAnglePidConfig(
      config::kPidPitchAngleKp, config::kPidPitchAngleKi,
      config::kPidPitchAngleKd, config::kPitchRateMax));
  return &pid;
}
