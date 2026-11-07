/**
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
 * @brief     : 云台板双板通讯对象实例化（GimbalChassisComm）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-16：G2C/C2G 双板帧单例
 *******************************************************************************
 * @attention :
 *  - 云台板：CodePart::Gimbal，发送 G2C(config::kCanIdG2C = 0x211)，
 *    接收 C2G(config::kCanIdC2G = 0x210)（契约 §3.2 / §4.1）。
 *  - 函数内 static：CommTaskInit 中首次调用完成构造，保证 CAN 接收中断打开前
 *    last_rx_tick_ 已初始化为当前 tick（首帧到达前按离线处理）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"

#include "config.hpp"
#include "gimbal_chassis_comm.hpp"

/* Exported function definitions ---------------------------------------------*/

robot::GimbalChassisComm *GetGimbalChassisComm()
{
  static robot::GimbalChassisComm comm(
      robot::GimbalChassisComm::CodePart::Gimbal, config::kCanIdG2C,
      config::kCanIdC2G);
  return &comm;
}
