/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     :双板通讯实例（本板=Chassis，收发 C2G 0x210 / G2C 0x211）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-10/17：双板通讯单例
 *******************************************************************************
 * @attention :
 *  - 帧语义/编解码在 RobotComponents/component/gimbal_chassis_comm.{hpp,cpp}
 *    （Lead 冻结，本文件只负责实例化，不改协议）。
 *  - CAN ID 唯一来源 config::kCanIdC2G / kCanIdG2C（契约 §4.1）。
 *  - 契约 §6.3：函数内 static 构造，避免全局构造函数早于 HAL/tick 初始化
 *    （GimbalChassisComm 构造函数会读 tick::GetTickMs()）。
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
  static robot::GimbalChassisComm comm(robot::GimbalChassisComm::CodePart::Chassis,
                                       config::kCanIdC2G,  /* 本板发送 C2G */
                                       config::kCanIdG2C); /* 本板接收 G2C */
  return &comm;
}

/* Private function definitions ----------------------------------------------*/
