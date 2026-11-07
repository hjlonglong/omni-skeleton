/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 云台板顶层状态机与云台运动模块实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-13/16：Robot/Gimbal 单例
 *******************************************************************************
 * @attention :
 *  - 函数内 static：首次调用（Robot::init 内）构造，构造顺序安全；
 *    运行期无动态内存（契约 §6.3 / §10）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "gimbal.hpp"
#include "robot.hpp"

/* Exported function definitions ---------------------------------------------*/

robot::Robot *GetRobot()
{
  static robot::Robot robot;
  return &robot;
}

robot::Gimbal *GetGimbal()
{
  static robot::Gimbal gimbal;
  return &gimbal;
}
