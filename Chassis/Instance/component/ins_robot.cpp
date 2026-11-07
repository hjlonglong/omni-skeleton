/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :顶层状态机 robot::Robot 与底盘模块 robot::Chassis 的实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：函数内 static 单例
 *******************************************************************************
 * @attention :
 *  - 契约 §6.3：不做全局对象（全局构造函数可能早于 HAL/时钟初始化），统一用
 *    函数内 static 局部对象，在第一次 GetRobot()/GetChassis() 时构造。
 *  - 真正的初始化（绑定电机/PID/逆解器、复位状态）在 Robot::init() 里，由
 *    MainTaskInit 在 TIM6 中断启动前调用。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"

/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot()
{
  static robot::Robot instance;
  return &instance;
}

robot::Chassis *GetChassis()
{
  static robot::Chassis instance;
  return &instance;
}

/* Private function definitions ----------------------------------------------*/
