/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() {
  /* TODO【新人任务】:
   * 返回本板顶层状态机 robot::Robot 单例。
   *
   * 你要做：
   *   1. 在本文件静态定义一个 robot::Robot 对象（静态存储期，程序启动即构造）。
   *   2. 返回它的地址。
   *
   * 约束 / 提示：
   *   - robot::Robot 类的定义由你在 RobotModules/component/robot.hpp 补全（见该文件 TODO），
   *     本文件只负责实例化并返回地址。
   */
  return nullptr;
}

robot::Chassis *GetChassis() {
  /* TODO【新人任务】:
   * 返回 Chassis 模块 robot::Chassis 单例，做法同 GetRobot()。
   * robot::Chassis 类定义由你在 RobotModules/component/chassis.hpp 补全。
   */
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
