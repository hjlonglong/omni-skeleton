/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
 * @brief     :底盘逆解器实例（O 型全向轮，几何参数来自 config.hpp）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：逆解器单例
 *******************************************************************************
 * @attention :
 *  - 逆解算法在 Chassis/inc/chassis_ik_solver.hpp（纯头文件、无 HAL 依赖）。
 *  - 几何参数（轮半径/四角位置/安装角/方向符号）全部来自 config.hpp，本文件只做
 *    函数内 static 实例化（契约 §6.3：不做全局对象）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"
#include "config.hpp"

/* Exported function definitions ---------------------------------------------*/
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver()
{
  static hello_world::chassis_ik_solver::ChassisIkSolver solver(
      config::kWheelRadius);
  return &solver;
}

/* Private function definitions ----------------------------------------------*/
