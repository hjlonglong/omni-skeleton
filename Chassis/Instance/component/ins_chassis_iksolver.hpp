/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_
#define CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace chassis_ik_solver {
class ChassisIkSolver;
}  // namespace chassis_ik_solver
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取底盘逆解器单例（底盘速度 → 各轮速 / 舵角的运动学解算）。 */
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver();

#endif /* CHASSIS_INSTANCE_INS_CHASSIS_IKSOLVER_HPP_ */
