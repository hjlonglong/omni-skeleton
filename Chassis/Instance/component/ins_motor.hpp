/**
 *******************************************************************************
 * @file      :ins_motor.hpp
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
#ifndef CHASSIS_INSTANCE_INS_MOTOR_HPP_
#define CHASSIS_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机单例（M3508，共 4 个，idx ∈ [0,4)）。 */
hello_world::motor::Motor *GetMotorWheel(int idx);

#endif /* CHASSIS_INSTANCE_INS_MOTOR_HPP_ */
