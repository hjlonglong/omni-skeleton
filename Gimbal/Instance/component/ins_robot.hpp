/**
 *******************************************************************************
 * @file      :ins_robot.hpp
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
#ifndef GIMBAL_INSTANCE_INS_ROBOT_HPP_
#define GIMBAL_INSTANCE_INS_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot {
class Robot;
class Gimbal;
}  // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板顶层状态机单例（robot::Robot），对象在 ins_robot.cpp 中静态定义。 */
robot::Robot *GetRobot();
/* 获取云台运动模块单例（robot::Gimbal），负责 yaw / pitch 关节空间控制。 */
robot::Gimbal *GetGimbal();

#endif /* GIMBAL_INSTANCE_INS_ROBOT_HPP_ */
