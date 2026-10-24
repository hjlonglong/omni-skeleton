/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.hpp
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
#ifndef GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_
#define GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace robot {
class GimbalChassisComm;
}  // namespace robot
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板双板通讯对象（robot::GimbalChassisComm）单例。 */
robot::GimbalChassisComm *GetGimbalChassisComm();

#endif /* GIMBAL_INSTANCE_INS_CHASSIS_GIMBAL_COMM_HPP_ */
