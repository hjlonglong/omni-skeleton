/**
 *******************************************************************************
 * @file      :ins_imu.hpp
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
#ifndef CHASSIS_INSTANCE_INS_IMU_HPP_
#define CHASSIS_INSTANCE_INS_IMU_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace hello_world {
namespace imu {
class Imu;
}  // namespace imu
}  // namespace hello_world
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板 IMU（姿态传感器）单例。Imu 类由你在 inc/imu.hpp 补全。 */
hello_world::imu::Imu *GetImu();

#endif /* CHASSIS_INSTANCE_INS_IMU_HPP_ */
