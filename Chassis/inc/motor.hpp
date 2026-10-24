/**
 *******************************************************************************
 * @file      : motor.hpp
 * @brief     : 电机类汇总
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-14      Caikunzhen      1. 添加电机简单工厂函数
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *  V2.0.0      2026-09-18      Hello World     1. 改装版：仅保留考核所需电机
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_DaMiao.hpp"
#include "motor_M2006.hpp"
#include "motor_M3508.hpp"
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world {
namespace motor {
/* Exported macro ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

enum class MotorType {
  kM2006,
  kM3508,
  kDM_J4310,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       根据所需的电机类型创建电机并初始化
 * @param        id: 电机 ID，取值范围由电机类型决定
 * @param        opt: 电机可选配置参数
 * @retval       创建的电机类指针
 * @note        返回的电机类是经过内存申请得到的，不使用时需要自行释放内存
 */
Motor *CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams &optinal_params = OptionalParams());

HW_OPTIMIZE_O2_END
} // namespace motor
} // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_ */
