/**
 *******************************************************************************
 * @file      : motor.cpp
 * @brief     : 电机汇总
 * @history   :
 *  Version     Date            Author          Note
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
/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

Motor* CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams& opt)
{
  switch (motor_type) {
    case MotorType::kM2006:
      return new M2006(id, opt);
    case MotorType::kM3508:
      return new M3508(id, opt);
    case MotorType::kDM_J4310:
      return new DM_J4310(id, opt);
    default:
      HW_ASSERT(0, "Error motor type");
      return nullptr;
  }
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
