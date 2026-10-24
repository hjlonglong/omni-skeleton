/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"

#include "config.hpp"
#include "motor_M3508.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {
/* 轮电机数量（索引顺序 LF、LB、RB、RF，见 config.hpp） */
constexpr int kWheelNum = 4;
}  // namespace

/* Private types -------------------------------------------------------------*/
namespace {
/**
 * 4 个轮电机实例（M3508）。
 *
 * 按契约 §6.3：不定义全局对象（避免全局构造函数在 HAL / 时钟初始化之前运行），
 * 改为函数内 static 局部对象，在首次调用 GetMotorWheel() 时构造并初始化。
 */
struct WheelMotorInstances {
  WheelMotorInstances() {
    for (int i = 0; i < kWheelNum; ++i) {
      hello_world::motor::OptionalParams opt;
      opt.dir = static_cast<hello_world::motor::Dir>(config::kWheelDir[i]);
      opt.input_type = hello_world::motor::InputType::kCurr;
      opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
      opt.max_curr_input_lim = config::kMaxWheelCurrent;
      motors[i].init(config::kWheelCanId[i], opt);
    }
  }

  hello_world::motor::M3508 motors[kWheelNum];
};
}  // namespace

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx) {
  /* 越界安全：非法下标返回 nullptr，调用方须判空 */
  if (idx < 0 || idx >= kWheelNum) {
    return nullptr;
  }

  /* 首次调用时才构造 4 个电机实例（函数内 static，见 WheelMotorInstances 注释） */
  static WheelMotorInstances wheel_motors;
  return &wheel_motors.motors[idx];
}

/* Private function definitions ----------------------------------------------*/
