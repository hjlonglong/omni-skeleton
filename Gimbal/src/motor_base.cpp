/**
 *******************************************************************************
 * @file      : motor_base.cpp
 * @brief     : 电机基类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-30      Caikunzhen      1. 修复电流输入转力矩问题
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *  V2.0.0      2026-09-18      Hello World     1. 改装版：去通信耦合，暴露纯字节接口
 *******************************************************************************
 * @attention :
 *  1. OptionalParams 结构体中具有三个输入限制，彼此存在转换关系，电机实际的约束会被设
 *  置为等效的最小值（例如，力矩与电流均被转换为报文输入，然后取最小值作为约束）。同时只
 *  有当设置的约束值小于电机原始约束值时才会起到作用
 *  2. 注意由于电机计圈功能依靠前后两次电机的反馈，因此对于有高速转动电机的需求，要求其
 *  反馈频率不能过低，最低要求为在两次反馈间隔中，电机转子端不能转动超过半圈
 *  3. 电机转向关系标定：先在电机配置中选择电机转动正方向与自定转动正方向相同，烧录程序，
 *  用手转动电机输出端，在 Debug 中查看数据变化是否与期望的相同，若不同则电机转动正方向
 *  与自定转动正方向相反，以此配置转向关系
 *  4. 电机减速比配置：组件中电机默认的的减速比为电机自带减速器的减速比。
 *    1）当电机实际使用时在自带减速器外还额外增加减速器时，应当在电机初始化时将初始化结
 *  构体中 ex_redu_rat 属性设置为外加减速器的减速比，同时将 remove_build_in_reducer
 *  属性设置为 false
 *    2）当电机实际使用时拆除了电机自带的减速器，又外加了减速器时，应当在电机初始化时将
 *  初始化结构体中 ex_redu_rat 属性设置为外加减速器的减速比，同时将
 *  remove_build_in_reducer 属性设置为 true
 *  5. 电机零点标定（在电机转向关系标定与电机减速比配置完成后进行）：
 *    1）通过机械限位标定：通过机械限位标定可采取在程序一开始使先使对电机进行速度闭环，
 *  使其往固定方向转动，当读取到反馈的速度的值小于设定转速一定程度时便可认为到达机械限位，
 *  此时可调用 setAngleValue 方法设定当前的角度，值为到达机械限位时应该到达的角度值。
 *  采用此方法的往往是没有输出端绝对编码的电机，因此对于这类电机在每次程序重启后都需要进
 *  行标定后才可正常使用
 *    2）通过绝对位置标定 通过绝对位置标定可采取在电机属性初始化时将 angle_offset 属性
 *  设置为 0，然后烧录程序，将电机输出端转动到实际定义的 0 位后，通过 Debug 读取此时对
 *  应的角度值，然后将 angle_offset 属性设置为读取到的角度值，便可完成标定
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

#include <cstring>
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

float Motor::raw2Torq(float raw) const
{
  if (motor_info_.raw_mapping_type == RawMappingType::kTorq) {
    return raw * motor_info_.torq_rat * motor_info_.redu_rat;
  } else if (motor_info_.raw_mapping_type == RawMappingType::kCurr) {
    return raw * motor_info_.curr_rat * motor_info_.torq_const *
           motor_info_.redu_rat;
  } else {
    return 0;
  }
}

float Motor::torq2Raw(float torq) const
{
  if (motor_info_.raw_mapping_type == RawMappingType::kTorq) {
    return torq / motor_info_.redu_rat / motor_info_.torq_rat;
  } else if (motor_info_.raw_mapping_type == RawMappingType::kCurr) {
    return torq / motor_info_.redu_rat / motor_info_.torq_const /
           motor_info_.curr_rat;
  } else {
    return 0;
  }
}

float Motor::raw2Curr(float raw) const
{
  if (motor_info_.raw_mapping_type == RawMappingType::kTorq) {
    return raw * motor_info_.torq_rat / motor_info_.torq_const;
  } else if (motor_info_.raw_mapping_type == RawMappingType::kCurr) {
    return raw * motor_info_.curr_rat;
  } else {
    return 0;
  }
}

float Motor::curr2Raw(float curr) const
{
  if (motor_info_.raw_mapping_type == RawMappingType::kTorq) {
    return curr * motor_info_.torq_const / motor_info_.torq_rat;
  } else if (motor_info_.raw_mapping_type == RawMappingType::kCurr) {
    return curr / motor_info_.curr_rat;
  } else {
    return 0;
  }
}

float Motor::torq2Curr(float torq) const
{
  return torq / motor_info_.redu_rat / motor_info_.torq_const;
}

float Motor::curr2Torq(float curr) const
{
  return curr * motor_info_.torq_const * motor_info_.redu_rat;
}

Status Motor::setInput(float input)
{
  float actual_input = 0;
  switch (motor_info_.input_type) {
    case InputType::kRaw:
      raw_input_ = hello_world::Bound(
          input, motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
      torq_input_ = raw2Torq(raw_input_);
      curr_input_ = raw2Curr(raw_input_);
      actual_input = raw_input_;
      break;
    case InputType::kTorq:
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      raw_input_ = torq2Raw(torq_input_);
      curr_input_ = torq2Curr(torq_input_);
      actual_input = torq_input_;
      break;
    case InputType::kCurr:
      curr_input_ = hello_world::Bound(
          input, motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
      raw_input_ = curr2Raw(curr_input_);
      torq_input_ = curr2Torq(curr_input_);
      actual_input = curr_input_;
      break;
    default:
      return Status::kInputTypeError;
  }

  if (fabsf(actual_input) < fabsf(input)) {
    return Status::kInputValueOverflow;
  } else {
    return Status::kOk;
  }
}

float Motor::getInput(void) const
{
  switch (motor_info_.input_type) {
    case InputType::kRaw:
      return raw_input_;
    case InputType::kTorq:
      return torq_input_;
    case InputType::kCurr:
      return curr_input_;
    default:
      return 0;
  }
}

Status Motor::set_input_type(InputType input_type)
{
  if (input_type == InputType::kRaw || input_type == InputType::kTorq ||
      input_type == InputType::kCurr) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

float Motor::normAngle(float angle)
{
  switch (motor_info_.angle_range) {
    case AngleRange::kNegPiToPosPi:
      return NormPeriodData(-PI, PI, angle);
    case AngleRange::k0To2Pi:
      return NormPeriodData(0, 2 * PI, angle);
    case AngleRange::kNegInfToPosInf:
    default:
      return angle;
  }
}

float Motor::calcVel(void)
{
  /* 改装版去掉了微分跟踪器（td），直接返回电机反馈的原始角速度；若需要角度差分速度，
   * 请由新人在自己的设备类之外自行实现 */
  return vel_raw_;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
