/**
 *******************************************************************************
 * @file      : ins_motor.cpp
 * @brief     : 云台板电机实例化（2 × DM-J4310，MIT 模式）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-08：yaw/pitch 两轴达妙电机实例化
 *******************************************************************************
 * @attention :
 *  - 函数内 static 对象：首次调用（CommTaskInit 内）构造，保证 CAN 接收中断打开前
 *    对象已存在；运行期无动态内存。
 *  - yaw 需要 360° 连续旋转，angle_range 取 kNegInfToPosInf（反馈为多圈连续角）；
 *    pitch 只在小角度范围运动，取默认 kNegPiToPosPi。
 *  - 达妙电机只有力矩输入（input_type = kTorq），位置控制在上层 PID 完成，
 *    下发用 MIT 纯力矩（setMitInput(0,0,0,0,τ)，契约 §7.3）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"

#include "config.hpp"
#include "motor.hpp"

namespace
{

using hello_world::motor::AngleRange;
using hello_world::motor::InputType;
using hello_world::motor::OptionalParams;

/**
 * @brief       生成达妙 DM-J4310 可选配置参数
 * @param       dir: 转动方向（config 定义，+1 同向 / -1 反向）
 * @param       angle_offset: 输出端零位偏置（rad）
 * @param       angle_range: 角度反馈范围
 * @retval      OptionalParams
 * @note        ex_redu_rat = 1：DM-J4310 直驱，无外置减速比；
 *              max_*_input_lim 保持默认（大值），力矩限幅由上层 config 保证。
 */
OptionalParams MakeDaMiaoOptionalParams(int8_t dir, float angle_offset,
                                        AngleRange angle_range)
{
  OptionalParams opt;
  opt.input_type = InputType::kTorq;
  opt.angle_range = angle_range;
  opt.dir = (dir >= 0) ? hello_world::motor::kDirFwd
                       : hello_world::motor::kDirRev;
  opt.angle_offset = angle_offset;
  opt.ex_redu_rat = 1.0f;
  opt.remove_build_in_reducer = false;
  return opt;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::motor::Motor *GetMotorYaw()
{
  /* yaw 关节：DM-J4310，CAN ID = config::kYawMotorId。
   * auto_enable = true：上电后 encode 自动补发使能指令（电机主动上报反馈）。 */
  static hello_world::motor::DM_J4310 motor(
      config::kYawMotorId,
      MakeDaMiaoOptionalParams(config::kYawMotorDir, config::kYawAngleOffset,
                               AngleRange::kNegInfToPosInf),
      true);
  return &motor;
}

hello_world::motor::Motor *GetMotorPitch()
{
  /* pitch 关节：DM-J4310，CAN ID = config::kPitchMotorId。 */
  static hello_world::motor::DM_J4310 motor(
      config::kPitchMotorId,
      MakeDaMiaoOptionalParams(config::kPitchMotorDir, config::kPitchAngleOffset,
                               AngleRange::kNegPiToPosPi),
      true);
  return &motor;
}
