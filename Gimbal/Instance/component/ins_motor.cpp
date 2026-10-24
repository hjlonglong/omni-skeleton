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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw() {
  /* TODO【新人任务】:
   * 返回云台 yaw 关节电机单例（DM-J4310）。
   *
   * 你要做：
   *   1. 静态定义 yaw 电机对象（CreateMotor(kDM_J4310, id, opt) 或直接构造 DM_J4310）。
   *   2. 配置 id / 方向 / 减速比 / 角度偏移，并确保使能（auto_enable / enable()），
   *      达妙电机失能状态下不会反馈数据。
   *   3. 返回其地址。
   *
   * 约束 / 提示：
   *   - 设备类见 inc/motor_DaMiao.hpp（DM_J4310），工厂 MotorType::kDM_J4310。
   *   - yaw 关节注意 360° 连续旋转的角度过零处理（MIT 位置控制内部不做多圈累计，
   *     见 motor_DaMiao.hpp 注释）。
   */
  return nullptr;
}

hello_world::motor::Motor *GetMotorPitch() {
  /* TODO【新人任务】:
   * 返回云台 pitch 关节电机单例（DM-J4310），做法同 GetMotorYaw()。
   * pitch 关节需用反馈角度做限位，防止堵转 / 撞结构。
   */
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
