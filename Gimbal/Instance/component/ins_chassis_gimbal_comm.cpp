/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
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
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() {
  /* TODO【新人任务】:
   * 返回本板双板通讯对象单例。
   *
   * 你要做：
   *   1. 静态定义一个 robot::GimbalChassisComm 对象，构造时传入：
   *      - CodePart：本板是 Chassis 还是 Gimbal（决定编解码方向）。
   *      - tx_id / rx_id：本板发送帧 / 接收帧的 CAN 标准帧 ID（两板协商一致）。
   *   2. 返回其地址。
   *
   * 约束 / 提示：
   *   - 类定义在 RobotComponents/component/gimbal_chassis_comm.hpp（encode/decode 由你实现）。
   *   - CAN ID 建议从 0x200 之后的空闲标准帧 ID 里选，避开电机反馈 ID。
   */
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
