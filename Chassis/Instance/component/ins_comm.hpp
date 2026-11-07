/**
 *******************************************************************************
 * @file      :ins_comm.hpp
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
#ifndef CHASSIS_INSTANCE_INS_COMM_HPP_
#define CHASSIS_INSTANCE_INS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取遥控器解码对象（DT7）单例。 */
hello_world::remote_control::DT7 *GetRemoteControl();

namespace hello_world {
namespace remote_control {
/* 刷新遥控在线时间戳：UART 空包回调在成功解包 18 字节后调用
 * （Task/component/comm_task.cpp）。收到第一帧前 IsRemoteOnline() 为 false。 */
void NotifyRemoteRx();
/* 距最近一次成功解包是否超过 config::kRemoteOfflineThresMs（回绕安全）。 */
bool IsRemoteOnline();
}  // namespace remote_control
}  // namespace hello_world

#endif /* CHASSIS_INSTANCE_INS_COMM_HPP_ */
