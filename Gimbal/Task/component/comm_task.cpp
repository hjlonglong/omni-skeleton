/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
}

void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
