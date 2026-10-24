/**
 *******************************************************************************
 * @file      : HW_can.hpp
 * @brief     : CAN 收发基础封装（滤波器初始化 + 发送）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：只提供 CAN filter 与发送，
 *                                                接收分发留给新人
 *******************************************************************************
 * @attention :
 *  1. 本组件只提供 CAN 滤波器初始化（全收）与一帧标准帧发送。
 *  2. 接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已预留在 HW_can.cpp 中，
 *     新人需在回调内按帧 ID 把 8 字节数据分发给电机 decode / 双板协议解码。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_BSP_COMM_HW_CAN_HPP_
#define HW_COMPONENTS_BSP_COMM_HW_CAN_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "stm32_hal.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief       CAN 滤波器初始化（全收：不过滤任何 ID）
 * @param       hcan: CAN 句柄（&hcan1 或 &hcan2）
 * @retval      None
 * @note        hcan1 配置到 FIFO0、bank0；hcan2 配置到 FIFO1、bank14
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief       向 CAN 总线发送一帧标准帧
 * @param       hcan: CAN 句柄
 * @param       msg: 待发送的 8 字节数据首地址
 * @param       id: 标准帧 ID（11 位）
 * @param       len: 数据长度（字节数，一般 8）
 * @retval      None
 */
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* HW_COMPONENTS_BSP_COMM_HW_CAN_HPP_ */
