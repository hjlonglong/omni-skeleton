/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      /* TODO【新人任务】: 在此按 rx_header1.StdId 路由 8 字节 can1_rx_data。
       * 判定方法：把 StdId 与各电机的接收 ID（GetRxId）及双板通讯帧 ID 逐一比对：
       *   - 命中某电机的接收 ID：调用该电机 decode(can1_rx_data) 解包反馈；
       *   - 命中双板通讯帧 ID：调用自己的 GimbalChassisComm 解码；
       *   - 都未命中：丢弃（返回前记得重新使能接收中断，见下方调用）。
       */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* TODO【新人任务】: 同 FIFO0，按 rx_header2.StdId 路由 8 字节 can2_rx_data。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
