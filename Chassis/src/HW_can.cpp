/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *  V1.1.0      2026-10-21      Agent B         1. WP-07：FIFO0 按 0x201~0x204 分发到
 *                                                4×M3508 decode；修正 CAN2SB 未初始化
 *******************************************************************************
 * @attention :
 *  HAL_CAN_RxFifo0MsgPendingCallback（CAN1）按 StdId 与各电机 GetRxId 匹配后调用
 *  decode；HAL_CAN_RxFifo1MsgPendingCallback（CAN2）留给 WP-10 双板协议解码。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "ins_motor.hpp"
#include "motor_M3508.hpp"

/* Private constants ---------------------------------------------------------*/
namespace
{
/* 轮电机数量（索引顺序 LF、LB、RB、RF，见 config.hpp） */
constexpr int kWheelNum = 4;
}  // namespace

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* 注意：CAN_FilterTypeDef 必须显式初始化（= {0}）。F4 双 CAN 实例下
   * HAL_CAN_ConfigFilter 会用 SlaveStartFilterBank 重写 CAN_FMR.CAN2SB；该字段
   * 未初始化（栈上随机值）时，可能把 CAN1 的滤波组错划给 CAN2，导致 CAN1 接收全丢。
   * 按契约 §4.2：CAN1 使用 bank 0~13（FIFO0），CAN2 使用 bank 14~27（FIFO1）。 */
  CAN_FilterTypeDef can_filter = {0};
  can_filter.SlaveStartFilterBank = 14;

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
 * @brief   CAN1 FIFO0 接收中断回调（轮电机反馈分发）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    按 StdId 匹配 4 个轮电机的接收 ID（0x201~0x204）后调用 decode
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      /* 按 StdId 路由：0x201~0x204 为 4 个轮电机的反馈帧（0x200 + 电调 ID，
       * 契约 §4.2）。用 M3508::GetRxId(id) 匹配，不硬编码反馈 ID。 */
      for (int i = 0; i < kWheelNum; ++i) {
        hello_world::motor::Motor *motor = GetMotorWheel(i);
        if (motor != nullptr &&
            rx_header1.StdId ==
                hello_world::motor::M3508::GetRxId(motor->id())) {
          (void)motor->decode(can1_rx_data);
          break;
        }
      }
      /* 其它 ID（如双板通讯帧 0x210/0x211，属 WP-10）当前直接丢弃。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（双板通讯帧预留）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    本 WP 只建立收发通道，解码由 WP-10 在此实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* TODO【WP-10】: CAN2 承载双板通讯帧（底盘 0x210 / 云台 0x211，契约 §4.1），
       * 届时在此调用 GimbalChassisComm 解码。本 WP 只建立 CAN 收发通道，
       * 收到后直接丢弃。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
