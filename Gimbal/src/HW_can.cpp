/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装 + 接收分发（电机反馈 / 双板帧）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：filter/发送 + 空回调
 *  V1.1.0      2026-11-07      Agent C         1. 接收分发落地；2. 修复 CAN2SB 缺陷
 *******************************************************************************
 * @attention :
 *  1. CAN1（FIFO0）：达妙电机反馈帧；CAN2（FIFO1）：底盘板 C2G 帧（0x210）。
 *  2. 每收一帧后必须重新 HAL_CAN_ActivateNotification，否则中断只来一次。
 *  3. 回调内不做阻塞/死等，decode 均为纯内存解析（契约 §1.3 / §10）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"
#include "tick.hpp"

/* Private constants ---------------------------------------------------------*/
namespace
{
constexpr int kAxisNum = 2;  ///< 云台轴数：yaw / pitch
}  // namespace

/* Exported variables --------------------------------------------------------*/
volatile uint32_t g_can_yaw_motor_last_rx_tick = 0;
volatile uint32_t g_can_pitch_motor_last_rx_tick = 0;
volatile uint32_t g_can_c2g_last_rx_tick = 0;
volatile uint32_t g_can_yaw_fdb_cnt = 0;
volatile uint32_t g_can_pitch_fdb_cnt = 0;
volatile uint32_t g_can_c2g_rx_cnt = 0;
volatile uint32_t g_can_err_instance = 0;
volatile uint32_t g_can_err_code = 0;
volatile uint32_t g_can_err_overflow_cnt = 0;
volatile uint32_t g_can_tx_fail_cnt = 0;

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* F4 双 CAN 共用 28 组滤波器：CAN1 用 bank0~13，CAN2 用 bank14 起
   * （SlaveStartFilterBank = 14）。结构体必须 {0} 清零初始化：HAL 的
   * HAL_CAN_ConfigFilter 会用 SlaveStartFilterBank 重算 CAN2 的 FilterBank，
   * 未清零时该字段为栈上随机值 → CAN2 收不到任何帧（task-2 缺陷根因）。 */
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.SlaveStartFilterBank = 14;
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
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  tx_header.TransmitGlobalTime = DISABLE;

  /* 无空闲邮箱时先取消挂起发送再重试，避免 1ms 周期下丢帧 */
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
    (void)HAL_CAN_AbortTxRequest(
        hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  }
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) != HAL_OK) {
    g_can_tx_fail_cnt = g_can_tx_fail_cnt + 1;  // 赋值而非 ++：避免 -Wvolatile
  }
}

/* Exported callback definitions (C linkage) --------------------------------*/

/**
 * @brief   CAN1 FIFO0 接收中断回调：分发达妙电机反馈帧
 * @note    一帧反馈只对应一个电机（0x11 = yaw、0x12 = pitch），命中后即 break；
 *          记录接收时刻供 Robot 做反馈超时判断。
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) ==
        HAL_OK) {
      const uint32_t now = hello_world::tick::GetTickMs();
      hello_world::motor::Motor *const motors[kAxisNum] = {GetMotorYaw(),
                                                           GetMotorPitch()};
      volatile uint32_t *const rx_ticks[kAxisNum] = {
          &g_can_yaw_motor_last_rx_tick, &g_can_pitch_motor_last_rx_tick};
      volatile uint32_t *const rx_cnts[kAxisNum] = {&g_can_yaw_fdb_cnt,
                                                    &g_can_pitch_fdb_cnt};

      for (int i = 0; i < kAxisNum; ++i) {
        hello_world::motor::Motor *motor = motors[i];
        if (motor == nullptr) {
          continue;
        }
        if (rx_header1.StdId ==
            hello_world::motor::DaMiao::GetRxId(motor->id())) {
          (void)motor->decode(can1_rx_data);  // 纯内存解析，不阻塞
          *rx_ticks[i] = now;
          *rx_cnts[i] = *rx_cnts[i] + 1;  // 赋值而非 ++：避免 -Wvolatile
          break;
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调：解码底盘板 C2G 帧（0x210）
 */
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) {
      robot::GimbalChassisComm *const comm = GetGimbalChassisComm();
      if (comm != nullptr && rx_header2.StdId == comm->rx_id()) {
        comm->decode(can2_rx_data);  // 刷新 last_rx_tick_ 与 chassis_data_
        g_can_c2g_last_rx_tick = hello_world::tick::GetTickMs();
        g_can_c2g_rx_cnt = g_can_c2g_rx_cnt + 1;  // 赋值而非 ++：避免 -Wvolatile
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   CAN 错误回调：仅做快照与计数，不做阻塞操作
 */
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  g_can_err_instance =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(hcan->Instance));
  g_can_err_code = hcan->ErrorCode;

  if ((hcan->ErrorCode & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) !=
      0U) {
    g_can_err_overflow_cnt = g_can_err_overflow_cnt + 1;  // 避免 -Wvolatile
  }
}
