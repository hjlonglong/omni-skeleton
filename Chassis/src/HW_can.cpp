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
 *  V1.2.0      2026-10-28      Lead            1. CAN 故障可观测性：发送失败清邮箱并计数，
 *                                                错误回调快照 ErrorCode/ESR/TSR（Ozone 可看）
 *  V1.3.0      2026-11-07      Agent B         1. WP-10：FIFO1 按 StdId 路由 G2C 双板
 *                                                帧（0x211）到 GimbalChassisComm::decode
 *  V1.4.0      2026-11-08      Lead            1. 按实物通讯拓扑调整：双板帧 G2C 改由
 *                                                CAN1 FIFO0 分发（两板 CAN1 相连）
 *******************************************************************************
 * @attention :
 *  - 2026-11-08 实物拓扑：两板 CAN1 物理相连，本板 CAN1 上同时有 4×M3508 反馈
 *    （0x201~0x204）与 G2C 双板帧（0x211），FIFO0 回调按 StdId 路由。
 *  - CAN2 本板暂未使用（保留 FIFO1 处理，兼容旧接线/后续扩展）。
 *  - 中断里只做 8 字节解码（memcpy + 整型/浮点运算），不得调用 HAL_Delay/打印。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "config.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
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

/* CAN 故障观测（Ozone Live Watch 用；volatile 防止 -O0 之外被优化掉）---------- */
volatile uint32_t g_can_err_instance = 0;  ///< 1=CAN1 2=CAN2（最近一次错误来源）
volatile uint32_t g_can_err_code = 0;      ///< 最近一次错误时的 HAL ErrorCode 快照
volatile uint32_t g_can_err_esr = 0;       ///< 最近一次错误时的 ESR 快照
volatile uint32_t g_can_err_tsr = 0;       ///< 最近一次错误时的 TSR 快照
volatile uint32_t g_can_err_cnt = 0;       ///< CAN 错误回调累计次数
volatile uint32_t g_can_tx_fail_cnt = 0;   ///< CAN_Send_Msg 发送失败累计次数
/* 接收分发计数（Ozone Live Watch：判断电机反馈/双板帧是否真的到达并解码）---- */
volatile uint32_t g_can_wheel_rx_cnt[4] = {0, 0, 0, 0};  ///< 各轮反馈解码次数
volatile uint32_t g_can_wheel_unknown_cnt = 0;  ///< CAN1 上未匹配到轮电机的帧数
volatile uint32_t g_can_g2c_rx_cnt = 0;         ///< 收到的 G2C（0x211）双板帧数

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
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) == HAL_OK) {
    return;
  }

  /* 发送失败：常见为 3 个发送邮箱全满（总线无 ACK / 错误被动 / bus-off 时帧一直
   * 挂在邮箱里）。清空一次发送邮箱，避免此后每个 1 ms 周期都返回
   * HAL_CAN_ERROR_PARAM（0x200000）且电机永远收不到新指令；同时做快照便于诊断。 */
  g_can_tx_fail_cnt += 1;
  g_can_err_instance = (hcan == &hcan1) ? 1u : 2u;
  g_can_err_code = hcan->ErrorCode;
  g_can_err_esr = hcan->Instance->ESR;
  g_can_err_tsr = hcan->Instance->TSR;
  (void)HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 |
                                         CAN_TX_MAILBOX2);
  hcan->ErrorCode = HAL_CAN_ERROR_NONE;
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（轮电机反馈 + G2C 双板帧分发）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    2026-11-08 实物拓扑：两板 CAN1 相连。按 StdId 路由：
 *          - 0x211（G2C）：云台板状态/关节角/IMU → GimbalChassisComm::decode；
 *          - 0x201~0x204：4 个轮电机反馈 → M3508::decode；
 *          - 其它 ID：计数后丢弃。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      bool matched = false;

      /* 1) 双板帧 G2C（0x211）：两板 CAN1 相连（2026-11-08 实测） */
      if (rx_header1.StdId == config::kCanIdG2C) {
        robot::GimbalChassisComm *comm = GetGimbalChassisComm();
        if (comm != nullptr) {
          comm->decode(can1_rx_data);
          g_can_g2c_rx_cnt += 1u;
          matched = true;
        }
      }

      /* 2) 轮电机反馈 0x201~0x204（用 M3508::GetRxId 匹配，不硬编码） */
      if (!matched) {
        for (int i = 0; i < kWheelNum; ++i) {
          hello_world::motor::Motor *motor = GetMotorWheel(i);
          if (motor != nullptr &&
              rx_header1.StdId ==
                  hello_world::motor::M3508::GetRxId(motor->id())) {
            (void)motor->decode(can1_rx_data);
            g_can_wheel_rx_cnt[i] += 1u;
            matched = true;
            break;
          }
        }
      }

      if (!matched) {
        /* 其它 ID（电调错误帧/其它设备） */
        g_can_wheel_unknown_cnt += 1u;
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（预留；本板 CAN2 暂未使用）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    2026-11-08 实物拓扑：双板帧走 CAN1，本板 CAN2 上没有设备。保留 FIFO1
 *          处理以兼容旧接线/后续扩展；收到 G2C 时仍解码（冗余无害）。
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      /* 用 config::kCanIdG2C 匹配，不硬编码（契约 §3.2/§4.2） */
      if (rx_header2.StdId == config::kCanIdG2C) {
        robot::GimbalChassisComm *comm = GetGimbalChassisComm();
        if (comm != nullptr) {
          comm->decode(can2_rx_data);
          g_can_g2c_rx_cnt += 1u;
        }
      }
      /* 其它 ID 丢弃：本板只接收云台板的 G2C 帧。 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   CAN 错误回调（错误中断打开后由 HAL_CAN_IRQHandler 调用）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    本工程不用 RTT 打印（避免在中断里做慢速 IO），只把 ErrorCode/ESR/TSR
 *          快照写入全局变量供 Ozone Live Watch 观察：
 *            g_can_err_code   bits: EWG(0x1) EPV(0x2) BOF(0x4) STF(0x8) FOR(0x10)
 *                                  ACK(0x20) BR(0x40) BD(0x80) CRC(0x100)
 *                                  PARAM(0x200000，邮箱满)
 *            g_can_err_esr    TEC/REC/LEC/BOFF/EPVF/EWGF（原始 ESR 寄存器）
 *            g_can_err_tsr    三个发送邮箱状态/TXOK/TERR
 *          快照后清空 hcan->ErrorCode，使下一次错误能反映"新"故障而非粘滞值。
 */
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    g_can_err_instance = 1u;
  } else if (hcan == &hcan2) {
    g_can_err_instance = 2u;
  } else {
    return;
  }
  g_can_err_cnt += 1;
  g_can_err_code = hcan->ErrorCode;
  g_can_err_esr = hcan->Instance->ESR;
  g_can_err_tsr = hcan->Instance->TSR;
  hcan->ErrorCode = HAL_CAN_ERROR_NONE;
}
