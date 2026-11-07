/**
 *******************************************************************************
 * @file      : HW_can.hpp
 * @brief     : CAN 收发基础封装（滤波器初始化 + 发送）与调试观测变量
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：只提供 CAN filter 与发送，
 *                                                接收分发留给新人
 *  V1.1.0      2026-11-07      Agent C         1. WP-08/16：接收分发落地，补调试观测变量
 *******************************************************************************
 * @attention :
 *  1. 本组件提供 CAN 滤波器初始化（全收）与一帧标准帧发送。
 *  2. 接收分发在 HW_can.cpp 的回调中完成：
 *     - CAN1 / FIFO0：达妙电机反馈（0x11/0x12）→ motor->decode()；
 *     - CAN2 / FIFO1：底盘板 C2G 帧（0x210）→ GimbalChassisComm::decode()。
 *  3. g_can_* 为调试可观测变量（Ozone Live Watch），用于无硬件自测与实车排查。
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
 * @note        hcan1 配置到 FIFO0、bank0；hcan2 配置到 FIFO1、bank14。
 *              必须用 {0} 清零结构体并显式设置 SlaveStartFilterBank = 14，
 *              否则 HAL 会把 CAN2 滤波器算到 CAN1 的 bank 区间（task-2 缺陷）。
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

/* Exported variables --------------------------------------------------------*/
/* 调试/诊断可观测变量（Ozone Live Watch；定义在 src/HW_can.cpp） */
extern volatile uint32_t
    g_can_yaw_motor_last_rx_tick;  ///< yaw DM 反馈帧最近接收时刻（ms，0=未收到）
extern volatile uint32_t
    g_can_pitch_motor_last_rx_tick;  ///< pitch DM 反馈帧最近接收时刻（ms）
extern volatile uint32_t g_can_c2g_last_rx_tick;  ///< C2G 帧最近接收时刻（ms）
extern volatile uint32_t g_can_yaw_fdb_cnt;       ///< yaw 反馈帧累计计数
extern volatile uint32_t g_can_pitch_fdb_cnt;     ///< pitch 反馈帧累计计数
extern volatile uint32_t g_can_c2g_rx_cnt;        ///< C2G 帧累计计数
extern volatile uint32_t g_can_err_instance;      ///< 最近一次错误的外设实例
extern volatile uint32_t g_can_err_code;          ///< 最近一次错误码
extern volatile uint32_t g_can_err_overflow_cnt;  ///< RX FIFO 溢出累计计数
extern volatile uint32_t g_can_tx_fail_cnt;       ///< 发送失败累计计数

#endif /* HW_COMPONENTS_BSP_COMM_HW_CAN_HPP_ */
