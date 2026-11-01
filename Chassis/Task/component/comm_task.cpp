/*
 * @Author: miunoki 13567781025@163.com
 * @Date: 2026-09-19 15:11:39
 * @LastEditors: miunoki 13567781025@163.com
 * @LastEditTime: 2026-09-19 21:14:38
 * @FilePath: \omni_skeleton\Chassis\Task\component\comm_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#include "HW_can.hpp"
#include "can.h"
#include "config.hpp"
#include "ins_motor.hpp"
#include "iwdg.h"

/* Private constants ---------------------------------------------------------*/
namespace {
/* 轮电机数量（索引顺序 LF、LB、RB、RF，见 config.hpp） */
constexpr int kWheelNum = 4;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 本 WP 只做 CAN 通道初始化；UART DBUS / IWDG 属 WP-09 / WP-15，见文末 TODO。 */

  /* 1. 先触发 Instance 层实例化 4 个轮电机（保证首次构造发生在开接收中断之前） */
  for (int i = 0; i < kWheelNum; ++i) {
    (void)GetMotorWheel(i);
  }

  /* 2. 接收滤波器（全收：CAN1→FIFO0，CAN2→FIFO1，契约 §4.2） */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* 3. 启动 CAN 外设并打开接收中断。HAL_CAN_Init 后外设仍处于初始化模式，
   *    不调用 HAL_CAN_Start 则永远无法收发。 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* 除接收外再打开错误类中断（ERRIE + 细分标志），由 HAL_CAN_IRQHandler 汇总后
   * 调用 HW_can.cpp 里的 HAL_CAN_ErrorCallback，供 bus-off/ACK 错误等故障诊断。 */
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                      CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    Error_Handler();
  }

  /* CAN2 预留给双板通讯帧（契约 §4.2），此处只做底层启动；接收分发由 WP-10 在
   * HW_can.cpp 的 FIFO1 回调中补齐。 */
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(
          &hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                      CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    Error_Handler();
  }

  /* TODO【WP-09】: 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）。
   * IWDG 已在 main.c 的 MX_IWDG_Init() 启动（契约 §8.2）：MainTaskInit 末尾喂一次，
   * 之后每 1 ms 在 CommTask() 末尾刷新；停喂复位演示属 WP-20。 */
}

void CommTask(void) {
  /* M3508 控制帧 0x200：电调 ID 1~4 共用同一帧，每个电机 encode() 只写自己的 2 字节，
   * 因此必须让 4 个电机依次 encode 到同一个 8 字节缓冲区，最后只发一帧（契约 §8.1）。
   * 每周期都发（输入为 0 也发），保证电调持续收到最新指令。 */
  uint8_t tx_data[8] = {0};
  for (int i = 0; i < kWheelNum; ++i) {
    hello_world::motor::Motor *motor = GetMotorWheel(i);
    if (motor == nullptr) {
      continue;
    }
    (void)motor->encode(tx_data);
  }
  CAN_Send_Msg(&hcan1, tx_data, config::kCanIdMotorTx, 8);

#if DEBUG_NO_IWDG == 0
  /* 契约 §8.2：在 1 ms 任务链末尾刷新 IWDG。本函数运行在 TIM6 中断里，
   * 若中断卡死/任务链停摆则会停止喂狗 → 底盘板 ~300 ms 后复位。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
#endif

  /* TODO【WP-10】: 双板通讯帧（底盘 0x210）encode 后经 CAN2 发出。 */
}

/* Private function definitions ----------------------------------------------*/
/* TODO【WP-09】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
