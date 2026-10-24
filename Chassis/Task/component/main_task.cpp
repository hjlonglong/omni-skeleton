/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :底盘任务层入口：1 ms 周期调度 + 各任务初始化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-21      Lead            1. 实现 TIM6 1 ms 中断分发与
 *                                                 MainTaskInit：CommTask +
 *                                                 ControlTask（单轮控制）接入
 *******************************************************************************
 * @attention :
 *  - 1 ms 调度顺序：MainTask() → ControlTask() → CommTask()。
 *    ControlTask 先算出发给电调的输入，CommTask 再把同一拍的 0x200 帧发出，
 *    避免电机指令晚一拍。
 *  - MainTask() 保留给整车状态机（WP-12：robot->update()），当前为空，
 *    符合"任务层保持薄"的约定。
 *  - MainTaskInit() 必须在 TIM6 中断启动前调用（main.c USER CODE 2），
 *    否则中断可能在 CAN 初始化完成前触发。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "control_task.hpp"
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void)
{
  /* 1. 通讯层：触发 Instance 实例化电机、配置 CAN 滤波器并启动收发中断 */
  CommTaskInit();

  /* 2. 控制层：绑定单轮电机与 PID、整定参数、清零历史状态 */
  ControlTaskInit();

  /* 3. 契约 §8.2：IWDG 上电即计时，耗时初始化完成后立刻喂一次；之后每 1 ms
   *    在 CommTask() 末尾刷新（DEBUG_NO_IWDG==1 时停止喂狗，仅限台架调试）。 */
  (void)HAL_IWDG_Refresh(&hiwdg);

  /* TODO【WP-09】: 遥控器 DBUS 接收启动在 CommTaskInit 内补齐。 */
}

void MainTask(void)
{
  /* TODO【WP-12】: 1 ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」整条链路。本函数保持薄一层，
   * 不要在这里堆业务逻辑。单轮闭环已拆到 ControlTask()（见下）。 */
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief   TIM6 1 ms 周期中断回调（HAL 弱函数重载）
 * @param   htim: 触发中断的定时器句柄
 * @retval  None
 * @note    先判中断来源是 TIM6，避免把 TIM2/TIM4 等其它定时器中断误当成 1 ms
 *          tick；再按「状态机 → 控制 → 通讯」顺序调用各任务。回调是 C 链接，
 *          HAL 在 C 文件中按 C 符号查找，因此必须用 extern "C" 包裹。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6) {
    return;
  }

  MainTask();
  ControlTask();
  CommTask();
}
