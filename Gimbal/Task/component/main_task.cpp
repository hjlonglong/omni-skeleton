/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :云台板任务层入口：1 ms 周期调度与 IMU/通讯初始化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2026-10-28      Lead            1. 骨架：MainTaskInit/MainTask 空体，
 *                                                等待任务层接入
 *  V1.0.0      2026-11-01      A1              1. 实现 MainTaskInit/MainTask 与 TIM6
 *                                                1 ms 回调：接入 CommTask 与 IMU
 *                                                更新（WP-06）
 *******************************************************************************
 * @attention :
 *  - 1 ms 调度顺序固定为 MainTask()（IMU 更新）→ CommTask()；云台控制业务
 *    （WP-08/12）之后接在 MainTask() 里，不要打乱 IMU 先更新的顺序。
 *  - 云台板没有 ControlTask：底盘板的 ControlTask 是单轮台架任务，云台不要照抄。
 *  - MainTaskInit() 必须在 TIM6 中断启动前调用（见 Core/Src/main.c 的 USER CODE 2），
 *    否则 CAN/SPI 初始化完成前中断就可能触发。
 *  - TODO【WP-10】：本板 CommTask() 仍为空壳，未周期喂 IWDG，上电约 0.6 s 会复位；
 *    联调前需补齐，详见 README「已知限制」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "imu.hpp"
#include "ins_imu.hpp"
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/

void MainTaskInit(void)
{
  /* 1. 通讯层：CAN 滤波器/收发模式初始化。
   *    当前 Gimbal 的 CommTaskInit() 为空壳（WP-10 补齐），此处保持与底盘板
   *    一致的分层顺序，后续只改 comm_task.cpp，不动本文件。 */
  CommTaskInit();

  /* 2. 传感层：实例化 Imu 并初始化 BMI088（SPI1 + 片选）。imuInit() 会读芯片
   *    ID/写配置寄存器，阻塞数毫秒；必须发生在 TIM6 启动之前。 */
  GetImu()->init();

  /* 3. IWDG 上电即计时（见 MX_IWDG_Init），阻塞初始化结束后立刻喂一次；
   *    之后需要周期喂狗，否则约 0.6 s 复位——见文件头 TODO【WP-10】。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void MainTask(void)
{
  /* 传感层：1 ms 更新 IMU（SPI 读六轴 → 零漂标定/Mahony → roll/pitch/yaw、
   * gyro_z）。放在最前面，保证同拍控制任务用的是最新姿态。 */
  GetImu()->update();

  /* TODO【WP-08/12】：云台控制/状态机周期入口在此接入（robot->update() 等），
   * 本函数保持薄一层，不要在这里堆业务逻辑。 */
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief  TIM6 1 ms 周期中断回调（重载 HAL 的弱函数）
 * @param  htim: 触发中断的定时器句柄
 * @retval None
 * @note   先判断中断来源是 TIM6，避免把 TIM2/TIM4 等定时器中断误当成 1 ms tick；
 *         再依次调用 MainTask() 与 CommTask()。云台板无 ControlTask。
 *         HAL 以 C 符号查找回调，故必须用 extern "C" 定义。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6) {
    return;
  }

  MainTask();
  CommTask();
}
