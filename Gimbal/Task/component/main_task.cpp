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
 *  V1.1.0      2026-11-07      Agent C         1. WP-08/13/16：接入 Robot 顶层（状态机+
 *                                                云台控制）；喂狗迁移到 CommTask
 *******************************************************************************
 * @attention :
 *  - 1 ms 调度顺序固定为 MainTask()（IMU 更新 → Robot 状态机/控制）→ CommTask()
 *    （电机帧 + G2C 帧 + 喂狗）；IMU 必须先更新，控制才能用最新姿态。
 *  - 云台板没有 ControlTask：底盘板的 ControlTask 是单轮台架任务，云台不要照抄。
 *  - MainTaskInit() 必须在 TIM6 中断启动前调用（见 Core/Src/main.c 的 USER CODE 2），
 *    否则 CAN/SPI 初始化完成前中断就可能触发。
 *  - IWDG 周期喂狗已按 WP-10 迁移到 CommTask 末尾（本文件不再兜底喂狗）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "ins_imu.hpp"
#include "ins_robot.hpp"
#include "iwdg.h"
#include "main.h"
#include "robot.hpp"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/

void MainTaskInit(void)
{
  /* 1. 通讯层：构造电机/双板通讯单例 + CAN 滤波器/中断启动（含 CAN2SB 修复）。
   *    分层顺序与底盘板一致：通讯 → 传感 → 顶层业务。 */
  CommTaskInit();

  /* 2. 传感层：实例化 Imu 并初始化 BMI088（SPI1 + 片选）。imu init() 会读芯片
   *    ID/写配置寄存器，阻塞数毫秒；必须发生在 TIM6 启动之前。 */
  GetImu()->init();

  /* 3. 顶层业务：从 ins_* 层装配 Robot/Gimbal，初始化云台并填首帧 G2C 数据
   *    （契约 §8：MainTaskInit = CommTaskInit → Imu init → Robot::Init → 喂狗）。 */
  GetRobot()->init();

  /* 4. IWDG 在 main.c 的 MX_IWDG_Init() 中已启动并开始计时：阻塞初始化结束后
   *    立刻喂一次，运行期由 CommTask 每 1 ms 喂狗（DEBUG_NO_IWDG=0 交付态）。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void MainTask(void)
{
  /* 传感层：1 ms 更新 IMU（SPI 读六轴 → 零漂标定/Mahony → roll/pitch/yaw、
   * gyro_z）。放在最前面，保证同拍控制任务用的是最新姿态。 */
  GetImu()->update();

  /* 顶层：安全判断 → 三态状态机 → 云台两轴控制 → 回填 G2C 帧（1ms 唯一入口）。 */
  GetRobot()->update();
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

  MainTask();  // IMU 更新 + 顶层状态机/云台控制
  CommTask();  // 电机 MIT 帧 + G2C 帧 + 喂 IWDG
}
