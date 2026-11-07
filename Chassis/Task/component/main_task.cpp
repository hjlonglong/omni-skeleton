/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     :底盘任务层入口：1 ms 周期调度 + 各任务初始化
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-21      Lead            1. 实现 TIM6 1 ms 中断分发与
 *                                                MainTaskInit：CommTask +
 *                                                ControlTask（单轮控制）接入
 *  V1.1.0      2026-11-07      Agent B         1. WP-12/15/17：MainTask 接入
 *                                                Robot::update（整车状态机），
 *                                                TEST_MODE==1 时控制权归
 *                                                ControlTask（台架扫掠）
 *******************************************************************************
 * @attention :
 *  - 1 ms 调度顺序：MainTask() → ControlTask() → CommTask()。
 *    MainTask 先跑整车状态机（含轮速环输出），ControlTask 仅在台架模式生效，
 *    CommTask 在同一拍把 0x200 / 0x210 帧发出，避免电机指令晚一拍。
 *  - TEST_MODE == 1（台架）时 MainTask 只更新 IMU、不跑 Robot（避免状态机与单轮
 *    扫掠抢控制权）；TEST_MODE == 0（交付）时控制权全归 Robot/Chassis。
 *  - MainTaskInit() 必须在 TIM6 中断启动前调用（main.c USER CODE 2），
 *    否则中断可能在 CAN/DBUS 初始化完成前触发。
 *  - 本函数保持薄一层：业务逻辑都在 Robot/Chassis/RobotModules 里。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "config.hpp"
#include "control_task.hpp"
#include "imu.hpp"
#include "ins_imu.hpp"
#include "ins_robot.hpp"
#include "iwdg.h"
#include "robot.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void)
{
  /* 1. 通讯层：触发 Instance 实例化电机/遥控/双板协议，配置 CAN 滤波器 + 收发
   *    中断 + DBUS 空闲 DMA 接收。 */
  CommTaskInit();

  /* 2. 传感层：实例化 Imu 并初始化 BMI088（SPI1 + 片选），阻塞数 ms；
   *    必须在 TIM6 中断启动前完成（MainTaskInit 由 main.c USER CODE 2 调用）。 */
  GetImu()->init();

  /* 3. 控制层：创建 Robot/Chassis 单例，绑定底盘、IMU、双板通讯、遥控器与
   *    4 轮电机/轮速环 PID，并把跟随环 PID 按 config 整定。 */
  GetRobot()->init();

#if TEST_MODE == 1
  /* 4. 台架模式：绑定单轮电机与 PID（扫掠目标在 ControlTask 里生成）。 */
  ControlTaskInit();
#endif

  /* 5. 契约 §8.2：IWDG 上电即计时，耗时初始化完成后立刻喂一次；之后每 1 ms
   *    在 CommTask() 末尾刷新（DEBUG_NO_IWDG==1 时停止喂狗，仅限台架调试）。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
}

void MainTask(void)
{
  /* 传感层：1 ms 更新 IMU（读 SPI → 零漂标定/Mahony → roll/pitch/yaw、gyro_z）。
   * 放在最前面，保证同拍 Robot/CommTask 用的是最新姿态。 */
  GetImu()->update();

#if TEST_MODE == 1
  /* 台架模式：控制权归 ControlTask（单轮三角波扫掠），不跑整车状态机；
   * C2G 遥测保持初始值（life_state=Dead），云台板据此不会动作。 */
#else
  /* 交付模式：整车状态机（遥控解码 → 三态迁移 → 模式/δ 变换 → 逆解 →
   * 4 轮速度环 → setInput）每 1 ms 跑一次。 */
  GetRobot()->update();
#endif
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
