/**
 *******************************************************************************
 * @file      :control_task.hpp
 * @brief     :底盘任务层——单轮控制（单电机速度闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      Lead            1. 新增 ControlTask：单轮速度环
 *                                                 （WP-07/ WP-05 之后的首个闭环）
 *******************************************************************************
 * @attention :
 *  1. 本任务在 TIM6 1 ms 中断中调用，顺序为 MainTask() → ControlTask() →
 *     CommTask()：控制先算出发给电调的输入，通讯任务再把同一拍的输入发出去。
 *  2. TEST_MODE == 1 时进入单轮台架扫掠（电流 ≤ kTestWheelCurrentLimit，车轮
 *     必须离地）；TEST_MODE == 0 时为安全兜底：无遥控/状态机输入时保持零电流。
 *  3. 目标速度目前只来自台架扫掠（TEST_MODE 1）；接入遥控与轮速分配（WP-09 /
 *     WP-12）后，把目标改由机器人状态机给出即可，本任务只负责闭环。
 *  4. 扫掠波形生成 CalcTestSweepTargetRadS() 是纯函数（不依赖 HAL / tick），
 *     可直接在主机上做单元测试。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_CONTROL_TASK_HPP_
#define CHASSIS_TASK_CONTROL_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "config.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       单轮台架扫掠目标速度（纯函数）
 * @param       elapsed_ms: 从 ControlTaskInit() 起经过的毫秒数
 * @retval      目标轮速，单位 rad/s
 * @note        波形：先等 kTestWheelStartDelayMs → 以 kTestWheelSweepRateRadS2
 *              的斜率在 [-kMax, +kMax] 三角波扫掠（2 s 一个周期）→ 超过
 *              kTestWheelDurationMs 后归零。纯函数便于主机测试（见 TEST_MODE）。
 */
inline float CalcTestSweepTargetRadS(uint32_t elapsed_ms)
{
  /* 上电等待阶段与超时阶段：目标归零（安全） */
  if (elapsed_ms < config::kTestWheelStartDelayMs) {
    return 0.0f;
  }
  const uint32_t t = elapsed_ms - config::kTestWheelStartDelayMs;
  if (t >= config::kTestWheelDurationMs) {
    return 0.0f;
  }

  /* 半程时间：幅值 / 斜率（ms）。0 表示参数非法，直接输出 0 防止除零。 */
  const uint32_t half_ms = static_cast<uint32_t>(
      1000.0f * config::kTestWheelSweepMaxRadS /
          config::kTestWheelSweepRateRadS2 +
      0.5f);
  if (half_ms == 0u) {
    return 0.0f;
  }

  /* 三角波：[-A, +A] 上升 2*half，再 [+A, -A] 下降 2*half（周期 4*half） */
  const uint32_t period_ms = 4u * half_ms;
  const uint32_t seg_ms = t % period_ms;
  const float rate_per_ms = config::kTestWheelSweepRateRadS2 / 1000.0f;
  if (seg_ms < 2u * half_ms) {
    return -config::kTestWheelSweepMaxRadS +
           rate_per_ms * static_cast<float>(seg_ms);
  }
  return config::kTestWheelSweepMaxRadS -
         rate_per_ms * static_cast<float>(seg_ms - 2u * half_ms);
}

/**
 * @brief       控制任务初始化（由 MainTaskInit 调用一次）
 * @retval      None
 * @note        取单轮电机与 PID 实例、按 config 整定 PID 参数、清零历史状态，
 *              并记录扫掠起始时刻。必须在 TIM6 中断启动前调用。
 */
void ControlTaskInit(void);

/**
 * @brief       单轮控制任务（TIM6 1 ms 周期调用）
 * @retval      None
 * @note        TEST_MODE == 1：台架扫掠闭环；TEST_MODE == 0：安全兜底零电流。
 */
void ControlTask(void);

#endif /* CHASSIS_TASK_CONTROL_TASK_HPP_ */
