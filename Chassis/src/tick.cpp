

/**
 *******************************************************************************
 * @file      : tick.cpp
 * @brief     : 系统滴答定时器驱动源文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-10-31      ZhouShichan     1. 完成正式版
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成格式修改
 *******************************************************************************
 * @attention :
 *  当使用 STM32 自带的滴答定时器时，依赖于滴答定时器回调的触发，由于默认优先级最低，因
 *  此当系统负载较高时，可能会出现滴答定时器计数不准确，甚至停止计数的情况
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tick.hpp"

#include <cmath>

#if defined(USE_UNIX_TICK)
#include <chrono>
#include <ctime>
#elif defined(USE_HAL_TICK)
#include "stm32_hal.hpp"
#endif
namespace hello_world
{
namespace tick
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined(USE_UNIX_TICK)
std::chrono::microseconds start_tick_us =
    std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch());
#endif
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#if defined(USE_UNIX_TICK)

uint32_t GetTickUs(void)
{
  auto now = std::chrono::system_clock::now();
  auto tick =
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch()) -
      start_tick_us;
  return (uint32_t)(tick.count());
}

uint32_t GetTickMs(void)
{
  return roundf(GetTickUs() / 1000.0f);
}

uint32_t GetTickS(void)
{
  return roundf(GetTickMs() / 1000.0f);
}

void DelayUs(uint32_t us)
{
  auto start = std::chrono::system_clock::now();
  auto end = start + std::chrono::microseconds(us);
  while (std::chrono::system_clock::now() < end);
}

#elif defined(USE_HAL_TICK)

/**
 * @brief       得到系统运行时间，单位微秒
 * @note        该函数可能出现“时间倒流”情况，谨慎使用。  原因如下：
 *              有时SysTick->LOAD已经递减为0，被重置为SysTick->VAL，
 *              但HAL_GetTick()的返回值尚未加 1，导致时间计算错误。
 */
uint64_t GetTickUs(void)
{
  return (uint32_t)(HAL_GetTick() * 1000 + 1000 - 
                    SysTick->VAL * 1000 / SysTick->LOAD);
}

uint32_t GetTickMs(void)
{
  return HAL_GetTick();
}

uint32_t GetTickS(void)
{
  return roundf(GetTickMs() / 1000.0f);
}

void DelayUs(uint32_t us)
{
  uint32_t reload = SysTick->LOAD;
  uint32_t ticks = us * (SystemCoreClock / 1e6f);
  uint32_t t_last = SysTick->VAL;

  uint32_t t_now = 0;
  uint32_t t_cnt = 0;

  while (t_cnt < ticks) {
    t_now = SysTick->VAL;
    if (t_now != t_last) {
      if (t_now < t_last) {
        t_cnt += t_last - t_now;
      } else {
        t_cnt += reload - t_now + t_last;
      }
      t_last = t_now;
    }
  }
}
#endif /* USE_UNIX_TICK */

/* Private function definitions -----------------------------------------------*/
}  // namespace tick
}  // namespace hello_world