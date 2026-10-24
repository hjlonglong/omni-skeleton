/**
 *******************************************************************************
 * @file      : system.hpp
 * @brief     : 用于实现系统相关的功能
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-11-25      ZhouShichan     1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_SYSTEM_HPP_
#define HW_COMPONENTS_TOOLS_SYSTEM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdlib>

#include "stm32_hal.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/

#ifndef HW_OPTIMIZE_CLOSE  // 是否关闭 O2 优化
#define HW_OPTIMIZE_O2_START  \
  _Pragma("GCC push_options") \
      _Pragma("GCC optimize(\"O2\")")
#define HW_OPTIMIZE_O2_END _Pragma("GCC pop_options")
#define HW_OPTIMIZE_O2 __attribute__((optimize("O2")))
#else
#define HW_OPTIMIZE_O2_START
#define HW_OPTIMIZE_O2_END
#define HW_OPTIMIZE_O2
#endif /* HW_OPTIMIZE_CLOSE */
/* RTOS 特性 */
#if USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#define MALLOC pvPortMalloc
#define FREE vPortFree
#define ENTER_CRITICAL() taskENTER_CRITICAL()
#define EXIT_CRITICAL() taskEXIT_CRITICAL()
#define ENTER_CRITICAL_FROM_ISR() taskENTER_CRITICAL_FROM_ISR()
#define EXIT_CRITICAL_FROM_ISR() taskEXIT_CRITICAL_FROM_ISR()
#define Mutex_t SemaphoreHandle_t
#define MUTEX_INIT() xSemaphoreCreateMutex()
#define MUTEX_LOCK(xSemaphore) \
  pdFALSE == xSemaphoreTake(xSemaphore, (TickType_t)10) ? SYS_ERROR : SYS_OK
#define MUTEX_UNLOCK(xSemaphore) \
  pdFALSE == xSemaphoreGive(xSemaphore) ? SYS_ERROR : SYS_OK
#define MUTEX_LOCK_FROM_ISR(xSemaphore, pxHigherPriorityTaskWoken) \
  pdFALSE == xSemaphoreTakeFromISR(                                \
                 xSemaphore, pxHigherPriorityTaskWoken)            \
      ? SYS_ERROR                                                  \
      : SYS_OK
#define MUTEX_UNLOCK_FROM_ISR(xSemaphore, pxHigherPriorityTaskWoken) \
  pdFALSE == xSemaphoreGiveFromISR(                                  \
                 xSemaphore, pxHigherPriorityTaskWoken)              \
      ? SYS_ERROR                                                    \
      : SYS_OK
#else
#define ENTER_CRITICAL() \
  do {                   \
    __disable_irq();     \
  } while (0);
#define EXIT_CRITICAL() \
  do {                  \
    __enable_irq();     \
  } while (0);
#define Mutex_t bool
#define MUTEX_INIT() true
#define MUTEX_LOCK(x) \
  do {                \
    (x) = false;      \
  } while (0)
#define MUTEX_UNLOCK(x) \
  do {                  \
    (x) = true;         \
  } while (0)
#endif /* USE_FREERTOS_CMSIS */
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_SYSTEM_HPP_ */
