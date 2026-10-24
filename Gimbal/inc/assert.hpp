/**
 *******************************************************************************
 * @file      : assert.hpp
 * @brief     : 用于实现自定义的断言功能
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  默认关闭自定义断言功能，若需要使用请在项目 CMakelist.txt 中添加
 *  USE_CUSTOM_DEFINE_ASSERT=1 的定义，启用后当断言失败时会进入死循环（关闭中断），若
 *  有进一步需求，如输出错误信息等，请自行修改 Loop 函数
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_ASSERT_HPP_
#define HW_COMPONENTS_TOOLS_ASSERT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "system.hpp"

namespace hello_world
{
namespace prv
{
/* Exported macro ------------------------------------------------------------*/
#ifndef USE_CUSTOM_DEFINE_ASSERT
#define USE_CUSTOM_DEFINE_ASSERT 1  //< 开启自定义 assert
#endif /* USE_CUSTOM_DEFINE_ASSERT */

#if USE_CUSTOM_DEFINE_ASSERT
/**
 * @brief       进行变量检查
 * @param        expr: 待检查表达式
 * @param        format: 格式化字符串
 * @retval       None
 * @note        None
 */
#define HW_ASSERT(expr, format, ...) \
  ((expr) ? (void)0U : hello_world::prv::Loop())
#else
#define HW_ASSERT(expr, format, ...) ((void)0U)
#endif /* USE_CUSTOM_DEFINE_ASSERT */

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       默认死循环函数
 * @retval       None
 * @note        进入死循环，同时关闭一切中断
 */
static inline void Loop(void)
{
  ENTER_CRITICAL();
  while (1) {
  }
}
}  // namespace prv
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_ASSERT_HPP_ */
