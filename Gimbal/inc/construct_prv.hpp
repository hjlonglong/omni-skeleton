/**
 *******************************************************************************
 * @file      : construct_prv.hpp
 * @brief     : 用于根据通过内存分配器分配的对象情况调用对应的析构函数
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该文件为内部文件，不应在外部使用
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_
#define HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_

/* Includes ------------------------------------------------------------------*/
#include <type_traits>

#include "system.hpp"

namespace hello_world
{
namespace prv
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

HW_OPTIMIZE_O2_START
/**
 * @brief       销毁一个平凡类型对象
 * @retval       None
 * @note        None
 */
template <typename T>
void DestroyOne(T*, std::true_type)
{
}

/**
 * @brief       销毁一个非平凡类型对象
 * @param        pointer: 对象指针
 * @retval       None
 * @note        None
 */
template <typename T>
void DestroyOne(T* pointer, std::false_type)
{
  if (pointer != nullptr) {
    pointer->~T();
  }
}

/**
 * @brief       销毁一个范围内的平凡类型对象
 * @retval       None
 * @note        None
 */
template <typename T>
void DestroyRange(T*, T*, std::true_type)
{
}

/**
 * @brief       销毁一个范围内的非平凡类型对象
 * @param        first: 起始指针
 * @param        last: 结束指针
 * @retval       None
 * @note        None
 */
template <typename T>
void DestroyRange(T* first, T* last, std::false_type)
{
  for (; first != last; ++first) {
    Destroy(first);
  }
}
HW_OPTIMIZE_O2_END
}  // namespace prv
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_ */
