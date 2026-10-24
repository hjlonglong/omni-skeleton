/**
 *******************************************************************************
 * @file      : construct.hpp
 * @brief     : 用于构造和销毁通过内存分配器分配的对象
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  该文件偏向内部使用，需要进行对象的构造与销毁时请使用 allocator.hpp 中提供的封装接
 *  口
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_
#define HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <new>
#include <type_traits>
#include <utility>

#include "construct_prv.hpp"
#include "system.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

HW_OPTIMIZE_O2_START
template <typename T>
void Construct(T* ptr)
{
  ::new (ptr) T();
}

template <typename T1, typename T2>
void Construct(T1* ptr, const T2& value)
{
  ::new (ptr) T1(value);
}

template <typename T1, typename T2>
void Construct(T1* ptr, T2&& value)
{
  ::new (ptr) T1(std::forward<T2>(value));
}

template <typename T, typename... Args>
void Construct(T* ptr, Args&&... args)
{
  ::new (ptr) T(std::forward<Args>(args)...);
}

template <typename T>
void Destroy(T* pointer)
{
  prv::DestroyOne(pointer, std::is_trivially_destructible<T>());
}

template <typename T>
void Destroy(T* first, T* last)
{
  prv::DestroyRange(first, last, std::is_trivially_destructible<T>());
}
HW_OPTIMIZE_O2_END
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_ */
