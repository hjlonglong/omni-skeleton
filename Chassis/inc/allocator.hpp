/**
 *******************************************************************************
 * @file      : allocator.hpp
 * @brief     : 实现内存分配器，程序中所有的内存分配都通过该类进行，对于类请继承
 *              MemMgr 类，以实现内存分配与释放
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *  当前版本使用 malloc 与 free 进行内存分配与释放，后续若引入操作系统，则需要更换内存
 *  申请与释放函数
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_
#define HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdlib>
#include <memory>
#include <utility>

#include "construct.hpp"
#include "system.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
template <typename T>
class Allocator : public std::allocator<T>
{
 public:
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  template <typename U>
  struct rebind {
    using other = Allocator<U>;
  };

  Allocator(void) = default;
  Allocator(const Allocator&) = default;
  Allocator& operator=(const Allocator&) = default;
  Allocator(Allocator&&) = default;
  Allocator& operator=(Allocator&&) = default;

  template <typename U>
  Allocator(const Allocator<U>&)
  {
  }

  ~Allocator(void) = default;

  static pointer address(reference x)
  {
    return &x;
  }

  static const_pointer address(const_reference x)
  {
    return &x;
  }

  static pointer allocate(size_type n, const void* hint = 0)
  {
    return static_cast<pointer>(malloc(n * sizeof(T)));
  }

  static void deallocate(pointer p, size_type n)
  {
    free(p);
  }

  static size_type max_size(void)
  {
    return size_type(-1) / sizeof(T);
  }

  template <typename U, typename... Args>
  static void construct(U* p, Args&&... args)
  {
    Construct(p, std::forward<Args>(args)...);
  }

  template <typename U>
  static void destroy(U* p)
  {
    Destroy(p);
  }
};

template <>
class Allocator<void> : public std::allocator<void>
{
 public:
  using value_type = void;
  using pointer = void*;
  using const_pointer = const void*;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  template <typename U>
  struct rebind {
    using other = Allocator<U>;
  };

  Allocator(void) = default;
  Allocator(const Allocator&) = default;
  Allocator& operator=(const Allocator&) = default;
  Allocator(Allocator&&) = default;
  Allocator& operator=(Allocator&&) = default;

  template <typename U>
  Allocator(const Allocator<U>&)
  {
  }

  ~Allocator(void) = default;

  static pointer allocate(size_type n, const void* hint = 0)
  {
    return static_cast<pointer>(malloc(n));
  }

  static void deallocate(pointer p, size_type n)
  {
    free(p);
  }

  template <typename U, typename... Args>
  static void construct(U* p, Args&&... args)
  {
    Construct(p, std::forward<Args>(args)...);
  }

  template <typename U>
  static void destroy(U* p)
  {
    Destroy(p);
  }
};

/** 用于重载类的内存申请，所有可能需要内存申请的类需要为该类的子类 */
class MemMgr
{
 public:
  void* operator new(size_t size) { return Allocator<void>::allocate(size); }
  void* operator new[](size_t size) { return Allocator<void>::allocate(size); }
  void operator delete(void* ptr) { Allocator<void>::deallocate(ptr, 0); }
  void operator delete[](void* ptr) { Allocator<void>::deallocate(ptr, 0); }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_ */
