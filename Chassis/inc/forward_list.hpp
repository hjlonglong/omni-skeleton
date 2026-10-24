/**
 *******************************************************************************
 * @file      : forward_list.hpp
 * @brief     : 简单单向链表
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-20      Caikunzhen      1. 完成正式版本
 *******************************************************************************
 * @attention : 仿照STL实现的一个简单的单向链表，因此部分命名与命名规范与 STL 保持一致
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_FORWARD_LIST_HPP_
#define HW_COMPONENTS_TOOLS_FORWARD_LIST_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "allocator.hpp"
#include "system.hpp"

namespace hello_world
{
namespace tools
{
namespace forward_list_impl
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
template <typename T>
struct forward_list_node_base;
template <typename T>
struct forward_list_node;

template <typename T>
struct node_traits {
  typedef forward_list_node_base<T>* base_ptr;
  typedef forward_list_node<T>* node_ptr;
};

template <typename T>
struct forward_list_node_base : public MemMgr {
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;

  base_ptr next_ = nullptr;

  forward_list_node_base(void) = default;
  virtual ~forward_list_node_base(void) { next_ = nullptr; }

  node_ptr as_node(void) { return static_cast<node_ptr>(this); }
};

template <typename T>
struct forward_list_node : public forward_list_node_base<T> {
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;

  T value_;

  forward_list_node(void) = default;
  forward_list_node(const T& v) : value_(v) {}
  forward_list_node(T&& v) : value_(std::move(v)) {}
  template <typename... Args>
  forward_list_node(Args&&... args) : value_(std::forward<Args>(args)...)
  {
  }

  virtual ~forward_list_node(void) = default;

  base_ptr as_base(void) { return static_cast<base_ptr>(this); }
};

template <typename T>
struct forward_list_iterator : public MemMgr {
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef forward_list_iterator<T> self;

  base_ptr node_ = nullptr;

  forward_list_iterator(void) = default;
  forward_list_iterator(base_ptr node) : node_(node) {}
  forward_list_iterator(node_ptr node) : node_(node->as_base()) {}
  forward_list_iterator(const forward_list_iterator& rhs)
      : node_(rhs.node_) {}
  forward_list_iterator& operator=(const forward_list_iterator& rhs)
  {
    node_ = rhs.node_;
    return *this;
  }

  virtual ~forward_list_iterator(void) { node_ = nullptr; }

  reference operator*(void) const { return node_->as_node()->value_; }

  pointer operator->(void) const { return &(operator*()); }

  self& operator++(void)
  {
    HW_ASSERT(node_ != nullptr, "Invalid iterator");
    node_ = node_->next_;
    return *this;
  }

  self operator++(int)
  {
    self tmp = *this;
    ++*this;
    return tmp;
  }

  bool operator==(const self& rhs) const { return node_ == rhs.node_; }

  bool operator!=(const self& rhs) const { return node_ != rhs.node_; }
};

template <typename T>
struct forward_list_const_iterator : public MemMgr {
  typedef T value_type;
  typedef const T* pointer;
  typedef const T& reference;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef forward_list_const_iterator<T> self;

  base_ptr node_ = nullptr;

  forward_list_const_iterator(void) = default;
  forward_list_const_iterator(base_ptr node) : node_(node) {}
  forward_list_const_iterator(node_ptr node) : node_(node->as_base()) {}
  forward_list_const_iterator(const forward_list_iterator<T>& rhs)
      : node_(rhs.node_) {}
  forward_list_const_iterator(const forward_list_const_iterator& rhs)
      : node_(rhs.node_) {}
  forward_list_const_iterator& operator=(const forward_list_const_iterator& rhs)
  {
    node_ = rhs.node_;
    return *this;
  }

  virtual ~forward_list_const_iterator(void) { node_ = nullptr; }

  reference operator*(void) const { return node_->as_node()->value_; }

  pointer operator->(void) const { return &(operator*()); }

  self& operator++(void)
  {
    HW_ASSERT(node_ != nullptr, "Invalid iterator");
    node_ = node_->next_;
    return *this;
  }

  self operator++(int)
  {
    self tmp = *this;
    ++*this;
    return tmp;
  }

  bool operator==(const self& rhs) const { return node_ == rhs.node_; }

  bool operator!=(const self& rhs) const { return node_ != rhs.node_; }
};

/** 被放置于 hello_world::tools 命名空间下 */
template <typename T>
class forward_list : public MemMgr
{
 public:
  typedef T value_type;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef forward_list_iterator<T> iterator;
  typedef forward_list_const_iterator<T> const_iterator;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef forward_list_node<T> node_t;
  typedef forward_list_node_base<T> node_base_t;
  typedef size_t size_type;

  forward_list(void) { before_head_ = new node_base_t(); }
  forward_list(const forward_list& other);
  forward_list(forward_list&& other)
  {
    before_head_ = new node_base_t();
    before_head_->next_ = other.before_head_->next_;
    other.before_head_->next_ = nullptr;
  }
  forward_list& operator=(const forward_list& other);
  forward_list& operator=(forward_list&& other)
  {
    if (&other != this) {
      clear();
      before_head_->next_ = other.before_head_->next_;
      other.before_head_->next_ = nullptr;
    }

    return *this;
  }

  forward_list(std::initializer_list<T> init);
  forward_list& operator=(std::initializer_list<T> nit);

  template <typename Iter>
  forward_list(Iter first, Iter last);

  ~forward_list(void)
  {
    clear();
    delete before_head_;
  }

  /**
   * @brief       判断两个 forward_list 是否为同一个对象
   * @param       rhs: forward_list 对象
   * @retval       是返回 true，否则返回 false
   * @note        None
   */
  bool operator==(const forward_list& rhs) const
  {
    if (&rhs == this) {
      return true;
    }

    return false;
  }

  reference front(void) { return *begin(); }

  const_reference front(void) const { return *begin(); }

  iterator before_begin(void) { return iterator(before_head_); }

  const_iterator before_begin(void) const
  {
    return const_iterator(before_head_);
  }

  const_iterator cbefore_begin(void) const
  {
    return const_iterator(before_head_);
  }

  iterator begin(void) { return iterator(before_head_->next_); }

  const_iterator begin(void) const
  {
    return const_iterator(before_head_->next_);
  }

  const_iterator cbegin(void) const
  {
    return const_iterator(before_head_->next_);
  }

  iterator end(void) { return iterator(static_cast<base_ptr>(nullptr)); }

  const_iterator end(void) const
  {
    return const_iterator(static_cast<base_ptr>(nullptr));
  }

  const_iterator cend(void) const
  {
    return const_iterator(static_cast<base_ptr>(nullptr));
  }

  bool empty(void) const { return before_head_->next_ == nullptr; }

  void clear(void);

  iterator erase_after(const_iterator pos);

  void push_front(const T& value)
  {
    node_ptr new_node = new node_t(value);
    new_node->next_ = before_head_->next_;
    before_head_->next_ = new_node->as_base();
  }

  void push_front(T&& value)
  {
    node_ptr new_node = new node_t(std::move(value));
    new_node->next_ = before_head_->next_;
    before_head_->next_ = new_node->as_base();
  }

  template <typename... Args>
  void emplace_front(Args&&... args)
  {
    node_ptr new_node = new node_t(std::forward<Args>(args)...);
    new_node->next_ = before_head_->next_;
    before_head_->next_ = new_node->as_base();
  }

  void pop_front(void)
  {
    if (before_head_->next_ != nullptr) {
      base_ptr node = before_head_->next_;
      before_head_->next_ = node->next_;
      delete node->as_node();
    }
  }

  void resize(size_type count);

 private:
  base_ptr before_head_;  ///< 头节点的前一个节点
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

template <typename T>
forward_list<T>::forward_list(const forward_list& other)
{
  before_head_ = new node_base_t();
  base_ptr tmp_node = before_head_;
  for (base_ptr node = other.before_head_->next_; node != nullptr;
       node = node->next) {
    tmp_node->next_ = new node_t(node->as_node()->value_);
    tmp_node = tmp_node->next_;
  }
}

template <typename T>
forward_list<T>& forward_list<T>::operator=(const forward_list& other)
{
  if (&other != this) {
    clear();
    base_ptr tmp_node = before_head_;
    for (base_ptr node = other.before_head_->next_; node != nullptr;
         node = node->next_) {
      tmp_node->next_ = new node_t(node->as_node()->value_);
      tmp_node = tmp_node->next_;
    }
  }
  return *this;
}

template <typename T>
forward_list<T>::forward_list(std::initializer_list<T> init)
{
  before_head_ = new node_base_t();
  base_ptr tmp_node = before_head_;
  for (auto iter = init.begin(); iter != init.end(); ++iter) {
    tmp_node->next_ = new node_t(*iter);
    tmp_node = tmp_node->next_;
  }
}

template <typename T>
forward_list<T>& forward_list<T>::operator=(std::initializer_list<T> init)
{
  clear();
  base_ptr tmp_node = before_head_;
  for (auto iter = init.begin(); iter != init.end(); ++iter) {
    tmp_node->next_ = new node_t(*iter);
    tmp_node = tmp_node->next_;
  }
  return *this;
}

template <typename T>
template <typename Iter>
forward_list<T>::forward_list(Iter first, Iter last)
{
  before_head_ = new node_base_t();
  base_ptr tmp_node = before_head_;
  for (auto iter = first; iter != last; ++iter) {
    tmp_node->next_ = new node_t(*iter);
    tmp_node = tmp_node->next_;
  }
}

template <typename T>
void forward_list<T>::clear(void)
{
  for (base_ptr node = before_head_->next_; node != nullptr;) {
    base_ptr next = node->next_;
    delete node->as_node();
    node = next;
  }
  before_head_->next_ = nullptr;
}

template <typename T>
forward_list_iterator<T> forward_list<T>::erase_after(const_iterator pos)
{
  for (base_ptr node = before_head_; node != nullptr; node = node->next_) {
    if (node == pos.node_) {
      base_ptr next = node->next_;
      if (next != nullptr) {
        node->next_ = next->next_;
        delete next->as_node();
      }
      return iterator(node->next_);
    }
  }
  return iterator(static_cast<base_ptr>(nullptr));
}

template <typename T>
void forward_list<T>::resize(size_type count)
{
  base_ptr node = before_head_;
  size_type i = 0;
  for (; i < count && node->next_ != nullptr; ++i) {
    node = node->next_;
  }

  if (i < count) {
    for (; i < count; ++i) {
      node->next_ = new node_t();
      node = node->next_;
    }
  } else {
    while (node->next_ != nullptr) {
      base_ptr next = node->next_;
      node->next_ = next->next_;
      delete next->as_node();
    }
  }
}
HW_OPTIMIZE_O2_END
}  // namespace forward_list_impl
using forward_list_impl::forward_list;
}  // namespace tools
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_TINY_FORWARD_LIST_HPP_ */
