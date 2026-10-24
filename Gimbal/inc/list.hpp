/**
 *******************************************************************************
 * @file      : list.hpp
 * @brief     : 简单双向链表实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-20      Caikunzhen      1. 完成正式版本
 *******************************************************************************
 * @attention : 仿照STL实现的一个简单的单向链表，因此部分命名与命名规范与STL保持一致
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_LIST_HPP_
#define HW_COMPONENTS_TOOLS_LIST_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "allocator.hpp"
#include "assert.hpp"
#include "system.hpp"

namespace hello_world
{
namespace tools
{
namespace list_impl
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
template <typename T>
struct list_node_base;
template <typename T>
struct list_node;

template <typename T>
struct node_traits {
  typedef list_node_base<T>* base_ptr;
  typedef list_node<T>* node_ptr;
};

template <typename T>
struct list_node_base : public MemMgr {
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;

  base_ptr next_ = nullptr;
  base_ptr prev_ = nullptr;

  list_node_base(void) { unlink(); }

  virtual ~list_node_base(void) { next_ = prev_ = nullptr; }

  node_ptr as_node(void) { return static_cast<node_ptr>(this); }

  void unlink(void) { next_ = prev_ = this; }
};

template <typename T>
struct list_node : public list_node_base<T> {
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;

  T value_ = T();

  list_node(void) = default;
  list_node(const T& v) : list_node_base<T>(), value_(v) {}
  list_node(T&& v) : list_node_base<T>(), value_(std::move(v)) {}
  template <typename... Args>
  list_node(Args&&... args) : value_(std::forward<Args>(args)...)
  {
  }

  virtual ~list_node(void) = default;

  base_ptr as_base(void) const { return static_cast<base_ptr>(this); }
};

template <typename T>
struct list_iterator : public MemMgr {
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef list_iterator<T> self;

  base_ptr node_ = nullptr;

  list_iterator(void) = default;
  list_iterator(base_ptr node) : node_(node) {}
  list_iterator(node_ptr node) : node_(node->as_base()) {}
  list_iterator(const list_iterator& rhs)
      : node_(rhs.node_) {}
  list_iterator& operator=(const list_iterator& rhs)
  {
    node_ = rhs.node_;
    return *this;
  }

  virtual ~list_iterator(void) { node_ = nullptr; }

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

  self& operator--(void)
  {
    HW_ASSERT(node_ != nullptr, "Invalid iterator");
    node_ = node_->prev_;
    return *this;
  }

  self operator--(int)
  {
    self tmp = *this;
    --*this;
    return tmp;
  }

  bool operator==(const self& rhs) const { return node_ == rhs.node_; }

  bool operator!=(const self& rhs) const { return node_ != rhs.node_; }
};

template <typename T>
struct list_const_iterator : public MemMgr {
  typedef T value_type;
  typedef const T* pointer;
  typedef const T& reference;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef list_const_iterator<T> self;

  base_ptr node_ = nullptr;

  list_const_iterator(void) = default;
  list_const_iterator(base_ptr node) : node_(node) {}
  list_const_iterator(node_ptr node) : node_(node->as_base()) {}
  list_const_iterator(const list_iterator<T>& rhs)
      : node_(rhs.node_) {}
  list_const_iterator(const list_const_iterator& rhs)
      : node_(rhs.node_) {}
  list_const_iterator& operator=(const list_const_iterator& rhs)
  {
    node_ = rhs.node_;
    return *this;
  }

  virtual ~list_const_iterator(void) { node_ = nullptr; }

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

  self& operator--(void)
  {
    HW_ASSERT(node_ != nullptr, "Invalid iterator");
    node_ = node_->prev_;
    return *this;
  }

  self operator--(int)
  {
    self tmp = *this;
    --*this;
    return tmp;
  }

  bool operator==(const self& rhs) const { return node_ == rhs.node_; }

  bool operator!=(const self& rhs) const { return node_ != rhs.node_; }
};

/** 被放置于 hello_world::tools 命名空间下 */
template <typename T>
class list : public MemMgr
{
 public:
  typedef T value_type;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef list_iterator<T> iterator;
  typedef list_const_iterator<T> const_iterator;
  typedef typename node_traits<T>::node_ptr node_ptr;
  typedef typename node_traits<T>::base_ptr base_ptr;
  typedef list_node<T> node_t;
  typedef list_node_base<T> node_base_t;
  typedef size_t size_type;

  list(void) = default;
  list(const list& other);
  list(list&& other)
  {
    end_.next_ = other.end_.next_;
    end_.next_->prev_ = &end_;
    end_.prev_ = other.end_.prev_;
    end_.prev_->next_ = &end_;
    size_ = other.size_;
    other.end_.unlink();
  }
  list& operator=(const list& other);
  list& operator=(list&& other)
  {
    if (&other != this) {
      clear();
      end_.next_ = other.end_.next_;
      end_.next_->prev_ = &end_;
      end_.prev_ = other.end_.prev_;
      end_.prev_->next_ = &end_;
      size_ = other.size_;
      other.end_.unlink();
    }

    return *this;
  }

  list(std::initializer_list<T> init);
  list& operator=(std::initializer_list<T> init);

  template <typename Iter>
  list(Iter first, Iter last);

  ~list(void) { clear(); }

  /**
   * @brief       判断两个 list 是否为同一个对象
   * @param       rhs: list 对象
   * @retval       是返回 true，否则返回 false
   * @note        None
   */
  bool operator==(const list& rhs) const
  {
    if (&rhs == this) {
      return true;
    }

    return false;
  }

  reference front(void) { return *begin(); }

  const_reference front(void) const { return *begin(); }

  reference back(void) { return *(--end()); }

  const_reference back(void) const { return *(--end()); }

  iterator begin(void) { return iterator(end_.next_); }

  const_iterator begin(void) const
  {
    return const_iterator(end_.next_);
  }

  const_iterator cbegin(void) const
  {
    return const_iterator(end_.next_);
  }

  iterator end(void) { return iterator(&end_); }

  const_iterator end(void) const
  {
    return const_iterator(const_cast<base_ptr>(&end_));  // 确保通过编译
  }

  const_iterator cend(void) const
  {
    return const_iterator(const_cast<base_ptr>(&end_));  // 确保通过编译
  }

  bool empty(void) const { return size_ == 0; }

  size_type size(void) const { return size_; }

  void clear(void);

  iterator erase(iterator pos) { return erase(const_iterator(pos)); }

  iterator erase(const_iterator pos);

  void push_back(const T& value)
  {
    node_ptr new_node = new node_t(value);
    new_node->next_ = &end_;
    new_node->prev_ = end_.prev_;
    end_.prev_->next_ = new_node;
    end_.prev_ = new_node;
    ++size_;
  }

  void push_back(T&& value)
  {
    node_ptr new_node = new node_t(std::move(value));
    new_node->next_ = &end_;
    new_node->prev_ = end_.prev_;
    end_.prev_->next_ = new_node;
    end_.prev_ = new_node;
    ++size_;
  }

  template <typename... Args>
  void emplace_back(Args&&... args)
  {
    node_ptr new_node = new node_t(std::forward<Args>(args)...);
    new_node->next_ = &end_;
    new_node->prev_ = end_.prev_;
    end_.prev_->next_ = new_node;
    end_.prev_ = new_node;
    ++size_;
  }

  void pop_back(void)
  {
    if (end_.prev_ != &end_) {
      base_ptr node = end_.prev_;
      node->prev_->next_ = &end_;
      end_.prev_ = node->prev_;
      delete node->as_node();
      --size_;
    }
  }

  void push_front(const T& value)
  {
    node_ptr new_node = new node_t(value);
    new_node->next_ = end_.next_;
    new_node->prev_ = &end_;
    end_.next_->prev_ = new_node;
    end_.next_ = new_node;
    ++size_;
  }

  void push_front(T&& value)
  {
    node_ptr new_node = new node_t(std::move(value));
    new_node->next_ = end_.next_;
    new_node->prev_ = &end_;
    end_.next_->prev_ = new_node;
    end_.next_ = new_node;
    ++size_;
  }

  template <typename... Args>
  void emplace_front(Args&&... args)
  {
    node_ptr new_node = new node_t(std::forward<Args>(args)...);
    new_node->next_ = end_.next_;
    new_node->prev_ = &end_;
    end_.next_->prev_ = new_node;
    end_.next_ = new_node;
    ++size_;
  }

  void pop_front(void)
  {
    if (end_.next_ != &end_) {
      base_ptr node = end_.next_;
      end_.next_ = node->next_;
      node->next_->prev_ = &end_;
      delete node->as_node();
      --size_;
    }
  }

  void resize(size_type count);

 private:
  node_base_t end_;         ///< next_ 指向链表头，prev_ 指向链表尾
  size_type size_ = 0;      ///< 链表大小
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

template <typename T>
list<T>::list(const list& other)
{
  for (base_ptr node = other.end_.next_; node != &other.end_;
       node = node->next_) {
    push_back(node->as_node()->value_);
  }
}

template <typename T>
list<T>& list<T>::operator=(const list& other)
{
  if (&other != this) {
    clear();
    for (base_ptr node = other.end_.next_; node != &other.end_;
         node = node->next_) {
      push_back(node->as_node()->value_);
    }
  }
  return *this;
}

template <typename T>
list<T>::list(std::initializer_list<T> init)
{
  for (auto iter = init.begin(); iter != init.end(); ++iter) {
    push_back(*iter);
  }
}

template <typename T>
list<T>& list<T>::operator=(std::initializer_list<T> init)
{
  clear();
  for (auto iter = init.begin(); iter != init.end(); ++iter) {
    push_back(*iter);
  }
  return *this;
}

template <typename T>
template <typename Iter>
list<T>::list(Iter first, Iter last)
{
  for (Iter iter = first; iter != last; ++iter) {
    push_back(*iter);
  }
}

template <typename T>
void list<T>::clear(void)
{
  for (base_ptr node = end_.next_; node != &end_;) {
    base_ptr next = node->next_;
    delete node->as_node();
    node = next;
  }
  end_.unlink();
  size_ = 0;
}

template <typename T>
list_iterator<T> list<T>::erase(const_iterator pos)
{
  for (base_ptr node = end_.next_; node != &end_; node = node->next_) {
    if (node == pos.node_) {
      iterator it(node->next_);
      node->prev_->next_ = node->next_;
      node->next_->prev_ = node->prev_;
      size_--;

      delete node->as_node();
      return it;
    }
  }
  return iterator(&end_);
}

template <typename T>
void list<T>::resize(size_type count)
{
  base_ptr node = &end_;
  size_type i = 0;
  for (; i < count && node->next_ != &end_; ++i) {
    node = node->next_;
  }

  if (i < count) {
    for (; i < count; ++i) {
      node_ptr new_node = new node_t();
      new_node->next_ = &end_;
      new_node->prev_ = end_.prev_;
      end_.prev_->next_ = new_node;
      end_.prev_ = new_node;
    }
  } else {
    while (node->next_ != &end_) {
      base_ptr next = node->next_;
      node->next_ = next->next_;
      next->next_->prev_ = node;
      delete next->as_node();
    }
  }

  size_ = count;
}
HW_OPTIMIZE_O2_END
}  // namespace list_impl
using list_impl::list;
}  // namespace tools
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_TINY_LIST_HPP_ */
