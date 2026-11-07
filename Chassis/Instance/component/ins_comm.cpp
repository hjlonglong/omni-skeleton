/**
 *******************************************************************************
 * @file      :ins_comm.cpp
 * @brief     :遥控器解码实例 + 遥控在线状态（DBUS 18 字节接收链路的 Instance 层）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      Lead            1. 返回函数内 static DT7 实例
 *  V1.1.0      2026-11-07      Agent B         1. WP-09/17：补 NotifyRemoteRx()/
 *                                                IsRemoteOnline() 供 Robot 离线判定
 *******************************************************************************
 * @attention :
 *  - DT7 只负责 18 字节解码；串口收发（USART3 空闲 + DMA）在
 *    Task/component/comm_task.cpp 的 HAL_UARTEx_RxEventCallback 中完成。
 *  - 收到第一帧之前 IsRemoteOnline() 恒为 false：上电/断线期间状态机不会误判
 *    “遥控在线”而进入 Working（安全默认）。
 *  - 全部函数运行在 1 ms 中断/主循环上下文，只做计数器读写，无阻塞。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"

#include "config.hpp"
#include "tick.hpp"

/* Private types -------------------------------------------------------------*/
namespace
{
/**
 * @brief 遥控接收状态（函数内 static 构造，避免全局构造函数早于 HAL/tick 初始化）
 */
struct RemoteRxState {
  hello_world::remote_control::DT7 remote{};
  uint32_t last_rx_tick_ms = 0u;  ///< 最近一次成功解包时刻（GetTickMs）
  bool ever_received = false;     ///< 是否收到过至少一帧（未收到前判离线，安全默认）
};

RemoteRxState &State()
{
  static RemoteRxState state;
  return state;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl()
{
  return &State().remote;
}

void hello_world::remote_control::NotifyRemoteRx()
{
  RemoteRxState &state = State();
  state.last_rx_tick_ms = hello_world::tick::GetTickMs();
  state.ever_received = true;
}

bool hello_world::remote_control::IsRemoteOnline()
{
  const RemoteRxState &state = State();
  if (!state.ever_received) {
    return false;  /* 上电后尚未收到任何有效 DBUS 帧：按离线处理 */
  }
  const uint32_t now = hello_world::tick::GetTickMs();
  int64_t diff =
      static_cast<int64_t>(now) - static_cast<int64_t>(state.last_rx_tick_ms);
  if (diff < 0) {
    diff += 0x100000000LL;  /* uint32 tick 回绕（2^32 ms ≈ 49.7 天） */
  }
  return diff <= static_cast<int64_t>(config::kRemoteOfflineThresMs);
}

/* Private function definitions ----------------------------------------------*/
