/**
 *******************************************************************************
 * @file      : DT7.cpp
 * @brief     : 遥控器 DT7 解码类实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：独立解码实现
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"

namespace hello_world {
namespace remote_control {

/* Private constants ---------------------------------------------------------*/

static const uint16_t kRcOffset = 1024u;
static const float kRcRatio = 1.0f / 660;

/**
 * Ozone 诊断镜像（C 链接全局符号）：拨轮通道的 11 位原始值（映射前，中位 = 1024）。
 * 诊断“满量程反向”时看它在各位移下的单调性：若拨到底时该值回绕到 <1024（甚至 0），
 * 则问题在接收机/帧（信号或通道压缩），不在本文件映射；否则是映射/符号问题。
 */
extern "C" {
__attribute__((used)) volatile uint16_t g_dbg_rc_wheel_raw = 1024u;
__attribute__((used)) volatile uint16_t g_dbg_rc_wheel_v16 = 1024u;
__attribute__((used)) volatile int32_t g_dbg_rc_wheel_unwrapped = 0;
/* 拨轮回中异常诊断：每帧记录 11 位原值与展开值（32 深环形缓冲，写指针 g_dbg_rc_wheel_log_idx）。 */
#define kRcWheelLogLen 32u
__attribute__((used)) volatile uint16_t g_dbg_rc_wheel_raw_log[kRcWheelLogLen] = {0};
__attribute__((used)) volatile int32_t g_dbg_rc_wheel_unwrapped_log[kRcWheelLogLen] = {0};
__attribute__((used)) volatile uint8_t g_dbg_rc_wheel_log_idx = 0;
}

/* Exported function definitions ---------------------------------------------*/

void DT7::decode(const uint8_t rx_data[kRcRxDataLen])
{
  uint16_t tmp;

  tmp = (rx_data[0] | (rx_data[1] << 8)) & 0x07FF;
  rc_rh_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((rx_data[1] >> 3) | (rx_data[2] << 5)) & 0x07FF;
  rc_rv_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((rx_data[2] >> 6) | (rx_data[3] << 2) | (rx_data[4] << 10)) & 0x07FF;
  rc_lh_ = (tmp - kRcOffset) * kRcRatio;
  tmp = ((rx_data[4] >> 1) | (rx_data[5] << 7)) & 0x07FF;
  rc_lv_ = (tmp - kRcOffset) * kRcRatio;
  /* 拨轮：该接收机 ch4 是 **11 位字段**（v16 的低 11 位），但电位器机械行程比 11 位量程大，
   * 2026-11-15 实测（raw16/raw17）：中位 1024；顺时针到底 1907（+883）；
   * 逆时针到底 1684 —— 是“从中位向下穿过 0 回绕”后到达的：展开位移
   * = -(1024 + (2047-1684)) = -1387，而不是 1684-1024=+660。
   * 之前按 13 位（模 8192）判回绕，0→2047 的跳变被当成 +2047，于是逆时针方向反掉，
   * 且幅度 660/7531=0.088 又落进拨轮死区 → 表现为“逆时针不转”。
   * 这里改为 11 位（模 2048、半量程 1024）增量展开，再按实测两侧行程归一化：
   * 顺时针 +883 → +1.0，逆时针 -1387 → -1.0。 */
  static constexpr int32_t kWheelMod = 2048;        ///< 11 位字段模数
  static constexpr int32_t kWheelWrapHalf = 1024;   ///< 回绕判定阈值（11 位半量程）
  static constexpr int32_t kWheelSpanPos = 883;     ///< 顺时针到底 +883 counts（实测）
  static constexpr int32_t kWheelSpanNeg = 1387;    ///< 逆时针到底 -1387 counts（实测，经 2048 回绕）
  static constexpr int32_t kWheelDetentIn = 25;     ///< 中位锁存进入阈值（|raw-1024|，counts）
  static constexpr int32_t kWheelDetentOut = 140;   ///< 中位锁存离开阈值（counts，对应 rc≈0.16 < 死区 0.20）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;  /* 未截断 16 位原值（诊断） */
  const int32_t wheel11 = static_cast<int32_t>(v16 & 0x07FFu);  /* 11 位 ch4 */
  g_dbg_rc_wheel_raw = static_cast<uint16_t>(wheel11);
  int32_t d = 0;
  if (!wheel_inited_) {
    wheel_raw13_ = static_cast<uint16_t>(wheel11);
    wheel_unwrapped_ = 0;
    {
      const int32_t err = wheel11 - 1024;
      wheel_in_detent_ = ((err >= 0 ? err : -err) <= kWheelDetentOut);
    }
    wheel_inited_ = true;
  } else {
    d = wheel11 - static_cast<int32_t>(wheel_raw13_);
    if (d > kWheelWrapHalf) {
      d -= kWheelMod;  /* 11 位正向回绕（0 → 2047 方向） */
    } else if (d < -kWheelWrapHalf) {
      d += kWheelMod;  /* 11 位负向回绕（2047 → 0 方向） */
    }
    wheel_raw13_ = static_cast<uint16_t>(wheel11);
    wheel_unwrapped_ += d;
    /* 中位磁滞锁存：拨轮带弹簧回中，回中过程中会在中位附近振荡；
     * 硬复位会在窗口边缘抖动时反复“归零再上升”（2026-11-15 实车：顺时针侧出现三次）。
     * 改为回差锁存：|raw-中位| <= 25 进入中位（展开值锁 0），> 140 才离开中位
     * （140/883≈0.16，仍落在拨轮死区 0.20 内，不会产生可见转动）。 */
    {
      const int32_t err = wheel11 - 1024;  /* 相对 11 位中位 */
      const int32_t err_abs = (err >= 0) ? err : -err;
      if (wheel_in_detent_) {
        if (err_abs > kWheelDetentOut) {
          wheel_in_detent_ = false;
          wheel_unwrapped_ = err;  /* 离开中位时以绝对偏差重新锚定 */
        } else {
          wheel_unwrapped_ = 0;
        }
      } else if (err_abs <= kWheelDetentIn) {
        wheel_in_detent_ = true;
        wheel_unwrapped_ = 0;
      }
    }
    if (wheel_unwrapped_ > kWheelSpanPos) {
      wheel_unwrapped_ = kWheelSpanPos;
    } else if (wheel_unwrapped_ < -kWheelSpanNeg) {
      wheel_unwrapped_ = -kWheelSpanNeg;
    }
  }
  rc_wheel_ = (wheel_unwrapped_ >= 0)
                  ? static_cast<float>(wheel_unwrapped_) / static_cast<float>(kWheelSpanPos)
                  : static_cast<float>(wheel_unwrapped_) / static_cast<float>(kWheelSpanNeg);
  g_dbg_rc_wheel_unwrapped = wheel_unwrapped_;
  g_dbg_rc_wheel_raw_log[g_dbg_rc_wheel_log_idx] = static_cast<uint16_t>(wheel11);
  g_dbg_rc_wheel_unwrapped_log[g_dbg_rc_wheel_log_idx] = wheel_unwrapped_;
  g_dbg_rc_wheel_log_idx =
      static_cast<uint8_t>((g_dbg_rc_wheel_log_idx + 1u) % kRcWheelLogLen);

  /* 拨杆位域（DBUS 第 5 字节高半字节）：bit4-5 = s2，bit6-7 = s1。
   * 2026-11-11 实物复核（用户）：物理**左**拨杆对应 bit6-7、右拨杆对应 bit4-5，
   * 与 11-08 依据官方 sbus_to_rc() 的判断相反；现象是"左拨杆下 = 急停"（急停读的是
   * rc_r_switch）。以实车为准：s1(左) = bits6-7、s2(右) = bits4-5。 */
  rc_l_switch_ = SwitchState(((rx_data[5] >> 4) & 0x000C) >> 2);
  rc_r_switch_ = SwitchState((rx_data[5] >> 4) & 0x0003);

  mouse_x_ = rx_data[6] | (rx_data[7] << 8);
  mouse_y_ = rx_data[8] | (rx_data[9] << 8);
  mouse_z_ = rx_data[10] | (rx_data[11] << 8);
  mouse_l_btn_ = rx_data[12];
  mouse_r_btn_ = rx_data[13];

  key_.data = rx_data[14] | (rx_data[15] << 8);
}

}  // namespace remote_control
}  // namespace hello_world
