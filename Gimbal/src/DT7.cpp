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
 * Ozone 诊断镜像（C 链接全局符号）：拨轮通道的 13 位原始值（映射前，中位 = 1024）。
 * 诊断“满量程反向”时看它在各位移下的单调性：若拨到底时该值回绕到 <1024（甚至 0），
 * 则问题在接收机/帧（信号或通道压缩），不在本文件映射；否则是映射/符号问题。
 */
extern "C" {
__attribute__((used)) volatile uint16_t g_dbg_rc_wheel_raw = 1024u;
__attribute__((used)) volatile uint16_t g_dbg_rc_wheel_v16 = 1024u;
__attribute__((used)) volatile int32_t g_dbg_rc_wheel_unwrapped = 0;
/* 拨轮回中异常诊断：每帧记录 13 位原值与展开值（32 深环形缓冲，写指针 g_dbg_rc_wheel_log_idx）。 */
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
  /* 拨轮：该接收机 ch4 是 **13 位字段**（v16 低 13 位，raw17 高 5 位，实测顺时针到底
   * raw17=31），但 2026-11-15 早期版本误把它当 11 位（`&0x07FF`），导致逆时针跨 0 回绕
   * 后被当成正向 +2047，展开值出现 +660 之类的错误；也曾用 v16&0x07FF 做 11 位展开，
   * 在 v16 每增加 2048 时再次落回中位 1024 附近，触发中位锁存 → rc 在 0/1 间跳动。
   * 现按 13 位字段处理（模 8192、半量程 4096）：
   *   中位 1024；顺时针到底 v13=8051（+7027）；逆时针到底先降到 0、回绕到 8191
   *   再降到 1684（-7531）。两侧行程实测：+7027 / -7531 counts。 */
  static constexpr int32_t kWheelMod = 8192;        ///< 13 位字段模数
  static constexpr int32_t kWheelWrapHalf = 4096;   ///< 回绕判定阈值（13 位半量程）
  static constexpr int32_t kWheelSpanPos = 7027;    ///< 顺时针到底 +7027 counts（8051-1024，实测）
  static constexpr int32_t kWheelSpanNeg = 7531;    ///< 逆时针到底 -7531 counts（1024+6507，实测，经 8192 回绕）
  static constexpr int32_t kWheelDetentIn = 25;     ///< 中位锁存进入阈值（|raw-1024|，counts）
  static constexpr int32_t kWheelDetentOut = 140;   ///< 中位锁存离开阈值（counts，对应 rc≈0.02 < 死区 0.20）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;  /* 未截断 16 位原值（诊断） */
  const int32_t wheel13 = static_cast<int32_t>(v16 & 0x1FFFu);  /* 13 位 ch4 */
  g_dbg_rc_wheel_raw = static_cast<uint16_t>(wheel13);
  int32_t d = 0;
  if (!wheel_inited_) {
    wheel_raw13_ = static_cast<uint16_t>(wheel13);
    wheel_unwrapped_ = 0;
    {
      const int32_t err = wheel13 - 1024;
      wheel_in_detent_ = ((err >= 0 ? err : -err) <= kWheelDetentOut);
    }
    wheel_inited_ = true;
  } else {
    d = wheel13 - static_cast<int32_t>(wheel_raw13_);
    if (d > kWheelWrapHalf) {
      d -= kWheelMod;  /* 13 位正向回绕（0 → 8191 方向） */
    } else if (d < -kWheelWrapHalf) {
      d += kWheelMod;  /* 13 位负向回绕（8191 → 0 方向） */
    }
    wheel_raw13_ = static_cast<uint16_t>(wheel13);
    wheel_unwrapped_ += d;
    /* 中位磁滞锁存：拨轮带弹簧回中，回中过程中会在中位附近振荡；
     * 现在方向判定基于 13 位连续值，只有真正靠近中位（|raw-1024|<=25）才锁 0，
     * 离开中位 140 counts 以上才重新锚定，避免“归零再上升”的抖动。 */
    {
      const int32_t err = wheel13 - 1024;  /* 相对 13 位中位 */
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
  /* 拨轮输出编码（2026-11-15 用户指定分段）：
   * 顺时针（展开值 > 0）：按 16 位原值 v16 分段——v16 在 [1024, 2048] 之间线性加快
   *   （0 → 1），v16 > 2048 后维持满速 1.0（顺时针物理行程到 8051 不再继续增大）；
   * 逆时针：展开值 / kWheelSpanNeg（0 → -1）。
   * 由于方向由 13 位展开值判定，v16 深行程时不会误判到逆时针侧。 */
  if (wheel_unwrapped_ > 0) {
    const int32_t ramp = static_cast<int32_t>(v16) - 1024;
    float t = static_cast<float>(ramp) / 1024.0f;
    if (t < 0.0f) {
      t = 0.0f;
    } else if (t > 1.0f) {
      t = 1.0f;
    }
    rc_wheel_ = t;
  } else {
    rc_wheel_ = static_cast<float>(wheel_unwrapped_) / static_cast<float>(kWheelSpanNeg);
  }
  g_dbg_rc_wheel_unwrapped = wheel_unwrapped_;
  g_dbg_rc_wheel_raw_log[g_dbg_rc_wheel_log_idx] = static_cast<uint16_t>(wheel13);
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
