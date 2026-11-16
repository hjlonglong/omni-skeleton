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
  /* 拨轮（2026-11-16 第三次实车复核：连续 13 位展开 + 中位硬复位，替换分段选基版）：
   * 实测编码：中位 1024；顺时针到底 8051（13 位段）；逆时针先降到 0、再在 2047 段
   * 继续降到 1684（11 位回绕）。两段在 0~8191 内都是连续计数，故统一按 13 位原值
   * `raw = v16 & 0x1FFF` 展开（v16<=2047 时该掩码等于 v16，无副作用）；差分
   * |d|>1024 时在 d、d±2048、d±8192 中取绝对值最小者。
   * 关键：旧版按 `v16>2048 ? 13位 : 11位` 分段取原始值，v16 恰为 2048 时 11 位
   * 掩码得到 0，造成 ~2048 counts 跳变丢失；顺时针到底后快速回拨会残留正展开值，
   * 表现为“先顺时针到底、再迅速逆时针到底，车轮仍同向转”（缓慢回拨因经过中位
   * 复位而正常）。本版统一 13 位连续值 + 中位硬复位，消除该跳变。
   * 中位复位：|raw-1024|<=25 时展开值清零（弹簧回中后不残留偏移）。
   * 展开值钳位 [-1387, +7027]（两侧实测行程）。
   * 输出归一化：正向 = min(1, u/1024)（前 1024 counts 即满速，深行程保持 1）；
   *             负向 = max(-1, u/1387)。 */
  static constexpr int32_t kWheelSpanPos = 7027;    ///< 顺时针到底 +7027 counts（8051-1024，实测）
  static constexpr int32_t kWheelSpanNeg = 1387;    ///< 逆时针到底 -1387 counts（1024+363，实测，11 位回绕）
  static constexpr int32_t kWheelDetentIn = 25;     ///< 中位硬复位阈值（|raw-1024|，counts）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;  /* 未截断 16 位原值（诊断） */
  const int32_t raw = static_cast<int32_t>(v16 & 0x1FFFu);  /* 13 位连续值 */
  g_dbg_rc_wheel_raw = static_cast<uint16_t>(raw);
  if (!wheel_inited_) {
    wheel_raw13_ = static_cast<uint16_t>(raw);
    wheel_unwrapped_ = 0;
    wheel_inited_ = true;
  } else {
    int32_t d = raw - static_cast<int32_t>(wheel_raw13_);
    /* 回绕只可能发生在 11 位段（两侧均 <=2048）：逆时针跨 0 时 v16 从 0 跳到 2047。
     * 13 位段（任一采样 >2048）计数连续，即使丢帧（增量可 >1024）也按直接差分，
     * 避免把顺时针大步误判成回绕。 */
    if (wheel_raw13_ <= 2048u && raw <= 2048 && (d > 1024 || d < -1024)) {
      const int32_t m = ((d + 1024) % 2048 + 2048) % 2048;
      d = m - 1024;
    }
    wheel_raw13_ = static_cast<uint16_t>(raw);
    wheel_unwrapped_ += d;
    /* 中位硬复位：真正回到中位附近即清零，消除长行程往返的累积误差。 */
    const int32_t err_mid = raw - 1024;
    if ((err_mid >= 0 ? err_mid : -err_mid) <= kWheelDetentIn) {
      wheel_unwrapped_ = 0;
    }
    if (wheel_unwrapped_ > kWheelSpanPos) {
      wheel_unwrapped_ = kWheelSpanPos;
    } else if (wheel_unwrapped_ < -kWheelSpanNeg) {
      wheel_unwrapped_ = -kWheelSpanNeg;
    }
  }
  /* 输出归一化（2026-11-16：正向 1024 counts 满速、负向 1387 counts 满速）。 */
  if (wheel_unwrapped_ >= 0) {
    float t = static_cast<float>(wheel_unwrapped_) / 1024.0f;
    if (t > 1.0f) {
      t = 1.0f;
    }
    rc_wheel_ = t;
  } else {
    float t = static_cast<float>(wheel_unwrapped_) / static_cast<float>(kWheelSpanNeg);
    if (t < -1.0f) {
      t = -1.0f;
    }
    rc_wheel_ = t;
  }
  g_dbg_rc_wheel_unwrapped = wheel_unwrapped_;
  g_dbg_rc_wheel_raw_log[g_dbg_rc_wheel_log_idx] = static_cast<uint16_t>(raw);
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
