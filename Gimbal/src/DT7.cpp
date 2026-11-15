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
  /* 拨轮（2026-11-15 第二次实车复核：两段编码基不同）：
   *  - 顺时针：v16 可到 8051（13 位段），按 13 位字段展开（模 8192）；
   *  - 逆时针：v16 先降到 0、再回到 2047 段继续降到 1684（11 位回绕，不是 0→8191），
   *    按 13 位判回绕会得到 +660。
   * 因此按 v16 分段选择原始值，并用“最小幅度回绕判定”自动识别两种基：
   *   raw = (v16 > 2048) ? (v16 & 0x1FFF) : (v16 & 0x07FF)；
   *   差分 |d| > 1024 时，在 d、d±2048、d±8192 中取绝对值最小者作为真实增量。
   * 中位磁滞锁存只在 v16 <= 2048 段进行（顺时针深行程 v16 > 2048 不参与，避免误判回中）。
   * 展开值钳位 [-1387, +7027]（两侧实测行程）。
   * 输出：顺时针 v16 在 [1024, 2048] 线性 0→1、> 2048 恒 1（用户指定分段）；
   *       逆时针 = 展开值 / 1387（0 → -1）。 */
  static constexpr int32_t kWheelSpanPos = 7027;    ///< 顺时针到底 +7027 counts（8051-1024，实测）
  static constexpr int32_t kWheelSpanNeg = 1387;    ///< 逆时针到底 -1387 counts（1024+363，实测，11 位回绕）
  static constexpr int32_t kWheelDetentIn = 25;     ///< 中位锁存进入阈值（|raw-1024|，counts）
  static constexpr int32_t kWheelDetentOut = 140;   ///< 中位锁存离开阈值（counts，对应 rc≈0.10 < 死区 0.20）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;  /* 未截断 16 位原值（诊断） */
  const int32_t raw = (v16 > 2048u) ? static_cast<int32_t>(v16 & 0x1FFFu)
                                    : static_cast<int32_t>(v16 & 0x07FFu);
  g_dbg_rc_wheel_raw = static_cast<uint16_t>(raw);
  int32_t d = 0;
  if (!wheel_inited_) {
    wheel_raw13_ = static_cast<uint16_t>(raw);
    wheel_unwrapped_ = 0;
    {
      const int32_t err = raw - 1024;
      wheel_in_detent_ = ((err >= 0 ? err : -err) <= kWheelDetentOut);
    }
    wheel_inited_ = true;
  } else {
    d = raw - static_cast<int32_t>(wheel_raw13_);
    if (d > 1024 || d < -1024) {
      /* 两种回绕基（2048 / 8192）统一按“最小幅度”解释，避免方向反转。 */
      int32_t best = d;
      const int32_t candidates[4] = {d - 2048, d + 2048, d - 8192, d + 8192};
      for (const int32_t c : candidates) {
        if ((c < 0 ? -c : c) < (best < 0 ? -best : best)) {
          best = c;
        }
      }
      d = best;
    }
    wheel_raw13_ = static_cast<uint16_t>(raw);
    wheel_unwrapped_ += d;
    /* 中位磁滞锁存：拨轮带弹簧回中，回中过程中会在中位附近振荡；
     * 只有真正靠近中位（|raw-1024|<=25）才锁 0，离开 140 counts 以上才重新锚定。 */
    if (v16 <= 2048u) {
      const int32_t err = raw - 1024;  /* 相对中位 */
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
  /* 输出编码（2026-11-15 用户指定分段）。 */
  if (v16 > 2048u || wheel_unwrapped_ > 0) {
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
