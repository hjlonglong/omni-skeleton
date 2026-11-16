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
  /* 拨轮解码（2026-11-16 第四次实车复核：重叠带速度预测消歧，替换 13 位连续展开版）：
   * 实测编码（同一电位器、两段映射）：
   *   顺时针：raw = v16（13 位段），s = raw - 1024，行程 +7027 counts（到 8051）；
   *   逆时针：raw∈[1684,2047] 时 s = raw - 3072，行程 -1387 counts（到 1685 → -1387）。
   *   其余 raw∈[0,1684) 一律 s = raw - 1024，计数连续。
   * 重叠带 raw∈[1684,2047] 有两种解释（A 段正小 / C 段负大），按上一帧增量预测
   * target = wheel_s_ + wheel_pred_ 选更近者；快速甩动（采样仅 2~3 帧、整程跨越）
   * 用“上一帧几乎不动却落在 C 深段”的兜底判为整程反向。
   * 中位硬复位：|raw-1024|<=25 时 s=0（弹簧回中后不残留偏移）。
   * 输出：正 = min(1, s/1024)；负 = max(-1, s/1387)。
   * 背景：13 位连续展开版在“顺时针到底→迅速逆时针到底”时，重叠带 raw=1684/1685
   * 被当成 A 段解释（s=+660），展开值停在 +661（rc≈+0.65，同向），表现为快速回拨
   * 仍同向转；缓慢回拨因经过中位复位而正常。本版用速度预测 + 整程兜底修正。 */
  static constexpr int32_t kWheelSpanPos = 7027;   ///< 顺时针到底 +7027 counts（8051-1024，实测）
  static constexpr int32_t kWheelSpanNeg = 1387;   ///< 逆时针到底 -1387 counts（实测）
  static constexpr int32_t kWheelDetentIn = 25;    ///< 中位硬复位阈值（|raw-1024|，counts）
  static constexpr int32_t kWheelPredMax = 4000;   ///< 单帧增量预测钳位（counts）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;  /* 未截断 16 位原值（诊断） */
  const int32_t raw = static_cast<int32_t>(v16 & 0x1FFFu);
  g_dbg_rc_wheel_raw = static_cast<uint16_t>(raw);
  if (!wheel_inited_) {
    wheel_s_ = raw - 1024;
    if (wheel_s_ > kWheelSpanPos) {
      wheel_s_ = kWheelSpanPos;
    } else if (wheel_s_ < -kWheelSpanNeg) {
      wheel_s_ = -kWheelSpanNeg;
    }
    wheel_pred_ = 0;
    wheel_inited_ = true;
  } else {
    int32_t s;
    if (raw >= 1684 && raw <= 2047) {
      const int32_t sa = raw - 1024; /* A 段解释：正小（+661~+1023） */
      const int32_t sc = raw - 3072; /* C 段解释：负大（-1388~-1025） */
      const int32_t target = wheel_s_ + wheel_pred_;
      const int32_t da = sa - target;
      const int32_t dc = sc - target;
      s = ((da >= 0 ? da : -da) <= (dc >= 0 ? dc : -dc)) ? sa : sc;
      /* 单帧整程兜底：上一帧几乎不动（|pred|<300）却落在 C 深段且此前在正大位移，
       * 视为“从顺时针到底快速甩到逆时针到底”的整程跨越，取 C 段解释。 */
      if (wheel_pred_ < 300 && wheel_pred_ > -300 && wheel_s_ > 3000 && raw <= 1740) {
        s = sc;
      }
    } else {
      s = raw - 1024;
    }
    if (s > kWheelSpanPos) {
      s = kWheelSpanPos;
    } else if (s < -kWheelSpanNeg) {
      s = -kWheelSpanNeg;
    }
    int32_t d = s - wheel_s_;
    const int32_t err_mid = raw - 1024;
    if ((err_mid >= 0 ? err_mid : -err_mid) <= kWheelDetentIn) {
      s = 0;
      d = s - wheel_s_;
    }
    if (d > kWheelPredMax) {
      d = kWheelPredMax;
    } else if (d < -kWheelPredMax) {
      d = -kWheelPredMax;
    }
    wheel_pred_ = d;
    wheel_s_ = s;
  }
  if (wheel_s_ >= 0) {
    float t = static_cast<float>(wheel_s_) / 1024.0f;
    if (t > 1.0f) {
      t = 1.0f;
    }
    rc_wheel_ = t;
  } else {
    float t = static_cast<float>(wheel_s_) / static_cast<float>(kWheelSpanNeg);
    if (t < -1.0f) {
      t = -1.0f;
    }
    rc_wheel_ = t;
  }
  g_dbg_rc_wheel_unwrapped = wheel_s_;
  g_dbg_rc_wheel_raw_log[g_dbg_rc_wheel_log_idx] = static_cast<uint16_t>(raw);
  g_dbg_rc_wheel_unwrapped_log[g_dbg_rc_wheel_log_idx] = wheel_s_;
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
