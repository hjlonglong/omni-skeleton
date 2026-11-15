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
  /* 拨轮：该接收机实际为 13 位编码（2026-11-15 实测 raw[17] 最大 31）。
   * 中位 = 1024=0x0400；顺时针到底 = 0x1F73=8051（+7027 counts）；
   * 逆时针到底 = 0x0694=1684，即 (1024-7531) mod 8192（跨 0 回绕）。
   * 旧实现 &0x07FF 只保留 11 位，8000+ 的值会截成 0~2047 的锯齿，
   * 于是“拨到高段”被算成负值 → 满量程反向。这里改为 13 位增量展开。 */
  static constexpr int32_t kWheelSpanPos = 7027;  ///< 顺时针到底 counts（实测，TODO(硬件) 复核）
  static constexpr int32_t kWheelSpanNeg = 7531;  ///< 逆时针到底 counts（实测，TODO(硬件) 复核）
  static constexpr int32_t kWheelWrapHalf = 4096;  ///< 13 位半量程（回绕判定阈值）

  const uint16_t v16 = static_cast<uint16_t>(rx_data[16] | (rx_data[17] << 8));
  g_dbg_rc_wheel_v16 = v16;                    /* 未截断 16 位原值（诊断） */
  const uint16_t v13 = static_cast<uint16_t>(v16 & 0x1FFFu);  /* 13 位编码值 */
  g_dbg_rc_wheel_raw = v13;                    /* 13 位原始值（诊断） */
  if (!wheel_inited_) {
    wheel_raw13_ = v13;
    wheel_unwrapped_ = 0;
    wheel_inited_ = true;
  } else {
    int32_t d = static_cast<int32_t>(v13) - static_cast<int32_t>(wheel_raw13_);
    if (d > kWheelWrapHalf) {
      d -= 8192;  /* 13 位正向回绕 */
    } else if (d < -kWheelWrapHalf) {
      d += 8192;  /* 13 位负向回绕 */
    }
    wheel_raw13_ = v13;
    wheel_unwrapped_ += d;
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
