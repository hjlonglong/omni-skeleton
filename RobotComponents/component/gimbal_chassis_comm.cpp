/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
 * @brief     :底盘↔云台双板 CAN 通讯协议实现（契约 §3，WP-04 冻结）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 按契约 §3.2/§3.3 实现
 *                                                 encode/decode/isOffline
 *******************************************************************************
 * @attention :
 *  - 帧布局/位域/缩放是两板联调契约，修改必须两板同步并更新契约 §3。
 *  - 大端（b2 = 高字节）；角度缩放 1e-3 rad/LSB；往返误差 ≤ 0.0005 rad。
 *  - 本文件被两板共同编译：kCommOfflineThresMs 取自各板自己的 inc/config.hpp。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>
#include <cstdint>

#include "config.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{
constexpr float kAngleScale = 1000.0f;                  ///< 1e-3 rad / LSB
constexpr float kAngleLimitScaled = 3.14f * kAngleScale;  ///< 打包限幅 ±3140
constexpr uint8_t kFlagsOnline = 0x01u;         ///< b0 bit0 在线（本板发送恒 1）
constexpr uint8_t kFlagsWorking = 0x02u;        ///< b0 bit1 工作中
constexpr uint8_t kFlagsEstop = 0x04u;          ///< b0 bit2 软急停
constexpr uint8_t kFlagsImuCalibrated = 0x08u;  ///< b0 bit3 IMU 已标定
constexpr uint8_t kFlagsRemoteOnline = 0x10u;   ///< b0 bit4 遥控在线
constexpr uint8_t kSeqMask = 0x0Fu;             ///< b1 bit4-7 序号掩码
}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace
{
/**
 * @brief  角度打包：wrap 到 [-pi,pi) → ×1000 → 限幅 ±3140 → 四舍五入为 int16
 */
int16_t PackAngle(float rad) {
  float scaled = robot::wrapPi(rad) * kAngleScale;
  if (scaled > kAngleLimitScaled) {
    scaled = kAngleLimitScaled;
  } else if (scaled < -kAngleLimitScaled) {
    scaled = -kAngleLimitScaled;
  }
  return static_cast<int16_t>(lroundf(scaled));
}

/**
 * @brief  大端写入 int16（buf[idx] = 高字节，buf[idx+1] = 低字节）
 */
void WriteInt16BE(uint8_t tx_data[8], uint32_t idx, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  tx_data[idx] = static_cast<uint8_t>(raw >> 8);
  tx_data[idx + 1] = static_cast<uint8_t>(raw & 0xFFu);
}

/**
 * @brief  大端读出 int16
 */
int16_t ReadInt16BE(const uint8_t rx_data[8], uint32_t idx) {
  return static_cast<int16_t>((static_cast<uint16_t>(rx_data[idx]) << 8) |
                              static_cast<uint16_t>(rx_data[idx + 1]));
}

/**
 * @brief  int16 定点角 → float rad
 */
float UnpackAngle(int16_t raw) { return static_cast<float>(raw) / kAngleScale; }

/**
 * @brief  控制源解码（非法值一律视为 kNone）
 */
robot::ControlSource UnpackSource(uint8_t raw) {
  return (raw == static_cast<uint8_t>(robot::ControlSource::kRemote))
             ? robot::ControlSource::kRemote
             : robot::ControlSource::kNone;
}

/**
 * @brief  底盘模式解码（非法值一律视为 kSeparate）
 */
robot::ChassisMode UnpackChassisMode(uint8_t raw) {
  switch (raw) {
    case static_cast<uint8_t>(robot::ChassisMode::kFollow):
      return robot::ChassisMode::kFollow;
    case static_cast<uint8_t>(robot::ChassisMode::kSpin):
      return robot::ChassisMode::kSpin;
    default:
      return robot::ChassisMode::kSeparate;
  }
}

/**
 * @brief  云台模式解码（本版仅 kManual，非法值一律视为 kManual）
 */
robot::GimbalMode UnpackGimbalMode(uint8_t raw) {
  (void)raw;
  return robot::GimbalMode::kManual;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id) {
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  /* 初始化为当前时刻：上电后需超过离线阈值才判离线（契约 §3.3） */
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  uint8_t flags = kFlagsOnline;  ///< b0 bit0：本板发送时恒为在线

  if (code_part_ == CodePart::Chassis) {
    /* ---------- C2G（0x210）：打包本板 chassis_data_ ---------- */
    ChassisData &d = chassis_data_;

    if (d.life_state == LifeState::kWorking) {
      flags |= kFlagsWorking;
    }
    if (d.estop) {
      flags |= kFlagsEstop;
    }
    if (d.imu_calibrated) {
      flags |= kFlagsImuCalibrated;
    }
    if (d.remote_online) {
      flags |= kFlagsRemoteOnline;
    }
    tx_data[0] = flags;
    tx_data[1] = static_cast<uint8_t>((static_cast<uint8_t>(d.mode) & 0x03u) |
                                      ((static_cast<uint8_t>(d.source) & 0x03u) << 2) |
                                      ((d.tx_seq & kSeqMask) << 4));
    WriteInt16BE(tx_data, 2, PackAngle(d.gimbal_yaw_target));    ///< b2-3 保留字段
    WriteInt16BE(tx_data, 4, PackAngle(d.gimbal_pitch_target));  ///< b4-5 保留字段
    WriteInt16BE(tx_data, 6, PackAngle(d.chassis_imu_yaw));      ///< b6-7
    d.tx_seq = static_cast<uint8_t>((d.tx_seq + 1u) & kSeqMask);
  } else {
    /* ---------- G2C（0x211）：打包本板 gimbal_data_ ---------- */
    GimbalData &d = gimbal_data_;

    if (d.life_state == LifeState::kWorking) {
      flags |= kFlagsWorking;
    }
    if (d.estop) {
      flags |= kFlagsEstop;
    }
    if (d.imu_calibrated) {
      flags |= kFlagsImuCalibrated;
    }
    if (d.remote_online) {
      flags |= kFlagsRemoteOnline;
    }
    tx_data[0] = flags;
    tx_data[1] = static_cast<uint8_t>((static_cast<uint8_t>(d.mode) & 0x03u) |
                                      ((static_cast<uint8_t>(d.source) & 0x03u) << 2) |
                                      ((d.tx_seq & kSeqMask) << 4));
    WriteInt16BE(tx_data, 2, PackAngle(d.yaw_joint));       ///< b2-3 δ 主来源
    WriteInt16BE(tx_data, 4, PackAngle(d.pitch_joint));     ///< b4-5
    WriteInt16BE(tx_data, 6, PackAngle(d.gimbal_imu_yaw));  ///< b6-7
    d.tx_seq = static_cast<uint8_t>((d.tx_seq + 1u) & kSeqMask);
  }
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  const uint8_t flags = rx_data[0];
  const uint8_t b1 = rx_data[1];
  const bool working = (flags & kFlagsWorking) != 0;

  if (code_part_ == CodePart::Chassis) {
    /* ---------- 本板是底盘板 → 收到 G2C，写入 gimbal_data_ ---------- */
    GimbalData &d = gimbal_data_;
    d.online = (flags & kFlagsOnline) != 0;
    d.life_state = working ? LifeState::kWorking : LifeState::kDead;
    d.estop = (flags & kFlagsEstop) != 0;
    d.imu_calibrated = (flags & kFlagsImuCalibrated) != 0;
    d.remote_online = (flags & kFlagsRemoteOnline) != 0;
    d.mode = UnpackGimbalMode(b1 & 0x03u);
    d.source = UnpackSource((b1 >> 2) & 0x03u);
    d.tx_seq = static_cast<uint8_t>((b1 >> 4) & kSeqMask);
    d.yaw_joint = UnpackAngle(ReadInt16BE(rx_data, 2));
    d.pitch_joint = UnpackAngle(ReadInt16BE(rx_data, 4));
    d.gimbal_imu_yaw = UnpackAngle(ReadInt16BE(rx_data, 6));
  } else {
    /* ---------- 本板是云台板 → 收到 C2G，写入 chassis_data_ ---------- */
    ChassisData &d = chassis_data_;
    d.online = (flags & kFlagsOnline) != 0;
    d.life_state = working ? LifeState::kWorking : LifeState::kDead;
    d.estop = (flags & kFlagsEstop) != 0;
    d.imu_calibrated = (flags & kFlagsImuCalibrated) != 0;
    d.remote_online = (flags & kFlagsRemoteOnline) != 0;
    d.mode = UnpackChassisMode(b1 & 0x03u);
    d.source = UnpackSource((b1 >> 2) & 0x03u);
    d.tx_seq = static_cast<uint8_t>((b1 >> 4) & kSeqMask);
    d.gimbal_yaw_target = UnpackAngle(ReadInt16BE(rx_data, 2));
    d.gimbal_pitch_target = UnpackAngle(ReadInt16BE(rx_data, 4));
    d.chassis_imu_yaw = UnpackAngle(ReadInt16BE(rx_data, 6));
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();
}

bool robot::GimbalChassisComm::isOffline() const {
  /* 回绕安全：uint32 相减后按 int64 处理，负值补 0xFFFFFFFF（契约 §3.3） */
  const uint32_t now = hello_world::tick::GetTickMs();
  int64_t diff = static_cast<int64_t>(now) - static_cast<int64_t>(last_rx_tick_);
  if (diff < 0) {
    diff += 0xFFFFFFFFLL;
  }
  return diff > static_cast<int64_t>(config::kCommOfflineThresMs);
}

/* Private function definitions ----------------------------------------------*/
