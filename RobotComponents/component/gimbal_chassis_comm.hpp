/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     :底盘↔云台双板 CAN 通讯协议（契约 §3，WP-04 冻结）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 按契约 §3 冻结帧字段与类型；
 *                                                 encode/decode/isOffline 一并实现
 *******************************************************************************
 * @attention :
 *  - 帧布局/位域/缩放是两板联调契约，任何修改必须两板同步并更新契约 §3。
 *  - encode() 末尾自动把本方向 tx_seq 自增（0..15 循环），调用方无需维护。
 *  - 解码得到的对板 LifeState 只能区分 Working / 非 Working（帧内只有 working
 *    位），非 Working 一律映射为 kDead（kResurrection 不参与双板判断）。
 *  - 本文件被两板共同编译，禁止包含板级外设头或板级 config。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  /* 底盘→云台（C2G，0x210）帧语义（契约 §3.2）。 */
  struct ChassisData {
    /* ---- b0 flags ---- */
    bool online = false;    ///< bit0 对板在线标志（encode 恒发 1；decode 为收包值）
    LifeState life_state = LifeState::kDead;  ///< bit1 仅传 working（映射见文件头）
    bool estop = false;         ///< bit2 软急停标志
    bool imu_calibrated = false;  ///< bit3 IMU 零漂/姿态是否标定完成
    bool remote_online = false;   ///< bit4 遥控器是否在线
    /* ---- b1 ---- */
    ChassisMode mode = ChassisMode::kSeparate;      ///< bit0-1 底盘工作模式
    ControlSource source = ControlSource::kNone;    ///< bit2-3 控制源
    uint8_t tx_seq = 0;                             ///< bit4-7 发送序号（0..15 循环）
    /* ---- b2-3：本版保留，恒 0 ---- */
    float gimbal_yaw_target = 0.0f;    ///< int16 BE，1e-3 rad（保留字段，填 0）
    /* ---- b4-5：本版保留，恒 0 ---- */
    float gimbal_pitch_target = 0.0f;  ///< int16 BE，1e-3 rad（保留字段，填 0）
    /* ---- b6-7 ---- */
    float chassis_imu_yaw = 0.0f;      ///< int16 BE，1e-3 rad，wrap 后的底盘 IMU yaw
  };

  /* 云台→底盘（G2C，0x211）帧语义（契约 §3.2）。 */
  struct GimbalData {
    /* ---- b0 flags ---- */
    bool online = false;    ///< bit0 对板在线标志（encode 恒发 1；decode 为收包值）
    LifeState life_state = LifeState::kDead;  ///< bit1 仅传 working（映射见文件头）
    bool estop = false;         ///< bit2 软急停标志
    bool imu_calibrated = false;  ///< bit3 IMU 零漂/姿态是否标定完成
    bool remote_online = false;   ///< bit4 遥控器是否在线
    /* ---- b1 ---- */
    GimbalMode mode = GimbalMode::kManual;          ///< bit0-1 云台工作模式
    ControlSource source = ControlSource::kNone;    ///< bit2-3 控制源
    uint8_t tx_seq = 0;                             ///< bit4-7 发送序号（0..15 循环）
    /* ---- b2-3 ---- */
    float yaw_joint = 0.0f;       ///< int16 BE，1e-3 rad，yaw 关节角（δ 的主来源）
    /* ---- b4-5 ---- */
    float pitch_joint = 0.0f;     ///< int16 BE，1e-3 rad，pitch 关节角（含零位偏置）
    /* ---- b6-7 ---- */
    float gimbal_imu_yaw = 0.0f;  ///< int16 BE，1e-3 rad，wrap 后的云台 IMU 世界 yaw
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节（tx_seq 末尾自增）。 */
  void encode(uint8_t tx_data[8]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存（刷新 last_rx_tick_）。 */
  void decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过阈值时长未收到新帧；回绕安全）。 */
  bool isOffline() const;

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

  /* 本板发送/接收 CAN ID（调试打印用；发送由 CommTask 完成）。 */
  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }
  CodePart code_part() const { return code_part_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳（定长 uint32）
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
