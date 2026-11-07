/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 云台板顶层状态机实现（WP-08/13/16）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. 三态状态机 + C2G 目标消费 + G2C 回填；
 *                                                急停/离线最外层短路归零
 *******************************************************************************
 * @attention :
 *  - 状态迁移：Dead →(安全条件保持 kStateDebounceMs)→ Resurrection
 *             →(自检保持 kResurrectionHoldMs)→ Working
 *             →(任一安全条件丢失)→ Dead（立即）
 *  - 目标只在 Working 生效；Dead/Resurrection 每周期调 Gimbal::stop() 输出零力矩。
 *  - 调试变量在 robot_debug 命名空间（Ozone Live Watch 可见）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include "HW_can.hpp"
#include "config.hpp"
#include "gimbal.hpp"
#include "gimbal_math.hpp"
#include "imu.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_imu.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "ins_robot.hpp"
#include "motor_DaMiao.hpp"
#include "tick.hpp"

namespace robot
{

namespace robot_debug
{
volatile uint8_t life_state = 0;
volatile uint8_t safety_ok = 0;
volatile uint8_t c2g_estop = 0;
volatile uint8_t c2g_online = 0;
volatile uint8_t c2g_remote_online = 0;
volatile float c2g_yaw_target = 0.0f;
volatile float c2g_pitch_target = 0.0f;
volatile float c2g_chassis_imu_yaw = 0.0f;
volatile uint32_t cycle_count = 0;
}  // namespace robot_debug

namespace
{

using hello_world::motor::DaMiao;
using hello_world::motor::Motor;

/**
 * @brief       电机反馈帧新鲜度判断（时间戳回绕安全）
 * @param       last_tick: 最近一次收到反馈时刻的指针（GetTickMs，0 = 从未收到）
 * @param       now_ms: 当前时刻
 * @retval      true = 在 kMotorOfflineThresMs 内收到过反馈
 * @note        用指针而非 volatile 值参数：避免 C++20 对 volatile 限定参数的弃用告警。
 */
bool FeedbackFresh(const volatile uint32_t *last_tick, uint32_t now_ms)
{
  const uint32_t tick = *last_tick;
  if (tick == 0) {
    return false;
  }
  return static_cast<uint32_t>(now_ms - tick) <= config::kMotorOfflineThresMs;
}

/**
 * @brief       DM 电机是否使能且无故障码（0x8~0xE 均为故障）
 * @param       motor: 电机对象（Instance 层保证为 DM_J4310）
 * @retval      true = 已使能且 status_code == kMotorEnabled
 */
bool MotorEnabledNoFault(const Motor *motor)
{
  if (motor == nullptr) {
    return false;
  }
  const auto *dm = static_cast<const DaMiao *>(motor);
  return dm->is_enabled() && dm->status_code() == DaMiao::kMotorEnabled;
}

}  // namespace

void Robot::registerGimbal(Gimbal *gimbal) { gimbal_ = gimbal; }

void Robot::registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }

void Robot::registerComm(GimbalChassisComm *comm) { comm_ = comm; }

void Robot::registerRemote(hello_world::remote_control::DT7 *remote)
{
  remote_ = remote;
}

void Robot::init()
{
  /* 1. 从 Instance 层装配（函数内 static，构造顺序安全，契约 §6.3） */
  registerGimbal(GetGimbal());
  registerImu(GetImu());
  registerComm(GetGimbalChassisComm());
  registerRemote(GetRemoteControl());  // 云台板无 DBUS：恒为 nullptr（遥控经 C2G）
  yaw_motor_ = GetMotorYaw();
  pitch_motor_ = GetMotorPitch();

  /* 2. 云台控制对象装配（DM 电机 + 两轴位置 PID + IMU 保留指针） */
  if (gimbal_ != nullptr && imu_ != nullptr) {
    gimbal_->init(yaw_motor_, pitch_motor_, GetPidMotorYaw(), GetPidMotorPitch(),
                  imu_);
  }

  /* 3. 上电初始状态 Dead，并填一帧 G2C 数据（首周期 CommTask 即可发出） */
  EnterState(LifeState::kDead);
  FillGimbalData();
}

bool Robot::SafetyCheck() const
{
  if (comm_ == nullptr || gimbal_ == nullptr || imu_ == nullptr) {
    return false;
  }
  const auto &cd = comm_->chassis_data();

  /* 急停最高优先级（契约 §7.4：急停/离线判断放最外层） */
  if (cd.estop) {
    return false;
  }
  /* 双板链路：C2G 帧未超时且帧内 online 位有效 */
  if (comm_->isOffline() || !cd.online) {
    return false;
  }
  /* 遥控在线（云台板无 DBUS，经 C2G 帧继承） */
  if (!cd.remote_online) {
    return false;
  }
  /* IMU 零漂标定完成且姿态收敛 */
  if (!imu_->isReady()) {
    return false;
  }
  /* DM 两轴反馈新鲜 + 已使能且无故障 */
  if (!MotorFeedbackOk()) {
    return false;
  }
  if (!MotorsEnabled()) {
    return false;
  }
  return true;
}

bool Robot::MotorFeedbackOk() const
{
  const uint32_t now = hello_world::tick::GetTickMs();
  return FeedbackFresh(&g_can_yaw_motor_last_rx_tick, now) &&
         FeedbackFresh(&g_can_pitch_motor_last_rx_tick, now);
}

bool Robot::MotorsEnabled() const
{
  return MotorEnabledNoFault(yaw_motor_) && MotorEnabledNoFault(pitch_motor_);
}

bool Robot::WorkingReady() const
{
  if (!SafetyCheck()) {
    return false;
  }
  /* Resurrection 自检：IMU 已标定（零漂收敛） */
  if (!imu_->isCalibrated()) {
    return false;
  }
  /* pitch 在软限位内（含裕度），否则判故障，避免顶机械限位堵转 */
  if (gimbal_math::IsPitchOutOfLimit(gimbal_->pitchJoint())) {
    return false;
  }
  return true;
}

void Robot::EnterState(LifeState state)
{
  life_state_ = state;
  state_entry_tick_ = hello_world::tick::GetTickMs();
  robot_debug::life_state = static_cast<uint8_t>(state);
  candidate_valid_ = false;

  if (state == LifeState::kResurrection && gimbal_ != nullptr &&
      comm_ != nullptr) {
    /* 进入自检态：目标初始化到当前关节角（yaw 用等价世界 yaw 表示），
     * 避免 Resurrection→Working 首个周期目标突变。 */
    const float chassis_yaw = comm_->chassis_data().chassis_imu_yaw;
    gimbal_->setChassisYaw(chassis_yaw);
    gimbal_->setYawTarget(chassis_yaw + gimbal_->yawJoint());
    gimbal_->setPitchTarget(gimbal_->pitchJoint());
  }
}

void Robot::update()
{
  robot_debug::cycle_count = robot_debug::cycle_count + 1;
  if (comm_ == nullptr || gimbal_ == nullptr || imu_ == nullptr) {
    return;
  }

  const uint32_t now = hello_world::tick::GetTickMs();
  const auto &cd = comm_->chassis_data();

  /* 调试镜像（Ozone Live Watch） */
  robot_debug::c2g_estop = cd.estop ? 1u : 0u;
  robot_debug::c2g_online = cd.online ? 1u : 0u;
  robot_debug::c2g_remote_online = cd.remote_online ? 1u : 0u;
  robot_debug::c2g_yaw_target = cd.gimbal_yaw_target;
  robot_debug::c2g_pitch_target = cd.gimbal_pitch_target;
  robot_debug::c2g_chassis_imu_yaw = cd.chassis_imu_yaw;

  /* 1. 最外层安全判断：任一条件不满足 → Dead + 输出归零并返回 */
  if (!SafetyCheck()) {
    robot_debug::safety_ok = 0;
    if (life_state_ != LifeState::kDead) {
      EnterState(LifeState::kDead);
    }
    candidate_valid_ = false;
    gimbal_->stop();
    FillGimbalData();
    return;
  }
  robot_debug::safety_ok = 1;

  /* 2. 三态状态机（此时安全条件已满足） */
  switch (life_state_) {
    case LifeState::kDead:
      /* 安全条件保持 kStateDebounceMs 后进入自检态（去抖） */
      if (!candidate_valid_) {
        candidate_valid_ = true;
        candidate_tick_ = now;
      }
      if ((now - candidate_tick_) >= config::kStateDebounceMs) {
        EnterState(LifeState::kResurrection);
      }
      break;

    case LifeState::kResurrection:
      /* 自检：任一条件丢失立即回 Dead；保持 kResurrectionHoldMs 后进 Working */
      if (!WorkingReady()) {
        EnterState(LifeState::kDead);
      } else if ((now - state_entry_tick_) >= config::kResurrectionHoldMs) {
        EnterState(LifeState::kWorking);
      }
      break;

    case LifeState::kWorking:
      /* Working 期间任一安全条件失效 → 立即 Dead（不等待去抖） */
      if (!WorkingReady()) {
        EnterState(LifeState::kDead);
      }
      break;

    default:
      EnterState(LifeState::kDead);
      break;
  }

  /* 3. 输出：仅 Working 跟踪 C2G 目标；其余状态输出零（DM 仍需每周期发帧） */
  if (life_state_ == LifeState::kWorking) {
    gimbal_->setChassisYaw(cd.chassis_imu_yaw);
    gimbal_->setYawTarget(cd.gimbal_yaw_target);      /* v1.1：期望世界 yaw */
    gimbal_->setPitchTarget(cd.gimbal_pitch_target);  /* v1.1：期望绝对 pitch */
    gimbal_->update();
  } else {
    gimbal_->stop();
  }

  /* 4. 回填 G2C 帧数据（CommTask 本周期 encode 后经 CAN2 发出） */
  FillGimbalData();
}

void Robot::FillGimbalData()
{
  if (comm_ == nullptr || gimbal_ == nullptr || imu_ == nullptr) {
    return;
  }
  const auto &cd = comm_->chassis_data();
  auto &gd = comm_->gimbal_data();
  gd.online = true;  // encode 恒发 1（对板在线标志）
  gd.life_state = life_state_;
  gd.estop = cd.estop;  // 本板无本地急停按钮：镜像 C2G 软急停状态
  gd.imu_calibrated = imu_->isCalibrated();
  gd.remote_online = cd.remote_online;  // 本板无 DBUS：遥控在线经 C2G 继承
  gd.mode = GimbalMode::kManual;        // 本版云台只有手动模式
  gd.source = cd.source;                // 控制源继承 C2G（契约 §3.2）
  gd.yaw_joint = gimbal_->yawJoint();   // encode 内 PackAngle wrap 到 [-π,π)
  gd.pitch_joint = gimbal_->pitchJoint();
  gd.gimbal_imu_yaw = imu_->yaw();
}

}  // namespace robot
