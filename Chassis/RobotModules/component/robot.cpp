/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现（安全最外层 → 三态迁移 → 三模式速度生成 → 下发）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：状态机/急停/三模式/C2G 遥测
 *******************************************************************************
 * @attention :
 *  - 1 ms 周期入口，运行在 TIM6 中断上下文：禁止阻塞、禁止动态内存。
 *  - 安全红线（硬性验收）：急停/遥控离线/双板离线/IMU 未就绪在 update() 开头判，
 *    非 Working 一律 Chassis::stop() 零输出；任何模式分支都不能绕过这段代码。
 *  - 本文件负责“读硬件/单例并接线”，纯逻辑（状态迁移/模式/速度生成）在 robot.hpp，
 *    便于主机单测（见交付自测报告）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>

#include "chassis.hpp"
#include "chassis_ik_solver.hpp"
#include "config.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "ins_comm.hpp"
#include "tick.hpp"

/* Private constants ---------------------------------------------------------*/
namespace
{
/* 4 轮索引（与 config.hpp 注释一致：LF, LB, RB, RF） */
constexpr int kWheelNum = config::kWheelNum;

/**
 * Ozone 调试镜像（volatile，Live Watch 直接添加下列表达式即可观察）：
 *   g_dbg_life_state   0=Dead 1=Resurrection 2=Working
 *   g_dbg_mode         0=分离 1=跟随 2=小陀螺
 *   g_dbg_source       0=None 1=Remote
 *   g_dbg_safe_ok      遥控在线且无急停
 *   g_dbg_remote_online / g_dbg_estop / g_dbg_imu_ready / g_dbg_peer_online
 *   g_dbg_delta        δ（rad，G2C 关节角换算）
 *   g_dbg_follow_w     跟随环输出 w（rad/s）
 *   g_dbg_target_vx/vy/w  本拍下发到 Chassis 的云台系目标速度
 *   g_dbg_yaw_target / g_dbg_pitch_target  C2G 里的云台目标角（v1.1）
 */
volatile uint8_t g_dbg_life_state = 0;
volatile uint8_t g_dbg_mode = 0;
volatile uint8_t g_dbg_source = 0;
volatile uint8_t g_dbg_safe_ok = 0;
volatile uint8_t g_dbg_remote_online = 0;
volatile uint8_t g_dbg_estop = 0;
volatile uint8_t g_dbg_imu_ready = 0;
volatile uint8_t g_dbg_peer_online = 0;
volatile float g_dbg_delta = 0.0f;
volatile float g_dbg_follow_w = 0.0f;
volatile float g_dbg_target_vx = 0.0f;
volatile float g_dbg_target_vy = 0.0f;
volatile float g_dbg_target_w = 0.0f;
volatile float g_dbg_yaw_target = 0.0f;
volatile float g_dbg_pitch_target = 0.0f;
}  // namespace

/* Private function definitions ----------------------------------------------*/
namespace
{
/* 把角度限幅到 [min, max]（非有限值按 min 处理，安全侧） */
float ClampAngle(float value, float min, float max)
{
  if (!std::isfinite(value)) {
    return min;
  }
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void robot::Robot::init()
{
  /* ---- 1. 从 Instance 层取齐单例并注册（接线层职责，MainTaskInit 调用一次）---- */
  registerChassis(GetChassis());
  registerImu(GetImu());
  registerComm(GetGimbalChassisComm());
  registerRemote(GetRemoteControl());

  /* ---- 2. 底盘模块依赖注入：4 轮电机 + 4 轮速度环 PID + 逆解器 ---- */
  if (chassis_ != nullptr) {
    hello_world::motor::Motor *wheels[kWheelNum] = {nullptr, nullptr, nullptr,
                                                    nullptr};
    hello_world::pid::Pid *wheel_pids[kWheelNum] = {nullptr, nullptr, nullptr,
                                                    nullptr};
    for (int i = 0; i < kWheelNum; ++i) {
      wheels[i] = GetMotorWheel(i);
      wheel_pids[i] = GetPidMotorWheel(i);
    }
    chassis_->init(wheels, wheel_pids, GetChassisIkSolver());
  }

  /* ---- 3. 跟随环 PID（输入 δ rad，输出自转 w rad/s；死区 kFollowDeadBandRad）---- */
  hello_world::pid::Pid::Config follow_cfg;
  follow_cfg.kp = config::kPidFollowKp;
  follow_cfg.ki = config::kPidFollowKi;
  follow_cfg.kd = config::kPidFollowKd;
  follow_cfg.out_min = -config::kFollowWMax;
  follow_cfg.out_max = config::kFollowWMax;
  follow_cfg.integ_min = -config::kPidFollowIntegMax;
  follow_cfg.integ_max = config::kPidFollowIntegMax;
  follow_cfg.dead_zone = config::kFollowDeadBandRad;
  follow_cfg.d_lpf_alpha = config::kPidFollowDlpfAlpha;
  follow_pid_.init(follow_cfg);  /* init() 内部会 reset() */

  /* ---- 4. 复位状态机/模式/输入缓存与云台目标，并先零输出 ---- */
  sm_.reset();
  mode_sel_.reset(ChassisMode::kSeparate);
  mode_ = ChassisMode::kSeparate;
  source_ = ControlSource::kNone;
  remote_online_ = false;
  estop_ = true;  /* 上电默认按急停处理，直到遥控被解包且无急停 */
  imu_ready_ = false;
  peer_online_ = false;
  delta_ = 0.0f;
  follow_w_ = 0.0f;
  gimbal_yaw_target_ = 0.0f;
  gimbal_pitch_target_ = 0.0f;
  last_update_ms_ = hello_world::tick::GetTickMs();
  if (chassis_ != nullptr) {
    chassis_->stop();
  }
  fillCommTelemetry();
  updateDebugMirrors();
}

void robot::Robot::update()
{
  const uint32_t now = hello_world::tick::GetTickMs();
  last_update_ms_ = now;

  /* ================= 1. 读本拍输入（安全判断依据，最外层） ================= */
  remote_online_ =
      (remote_ != nullptr) && hello_world::remote_control::IsRemoteOnline();
  /* 右拨杆 Down = 急停（契约 §7.1：Mid/Up 为正常）。遥控离线时遥控数据不可信，
   * 保守按“急停”处理；但安全判断同时要求 remote_online_，两者任一不满足都回 Dead。 */
  estop_ = (remote_ == nullptr) ||
           (remote_->rc_r_switch() ==
            hello_world::remote_control::kSwitchStateDown);
  imu_ready_ = (imu_ != nullptr) && imu_->isReady();
  peer_online_ = (comm_ != nullptr) && !comm_->isOffline();
  source_ = (remote_online_ && !estop_) ? ControlSource::kRemote
                                        : ControlSource::kNone;

  const bool safe_ok = remote_online_ && !estop_;
  const bool ready_ok = imu_ready_ && peer_online_;
  const bool sticks_centered =
      (remote_ != nullptr) && SticksCentered(*remote_);

  /* ================= 2. 状态机迁移（唯一入口；安全条件不满足无条件回 Dead） ================= */
  const LifeState prev = sm_.state;
  const LifeState next = sm_.step(safe_ok, ready_ok, sticks_centered, now);
  if (next != prev) {
    onStateTransit(prev, next);
  }

  /* ================= 3. 非 Working：填遥测 → 强制零输出 → 返回 ================= */
  if (next != LifeState::kWorking) {
    follow_w_ = 0.0f;
    delta_ = 0.0f;
    g_dbg_target_vx = 0.0f;
    g_dbg_target_vy = 0.0f;
    g_dbg_target_w = 0.0f;
    if (chassis_ != nullptr) {
      chassis_->stop();  /* Dead/Resurrection：每周期零输出（契约 §7.4 输出归零矩阵） */
    }
    fillCommTelemetry();
    updateDebugMirrors();
    return;
  }

  /* ================= 4. Working：模式 → δ → 跟随环 → 速度生成 → 下发 ================= */
  /* 4.1 模式选择（左拨杆，30 ms 去抖：Up=跟随 / Mid=分离 / Down=小陀螺） */
  mode_ = mode_sel_.step(remote_->rc_l_switch(), now);

  /* 4.2 δ：G2C yaw 关节角为主来源（契约 §1.2/§3.2），wrapPi 后按 config::kDeltaSign 取号 */
  if (comm_ != nullptr) {
    delta_ = robot::wrapPi(static_cast<float>(config::kDeltaSign) *
                           comm_->gimbal_data().yaw_joint);
  } else {
    delta_ = 0.0f;
  }

  /* 4.3 跟随环：Pid::calc(0, δ) 的输出 = −(kp·δ + kd·δ')，取负号得到与 δ 同号的 w：
   *     δ>0 表示云台在车头左侧，底盘需要逆时针（w>0）转去对齐云台 → 减小 δ。 */
  if (mode_ == ChassisMode::kFollow) {
    const float raw = follow_pid_.calc(0.0f, delta_, config::kTs);
    follow_w_ = ClampValue(-raw, config::kFollowWMax);
  } else {
    follow_pid_.reset();
    follow_w_ = 0.0f;
  }

  /* 4.4 C2G v1.1 云台目标角：底盘板按遥控右摇杆积分生成（契约 §3.2 v1.1 扩展）。
   *     - yaw_target：世界系 yaw 目标，wrapPi；进入 Working 时用反馈初始化（见
   *       onStateTransit），保证云台不因上电目标为 0 而甩动。
   *     - pitch_target：绝对值目标，钳在软限位 [kGimbalPitchMin, kGimbalPitchMax]。
   *     - 非 Working 时本段不执行 → 目标冻结（C2G 的 working 位=0，云台板可据此丢弃）。 */
  gimbal_yaw_target_ = robot::wrapPi(
      gimbal_yaw_target_ + ApplyDeadZone(remote_->rc_rh()) *
                               config::kGimbalYawRateMax * config::kTs);
  gimbal_pitch_target_ = ClampAngle(
      gimbal_pitch_target_ + ApplyDeadZone(remote_->rc_rv()) *
                                 config::kGimbalPitchRateMax * config::kTs,
      config::kGimbalPitchMin, config::kGimbalPitchMax);

  /* 4.5 云台系目标速度（vx/vy 来自左摇杆，w 按模式），交给底盘逆解+轮速环 */
  const ChassisVelocity target = GenerateVelocity(*remote_, mode_, follow_w_);
  g_dbg_target_vx = target.vx;
  g_dbg_target_vy = target.vy;
  g_dbg_target_w = target.w;
  if (chassis_ != nullptr) {
    chassis_->setTarget(target, delta_, mode_);
    chassis_->update();
  }

  /* ================= 5. 遥测与调试镜像 ================= */
  fillCommTelemetry();
  updateDebugMirrors();
}

void robot::Robot::onStateTransit(LifeState from, LifeState to)
{
  (void)from;
  if (to == LifeState::kWorking) {
    /* 进入 Working：清跟随环历史、按当前拨杆重置模式去抖、用反馈初始化云台目标角 */
    follow_pid_.reset();
    if (remote_ != nullptr) {
      mode_sel_.reset(DecodeModeFromSwitch(remote_->rc_l_switch()));
    } else {
      mode_sel_.reset(ChassisMode::kSeparate);
    }
    mode_ = mode_sel_.mode;
    initGimbalTargetsFromFeedback();
  } else if (to == LifeState::kDead) {
    /* 回 Dead：清跟随环/模式/控制源；输出由 update() 每周期 Chassis::stop() 归零 */
    follow_pid_.reset();
    mode_sel_.reset(ChassisMode::kSeparate);
    mode_ = ChassisMode::kSeparate;
    source_ = ControlSource::kNone;
    follow_w_ = 0.0f;
    delta_ = 0.0f;
  } else {
    /* Dead → Resurrection：输出仍为零，只需保证跟随环/模式是无历史状态 */
    follow_pid_.reset();
    mode_sel_.reset(ChassisMode::kSeparate);
    mode_ = ChassisMode::kSeparate;
    follow_w_ = 0.0f;
  }
}

void robot::Robot::initGimbalTargetsFromFeedback()
{
  /* 进入 Working 时把云台目标角对齐当前实际角度，避免 C2G 目标跳变导致云台甩头：
   * 云台世界 yaw ≈ 底盘 IMU 世界 yaw + δ（δ 来自 G2C yaw 关节角）。 */
  const float chassis_yaw = (imu_ != nullptr) ? imu_->yaw() : 0.0f;
  const float yaw_joint =
      (comm_ != nullptr) ? comm_->gimbal_data().yaw_joint : 0.0f;
  const float delta =
      robot::wrapPi(static_cast<float>(config::kDeltaSign) * yaw_joint);
  gimbal_yaw_target_ = robot::wrapPi(chassis_yaw + delta);
  gimbal_pitch_target_ =
      ClampAngle((comm_ != nullptr) ? comm_->gimbal_data().pitch_joint : 0.0f,
                 config::kGimbalPitchMin, config::kGimbalPitchMax);
}

void robot::Robot::fillCommTelemetry()
{
  if (comm_ == nullptr) {
    return;
  }
  /* 每 1 ms 刷新 C2G 语义字段；实际发送在 CommTask（Task 层）里 encode + CAN 发送。
   * online 由 GimbalChassisComm::encode() 恒置 1，这里保留字段同步即可。 */
  GimbalChassisComm::ChassisData &d = comm_->chassis_data();
  d.online = true;
  d.life_state = sm_.state;
  d.estop = estop_;
  d.imu_calibrated = imu_ready_;
  d.remote_online = remote_online_;
  d.mode = mode_;
  d.source = source_;
  /* v1.1：C2G b2-3/b4-5 由“保留填 0”升级为云台目标角（契约 §3.2 v1.1） */
  d.gimbal_yaw_target = gimbal_yaw_target_;
  d.gimbal_pitch_target = gimbal_pitch_target_;
  d.chassis_imu_yaw =
      (imu_ != nullptr) ? robot::wrapPi(imu_->yaw()) : 0.0f;
}

void robot::Robot::updateDebugMirrors() const
{
  g_dbg_life_state = static_cast<uint8_t>(sm_.state);
  g_dbg_mode = static_cast<uint8_t>(mode_);
  g_dbg_source = static_cast<uint8_t>(source_);
  g_dbg_safe_ok =
      static_cast<uint8_t>(remote_online_ && !estop_);
  g_dbg_remote_online = static_cast<uint8_t>(remote_online_);
  g_dbg_estop = static_cast<uint8_t>(estop_);
  g_dbg_imu_ready = static_cast<uint8_t>(imu_ready_);
  g_dbg_peer_online = static_cast<uint8_t>(peer_online_);
  g_dbg_delta = delta_;
  g_dbg_follow_w = follow_w_;
  g_dbg_yaw_target = gimbal_yaw_target_;
  g_dbg_pitch_target = gimbal_pitch_target_;
}
