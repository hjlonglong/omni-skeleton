/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     :底盘板顶层状态机（生命状态 + 控制源 + 底盘模式 + 指令生成）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：三态状态机、急停最外层、
 *                                                三模式速度生成、C2G 遥测填充
 *******************************************************************************
 * @attention :
 *  - 状态机/模式/速度生成的【纯逻辑】用 inline 函数与普通 struct 写在本头文件里，
 *    不依赖 HAL/外设，可在主机上直接单元测试（交付自测即复用这些函数）。
 *  - 硬件相关的读取（DT7/IMU/双板帧）只在 robot.cpp 的 Robot::update() 里完成。
 *  - 安全红线（契约 §7.5）：急停/遥控离线/双板离线/IMU 未就绪必须【最外层】先判，
 *    任何模式分支都不能绕过；非 Working 一律零输出（由 Robot 调用 Chassis::stop()）。
 *  - δ 与逆解公式在 chassis_ik_solver.hpp / chassis.cpp；本文件只生成云台系速度。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

#include "DT7.hpp"
#include "config.hpp"
#include "module_state.hpp"
#include "pid.hpp"

/* Exported types ------------------------------------------------------------*/
namespace hello_world
{
namespace imu
{
class Imu;
}  // namespace imu
}  // namespace hello_world

namespace robot
{
class Chassis;
class GimbalChassisComm;

/* ============================ 纯逻辑（主机可测） ============================ */

/**
 * @brief  把 x 限幅到 ±limit（非有限值按 0 处理）
 */
inline float ClampValue(float x, float limit)
{
  if (!std::isfinite(x)) {
    return 0.0f;
  }
  if (x > limit) {
    return limit;
  }
  if (x < -limit) {
    return -limit;
  }
  return x;
}

/**
 * @brief  摇杆死区（|x| <= config::kStickDeadZone 时输出 0，否则原样输出）
 * @note   只做截断不做重标定，保证死区外的手感/满量程不变。
 */
inline float ApplyDeadZone(float x)
{
  if (!std::isfinite(x)) {
    return 0.0f;
  }
  return (std::fabs(x) <= config::kStickDeadZone) ? 0.0f : x;
}

/**
 * @brief  摇杆是否全部回中（上电/Resurrection 期间进 Working 的前置条件，契约 §7.1）
 */
inline bool SticksCentered(const hello_world::remote_control::DT7 &rc)
{
  return std::fabs(rc.rc_lv()) <= config::kStickCenterTol &&
         std::fabs(rc.rc_lh()) <= config::kStickCenterTol &&
         std::fabs(rc.rc_rv()) <= config::kStickCenterTol &&
         std::fabs(rc.rc_rh()) <= config::kStickCenterTol &&
         std::fabs(rc.rc_wheel()) <= config::kStickCenterTol;
}

/**
 * @brief  遥控器左拨杆 → 底盘模式（契约 §7.1：Up=跟随 / Mid=分离 / Down=小陀螺）
 */
inline ChassisMode DecodeModeFromSwitch(hello_world::remote_control::SwitchState sw)
{
  switch (sw) {
    case hello_world::remote_control::kSwitchStateUp:
      return ChassisMode::kFollow;
    case hello_world::remote_control::kSwitchStateDown:
      return ChassisMode::kSpin;
    default:
      return ChassisMode::kSeparate;
  }
}

/**
 * @brief  拨杆模式选择器（带 config::kModeDebounceMs 去抖）
 * @note   纯状态机、无 HAL 依赖；boot 时用 reset() 进入 kSeparate。
 */
struct ModeSelector {
  ChassisMode mode = ChassisMode::kSeparate;
  ChassisMode candidate = ChassisMode::kSeparate;
  uint32_t candidate_since_ms = 0;
  bool timing = false;

  void reset(ChassisMode initial = ChassisMode::kSeparate)
  {
    mode = initial;
    candidate = initial;
    candidate_since_ms = 0;
    timing = false;
  }

  /**
   * @brief 每周期调用：拨杆值稳定 kModeDebounceMs 后才切换模式
   * @param sw       当前左拨杆值（DT7::rc_l_switch()）
   * @param now_ms   当前 tick（ms）
   * @retval 去抖后的当前模式
   */
  ChassisMode step(hello_world::remote_control::SwitchState sw, uint32_t now_ms)
  {
    const ChassisMode want = DecodeModeFromSwitch(sw);
    if (want == mode) {
      timing = false;
      candidate = mode;
      return mode;
    }
    if (!timing || want != candidate) {
      timing = true;
      candidate = want;
      candidate_since_ms = now_ms;
    }
    if (now_ms - candidate_since_ms >= config::kModeDebounceMs) {
      mode = candidate;
      timing = false;
    }
    return mode;
  }
};

/**
 * @brief  生命状态三态机（Dead → Resurrection → Working，任一前置条件失效立即 Dead）
 * @note   - safe_ok      = 遥控在线 && 无急停（Dead→Resurrection 的 100 ms 去抖条件）
 *         - ready_ok     = IMU 已就绪 && 双板在线（Resurrection→Working 的 200 ms 保持条件）
 *         - sticks_centered 仅在 Resurrection→Working 期间参与判定；
 *           Working 态允许推杆（否则一动杆就会掉回 Dead）。
 *         所有时间比较用 uint32 相减，回绕安全。
 */
struct LifeStateMachine {
  LifeState state = LifeState::kDead;
  bool safe_timing = false;
  uint32_t safe_since_ms = 0;
  bool ready_timing = false;
  uint32_t ready_since_ms = 0;

  void reset()
  {
    state = LifeState::kDead;
    safe_timing = false;
    ready_timing = false;
  }

  /**
   * @brief  推进一步状态机
   * @param safe_ok          遥控在线且无急停
   * @param ready_ok         IMU 已标定且双板在线
   * @param sticks_centered  摇杆是否回中（仅进 Working 前检查）
   * @param now_ms           当前 tick（ms）
   * @retval 迁移后的状态
   */
  LifeState step(bool safe_ok, bool ready_ok, bool sticks_centered, uint32_t now_ms)
  {
    /* 安全最外层：只要 safe 不成立，无条件回 Dead 并取消所有计时 */
    if (!safe_ok) {
      reset();
      return state;
    }

    switch (state) {
      case LifeState::kDead:
        if (!safe_timing) {
          safe_timing = true;
          safe_since_ms = now_ms;
        }
        if (now_ms - safe_since_ms >= config::kStateDebounceMs) {
          state = LifeState::kResurrection;
          ready_timing = false;
        }
        break;

      case LifeState::kResurrection: {
        const bool can_work = ready_ok && sticks_centered;
        if (can_work) {
          if (!ready_timing) {
            ready_timing = true;
            ready_since_ms = now_ms;
          }
          if (now_ms - ready_since_ms >= config::kResurrectionHoldMs) {
            state = LifeState::kWorking;
          }
        } else {
          ready_timing = false;
        }
        break;
      }

      case LifeState::kWorking:
        /* Working 期间失条件（双板离线 / IMU 未就绪 / 急停 / 遥控离线）立即回 Dead */
        if (!ready_ok) {
          reset();
        }
        break;
    }
    return state;
  }
};

/**
 * @brief  遥控输入 → 云台系底盘速度指令（契约 §7.1/§7.2）
 * @param rc        已解码的 DT7 数据
 * @param mode      当前底盘模式
 * @param follow_w  跟随环本拍输出（kFollow 模式使用，其余模式忽略）
 * @retval 云台系 (vx, vy, w)；所有分量已限幅（kMaxVxVy / kMaxW / kSpinWMax / kFollowWMax）
 * @note   vx：左摇杆上推为正（前进）；vy：左摇杆左推为正（左移）；
 *         分离/小陀螺的 w 来自拨轮，跟随的 w 来自跟随环。
 */
inline ChassisVelocity GenerateVelocity(const hello_world::remote_control::DT7 &rc,
                                        ChassisMode mode, float follow_w)
{
  ChassisVelocity v;
  v.vx = ClampValue(ApplyDeadZone(rc.rc_lv()) * config::kMaxVxVy, config::kMaxVxVy);
  v.vy = ClampValue(ApplyDeadZone(rc.rc_lh()) * config::kMaxVxVy, config::kMaxVxVy);

  const float wheel = ApplyDeadZone(rc.rc_wheel());
  switch (mode) {
    case ChassisMode::kFollow:
      v.w = ClampValue(follow_w, config::kFollowWMax);
      break;
    case ChassisMode::kSpin:
      v.w = ClampValue(wheel * config::kSpinWMax, config::kSpinWMax);
      break;
    case ChassisMode::kSeparate:
    default:
      v.w = ClampValue(wheel * config::kMaxW, config::kMaxW);
      break;
  }
  return v;
}

/* ============================ 顶层状态机类 ============================ */

/**
 * @brief 底盘板顶层状态机：1 ms 周期入口，汇总数据 → 迁移 → 生成指令 → 下发
 */
class Robot
{
 public:
  Robot() = default;

  /* ---- 模块注册（Robot::init 内部完成接线；也可由测试/其它接线层显式调用）---- */
  void registerChassis(Chassis *chassis) { chassis_ = chassis; }
  void registerImu(hello_world::imu::Imu *imu) { imu_ = imu; }
  void registerComm(GimbalChassisComm *comm) { comm_ = comm; }
  void registerRemote(hello_world::remote_control::DT7 *remote) { remote_ = remote; }

  /**
   * @brief  初始化：从 Instance 层取齐各单例并注册，复位状态机/PID/目标
   * @retval None
   * @note   必须在 TIM6 中断启动前（MainTaskInit）调用；本函数会零输出并 reset PID。
   */
  void init();

  /**
   * @brief  1 ms 周期入口（由 MainTask 调用）
   * @retval None
   * @note  顺序：读输入 → 状态机迁移 → 填 C2G 遥测 → Working 生成速度并下发 /
   *        非 Working 强制 stop()。任何分支都不允许绕过最外层的安全判断。
   */
  void update();

  /* ---- 状态查询（调试/Ozone/遥测填充用）---- */
  LifeState lifeState() const { return sm_.state; }
  ChassisMode mode() const { return mode_; }
  ControlSource source() const { return source_; }
  bool estop() const { return estop_; }
  bool remoteOnline() const { return remote_online_; }

 private:
  void onStateTransit(LifeState from, LifeState to);
  void fillCommTelemetry();
  void initGimbalTargetsFromFeedback();
  void updateDebugMirrors() const;

  /* ---- 模块指针（register* 注入）---- */
  Chassis *chassis_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::remote_control::DT7 *remote_ = nullptr;

  /* ---- 状态机 ---- */
  LifeStateMachine sm_;
  ModeSelector mode_sel_;
  hello_world::pid::Pid follow_pid_;

  /* ---- 本拍输入/输出缓存（Ozone 可观测）---- */
  ChassisMode mode_ = ChassisMode::kSeparate;
  ControlSource source_ = ControlSource::kNone;
  bool remote_online_ = false;
  bool estop_ = true;
  bool imu_ready_ = false;
  bool peer_online_ = false;
  float delta_ = 0.0f;
  float follow_w_ = 0.0f;
  float gimbal_yaw_target_ = 0.0f;
  float gimbal_pitch_target_ = 0.0f;
  uint32_t last_update_ms_ = 0;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
