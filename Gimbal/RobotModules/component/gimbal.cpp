/**
 *******************************************************************************
 * @file      : gimbal.cpp
 * @brief     : 云台两轴位置控制实现（WP-13）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. 位置环+速度环级联，DM MIT 纯力矩下发；
 *                                                yaw 世界目标→关节目标，pitch 软限位
 *******************************************************************************
 * @attention :
 *  - 控制链：角度误差 →(位置 PID, 输出关节速度，限幅 ±k?RateMax)→ 速度误差
 *    →(速度环, 输出 MIT 力矩，限幅 ±k?TorqueMax)→ setMitInput(0,0,0,0,torq)。
 *  - 位置 PID 参数见 ins_pid.cpp（config kPid?AngleKp/Ki/Kd，输出限幅 ±k?RateMax）；
 *    速度环参数取 config::kPid?VelKp/Ki（Ki 当前为 0，纯 P 阻尼/前馈）。
 *  - yaw 反馈为多圈连续角：先把关节反馈幂等到目标所在 2π 分支再进 PID，
 *    这样 D 项（对测量值微分）仍等于关节角速度，不产生 wrap 冲击。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include <cmath>

#include "config.hpp"
#include "gimbal_math.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace robot
{

namespace gimbal_debug
{
volatile float yaw_world_target = 0.0f;
volatile float yaw_joint_target = 0.0f;
volatile float yaw_joint = 0.0f;
volatile float yaw_torque = 0.0f;
volatile float pitch_target = 0.0f;
volatile float pitch_joint = 0.0f;
volatile float pitch_torque = 0.0f;
volatile float chassis_yaw_rate = 0.0f;
volatile float yaw_vel_ff = 0.0f;
volatile float world_yaw_est = 0.0f;
volatile float yaw_err = 0.0f;
volatile float yaw_vel_target = 0.0f;
volatile float pitch_err = 0.0f;
volatile float pitch_vel_target = 0.0f;
volatile float pitch_vel_ff = 0.0f;
volatile float pitch_gravity_ff = 0.0f;  ///< pitch 重力前馈力矩（N·m，2026-11-17）
volatile uint32_t cycle_count = 0;
}  // namespace gimbal_debug

namespace
{

using hello_world::motor::DaMiao;
using hello_world::motor::Motor;

/**
 * @brief       速度环：速度误差 → MIT 力矩（P + I，I 项带抗饱和限幅）
 * @param       vel_target: 位置环输出的目标关节角速度（rad/s）
 * @param       vel_actual: 关节角速度反馈（rad/s）
 * @param       kp / ki: 速度环增益（config）
 * @param       torque_max: 力矩限幅（N·m）
 * @param       integ: 积分状态（跨周期保持）
 * @retval      限幅后的 MIT 力矩（N·m）
 * @note        config 中 kPid?VelKi 当前为 0，退化为纯 P 阻尼/前馈；保留 I 结构
 *              供实车调参消静差（改动只需调整 config，不动本函数）。
 */
float CalcVelocityLoopTorque(float vel_target, float vel_actual, float kp,
                             float ki, float torque_max, float *integ)
{
  const float err = vel_target - vel_actual;
  *integ = gimbal_math::ClampValue(*integ + ki * err * config::kTs, -torque_max,
                                   torque_max);
  return gimbal_math::ClampValue(kp * err + *integ, -torque_max, torque_max);
}

/**
 * @brief       以 MIT 纯力矩方式下发：kp = kd = 0，pos_des = vel_des = 0
 * @param       motor: 达妙电机（Instance 层保证为 DM_J4310）
 * @param       torque: 关节空间力矩（N·m），符号按电机规定正方向
 * @retval      None
 * @note        达妙 MIT 帧中 kp = kd = 0 时位置/速度项不参与控制，等价纯力矩输入
 *              （契约 §7.3：setMitInput(0,0,0,0,τ)）。
 */
void SendMitTorque(Motor *motor, float torque)
{
  auto *dm = static_cast<DaMiao *>(motor);
  (void)dm->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, torque);
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void Gimbal::init(hello_world::motor::Motor *yaw,
                  hello_world::motor::Motor *pitch,
                  hello_world::pid::Pid *yaw_pid,
                  hello_world::pid::Pid *pitch_pid,
                  hello_world::imu::Imu *imu)
{
  yaw_ = yaw;
  pitch_ = pitch;
  yaw_pid_ = yaw_pid;
  pitch_pid_ = pitch_pid;
  imu_ = imu;  // 保留接口：控制不使用 IMU（位置环用 DM 角度反馈；IMU 就绪由 Robot 判）

  yaw_world_target_ = 0.0f;
  chassis_yaw_ = 0.0f;
  pitch_target_ = 0.0f;
  yaw_vel_integ_ = 0.0f;
  pitch_vel_integ_ = 0.0f;
  chassis_yaw_inited_ = false;  /* 重置底盘 yaw 速率前馈的差分基准 */
  chassis_yaw_rate_ = 0.0f;
  pitch_target_inited_ = false;  /* 重置 pitch 目标速率前馈的差分基准 */
  pitch_vel_ff_ = 0.0f;
  yaw_torque_ = 0.0f;
  pitch_torque_ = 0.0f;

  if (yaw_pid_ != nullptr) {
    yaw_pid_->reset();
  }
  if (pitch_pid_ != nullptr) {
    pitch_pid_->reset();
  }
}

bool Gimbal::IsInited() const
{
  return yaw_ != nullptr && pitch_ != nullptr && yaw_pid_ != nullptr &&
         pitch_pid_ != nullptr;
}

void Gimbal::setYawTarget(float yaw_world)
{
  yaw_world_target_ = yaw_world;
  gimbal_debug::yaw_world_target = yaw_world;
}

void Gimbal::setChassisYaw(float chassis_imu_yaw)
{
  chassis_yaw_ = chassis_imu_yaw;
}

void Gimbal::setPitchTarget(float pitch_joint) { pitch_target_ = pitch_joint; }

void Gimbal::update()
{
  if (!IsInited()) {
    return;
  }
  gimbal_debug::cycle_count = gimbal_debug::cycle_count + 1;

  /* 1. 读关节反馈（CAN 中断里已完成 decode） */
  const float yaw_joint = yaw_->angle();
  const float pitch_joint = pitch_->angle();

  /* 2. yaw：期望世界 yaw − 底盘 IMU yaw → 关节目标（契约 v1.1） */
  const float yaw_joint_target =
      gimbal_math::YawJointTarget(yaw_world_target_, chassis_yaw_);

  /* 2.1 底盘 yaw 速率前馈（2026-11-15）：差分 C2G 底盘 yaw（1e-3 rad 量化）并强低通，
   *     反向叠加到关节速度目标。小陀螺底盘自转 ~3 rad/s 时，纯位置环会有
   *     “角速度/Kp”的稳态滞后；前馈直接给出所需的关节反向速度，云台不再被拖着转。
   *     跟随模式下底盘是“缓慢转动去对齐云台”，前馈负责让云台真正世界固定；量化
   *     差分噪声用 α=0.06（τ≈16 ms）强低通 + 0.08 rad/s 死区抑制，避免低速摆动。 */
  if (!chassis_yaw_inited_) {
    prev_chassis_yaw_ = chassis_yaw_;
    chassis_yaw_inited_ = true;
  }
  const float yaw_rate_raw =
      gimbal_math::WrapPi(chassis_yaw_ - prev_chassis_yaw_) / config::kTs;
  prev_chassis_yaw_ = chassis_yaw_;
  const float yaw_rate_cmd =
      ((yaw_rate_raw > -0.08f) && (yaw_rate_raw < 0.08f)) ? 0.0f : yaw_rate_raw;
  chassis_yaw_rate_ = gimbal_math::ClampValue(
      chassis_yaw_rate_ * (1.0f - 0.06f) + yaw_rate_cmd * 0.06f,
      -yaw_rate_max_, yaw_rate_max_);
  const float yaw_vel_ff = -chassis_yaw_rate_;

  /* 3. yaw 位置环：把关节反馈幂等到目标分支后进 PID（误差 = 最短路径 wrap） */
  const float yaw_err = gimbal_math::WrapPi(yaw_joint_target - yaw_joint);
  const float yaw_fdb_equiv = yaw_joint_target - yaw_err;
  const float yaw_vel_target = gimbal_math::ClampValue(
      yaw_pid_->calc(yaw_joint_target, yaw_fdb_equiv, config::kTs) + yaw_vel_ff,
      -yaw_rate_max_, yaw_rate_max_);

  /* 4. pitch：目标软限位 clamp 后做位置环（pitch 反馈无 wrap） */
  const float pitch_target = gimbal_math::ClampPitchTarget(pitch_target_);

  /* 4.1 pitch 目标角速率前馈（2026-11-15）：目标以速率上限斜坡变化时，纯位置环
   *     稳态滞后 = 目标速率/Kp（1.5/8 ≈ 0.19 rad ≈ 11°），即“跟不上目标”。
   *     差分目标角 + 强低通/死区得到前馈，直接叠加到速度环目标，消除斜坡滞后；
   *     位置环输出限幅 kPitchRateMax 同时由 1.5 提高到 3.0，给瞬态留余量。 */
  if (!pitch_target_inited_) {
    prev_pitch_target_ = pitch_target;
    pitch_target_inited_ = true;
  }
  const float pitch_rate_raw =
      (pitch_target - prev_pitch_target_) / config::kTs;
  prev_pitch_target_ = pitch_target;
  const float pitch_rate_cmd =
      ((pitch_rate_raw > -0.08f) && (pitch_rate_raw < 0.08f)) ? 0.0f
                                                              : pitch_rate_raw;
  pitch_vel_ff_ = gimbal_math::ClampValue(
      pitch_vel_ff_ * (1.0f - 0.06f) + pitch_rate_cmd * 0.06f,
      -config::kPitchRateMax, config::kPitchRateMax);
  const float pitch_vel_target = gimbal_math::ClampValue(
      pitch_pid_->calc(pitch_target, pitch_joint, config::kTs) + pitch_vel_ff_,
      -config::kPitchRateMax, config::kPitchRateMax);

  /* 5. 速度环 → MIT 力矩（力矩限幅在速度环内完成） */
  const float yaw_torque =
      CalcVelocityLoopTorque(yaw_vel_target, yaw_->vel(), config::kPidYawVelKp,
                             config::kPidYawVelKi, config::kYawTorqueMax,
                             &yaw_vel_integ_);
  const float pitch_torque =
      CalcVelocityLoopTorque(pitch_vel_target, pitch_->vel(),
                             config::kPidPitchVelKp, config::kPidPitchVelKi,
                             config::kPitchTorqueMax, &pitch_vel_integ_);

  /* 5.1 pitch 重力前馈（2026-11-17 用户要求“加入重力前馈”）：pitch 轴重心偏置
   *     产生的重力力矩 τ_g(θ) ≈ A·cos(θ)（θ=0 水平最大，±90° 为 0）。纯位置环
   *     靠静差才能积累出力矩平衡重力 → 稳定时“偏一点才停”；直接按模型前馈，
   *     可显著减小静差、抑制缓慢低头/抬头。方向由 kPitchGravityTorqueSign 控制：
   *     实车验证本机构 MIT 正力矩对应 pitch 向下，故取 −1 反向补偿（2026-11-17
   *     用户反馈“方向加反了、更易下垂”后修正）。 */
  const float pitch_gravity_ff = config::kPitchGravityTorqueSign *
                                 config::kPitchGravityTorqueAmp *
                                 std::cos(pitch_joint);
  const float pitch_torque_total = gimbal_math::ClampValue(
      pitch_torque + pitch_gravity_ff, -config::kPitchTorqueMax,
      config::kPitchTorqueMax);

  /* 6. 下发 MIT 纯力矩帧（每轴一帧，CommTask 统一发送 encode 结果） */
  SendMitTorque(yaw_, yaw_torque);
  SendMitTorque(pitch_, pitch_torque_total);

  yaw_torque_ = yaw_torque;
  pitch_torque_ = pitch_torque_total;

  /* 7. 调试观测（Ozone Live Watch） */
  gimbal_debug::yaw_joint_target = yaw_joint_target;
  gimbal_debug::yaw_joint = yaw_joint;
  gimbal_debug::yaw_torque = yaw_torque;
  gimbal_debug::chassis_yaw_rate = chassis_yaw_rate_;
  gimbal_debug::yaw_vel_ff = yaw_vel_ff;
  gimbal_debug::world_yaw_est =
      gimbal_math::WrapPi(chassis_yaw_ + yaw_joint);
  gimbal_debug::yaw_err = yaw_err;
  gimbal_debug::yaw_vel_target = yaw_vel_target;
  gimbal_debug::pitch_err = pitch_target - pitch_joint;
  gimbal_debug::pitch_vel_target = pitch_vel_target;
  gimbal_debug::pitch_vel_ff = pitch_vel_ff_;
  gimbal_debug::pitch_gravity_ff = pitch_gravity_ff;
  gimbal_debug::pitch_target = pitch_target;
  gimbal_debug::pitch_joint = pitch_joint;
  gimbal_debug::pitch_torque = pitch_torque_total;
}

void Gimbal::stop()
{
  if (yaw_ != nullptr) {
    SendMitTorque(yaw_, 0.0f);
  }
  if (pitch_ != nullptr) {
    SendMitTorque(pitch_, 0.0f);
  }
  if (yaw_pid_ != nullptr) {
    yaw_pid_->reset();
  }
  if (pitch_pid_ != nullptr) {
    pitch_pid_->reset();
  }
  yaw_vel_integ_ = 0.0f;
  pitch_vel_integ_ = 0.0f;
  chassis_yaw_inited_ = false;  /* 重置底盘 yaw 速率前馈的差分基准 */
  chassis_yaw_rate_ = 0.0f;
  pitch_target_inited_ = false;  /* 重置 pitch 目标速率前馈的差分基准 */
  pitch_vel_ff_ = 0.0f;
  yaw_torque_ = 0.0f;
  pitch_torque_ = 0.0f;
  gimbal_debug::yaw_torque = 0.0f;
  gimbal_debug::pitch_torque = 0.0f;
  gimbal_debug::pitch_gravity_ff = 0.0f;
}

float Gimbal::yawJoint() const
{
  return (yaw_ != nullptr) ? yaw_->angle() : 0.0f;
}

float Gimbal::pitchJoint() const
{
  return (pitch_ != nullptr) ? pitch_->angle() : 0.0f;
}

}  // namespace robot
