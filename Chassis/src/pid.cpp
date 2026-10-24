/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : PID 控制器（位置式）实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      A1              1. 按《接口契约 v1.0》§6.1 实现
 *  V1.0.1      2026-10-21      A1              1. 统一 d_lpf_alpha 为滤波强度语义（Lead 决策）
 *******************************************************************************
 * @attention :
 *  1. 本文件与 Gimbal 板副本逐字节一致（由同一份源文件拷贝），修改时两板必须
 *     同步更新，否则 md5 将不一致并被 CI 检查发现；
 *  2. 不使用任何 HAL/STM32 头文件，保证可在主机上单测；
 *  3. 实现要点见各步骤注释：输入防御、微分对测量值+一阶低通、死区不污染微分
 *     历史、条件积分抗饱和、积分/输出限幅、位置式输出。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "pid.hpp"

#include <cmath>

/* Private constants --------------------------------------------------------*/
/* Private types ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Exported function definitions --------------------------------------------*/
namespace hello_world
{
namespace pid
{
void Pid::init(const Config& cfg)
{
  cfg_ = cfg;
  reset();
}

void Pid::reset()
{
  integ_ = 0.0f;
  d_filt_ = 0.0f;
  out_ = 0.0f;
  last_fdb_ = 0.0f;
  has_last_fdb_ = false;
}

float Pid::clampOutput(float value) const
{
  /* 仅当上下限有效（max > min）时限幅；默认 0,0 视为不限幅 */
  if (cfg_.out_max > cfg_.out_min) {
    if (value > cfg_.out_max) {
      return cfg_.out_max;
    }
    if (value < cfg_.out_min) {
      return cfg_.out_min;
    }
  }
  return value;
}

float Pid::clampIntegral(float value) const
{
  /* 仅当上下限有效（max > min）时限幅；默认 0,0 视为不限幅 */
  if (cfg_.integ_max > cfg_.integ_min) {
    if (value > cfg_.integ_max) {
      return cfg_.integ_max;
    }
    if (value < cfg_.integ_min) {
      return cfg_.integ_min;
    }
  }
  return value;
}

float Pid::calc(float ref, float fdb, float dt)
{
  /* 1. 输入合法性防御：非法输入（NaN/Inf/dt<=0）时冻结内部状态，返回上一拍输出，
   *    避免一次异常采样污染积分/微分历史，也避免 NaN 在控制回路中扩散。 */
  if (!std::isfinite(ref) || !std::isfinite(fdb) || !std::isfinite(dt) ||
      dt <= 0.0f) {
    return out_;
  }

  /* 2. 计算误差 */
  const float err = ref - fdb;

  /* 3. 微分项：对【测量值】微分（而非对误差微分），设定值突跳不会产生微分冲击；
   *    再对微分值做一阶低通滤波，用于抑制编码器量化噪声被 kd 放大。
   *    注意：微分历史在死区/饱和时也照常更新，保证退出死区时微分连续、不产生
   *    历史断档导致的尖峰。 */
  const float kp = cfg_.kp;
  const float ki = cfg_.ki;
  const float kd = cfg_.kd;
  float d_raw = 0.0f;
  if (has_last_fdb_) {
    d_raw = (fdb - last_fdb_) / dt;
  }
  last_fdb_ = fdb;
  has_last_fdb_ = true;

  /* d_lpf_alpha 是【滤波强度】而非"新值权重"：alpha = 0 不滤波（直接跟随
   * 原始微分）；alpha 越大越相信历史值、滤波越重，新值权重为 (1-alpha)；
   * alpha = 1 时完全保持上一拍滤波值（最重滤波）；非有限值按 0 处理。 */
  float alpha = cfg_.d_lpf_alpha;
  if (!std::isfinite(alpha) || alpha <= 0.0f) {
    alpha = 0.0f;
  } else if (alpha > 1.0f) {
    alpha = 1.0f;
  }
  if (alpha <= 0.0f) {
    d_filt_ = d_raw; /* 0：不滤波，直接跟随原始微分 */
  } else if (alpha < 1.0f) {
    d_filt_ = (1.0f - alpha) * d_raw + alpha * d_filt_;
  }
  /* alpha >= 1：保持上一拍滤波值不变（最重滤波） */
  const float d_term = -kd * d_filt_;

  /* 4. 死区：|err| <= dead_zone 时输出 0（积分冻结，微分历史已在上方更新）。
   *    dead_zone <= 0 表示不启用死区。 */
  const float dead_zone = cfg_.dead_zone;
  if (dead_zone > 0.0f && std::fabs(err) <= dead_zone) {
    out_ = 0.0f;
    return out_;
  }

  /* 5. 抗积分饱和（条件积分 / conditional integration）：
   *    先按当前积分状态预估输出，若已饱和且误差方向与饱和方向相同（继续积分只会
   *    让输出更深地饱和），则本拍停止积分；否则正常累加。 */
  const float u_pre = kp * err + ki * integ_ + d_term;
  bool saturated_high = false;
  bool saturated_low = false;
  if (cfg_.out_max > cfg_.out_min) {
    saturated_high = (u_pre > cfg_.out_max);
    saturated_low = (u_pre < cfg_.out_min);
  }
  const bool windup = (saturated_high && err > 0.0f) ||
                      (saturated_low && err < 0.0f);
  if (ki != 0.0f && !windup) {
    integ_ += err * dt;
    if (!std::isfinite(integ_)) {
      /* 极端输入下 err*dt 可能溢出，直接复位积分，避免 Inf 永久污染状态 */
      integ_ = 0.0f;
    }
    integ_ = clampIntegral(integ_);
  }

  /* 6. 位置式输出 + 输出限幅。若计算结果非有限值（极端参数组合），保持上一拍
   *    输出，保证 calc() 永远不向控制回路返回 NaN。 */
  float u = kp * err + ki * integ_ + d_term;
  if (!std::isfinite(u)) {
    return out_;
  }
  u = clampOutput(u);
  out_ = u;
  return out_;
}

}  // namespace pid
}  // namespace hello_world
