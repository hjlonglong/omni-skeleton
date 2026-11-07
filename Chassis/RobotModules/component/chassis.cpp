/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :底盘运动模块实现（逆解 + 4 轮轮速环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：O 型全向轮逆解 +
 *                                                每轮速度环 + 非 Working 归零
 *******************************************************************************
 * @attention :
 *  - 安全链路：本模块不判断状态/遥控，只执行；但 stop() 必须能被每周期调用且无副作用。
 *  - 反馈异常（NaN/Inf）或 PID 输出异常时，本拍直接 setInput(0) 并 reset PID，避免
 *    把异常值持续送给电调。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include <cmath>

#include "chassis_ik_solver.hpp"
#include "motor_base.hpp"
#include "pid.hpp"

/* Private constants ---------------------------------------------------------*/
namespace
{
/* 4 轮索引（与 config.hpp 注释一致：LF, LB, RB, RF） */
constexpr int kLF = 0;
constexpr int kLB = 1;
constexpr int kRB = 2;
constexpr int kRF = 3;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
void robot::Chassis::init(hello_world::motor::Motor *wheels[config::kWheelNum],
                          hello_world::pid::Pid *wheel_pid[config::kWheelNum],
                          hello_world::chassis_ik_solver::ChassisIkSolver *ik_solver)
{
  ik_solver_ = ik_solver;
  bool ok = (ik_solver_ != nullptr);
  for (int i = 0; i < kWheelNum; ++i) {
    wheels_[i] = wheels[i];
    wheel_pid_[i] = wheel_pid[i];
    target_rad_s_[i] = 0.0f;
    actual_rad_s_[i] = 0.0f;
    if (wheels_[i] == nullptr || wheel_pid_[i] == nullptr) {
      ok = false;
      continue;
    }
    /* 上电清历史：先零电流，再清 PID 状态，避免残留积分/微分冲击 */
    (void)wheels_[i]->setInput(0.0f);
    wheel_pid_[i]->reset();
  }
  target_.zero();
  delta_ = 0.0f;
  mode_ = ChassisMode::kSeparate;
  inited_ = ok;

  /* 静态断言：轮序常量与数组长度一致（编译期即可发现配置改动） */
  static_assert(kLF == 0 && kLB == 1 && kRB == 2 && kRF == 3 && kWheelNum == 4,
                "wheel index layout mismatch");
}

void robot::Chassis::setTarget(const ChassisVelocity &target_gimbal, float delta,
                               ChassisMode mode)
{
  target_ = target_gimbal;
  delta_ = delta;
  mode_ = mode;
}

void robot::Chassis::update()
{
  if (!inited_ || ik_solver_ == nullptr) {
    return;
  }

  /* 1. 逆解：云台系速度 → 车体系 → 4 轮目标轮速（未限幅） */
  float solved[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  ik_solver_->solve(target_, delta_, solved);

  for (int i = 0; i < kWheelNum; ++i) {
    /* 2. 目标轮速限幅（kMaxWheelRadS）；NaN 目标按 0 处理，避免污染 PID */
    float target = solved[i];
    if (!std::isfinite(target)) {
      target = 0.0f;
    }
    if (target > config::kMaxWheelRadS) {
      target = config::kMaxWheelRadS;
    } else if (target < -config::kMaxWheelRadS) {
      target = -config::kMaxWheelRadS;
    }
    target_rad_s_[i] = target;

    /* 3. 反馈读取（CAN 反馈由 HW_can.cpp 的 FIFO0 回调 decode） */
    const float actual = wheels_[i]->vel();
    actual_rad_s_[i] = actual;
    if (!std::isfinite(actual)) {
      /* 反馈异常：本拍零电流并清 PID，等下一帧正常反馈再恢复 */
      wheel_pid_[i]->reset();
      (void)wheels_[i]->setInput(0.0f);
      continue;
    }

    /* 4. 轮速环 PID：输入轮速误差 rad/s，输出电流 A；输出再做一次电流限幅兜底 */
    float out = wheel_pid_[i]->calc(target, actual, config::kTs);
    if (!std::isfinite(out)) {
      out = 0.0f;
      wheel_pid_[i]->reset();
    }
    if (out > config::kMaxWheelCurrent) {
      out = config::kMaxWheelCurrent;
    } else if (out < -config::kMaxWheelCurrent) {
      out = -config::kMaxWheelCurrent;
    }
    (void)wheels_[i]->setInput(out);
  }
}

void robot::Chassis::stop()
{
  for (int i = 0; i < kWheelNum; ++i) {
    target_rad_s_[i] = 0.0f;
    if (wheels_[i] != nullptr) {
      (void)wheels_[i]->setInput(0.0f);
    }
    if (wheel_pid_[i] != nullptr) {
      wheel_pid_[i]->reset();
    }
  }
  target_.zero();
  /* 保留 mode_ 供 Ozone 观测（不影响输出） */
}
