/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     :PID 实例（每轮一个轮速环 PID）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-21      Lead            1. 实现 GetPidMotorWheel()：4 个
 *                                                 轮速环 PID 实例（参数来自 config）
 *******************************************************************************
 * @attention :
 *  - 与 ins_motor.cpp 同风格：函数内 static 一次性构造，避免全局构造函数在
 *    HAL / 时钟初始化之前运行（契约 §6.3）。
 *  - PID 参数唯一来源为 config.hpp（kPidWheelKp/Ki/Kd、kPidWheelIntegMax、
 *    kMaxWheelCurrent），业务代码禁止散落魔法数。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "config.hpp"
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{
/* 轮速环数量（索引顺序 LF、LB、RB、RF，见 config.hpp） */
constexpr int kWheelNum = 4;
}  // namespace

/* Private types -------------------------------------------------------------*/
namespace
{
/**
 * 4 个轮速环 PID 实例（函数内 static 构造，见文件头 @attention）。
 *
 * 一个 PID 只对应一个被控对象（一个轮的转速），因此 4 个轮各持有一个实例，
 * 积分/微分历史互不串扰。
 */
struct WheelPidInstances {
  WheelPidInstances()
  {
    hello_world::pid::Pid::Config cfg;
    cfg.kp = config::kPidWheelKp;          /* A/(rad/s) */
    cfg.ki = config::kPidWheelKi;          /* A/rad */
    cfg.kd = config::kPidWheelKd;          /* A·s/rad */
    cfg.out_min = -config::kMaxWheelCurrent;
    cfg.out_max = config::kMaxWheelCurrent;
    cfg.integ_min = -config::kPidWheelIntegMax;
    cfg.integ_max = config::kPidWheelIntegMax;
    cfg.dead_zone = 0.0f;    /* 轮速环不设死区（死区交给整车状态机） */
    cfg.d_lpf_alpha = 0.0f;  /* kd=0，微分低通暂不启用 */
    for (int i = 0; i < kWheelNum; ++i) {
      pids[i].init(cfg);
    }
  }

  hello_world::pid::Pid pids[kWheelNum];
};
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx)
{
  /* 越界安全：非法下标返回 nullptr，调用方须判空 */
  if (idx < 0 || idx >= kWheelNum) {
    return nullptr;
  }

  /* 首次调用时才构造 4 个 PID 实例（函数内 static，见文件头 @attention） */
  static WheelPidInstances wheel_pids;
  return &wheel_pids.pids[idx];
}

/* Private function definitions ----------------------------------------------*/
