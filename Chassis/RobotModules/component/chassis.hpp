/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     :底盘运动模块（云台系速度 → O 型全向轮逆解 → 4 轮速度环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：底盘逆解 + 轮速环实现
 *******************************************************************************
 * @attention :
 *  - 本模块只负责“目标速度 → 轮速 → 电流”的运动执行，不含状态机/遥控逻辑：
 *    Robot 在 Working 态调用 setTarget()/update()，非 Working 态调用 stop()。
 *  - 逆解公式在 hello_world::chassis_ik_solver::ChassisIkSolver（纯算法、主机可测），
 *    本类只做限幅（kMaxWheelRadS）+ 每轮 PID（kPidWheel*）+ 电流限幅（kMaxWheelCurrent）。
 *  - 无动态内存、无阻塞调用，可在 1 ms 定时中断里直接调用。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "config.hpp"
#include "module_state.hpp"

/* Exported types ------------------------------------------------------------*/
namespace hello_world
{
namespace motor
{
class Motor;
}  // namespace motor
namespace pid
{
class Pid;
}  // namespace pid
namespace chassis_ik_solver
{
class ChassisIkSolver;
}  // namespace chassis_ik_solver
}  // namespace hello_world

namespace robot
{
/**
 * @brief 底盘运动模块（4×M3508 轮速闭环）
 */
class Chassis
{
 public:
  Chassis() = default;

  /**
   * @brief  初始化：绑定 4 个轮电机、4 个轮速环 PID 与逆解器，并清零历史状态
   * @param  wheels     4 个轮电机指针（索引 LF/LB/RB/RF），不可为 nullptr
   * @param  wheel_pid  4 个轮速环 PID 指针，不可为 nullptr
   * @param  ik_solver  逆解器指针（函数内 static 单例），不可为 nullptr
   * @retval None
   * @note   由 Robot::init() 在 TIM6 中断启动前调用一次；本函数会 setInput(0) 与
   *         PID reset()，保证上电无残留指令。
   */
  void init(hello_world::motor::Motor *wheels[config::kWheelNum],
            hello_world::pid::Pid *wheel_pid[config::kWheelNum],
            hello_world::chassis_ik_solver::ChassisIkSolver *ik_solver);

  /**
   * @brief  下发目标速度（云台系）
   * @param  target_gimbal 目标 (vx, vy, w)，契约 §1.2 云台系
   * @param  delta         δ（rad，已 wrapPi 并按 config::kDeltaSign 取号）
   * @param  mode          当前底盘模式（仅记录用于 Ozone 观察，不改变逆解）
   * @retval None
   * @note   只在 Robot 的 Working 态调用；非 Working 态必须调用 stop()。
   */
  void setTarget(const ChassisVelocity &target_gimbal, float delta,
                 ChassisMode mode);

  /**
   * @brief  1 ms 周期执行：逆解 → 每轮限幅 → 轮速 PID → 电流限幅 → setInput
   * @retval None
   * @note   未初始化或任一依赖为空时直接返回（不驱动电机）。
   */
  void update();

  /**
   * @brief  安全停止：全部电机 setInput(0)、PID reset、目标清零
   * @retval None
   * @note   Dead/Resurrection 态每周期调用，保证「非 Working 无输出」安全红线。
   */
  void stop();

  /* ---- 调试/观测接口（Ozone / 双板帧校验用）---- */
  const float *wheelTarget() const { return target_rad_s_; }  ///< 4 轮目标轮速 rad/s
  const float *wheelActual() const { return actual_rad_s_; }  ///< 4 轮实际轮速 rad/s
  bool inited() const { return inited_; }
  ChassisMode mode() const { return mode_; }

 private:
  static constexpr int kWheelNum = config::kWheelNum;

  hello_world::motor::Motor *wheels_[kWheelNum] = {nullptr, nullptr, nullptr,
                                                   nullptr};
  hello_world::pid::Pid *wheel_pid_[kWheelNum] = {nullptr, nullptr, nullptr,
                                                  nullptr};
  hello_world::chassis_ik_solver::ChassisIkSolver *ik_solver_ = nullptr;

  float target_rad_s_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  float actual_rad_s_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  ChassisVelocity target_{};
  float delta_ = 0.0f;
  ChassisMode mode_ = ChassisMode::kSeparate;
  bool inited_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
