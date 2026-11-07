/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 云台板顶层状态机（安全判断 → 三态迁移 → 云台控制 → 回填 G2C 帧）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-08/13/16：三态状态机（Dead/
 *                                                Resurrection/Working）+ 双板帧消费/回填
 *******************************************************************************
 * @attention :
 *  - 本类是本板 1ms 唯一入口（main_task.cpp 的 MainTask 调用 update()），
 *    自己不开中断、不死等。
 *  - 急停/离线判断放最外层：任何安全条件不满足 → Dead + 输出归零并 return（契约 §7.4）。
 *  - 与底盘板之间只通过 GimbalChassisComm 的 CAN 帧交换信息（禁止直接调用/共享变量）。
 *  - C2G 目标语义按契约 v1.1：b2-3 = 期望世界 yaw，b4-5 = 期望绝对 pitch。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "gimbal_chassis_comm.hpp"
#include "module_state.hpp"

namespace hello_world
{
namespace imu
{
class Imu;
}  // namespace imu
namespace motor
{
class Motor;
}  // namespace motor
namespace remote_control
{
class DT7;
}  // namespace remote_control
}  // namespace hello_world

namespace robot
{
class Gimbal;

/* 云台板状态机调试观测变量（Ozone Live Watch 用；无硬件自测观察状态迁移/输出归零） */
namespace robot_debug
{
extern volatile uint8_t life_state;         ///< 当前生命状态（LifeState 枚举值）
extern volatile uint8_t safety_ok;          ///< 1 = 最外层安全条件通过
extern volatile uint8_t c2g_estop;          ///< C2G 软急停标志（镜像）
extern volatile uint8_t c2g_online;         ///< C2G 对板在线标志
extern volatile uint8_t c2g_remote_online;  ///< C2G 遥控在线标志
extern volatile float c2g_yaw_target;       ///< C2G 期望世界 yaw（rad）
extern volatile float c2g_pitch_target;     ///< C2G 期望 pitch（rad）
extern volatile float c2g_chassis_imu_yaw;  ///< C2G 底盘 IMU yaw（rad）
extern volatile uint32_t cycle_count;       ///< update() 调用计数
}  // namespace robot_debug

/**
 * @brief 云台板顶层状态机（契约 §6.3 / §7.4，WP-08/13/16）
 */
class Robot
{
 public:
  Robot() = default;

  /* 模块注册（契约 §6.3；ins_* 层装配后调用，便于主机单测替换实现） */
  void registerGimbal(Gimbal *gimbal);
  void registerImu(hello_world::imu::Imu *imu);
  void registerComm(GimbalChassisComm *comm);
  void registerRemote(hello_world::remote_control::DT7 *remote);

  /** @brief 装配：从 ins_* 层取各模块实例并初始化云台（MainTaskInit 调用一次） */
  void init();

  /** @brief 1ms 唯一入口：安全判断 → 状态机 → 输出 → 回填 G2C 帧 */
  void update();

  /** @brief 当前生命状态（G2C 帧/调试用） */
  LifeState lifeState() const { return life_state_; }

 private:
  /** @brief 最外层安全条件：急停/双板链路/遥控/IMU/DM 反馈与使能 */
  bool SafetyCheck() const;
  /** @brief Working 就绪条件：安全条件 + IMU 已标定 + pitch 在软限位内 */
  bool WorkingReady() const;
  /** @brief 两轴 DM 反馈帧是否新鲜（未超时） */
  bool MotorFeedbackOk() const;
  /** @brief 两轴 DM 是否使能且无故障码 */
  bool MotorsEnabled() const;
  /** @brief 进入指定状态并做进入动作（目标初始化/复位） */
  void EnterState(LifeState state);
  /** @brief 把本板状态/关节角/IMU yaw 回填到 G2C 帧（CommTask 负责发送） */
  void FillGimbalData();

  Gimbal *gimbal_ = nullptr;
  hello_world::imu::Imu *imu_ = nullptr;
  GimbalChassisComm *comm_ = nullptr;
  hello_world::remote_control::DT7 *remote_ = nullptr;  ///< 云台板恒 nullptr（无 DBUS）
  hello_world::motor::Motor *yaw_motor_ = nullptr;
  hello_world::motor::Motor *pitch_motor_ = nullptr;

  LifeState life_state_ = LifeState::kDead;  ///< 当前生命状态
  uint32_t state_entry_tick_ = 0;            ///< 进入当前状态时刻（ms）
  uint32_t candidate_tick_ = 0;              ///< Dead→Resurrection 条件满足起始时刻（ms）
  bool candidate_valid_ = false;             ///< candidate_tick_ 是否有效
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
