/**
 *******************************************************************************
 * @file      :control_task.cpp
 * @brief     :底盘任务层——单轮控制（单电机速度闭环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      Lead            1. 新增 ControlTask：单轮速度环实现
 *******************************************************************************
 * @attention :
 *  - 控制链路：TIM6 1 ms 中断 → MainTask() → ControlTask() → CommTask()。
 *    ControlTask 用 PID（hello_world::pid::Pid）把轮速拉向目标，setInput()
 *    写电流（InputType::kCurr），CommTask 在同一拍把控制帧 0x200 发出。
 *  - TEST_MODE == 1（台架，车轮必须离地）：目标来自 CalcTestSweepTargetRadS()
 *    三角波扫掠，电流限幅 kTestWheelCurrentLimit（2 A）；超过
 *    kTestWheelDurationMs 后目标归零。
 *  - TEST_MODE == 0（交付态）：控制权归 Robot/Chassis（WP-12），本任务不覆盖
 *    电机指令（只在 Ozone 里提供观测值），避免把状态机同拍下发的电流清零。
 *  - TODO【WP-10/WP-12】：增加反馈离线监督（>100 ms 无反馈时零电流 + 状态机
 *    降级）；当前单轮台架阶段先用 2 A 限幅 + 离地测试兜底。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "control_task.hpp"

#include <cmath>
#include <cstdint>

#include "config.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "tick.hpp"

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace
{
/* 单轮控制使用的电机 / PID 实例（ControlTaskInit 中绑定） */
hello_world::motor::Motor *g_motor = nullptr;
hello_world::pid::Pid *g_pid = nullptr;
uint32_t g_start_tick_ms = 0u;  ///< 扫掠计时起点（ControlTaskInit 时刻）
bool g_inited = false;          ///< 初始化完成标志
}  // namespace

#if TEST_MODE == 1
/* ---- Ozone 调试镜像 --------------------------------------------------------
 * 仅台架模式（TEST_MODE == 1）编译：把 ControlTask 的局部变量复制到全局
 * volatile，Ozone 的 Live Watch 可直接添加下列表达式、运行中连续刷新：
 *   g_dbg_target     目标轮速 rad/s（三角波扫掠值）
 *   g_dbg_vel        实际轮速 rad/s（CAN 反馈）
 *   g_dbg_out        本拍 PID 输出电流 A（限幅后）
 *   g_dbg_elapsed_ms 扫掠已运行时间 ms（从 ControlTaskInit 起算）
 * 正式固件（TEST_MODE == 0）中不生成这些变量，无需删除本段。 */
volatile float g_dbg_target = 0.0f;      ///< 目标轮速，rad/s
volatile float g_dbg_vel = 0.0f;         ///< 实际轮速，rad/s
volatile float g_dbg_out = 0.0f;         ///< PID 输出电流，A
volatile uint32_t g_dbg_elapsed_ms = 0u; ///< 扫掠已运行时间，ms
#endif

/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void ControlTaskInit(void)
{
  g_motor = GetMotorWheel(config::kTestWheelIdx);
  g_pid = GetPidMotorWheel(config::kTestWheelIdx);
  g_start_tick_ms = hello_world::tick::GetTickMs();
  g_inited = false;

  if (g_motor == nullptr || g_pid == nullptr) {
    return;  /* 索引非法（config::kTestWheelIdx 越界）：本任务不动作 */
  }

  /* 轮速环 PID：输入为速度误差（rad/s），输出为电流（A） */
  constexpr float kLimit =
#if TEST_MODE == 1
      config::kTestWheelCurrentLimit;  /* 台架调试：≤2 A */
#else
      config::kMaxWheelCurrent;  /* 正常模式：沿用整车限幅 */
#endif
  hello_world::pid::Pid::Config cfg;
  cfg.kp = config::kPidWheelKp;
  cfg.ki = config::kPidWheelKi;
  cfg.kd = config::kPidWheelKd;
  cfg.out_min = -kLimit;
  cfg.out_max = kLimit;
  cfg.integ_min = -config::kPidWheelIntegMax;
  cfg.integ_max = config::kPidWheelIntegMax;
  cfg.dead_zone = 0.0f;      /* 轮速环死区由整车状态机处理，避免死区造成稳态误差 */
  cfg.d_lpf_alpha = 0.0f;    /* 轮速环 kd=0，微分低通暂不启用 */
  g_pid->init(cfg);

  /* 上电先清零输入，避免残留非零指令 */
  (void)g_motor->setInput(0.0f);
  g_inited = true;

#if TEST_MODE == 1
  /* Ozone 调试镜像清零（见文件内 g_dbg_* 说明） */
  g_dbg_target = 0.0f;
  g_dbg_vel = 0.0f;
  g_dbg_out = 0.0f;
  g_dbg_elapsed_ms = 0u;
#endif
}

void ControlTask(void)
{
  if (!g_inited || g_motor == nullptr || g_pid == nullptr) {
    return;
  }

  const float vel = g_motor->vel();  /* 输出端角速度，rad/s（由 CAN 反馈 decode） */

#if TEST_MODE == 1
  /* ---- 台架模式：单轮三角波扫掠（车轮必须离地）---- */
  const uint32_t elapsed_ms =
      hello_world::tick::GetTickMs() - g_start_tick_ms;  /* uint32 减法天然回绕安全 */
  const float target = CalcTestSweepTargetRadS(elapsed_ms);

  /* 调试镜像：Ozone Live Watch 用（仅 TEST_MODE == 1 存在） */
  g_dbg_elapsed_ms = elapsed_ms;
  g_dbg_target = target;
  g_dbg_vel = vel;

  /* 反馈异常（NaN/Inf）时直接零电流，避免把异常值送进 PID 与电机 */
  if (!std::isfinite(vel)) {
    g_dbg_out = 0.0f;
    (void)g_motor->setInput(0.0f);
    return;
  }

  float out = g_pid->calc(target, vel, config::kTs);
  if (!std::isfinite(out)) {
    out = 0.0f;
    g_pid->reset();
  }
  (void)g_motor->setInput(out);
  g_dbg_out = out;
#else
  /* ---- 交付态（TEST_MODE == 0）：控制权归 Robot/Chassis ----
   * 本任务不得覆盖电机指令：状态机刚在 MainTask() 里下发的电流如果被这里同一拍
   * 清零，底盘就永远动不了。因此只保留观测（Ozone 可看 vel/PID 状态），
   * 不再调用 setInput()。 */
  (void)vel;
#endif
}

/* Private function definitions ----------------------------------------------*/
