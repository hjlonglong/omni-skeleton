/**
 *******************************************************************************
 * @file      :chassis_ik_solver.hpp
 * @brief     :底盘运动学逆解（云台系速度 → 4 轮轮速，O 型全向轮）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent B         1. WP-12/14/17：O 型全向轮通用逆解
 *                                                （四角位置 + 每轮安装角 + 方向符号）
 *******************************************************************************
 * @attention :
 *  - 纯头文件、无 HAL/外设依赖，可在主机上直接单元测试（见交付自测）。
 *  - 输入速度定义在【云台系】（契约 §1.2），先按契约公式旋到车体系，再逐轮投影：
 *      vx_chassis =  vx_gimbal·cosδ + vy_gimbal·sinδ
 *      vy_chassis = -vx_gimbal·sinδ + vy_gimbal·cosδ
 *    （δ 由调用方 wrapPi 并按 config::kDeltaSign 取号后传入）
 *  - 全向轮逆解通用公式（轮 i 轮心 r_i、驱动方向角 α_i、半径 R）：
 *      ω_i = sign_i · [ vx·cosα_i + vy·sinα_i + w·(x_i·sinα_i − y_i·cosα_i) ] / R
 *    其中 w×(x_i,y_i) = w·(−y_i, x_i)，与驱动方向点乘即旋转项。
 *  - 默认假设为 O 型布置（四轮驱动方向沿以车体中心为圆心的切线），所有几何量来自
 *    config.hpp 的四角数组：实车安装角/位置/方向确认后只改 config，不改本文件。
 *  - 本类只算【目标轮速】，不做限幅、不做闭环；限幅（kMaxWheelRadS）与轮速环在
 *    robot::Chassis 内完成。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_
#define CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

#include "config.hpp"
#include "module_state.hpp"

/* Exported types ------------------------------------------------------------*/
namespace hello_world
{
namespace chassis_ik_solver
{
/**
 * @brief 底盘逆解器（O 型全向轮，四轮）
 * @note  无状态、可重入：每周期调用 solve() 即可，可安全地在中断上下文中使用。
 */
class ChassisIkSolver
{
 public:
  ChassisIkSolver() = default;
  explicit ChassisIkSolver(float wheel_radius) : wheel_radius_(wheel_radius) {}

  /**
   * @brief  把云台系平移速度旋到车体系（契约 §1.2 硬性公式，全工程唯一实现）
   * @param  v_gimbal    云台系速度 (vx, vy, w)，m/s 与 rad/s
   * @param  delta       δ（已 wrapPi 到 [-π, π)），rad
   * @param  vx_chassis  输出：车体系 vx，m/s
   * @param  vy_chassis  输出：车体系 vy，m/s
   * @note   w 为标量，不参与旋转。
   */
  static void RotateToChassis(const robot::ChassisVelocity &v_gimbal, float delta,
                              float *vx_chassis, float *vy_chassis)
  {
    const float c = std::cos(delta);
    const float s = std::sin(delta);
    *vx_chassis = v_gimbal.vx * c + v_gimbal.vy * s;
    *vy_chassis = -v_gimbal.vx * s + v_gimbal.vy * c;
  }

  /**
   * @brief  逆解：云台系速度 → 4 轮目标轮速
   * @param  v_gimbal     云台系速度指令（m/s, rad/s）
   * @param  delta        δ（已 wrapPi，rad）
   * @param  wheel_rad_s  输出：4 轮目标角速度（rad/s，未限幅），索引顺序 LF/LB/RB/RF
   * @note   纯计算，无副作用；调用方负责限幅（config::kMaxWheelRadS）。
   */
  void solve(const robot::ChassisVelocity &v_gimbal, float delta,
             float wheel_rad_s[config::kWheelNum]) const
  {
    float vx_chassis = 0.0f;
    float vy_chassis = 0.0f;
    RotateToChassis(v_gimbal, delta, &vx_chassis, &vy_chassis);

    for (int i = 0; i < config::kWheelNum; ++i) {
      const float angle = config::kWheelMountAngleRad[i];
      const float ca = std::cos(angle);
      const float sa = std::sin(angle);
      /* 平移项：车体系速度在轮驱动方向上的投影；
       * 旋转项：w×(x_i,y_i) 与驱动方向的点乘 = w·(x_i·sinα_i − y_i·cosα_i)。 */
      const float v_wheel = vx_chassis * ca + vy_chassis * sa +
                            v_gimbal.w * (config::kWheelPosX[i] * sa -
                                          config::kWheelPosY[i] * ca);
      wheel_rad_s[i] = static_cast<float>(config::kWheelSign[i]) * v_wheel /
                       wheel_radius_;
    }
  }

  /// 轮半径（m），默认取 config::kWheelRadius
  float wheel_radius() const { return wheel_radius_; }

 private:
  float wheel_radius_ = config::kWheelRadius;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* CHASSIS_INC_CHASSIS_IK_SOLVER_HPP_ */
