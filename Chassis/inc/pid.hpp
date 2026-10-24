/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : PID 控制器（位置式，可用于电机速度/位置等闭环控制）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-21      A1              1. 按《接口契约 v1.0》§6.1 实现
 *  V1.0.1      2026-10-21      A1              1. 统一 d_lpf_alpha 为滤波强度语义（Lead 决策）
 *******************************************************************************
 * @attention :
 *  1. 本文件不包含任何 HAL/STM32 头文件，可直接在主机上做单元测试；
 *  2. 位置式 PID，控制周期由调用方通过 calc() 的 dt 参数显式传入，不使用
 *     内部写死的采样时间（当前调度周期为 kTs = 0.001f）；
 *  3. Config 中 out_max <= out_min（默认 0,0）表示【不做输出限幅】，
 *     integ_max <= integ_min（默认 0,0）表示【不做积分限幅】；dead_zone <= 0
 *     表示【不启用死区】；d_lpf_alpha 是微分一阶低通的【滤波强度】，范围
 *     [0,1]：0 表示不滤波，越大滤波越重（越平滑），1 表示完全保持上一拍
 *     （最重滤波）；非有限值按 0 处理；
 *  4. calc() 返回的永远是输出限幅后的值，与 out() 一致；微分项对【测量值】
 *     微分并做一阶低通，因此设定值突跳不会产生微分冲击。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HELLO_WORLD_PID_HPP_
#define HELLO_WORLD_PID_HPP_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types --------------------------------------------------------*/
namespace hello_world
{
namespace pid
{
class Pid
{
 public:
  /**
   * @brief       PID 配置参数
   * @note        各字段默认值均为 0，即默认不做限幅/死区/滤波；
   *              限幅生效条件为上限 > 下限（见文件头 @attention）
   */
  struct Config {
    float kp = 0.0f;  ///< 比例系数
    float ki = 0.0f;  ///< 积分系数（单位：1/s）
    float kd = 0.0f;  ///< 微分系数（单位：s）
    float out_min = 0.0f;  ///< 输出下限
    float out_max = 0.0f;  ///< 输出上限
    float integ_min = 0.0f;  ///< 积分项下限（积分状态 integ 的限幅）
    float integ_max = 0.0f;  ///< 积分项上限
    float dead_zone = 0.0f;  ///< 误差死区宽度（|err| <= dead_zone 时输出 0）
    float d_lpf_alpha = 0.0f;  ///< 微分一阶低通【滤波强度】[0,1]：0 不滤波，越大越平滑，1 保持上一拍
  };

  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象后，必须调用 init() 设置参数
   */
  Pid() = default;

  /**
   * @brief       带配置的构造函数
   * @param        cfg: PID 配置参数
   * @retval       None
   * @note        None
   */
  explicit Pid(const Config& cfg) { init(cfg); }

  /* 配置方法 */

  /**
   * @brief       PID 初始化/重新配置
   * @param        cfg: PID 配置参数
   * @retval       None
   * @note        会调用 reset()，清空积分/微分历史与输出
   */
  void init(const Config& cfg);

  /**
   * @brief       复位控制器
   * @retval       None
   * @note        清空积分状态、微分滤波状态、上一拍反馈与输出，保留配置参数
   */
  void reset();

  /* 计算方法 */

  /**
   * @brief       PID 计算（位置式）
   * @param        ref: 设定值
   * @param        fdb: 反馈值（测量值）
   * @param        dt: 距上次计算的采样时间，单位：s，必须 > 0
   * @retval       限幅后的控制输出
   * @note        若 ref/fdb/dt 不是有限值或 dt <= 0，则冻结内部状态并返回上一拍
   *              输出（不产生 NaN）；死区内输出 0，积分冻结但微分历史照常更新
   */
  float calc(float ref, float fdb, float dt);

  /* 状态查询方法 */

  /**
   * @brief       获取上一拍输出（限幅后的值）
   * @retval       上一拍控制输出
   */
  float out() const { return out_; }

  /**
   * @brief       获取积分项状态（∫err dt）
   * @retval       积分项状态
   */
  float integral() const { return integ_; }

  /**
   * @brief       获取当前配置参数
   * @retval       配置参数常量引用
   */
  const Config& config() const { return cfg_; }

 private:
  /**
   * @brief       对输出限幅（out_max > out_min 时生效）
   * @param        value: 原始输出
   * @retval       限幅后的输出
   */
  float clampOutput(float value) const;

  /**
   * @brief       对积分项限幅（integ_max > integ_min 时生效）
   * @param        value: 原始积分状态
   * @retval       限幅后的积分状态
   */
  float clampIntegral(float value) const;

  Config cfg_{};              ///< 配置参数
  float integ_ = 0.0f;        ///< 积分项状态 ∫err dt
  float d_filt_ = 0.0f;       ///< 微分项状态：-d(fdb)/dt 的一阶低通滤波值
  float out_ = 0.0f;          ///< 上一拍输出（限幅后）
  float last_fdb_ = 0.0f;     ///< 上一拍反馈值（用于对测量值微分）
  bool has_last_fdb_ = false; ///< 是否已有有效的上一拍反馈值
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace pid
}  // namespace hello_world

#endif /* HELLO_WORLD_PID_HPP_ */
