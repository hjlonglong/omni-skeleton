/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 封装类（WP-06 骨架：BMI088 原始数据 + 零漂估计 + Mahony 姿态）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2026-10-24      Lead            1. 骨架：接口/状态机/纯函数零漂估计，
 *                                                算法主体留 TODO(STEP 4/5/6)
 *  V1.0.0      2026-10-28      Lead            1. 完成实现：修正 Mahony 命名空间
 *                                                （hello_world::ahrs）
 *******************************************************************************
 * @attention :
 *  - 两板同源文件：Chassis/inc/imu.hpp 与 Gimbal/inc/imu.hpp 必须逐字节一致（md5）。
 *  - 本类只在 MainTaskInit()（非中断上下文）调用 init()；update() 由 TIM6 1 ms
 *    周期任务调用。
 *  - 零漂未收敛 / 初始化失败时 isReady() 恒为 false，状态机不得进入 Working。
 *  - 禁止 include HAL 之外的板级差异代码；SPI/GPIO 句柄只在 src/imu.cpp 里引用。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

#include "BMI088.hpp"
#include "mahony.hpp"

namespace hello_world
{
namespace imu
{
/* ============================ 可调参数（只改这里） ============================ */
inline constexpr float kSampleFreqHz = 1000.0f;   ///< Hz，必须与 TIM6 1 ms 一致
inline constexpr uint32_t kCalibFrames = 750;     ///< 帧，开机零漂采样窗口（0.75 s）
inline constexpr float kStillStdMaxRadS = 0.02f;  ///< rad/s，静止判定：三轴标准差上限
inline constexpr float kStillMeanMaxRadS = 0.05f; ///< rad/s，静止判定：三轴均值上限
inline constexpr uint32_t kReadyExtraFrames = 100;///< 帧，标定完成后再跑 100 帧才 isReady
inline constexpr float kMahonyKp = 0.5f;          ///< Mahony 比例增益（初值 0.5~1.0）
inline constexpr float kMahonyKi = 0.0f;          ///< Mahony 积分增益（初值 0）

/* ============================ 零漂估计（纯函数，可主机单测） ============================ */
/**
 * @brief 开机零漂估计器：流式采样角速度，窗口满后给出均值与"是否静止"的判定。
 * @note  不依赖 HAL，可放到 tests/host/test_imu_math.cpp 里单独验证。
 *        判据：三轴标准差 < kStillStdMaxRadS 且三轴均值 < kStillMeanMaxRadS。
 */
class ZeroDriftEstimator
{
 public:
  /** @brief 清空窗口，重新开始采样 */
  void reset()
  {
    for (int i = 0; i < 3; ++i) {
      sum_[i] = 0.0f;
      sum_sq_[i] = 0.0f;
      min_[i] = 0.0f;
      max_[i] = 0.0f;
    }
    n_ = 0;
  }

  /**
   * @brief  采样一帧角速度
   * @param  gyro: [wx wy wz]，单位 rad/s
   * @retval true = 窗口已满（可以调用 isStationary()/bias() 判定并取零漂）
   */
  bool addSample(const float gyro[3])
  {
    for (int i = 0; i < 3; ++i) {
      if (n_ == 0 || gyro[i] < min_[i]) { min_[i] = gyro[i]; }
      if (n_ == 0 || gyro[i] > max_[i]) { max_[i] = gyro[i]; }
      sum_[i] += gyro[i];
      sum_sq_[i] += gyro[i] * gyro[i];
    }
    ++n_;
    return n_ >= kCalibFrames;
  }

  /** @brief 当前窗口是否满足"静止"判据（窗口未满返回 false） */
  bool isStationary() const
  {
    if (n_ < kCalibFrames) { return false; }
    for (int i = 0; i < 3; ++i) {
      const float mean = sum_[i] / static_cast<float>(n_);
      if (std::fabs(mean) > kStillMeanMaxRadS) { return false; }
      float var = sum_sq_[i] / static_cast<float>(n_) - mean * mean;
      if (var < 0.0f) { var = 0.0f; }
      if (std::sqrt(var) > kStillStdMaxRadS) { return false; }
    }
    return true;
  }

  /** @brief 取窗口均值作为零漂（单位 rad/s；窗口为空时输出 0） */
  void bias(float out_bias[3]) const
  {
    for (int i = 0; i < 3; ++i) {
      out_bias[i] = (n_ == 0) ? 0.0f : sum_[i] / static_cast<float>(n_);
    }
  }

  /** @brief 已采样帧数 */
  uint32_t count() const { return n_; }

 private:
  float sum_[3] = {0.0f, 0.0f, 0.0f};
  float sum_sq_[3] = {0.0f, 0.0f, 0.0f};
  float min_[3] = {0.0f, 0.0f, 0.0f};
  float max_[3] = {0.0f, 0.0f, 0.0f};
  uint32_t n_ = 0;
};

/* ============================ IMU 封装类（契约 §6.2） ============================ */
class Imu
{
 public:
  Imu() = default;

  /**
   * @brief 初始化：配置 BMI088（SPI1 + CS）→ 芯片自检/寄存器配置 → Mahony 初始化
   * @note  会阻塞数 ms（imuInit 读 ID/写寄存器），只能在 MainTaskInit 里调用。
   */
  void init();

  /**
   * @brief 1 ms 周期更新：读原始数据 → （未标定时）零漂采样 → 扣零漂 → Mahony 更新
   * @note  由 TIM6 1 ms 任务调用；标定期间也要持续调用。
   */
  void update();

  bool isInitOk() const { return initialized_; }   ///< BMI088 初始化是否成功
  bool isCalibrated() const { return calibrated_; }///< 零漂是否已收敛
  bool isReady() const { return ready_; }          ///< 标定完成且姿态已收敛，可进 Working

  float roll() const { return euler_[0]; }   ///< rad，相对开机姿态
  float pitch() const { return euler_[1]; }  ///< rad
  float yaw() const { return euler_[2]; }    ///< rad
  float gyroZ() const { return gyro_z_; }    ///< rad/s，扣零漂后的 z 轴角速度

  /** @brief 当前零漂估值，[wx wy wz]，rad/s（调试/RTT 用） */
  const float* gyroBias() const { return bias_; }

 private:
  /** @brief 复位标定/姿态状态（init 末尾调用） */
  void resetState();

  BMI088 bmi_;                  ///< 芯片级驱动（原始 acc/gyro/temp）
  hello_world::ahrs::Mahony mahony_;  ///< 姿态解算（现成 Mahony，见 mahony.hpp）
  ZeroDriftEstimator drift_;    ///< 开机零漂估计
  float bias_[3] = {0.0f, 0.0f, 0.0f};     ///< 零漂
  float euler_[3] = {0.0f, 0.0f, 0.0f};    ///< [roll pitch yaw]
  float gyro_z_ = 0.0f;                    ///< 扣零漂后 z 轴角速度
  uint32_t frames_after_calib_ = 0;        ///< 标定完成后累计帧数
  bool initialized_ = false;               ///< BMI088 init 成功
  bool calibrated_ = false;                ///< 零漂收敛
  bool ready_ = false;                     ///< 可进 Working
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
