/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 封装类实现（WP-06 骨架）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2026-10-24      Lead            1. 骨架：init/update 分段留 TODO(STEP)
 *******************************************************************************
 * @attention :
 *  - 两板同源文件（Chassis/src/imu.cpp 与 Gimbal/src/imu.cpp 逐字节一致）。
 *  - 业务逻辑不要写进中断里做阻塞操作；init() 只在 MainTaskInit() 调用。
 *  - 本文件是唯一引用板级 SPI/GPIO 句柄的位置，方便两板共用同一份 Imu 类。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

#include "main.h"
#include "spi.h"

namespace hello_world
{
namespace imu
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void Imu::init()
{
  /* ===== STEP 1：BMI088 硬件配置（SPI1 + 两个片选） =====
   * 假设（WP-06 任务卡）：SPI1；ACC CS = PA4；GYRO CS = PB0；
   * 两板 gpio.c 已把它们配成推挽输出且默认拉高。
   * TODO(硬件)：焊好后先用示波器看 CS 波形 / 读芯片 ID 再确认，不要想当然。 */
  BMI088HWConfig hw_config;
  hw_config.hspi = &hspi1;
  hw_config.acc_cs_port = GPIOA;
  hw_config.acc_cs_pin = GPIO_PIN_4;    // TODO(硬件): 加速度计 CS
  hw_config.gyro_cs_port = GPIOB;
  hw_config.gyro_cs_pin = GPIO_PIN_0;   // TODO(硬件): 陀螺仪 CS

  /* 安装方向：单位阵 = 芯片坐标与车体坐标同向（x 前 y 左 z 上）。
   * TODO(硬件)：若 IMU 装反/旋转 90°，改这里（行优先 3x3），不要改算法。 */
  static const float kRotMatIdentity[9] = {1.0f, 0.0f, 0.0f,
                                           0.0f, 1.0f, 0.0f,
                                           0.0f, 0.0f, 1.0f};
  bmi_.init(hw_config, kRotMatIdentity, BMI088Config());

  /* ===== STEP 2：芯片初始化 + 自检 =====
   * imuInit() 会读芯片 ID、写配置寄存器，阻塞数毫秒；返回错误位或 0。
   * TODO(你)：STEP 2 完成后用 RTT 打印这个返回值，确认 == kBMI088ErrStateNoErr。 */
  const BMI088ErrState err = bmi_.imuInit(false);
  initialized_ = (err == kBMI088ErrStateNoErr);
  (void)err;  // 暂未使用，避免编译告警；STEP 2 改成 RTT 打印

  /* ===== STEP 3：Mahony 初始化（采样率必须与 1 ms 调度一致） ===== */
  mahony_.init(kSampleFreqHz, kMahonyKp, kMahonyKi);

  resetState();
}

void Imu::update()
{
  /* 初始化失败时不做任何事：isReady() 保持 false，上层不会进 Working。 */
  if (!initialized_) {
    return;
  }

  float acc[3] = {0.0f, 0.0f, 0.0f};
  float gyro_raw[3] = {0.0f, 0.0f, 0.0f};
  bmi_.getData(acc, gyro_raw, nullptr);  // acc: m/s²（含重力）；gyro: rad/s

  if (!calibrated_) {
    if (drift_.addSample(gyro_raw)) {        // 窗口满（kCalibFrames=750 帧）
      if (drift_.isStationary()) {           // 三轴均值/标准差都足够小
        drift_.bias(bias_);                  // 取均值当零漂
        calibrated_ = true;
      } else {
        drift_.reset();                      // 有运动，重新采 0.75 s
      }
  }
  return;                                  // 标定期间不做 Mahony 更新
}

  /* ===== STEP 5：扣零漂 + Mahony 姿态更新（TODO 你来实现） =====
   * 提示：
   *   float gyro[3];
   *   for (int i = 0; i < 3; ++i) { gyro[i] = gyro_raw[i] - bias_[i]; }
   *   mahony_.update(acc, gyro);        // acc 必须含重力（getData 已给 m/s²）
   *   mahony_.getEulerAngle(euler_);    // [roll pitch yaw]，rad
   *   gyro_z_ = gyro[2];
   */
  (void)acc;
  (void)gyro_raw;

  /* ===== STEP 6：就绪判定（TODO 你来实现） =====
   * 提示：标定完成后再跑 kReadyExtraFrames 帧，姿态稳定后才允许进 Working：
   *   if (++frames_after_calib_ >= kReadyExtraFrames) { ready_ = true; }
   * 注意：ready_ 一旦置 true 不要自动清零；异常复位由上层重启流程处理。 */
}

/* Private function definitions ----------------------------------------------*/

void Imu::resetState()
{
  drift_.reset();
  for (int i = 0; i < 3; ++i) {
    bias_[i] = 0.0f;
    euler_[i] = 0.0f;
  }
  gyro_z_ = 0.0f;
  frames_after_calib_ = 0;
  calibrated_ = false;
  ready_ = false;
}

}  // namespace imu
}  // namespace hello_world
