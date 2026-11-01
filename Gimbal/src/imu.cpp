/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 封装类实现（BMI088 原始数据 + 零漂标定 + Mahony 姿态解算）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2026-10-24      Lead            1. 骨架：init/update 分段留 TODO(STEP)
 *  V1.0.0      2026-10-28      Lead            1. 补全 STEP 4/5/6：零漂标定、扣零漂 +
 *                                                Mahony 更新、就绪判定；修正 Mahony
 *                                                命名空间（hello_world::ahrs）
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
   * 返回非 0 时 initialized_ 保持 false，update() 直接返回、isReady() 恒 false。 */
  const BMI088ErrState err = bmi_.imuInit(false);
  initialized_ = (err == kBMI088ErrStateNoErr);
  (void)err;  // 预留：接 RTT 后打印 err 便于定位硬件问题

  /* ===== STEP 3：Mahony 初始化（采样率必须与 1 ms 调度一致） =====
   * 注意：Mahony 在 hello_world::ahrs 命名空间（见 mahony.hpp）。 */
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
    /* ===== STEP 4：开机零漂标定 =====
     * 静止采样 kCalibFrames（750 帧 = 0.75 s）角速度；
     * 均值做零漂，同时用均值/标准差判静止；有运动则重采。 */
    if (drift_.addSample(gyro_raw)) {          // 窗口满
      if (drift_.isStationary()) {             // 确认静止
        drift_.bias(bias_);                    // 取窗口均值作为零漂
        calibrated_ = true;                    // 标定完成
      } else {
        drift_.reset();                        // 有运动 → 丢弃窗口重采
      }
    }
    return;  // 标定期间不跑 Mahony，也不置 ready_
  }

  /* ===== STEP 5：扣零漂 + Mahony 姿态更新 =====
   * acc 必须含重力（getData 已给 m/s²），不要归一化/乘 9.8；
   * gyro 为 rad/s。Mahony 输出 Z-Y-X 欧拉角 [roll pitch yaw]，单位 rad。 */
  float gyro[3];
  for (int i = 0; i < 3; ++i) {
    gyro[i] = gyro_raw[i] - bias_[i];
  }
  mahony_.update(acc, gyro);
  mahony_.getEulerAngle(euler_);
  gyro_z_ = gyro[2];  // 底盘 yaw 阻尼/角速度环直接用陀螺 z 轴

  /* ===== STEP 6：就绪判定 =====
   * 标定完成后再跑 kReadyExtraFrames（100 帧 = 0.1 s）等 Mahony 收敛，
   * 才允许状态机进 Working；ready_ 一旦置位不自动清零（上层重启才复位）。 */
  if (!ready_ && ++frames_after_calib_ >= kReadyExtraFrames) {
    ready_ = true;
  }
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
