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
 *  V1.1.0      2026-11-01      A1              1. 补全 Chassis 板 STEP 5/6（姿态更新与
 *                                                就绪判定）；两板 src/imu.cpp 逐字节同源
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
/* ===== Ozone Live Watch 调试镜像（2026-11-08 新增） =====
 * 原因：`GetImu()->euler_[0]` 这类表达式含"函数调用 + 函数内 static 局部变量"，
 * Ozone 的 Live Watch/Graph 不能执行目标代码，会报
 * "the expression contains at least one operand with an unknown data location"。
 * 解决：每 1 ms 把 Imu 关键状态镜像到这些普通全局变量（只写不参与控制）。
 * 用法：Ozone Live Watch 直接加 `g_dbg_imu_*`，即可连续采样/画曲线。 */
volatile uint8_t g_dbg_imu_init_ok = 0U;    ///< 1 = BMI088 初始化成功
volatile uint8_t g_dbg_imu_calib_ok = 0U;   ///< 1 = 零漂标定完成
volatile uint8_t g_dbg_imu_ready_flag = 0U; ///< 1 = 姿态就绪（可进 Working）
volatile uint32_t g_dbg_imu_drift_n = 0U;   ///< 标定窗口当前样本数（0~750 循环）
volatile uint32_t g_dbg_imu_frames = 0U;    ///< 标定完成后累计帧数
volatile float g_dbg_imu_bias_x = 0.0f;     ///< 零漂估计 rad/s
volatile float g_dbg_imu_bias_y = 0.0f;
volatile float g_dbg_imu_bias_z = 0.0f;
volatile float g_dbg_imu_roll = 0.0f;       ///< rad
volatile float g_dbg_imu_pitch = 0.0f;      ///< rad
volatile float g_dbg_imu_yaw = 0.0f;        ///< rad
volatile float g_dbg_imu_gyro_z = 0.0f;     ///< rad/s（扣零漂后）
volatile float g_dbg_imu_quat_w = 1.0f;     ///< 四元数 w/x/y/z（模应≈1）
volatile float g_dbg_imu_quat_x = 0.0f;
volatile float g_dbg_imu_quat_y = 0.0f;
volatile float g_dbg_imu_quat_z = 0.0f;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void Imu::init()
{
  /* ===== STEP 1：BMI088 硬件配置（SPI1 + 两个片选） =====
   * 两板一致：SPI1；ACC CS = PA4；GYRO CS = PB0；
   * 两板 gpio.c 已把它们配成推挽输出且默认拉高。
   * TODO(硬件)：焊好后先用示波器看 CS 波形 / 读芯片 ID 再确认，不要想当然。 */
  BMI088HWConfig hw_config;
  hw_config.hspi = &hspi1;
  hw_config.acc_cs_port = GPIOA;
  hw_config.acc_cs_pin = GPIO_PIN_4;    // TODO(硬件): 加速度计 CS
  hw_config.gyro_cs_port = GPIOB;
  hw_config.gyro_cs_pin = GPIO_PIN_0;   // TODO(硬件): 陀螺仪 CS

  /* 安装方向：单位阵 = 芯片坐标与车体坐标同向（x 前 y 左 z 上）。
   * 两板当前安装方向一致；若某板装反/旋转 90°，只改这里的 rot_mat（行优先 3x3），
   * 不要改算法。TODO(硬件)：实车确认。 */
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
  /* 调试镜像：初始化状态每帧刷新（未初始化时也刷新，便于定位硬件问题）。 */
  g_dbg_imu_init_ok = initialized_ ? 1U : 0U;

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
    /* 调试镜像：标定期间的值 */
    g_dbg_imu_calib_ok = 0U;
    g_dbg_imu_ready_flag = 0U;
    g_dbg_imu_drift_n = drift_.count();
    g_dbg_imu_frames = frames_after_calib_;
    g_dbg_imu_bias_x = bias_[0];
    g_dbg_imu_bias_y = bias_[1];
    g_dbg_imu_bias_z = bias_[2];
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
  gyro_z_ = gyro[2];  // yaw 阻尼/角速度环直接用陀螺 z 轴（两板通用）

  /* ===== STEP 6：就绪判定 =====
   * 标定完成后再跑 kReadyExtraFrames（100 帧 = 0.1 s）等 Mahony 收敛，
   * 才允许状态机进 Working；ready_ 一旦置位不自动清零（上层重启才复位）。 */
  if (!ready_ && ++frames_after_calib_ >= kReadyExtraFrames) {
    ready_ = true;
  }

  /* 调试镜像：标定完成后的完整状态（Ozone Live Watch 直接看这些全局变量）。 */
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  mahony_.getQuat(quat);
  g_dbg_imu_calib_ok = calibrated_ ? 1U : 0U;
  g_dbg_imu_ready_flag = ready_ ? 1U : 0U;
  g_dbg_imu_drift_n = drift_.count();
  g_dbg_imu_frames = frames_after_calib_;
  g_dbg_imu_bias_x = bias_[0];
  g_dbg_imu_bias_y = bias_[1];
  g_dbg_imu_bias_z = bias_[2];
  g_dbg_imu_roll = euler_[0];
  g_dbg_imu_pitch = euler_[1];
  g_dbg_imu_yaw = euler_[2];
  g_dbg_imu_gyro_z = gyro_z_;
  g_dbg_imu_quat_w = quat[0];
  g_dbg_imu_quat_x = quat[1];
  g_dbg_imu_quat_y = quat[2];
  g_dbg_imu_quat_z = quat[3];
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
