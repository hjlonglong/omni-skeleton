/**
 *******************************************************************************
 * @file      :ins_imu.cpp
 * @brief     :IMU 实例（单例 getter）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.1.0      2026-10-24      Lead            1. 返回函数内 static Imu 实例
 *******************************************************************************
 * @attention :
 *  - Imu 类的定义/实现见 inc/imu.hpp、src/imu.cpp；本文件只负责实例化。
 *  - 函数内 static：首次 GetImu() 时构造（C++11 起线程安全），避免全局对象
 *    在 HAL/SPI 初始化前构造。
 *  - 真正的 init() 由 MainTaskInit() 调用，不要在 getter 里做阻塞初始化。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_imu.hpp"

#include "imu.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *GetImu()
{
  static hello_world::imu::Imu imu;
  return &imu;
}

/* Private function definitions ----------------------------------------------*/
