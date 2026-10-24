/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（新人填写）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类，
 *                                                只保留 BMI088 芯片驱动 + Mahony 姿态
 *******************************************************************************
 * @attention :
 *  本文件已掏空。新人需基于以下两个已提供的组件自行实现 Imu 封装类：
 *    - BMI088：芯片级原始读取（加速度 m/s² / 角速度 rad/s / 温度）
 *    - ahrs::Mahony：姿态解算（输入角速度 + 加速度，输出四元数/姿态角）
 *  需要新人实现：
 *    - 开机零漂计算（静止采样求角速度均值）
 *    - 姿态角输出 roll()/pitch()/yaw()
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"

/* ============================================================================
 * 【新人任务】在本文件编写 Imu 封装类（namespace hello_world::imu）。
 *
 * 你要做：
 *   1. 组合已提供的 BMI088（芯片原始读取，加速度 m/s² / 角速度 rad/s，见 BMI088.hpp）
 *      与 Mahony（姿态解算 update(acc, gyro) → getEulerAngle，见 mahony.hpp / ahrs.hpp）。
 *   2. 实现开机零漂：静止采样一段时间的角速度均值，后续每帧扣除该零漂再送入 Mahony。
 *   3. 对外提供 roll() / pitch() / yaw()（单位 rad），以及标定是否完成的查询。
 *
 * 约束 / 提示：
 *   - Mahony 的加速度输入需含重力加速度项，单位与坐标系（x 前 y 左 z 上）见 ahrs_base.hpp。
 *   - 零漂未收敛前，底盘 / 云台状态机应保持在 Dead / Resurrection，不得进入 Working。
 *   - 本骨架只提供了 imu.hpp（无 imu.cpp）；若需 .cpp 实现文件，可自行在 src/ 下新增
 *     imu.cpp（CMake 已递归 glob *.c*，无需改配置）。
 * ==========================================================================*/

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
