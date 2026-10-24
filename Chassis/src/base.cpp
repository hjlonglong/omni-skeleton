/**
 *******************************************************************************
 * @file      : base.cpp
 * @brief     : 提供一些基础的工具类、函数以及常量定义
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "base.hpp"

namespace hello_world
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

HW_OPTIMIZE_O2_START

PeriodData2ContData::PeriodData2ContData(float period)
    : period_(period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period > 0, "period <= 0");
#pragma endregion
}

PeriodData2ContData& PeriodData2ContData::operator=(
    const PeriodData2ContData& other)
{
  if (this != &other) {
    period_ = other.period_;
    last_data_ = other.last_data_;
  }
  return *this;
}

PeriodData2ContData& PeriodData2ContData::operator=(
    PeriodData2ContData&& other)
{
  if (this != &other) {
    period_ = other.period_;
    last_data_ = other.last_data_;
  }
  return *this;
}

void PeriodData2ContData::init(float period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period > 0, "period <= 0");
#pragma endregion

  period_ = period;
  is_init_ = false;
}

PeriodAngle2ContAngleRad& PeriodAngle2ContAngleRad::operator=(
    const PeriodAngle2ContAngleRad& other)
{
  if (this != &other) {
    last_data_ = other.last_data_;
  }
  return *this;
}

PeriodAngle2ContAngleRad& PeriodAngle2ContAngleRad::operator=(
    PeriodAngle2ContAngleRad&& other)
{
  if (this != &other) {
    last_data_ = other.last_data_;
  }
  return *this;
}

PeriodAngle2ContAngleDeg& PeriodAngle2ContAngleDeg::operator=(
    const PeriodAngle2ContAngleDeg& other)
{
  if (this != &other) {
    last_data_ = other.last_data_;
  }
  return *this;
}

PeriodAngle2ContAngleDeg& PeriodAngle2ContAngleDeg::operator=(
    PeriodAngle2ContAngleDeg&& other)
{
  if (this != &other) {
    last_data_ = other.last_data_;
  }
  return *this;
}
/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace hello_world
