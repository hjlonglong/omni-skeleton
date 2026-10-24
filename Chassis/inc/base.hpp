/**
 *******************************************************************************
 * @file      : base.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_BASE_HPP_
#define HW_COMPONENTS_TOOLS_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>
#include <limits>

#include "allocator.hpp"
#include "assert.hpp"
#include "system.hpp"
namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
#ifndef PI
#define PI 3.14159265358979f
#endif /* PI */
/* Exported constants --------------------------------------------------------*/

static constexpr float kGravAcc = 9.7936f;       ///< 重力加速度（杭州）
static constexpr float kRad2DegCoff = 180 / PI;  ///< 弧度制转角度制系数
static constexpr float kDeg2RadCoff = PI / 180;  ///< 角度制转弧度制系数
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START
/* 周期数据转连续数据 */
class PeriodData2ContData : public MemMgr
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  PeriodData2ContData(void) = default;
  /**
   * @brief       周期数据转连续数据初始化
   * @param        period: 周期
   * @retval       None
   * @note        None
   */
  explicit PeriodData2ContData(float perioda);
  PeriodData2ContData(const PeriodData2ContData&) = default;
  PeriodData2ContData& operator=(const PeriodData2ContData& other);
  PeriodData2ContData(PeriodData2ContData&&) = default;
  PeriodData2ContData& operator=(PeriodData2ContData&& other);

  ~PeriodData2ContData(void) = default;

  /* 配置方法 */

  /**
   * @brief       周期数据转连续数据初始化，使用默认构造函数后请务必调用此函数
   * @param        period: 周期
   * @retval       None
   * @note        None
   */
  virtual void init(float period);

  /* 功能性方法 */

  /**
   * @brief       周期数据转连续数据
   * @param        data: 待转换的周期数据
   * @retval       转换后的连续数据
   * @note        None
   */
  float operator()(float data)
  {
    if (!is_init_) {
      last_data_ = data;
      is_init_ = true;
    }
    float delta_k = roundf((data - last_data_) / period_);
    last_data_ = data - delta_k * period_;
    return last_data_;
  }

  /**
   * @brief       重置转换器
   * @retval       None
   * @note        None
   */
  void reset(void) { is_init_ = false; }

 protected:
  float period_ = std::numeric_limits<float>::max();  ///< 周期
  float last_data_ = 0;                               ///< 上一次的数据
  bool is_init_ = false;                              ///< 是否初始化
};

/* 周期角度转连续角度，单位：rad */
class PeriodAngle2ContAngleRad : public PeriodData2ContData
{
 public:
  PeriodAngle2ContAngleRad(void) : PeriodData2ContData(2 * PI) {}
  PeriodAngle2ContAngleRad(const PeriodAngle2ContAngleRad&) = default;
  PeriodAngle2ContAngleRad& operator=(const PeriodAngle2ContAngleRad& other);
  PeriodAngle2ContAngleRad(PeriodAngle2ContAngleRad&&) = default;
  PeriodAngle2ContAngleRad& operator=(PeriodAngle2ContAngleRad&& other);

  ~PeriodAngle2ContAngleRad(void) = default;

  /**
   * @brief       无效化初始化函数，不允许修改周期
   * @param        period: 周期，无效参数
   * @retval       None
   * @note        None
   */
  virtual void init(float peroid) override {}
};

/* 周期角度转连续角度，单位：deg */
class PeriodAngle2ContAngleDeg : public PeriodData2ContData
{
 public:
  PeriodAngle2ContAngleDeg(void) : PeriodData2ContData(360) {}
  PeriodAngle2ContAngleDeg(const PeriodAngle2ContAngleDeg&) = default;
  PeriodAngle2ContAngleDeg& operator=(const PeriodAngle2ContAngleDeg& other);
  PeriodAngle2ContAngleDeg(PeriodAngle2ContAngleDeg&&) = default;
  PeriodAngle2ContAngleDeg& operator=(PeriodAngle2ContAngleDeg&& other);

  ~PeriodAngle2ContAngleDeg(void) = default;

  /**
   * @brief       无效化初始化函数，不允许修改周期
   * @param        period: 周期，无效参数
   * @retval       None
   * @note        None
   */
  virtual void init(float peroid) override {}
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       获取变量的符号
 * @param        x: 输入变量
 * @retval       输入变量的符号，当 x = 0 时返回 0
 * @note        只有支持大小比较的变量才可使用该函数
 */
template <typename T>
inline int8_t GetSign(T x)
{
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

/**
 * @brief       将输入值限制在一个范围内
 * @param        x: 待范围限制变量
 * @param        lim1: 范围边界1
 * @param        lim2: 范围边界2
 * @retval       范围限制后的值
 * @note        无需考虑两个范围边界的相对大小关系，变量需要支持大小比较
 */
template <typename T1, typename T2, typename T3>
inline T1 Bound(T1 x, T2 lim1, T3 lim2)
{
  float max_lim, min_lim;

  /* 设置上下限 */
  if (lim1 >= lim2) {
    max_lim = lim1;
    min_lim = lim2;
  } else {
    max_lim = lim2;
    min_lim = lim1;
  }

  /* 限制范围 */
  if (x > max_lim) {
    return max_lim;
  } else if (x < min_lim) {
    return min_lim;
  } else {
    return x;
  }
}

/**
 * @brief       判断变量是否位于给定的范围内
 * @param        bound1: 范围边界1（含）
 * @param        bound2: 范围边界2（含）
 * @retval       变量是否位于给定的范围内
 * @note        无需考虑两个范围边界的相对大小关系，变量需要支持大小比较
 */
template <typename T1, typename T2, typename T3>
inline bool IsInRange(T1 x, T2 bound1, T3 bound2)
{
  if (bound1 <= bound2) {
    return x >= bound1 && x <= bound2;
  } else {
    return x <= bound1 && x >= bound2;
  }
}

/**
 * @brief       将周期性数据归一化到指定周期区间
 * @param        period_lb: 周期区间下界（含）
 * @param        period_ub: 周期区间上界
 * @param        data: 待归一化数据
 * @retval       归一化后的数据
 * @note        period_lb 必须小于 period_ub
 */
inline float NormPeriodData(float period_lb, float period_ub, float data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period_ub > period_lb, "lower bound is larger than upper bound");
#pragma endregion

  float period = period_ub - period_lb;
  float tmp = fmodf(data - period_lb, period);  // (-period, period)

  if (tmp < 0) {
    return tmp + period + period_lb;
  } else {
    return tmp + period_lb;
  }
}

/**
 * @brief       将待处理数据平移到参考数据的所在周期中
 * @param        period_lb: 周期区间下界（含）
 * @param        period_ub: 周期区间上界
 * @param        ref_data: 参考数据（无需再周期区间之内）
 * @param        data: 待处理数据
 * @retval       平移后的数据
 * @note        period_lb 必须小于 period_ub
 */
inline float PeriodData2SameRegion(float period_lb, float period_ub,
                                   float ref_data, float data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period_ub > period_lb, "lower bound is larger than upper bound");
#pragma endregion

  float period = period_ub - period_lb;

  float delta_k = floorf((ref_data - period_lb) / period) -
                  floorf((data - period_lb) / period);
  return data + delta_k * period;
}

/**
 * @brief       将待处理数据平移到距离参考数据最近的值
 * @param        period: 周期
 * @param        ref_data: 参考数据
 * @param        data: 待处理数据
 * @retval       平移后的数据
 * @note        None
 */
inline float PeriodData2NearestDist(float period, float ref_data, float data)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period > 0, "period <= 0");
#pragma endregion

  float delta_k = roundf((ref_data - data) / period);
  return data + delta_k * period;
}

/**
 * @brief       将角度归一化到 [-π, π)
 * @param        angle: 待归一化角度，单位：rad
 * @retval       归一化后的角度，单位：rad
 * @note        None
 */
inline float AngleNormRad(float angle)
{
  return NormPeriodData(-PI, PI, angle);
}

/**
 * @brief       将角度归一化到 [-180°, 180°)
 * @param        angle: 待归一化角度，单位：deg
 * @retval       归一化后的角度，单位：deg
 * @note        None
 */
inline float AngleNormDeg(float angle)
{
  return NormPeriodData(-180.0f, 180.0f, angle);
}

/**
 * @brief       处理周期性数据的差值
 * @param        minuend: 被减数
 * @param        subtrahend: 减数
 * @param        period: 周期
 * @retval       返回处理后的差值，差值的范围在 -period / 2 到 period / 2 之间
 * @note        None
 */
inline float PeriodDataSub(float minuend, float subtrahend, float period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(period > 0, "val must be larger than 0");
#pragma endregion

  float diff = minuend - subtrahend;
  float times = roundf(diff / period);
  float res = diff - times * period;

  return res;
}

/**
 * @brief       将周期性数据环形限幅到指定区间
 * @param        data: 待处理数据 
 * @param        bound_lb: 限幅区间下界（含）
 * @param        bound_ub: 限幅区间上界
 * @param        period_lb: 周期区间下界（含）
 * @param        period_ub: 周期区间上界
 * @retval       限幅后的数据
 * @note        period_lb 必须小于 period_ub
 *              bound_lb 小于 bound_ub 时，data 限幅到 [bound_lb, bound_ub)
 *              bound_lb 大于 bound_ub 时，data 限幅到 [bound_lb, period_ub) ∪ [period_lb, bound_ub)
 */
inline float BoundPeriodData(float data, float bound_lb, float bound_ub, float period_lb, float period_ub)
{
#pragma region
  HW_ASSERT(period_ub > period_lb, "lower bound is larger than upper bound");
#pragma endregion

  float norm_bound_lb = NormPeriodData(period_lb, period_ub, bound_lb);
  float norm_bound_ub = NormPeriodData(period_lb, period_ub, bound_ub);
  float norm_data = NormPeriodData(period_lb, period_ub, data);
  float period = period_ub - period_lb;

  // 判断是否在限幅区间内
  if (norm_bound_lb < norm_bound_ub) {
    if (norm_data >= norm_bound_lb && norm_data < norm_bound_ub) {
      return norm_data;
    }
  } else {  // 跨周期区间
    if (norm_data >= norm_bound_lb || norm_data < norm_bound_ub) {
      return norm_data;
    }
  }

  // 超出范围，返回最近端点
  float dist_to_lb = fabsf(PeriodDataSub(norm_data, norm_bound_lb, period));
  float dist_to_ub = fabsf(PeriodDataSub(norm_data, norm_bound_ub, period));

  return (dist_to_lb <= dist_to_ub) ? norm_bound_lb : norm_bound_ub;
}

/**
 * @brief       将周期性数据环形限幅到 [-PI, PI)
 * @param        data: 待处理数据
 * @param        bound_lb: 限幅区间下界（含）
 * @param        bound_ub: 限幅区间上界
 * @retval       限幅后的数据
 * @note        bound_lb 小于 bound_ub 时，data 限幅到 [bound_lb, bound_ub)
 *              bound_lb 大于 bound_ub 时，data 限幅到 [bound_lb, PI) ∪ [-PI, bound_ub)
 */
inline float BoundDataRad(float data, float bound_lb, float bound_ub) { return BoundPeriodData(data, bound_lb, bound_ub, -PI, PI); }

/**
 * @brief       将周期性数据环形限幅到 [-180°, 180°)
 * @param        data: 待处理数据
 * @param        bound_lb: 限幅区间下界（含）
 * @param        bound_ub: 限幅区间上界
 * @retval       限幅后的数据
 * @note        bound_lb 小于 bound_ub 时，data 限幅到 [bound_lb, bound_ub)
 *              bound_lb 大于 bound_ub 时，data 限幅到 [bound_lb, 180°) ∪ [-180°, bound_ub)
 */
inline float BoundDataDeg(float data, float bound_lb, float bound_ub) { return BoundPeriodData(data, bound_lb, bound_ub, -180.0f, 180.0f); }

/**
 * @brief       处理角度跨越 0 的情况，单位：rad
 * @param        minuend: 被减数，也就是目标角度
 * @param        subtrahend: 减数，也就是实际反馈的角度值
 * @retval       返回处理后的角度值，角度值的范围在 -PI 到 PI 之间
 * @note        None
 */
inline float HandleAngleCross0Rad(float minuend, float subtrahend)
{
  return subtrahend + PeriodDataSub(minuend, subtrahend, PI * 2);
}

/**
 * @brief       处理角度跨越 0 的情况，单位：deg
 * @param        minuend: 被减数，也就是目标角度
 * @param        subtrahend: 减数，也就是实际反馈的角度值
 * @retval       返回处理后的角度值，角度值的范围在 -180 到 180 之间
 * @note        None
 */
inline float HandleAngleCross0Deg(float minuend, float subtrahend)
{
  return subtrahend + PeriodDataSub(minuend, subtrahend, 360.0f);
}

/**
 * @brief       将弧度制角度转换为角度制
 * @param        angle: 待转换角度，单位：rad
 * @retval       输入角度的角度制表示
 * @note        None
 */
inline float Rad2Deg(float angle) { return angle * kRad2DegCoff; }

/**
 * @brief       将角度制角度转换为弧度制
 * @param        angle: 待转换角度，单位：deg
 * @retval       输入角度的弧度制表示
 * @note        None
 */
inline float Deg2Rad(float angle) { return angle * kDeg2RadCoff; }

/**
 * @brief       设置变量的某些位为 1
 * @param        mask: 位掩码
 * @param        data: 待设置的变量
 * @retval       None
 * @note        类型 T1 与 T2 必须支持位运算 |
 */
template <typename T1, typename T2>
inline void SetBits(const T1& mask, T2& data)
{
  data = T2(data | mask);
}

/**
 * @brief       判断变量的某些位是否为 1
 * @param        mask: 位掩码
 * @param        data: 待判断的变量
 * @retval       变量的某些位是否为1
 * @note        类型 T1 与 T2 必须支持位运算 & 与 ==
 */
template <typename T1, typename T2>
inline bool IsBitsSet(const T1& mask, const T2& data)
{
  return (data & mask) == mask;
}

/**
 * @brief       清除变量的某些位为 0
 * @param        mask: 位掩码
 * @param        data: 待清除的变量
 * @retval       None
 * @note        类型 T1 与 T2 必须支持位运算 & 与 ~
 */
template <typename T1, typename T2>
inline void ClearBits(const T1& mask, T2& data)
{
  data = T2(data & (~mask));
}
HW_OPTIMIZE_O2_END
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_BASE_HPP_ */
