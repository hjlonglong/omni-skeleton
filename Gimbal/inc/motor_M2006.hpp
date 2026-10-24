/**
 *******************************************************************************
 * @file      : motor_M2006.hpp
 * @brief     : 大疆 M2006 电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.1.0      2024-07-11      Caikunzhen      1. 完成正式版
 *  V2.0.0      2026-09-18      Hello World     1. 改装版：去通信耦合，暴露纯字节接口
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项
 *  2. 电机 ID 范围为 1~8，其中 1~4 为同一条发送报文，5~8 为同一条发送报文
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq 和 InputType::kCurr
 *  4. 需结合 C610 电调使用
 *  5. 改装版中 decode/encode 为纯字节接口：新人需在自己的 CAN 层根据反馈 ID 判断报文
 *     归属后再调用 decode，并根据 GetTxId 将 encode 出的 8 字节填入对应 CAN 帧发出
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"
#include "system.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

HW_OPTIMIZE_O2_START

class M2006 : public Motor
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  M2006(void) = default;

  /**
   * @brief       M2006 初始化
   * @param        id: 电机 ID，1~8
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  explicit M2006(
      uint8_t id, const OptionalParams& opt = OptionalParams());
  M2006(const M2006&) = default;
  M2006& operator=(const M2006& other);
  M2006(M2006&& other);
  M2006& operator=(M2006&& other);

  virtual ~M2006(void) = default;

  /* 重载方法 */

  /**
   * @brief       将电调发回的 8 字节 CAN 反馈报文进行解包
   * @param        data: 电调发回的 8 字节 CAN 报文
   * @retval       是否解包成功
   * @note        新人需先在自己的 CAN 层根据反馈 ID 判断报文归属，再调用此接口
   */
  virtual bool decode(const uint8_t data[8]) override;

  /**
   * @brief       将要发给电调的期望输入编码为 8 字节 CAN 控制报文
   * @param        data: 输出的 8 字节 CAN 报文缓冲区
   * @retval       是否编码成功
   * @note        新人需根据该电机所属发送 ID（见 GetTxId）将 data 填入 CAN 帧发出
   */
  virtual bool encode(uint8_t data[8]) override;

  /* 配置方法 */

  /**
   * @brief       M2006 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，1~8
   * @param        opt: 电机可选配置参数
   * @retval       None
   * @note        None
   */
  void init(uint8_t id, const OptionalParams& opt = OptionalParams());

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 1~8
   * @retval       对应的发送 ID
   * @note        1~4 为同一条发送报文，5~8 为同一条发送报文
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
#pragma endregion

    if (id <= 4) {
      return kTx1_4_;
    } else {
      return kTx5_8_;
    }
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 1~8
   * @retval       对应的接收 ID
   * @note        None
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 8, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

 private:
  static constexpr uint32_t kTx1_4_ = 0x200;
  static constexpr uint32_t kTx5_8_ = 0x1FF;
  static constexpr uint32_t kRx0_ = 0x200;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_ */
