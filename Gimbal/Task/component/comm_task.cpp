/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 通讯周期任务（CAN 发送：DM 电机 MIT 帧 + G2C 双板帧 + 喂狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-07      Agent C         1. WP-08/16：CAN 启动、电机帧、G2C 帧、IWDG
 *******************************************************************************
 * @attention :
 *  1. 本板不接 DBUS 遥控器：遥控状态经底盘板 C2G 帧下发，故无 UART 空闲/DMA 接收
 *     （ins_comm.cpp 的 GetRemoteControl() 恒返回 nullptr，契约 §7.1/§7.4）。
 *  2. CommTaskInit 中必须先构造电机/双板通讯对象，再开 CAN 接收中断：
 *     中断回调里会用到这些函数内 static 对象。
 *  3. 本任务 1ms 执行一次：两轴 DM 各一帧 + G2C 一帧；IWDG 只在这里喂
 *     （DEBUG_NO_IWDG=0 交付态）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "config.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"
#include "iwdg.h"
#include "main.h"
#include "motor_DaMiao.hpp"

/* Private constants ---------------------------------------------------------*/
namespace
{
constexpr int kAxisNum = 2;                 ///< 云台轴数：yaw / pitch
constexpr uint32_t kCanFrameLen = 8;        ///< 标准帧数据长度
constexpr uint32_t kCanErrIrqMask =
    CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING |
    CAN_IT_LAST_ERROR_CODE;  ///< 需要开中断的 CAN 错误类型
}  // namespace

/* 固件标识（2026-11-08 云台 IMU 检测版）：Ozone 里确认烧的是本次 ELF。
 * 必须 used+retain，否则未被引用的全局会被 --gc-sections 删除，Ozone 会把它
 * 解析到无效地址（读到 0x20020000 = 初始栈顶），看起来像"烧错固件"。 */
__attribute__((used, retain)) volatile uint32_t g_dbg_fw_tag = 0x20261116u;

/* DM 电机状态镜像（Ozone Live Watch；每 1 ms 刷新，只读不影响控制）----------
 * 数组索引：0 = yaw，1 = pitch（与 GetMotorYaw()/GetMotorPitch() 顺序一致）。
 *  - vel/curr/torq/angle：按电机反馈解码（rad/s、A、N·m、rad）；
 *  - input ：本周期编码进 MIT 帧的目标量（Robot 下发，纯力矩模式为 N·m）；
 *  - enabled：DM 反馈帧里的使能状态；rotor/mos temp：温度 ℃。 */
volatile float g_dbg_dm_vel[2] = {0.0f};
volatile float g_dbg_dm_curr[2] = {0.0f};
volatile float g_dbg_dm_torq[2] = {0.0f};
volatile float g_dbg_dm_angle[2] = {0.0f};
volatile float g_dbg_dm_input[2] = {0.0f};
volatile uint8_t g_dbg_dm_rotor_temp[2] = {0};
volatile uint8_t g_dbg_dm_mos_temp[2] = {0};
volatile uint8_t g_dbg_dm_enabled[2] = {0};
/* DM 状态码（反馈 data[0]>>4）：0=失能 1=使能 8=过压 9=欠压 A=过流
 * B=MOS过温 C=线圈过温 D=通信丢失 E=过载；非 0/1 即电机进入保护。 */
volatile uint8_t g_dbg_dm_status_code[2] = {0};
volatile uint32_t g_dbg_can1_tx_cnt = 0;  ///< CAN1（DM 电机帧）发送次数
volatile uint32_t g_dbg_can2_tx_cnt = 0;  ///< CAN2（G2C 双板帧）发送次数

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* 1. 先构造电机/双板通讯单例（函数内 static，CAN 中断打开前必须已存在） */
  (void)GetMotorYaw();
  (void)GetMotorPitch();
  (void)GetGimbalChassisComm();

  /* 2. 初始化 CAN1（云台电机总线）/ CAN2（双板总线）滤波器并启动外设 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }

  /* 3. 打开接收中断 + 错误中断（收完一帧后在回调内重新激活） */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
                                                kCanErrIrqMask) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING |
                                                kCanErrIrqMask) != HAL_OK) {
    Error_Handler();
  }

  /* 4. IWDG 已在 main.c 的 MX_IWDG_Init() 中启动，阻塞初始化结束后由
   *    MainTaskInit 统一喂一次；本任务负责运行期周期喂狗。 */
  /* 5. 调试镜像：写入固件标识，便于 Ozone 确认已烧录本次云台 ELF。 */
  g_dbg_fw_tag = 0x20261116u;
}

void CommTask(void)
{
  uint8_t tx_data[kCanFrameLen] = {0};
  hello_world::motor::Motor *const motors[kAxisNum] = {GetMotorYaw(),
                                                       GetMotorPitch()};

  /* 1. 云台两轴 DM 电机：encode 一帧（使能/MIT 纯力矩/模式切换）后经 CAN1 发出 */
  for (int i = 0; i < kAxisNum; ++i) {
    hello_world::motor::Motor *motor = motors[i];
    if (motor == nullptr) {
      continue;
    }
    if (motor->encode(tx_data)) {
      const auto *dm = static_cast<hello_world::motor::DaMiao *>(motor);
      CAN_Send_Msg(&hcan1, tx_data, dm->tx_id(),
                   static_cast<uint8_t>(kCanFrameLen));
      g_dbg_can1_tx_cnt += 1u;
    }
  }

  /* 2. 双板通讯：Robot 已填好 G2C 数据，encode 后经 CAN2 发出（契约 §3.2 / §8） */
  robot::GimbalChassisComm *const comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    comm->encode(tx_data);
    CAN_Send_Msg(&hcan2, tx_data, comm->tx_id(),
                 static_cast<uint8_t>(kCanFrameLen));
    g_dbg_can2_tx_cnt += 1u;
  }

  /* 3. 电机状态镜像（只读，供 Ozone 检查 CAN 反馈 / 使能 / 温度） */
  for (int i = 0; i < kAxisNum; ++i) {
    hello_world::motor::Motor *const motor = motors[i];
    if (motor == nullptr) {
      continue;
    }
    const auto *dm = static_cast<hello_world::motor::DaMiao *>(motor);
    g_dbg_dm_vel[i] = motor->vel();
    g_dbg_dm_curr[i] = motor->curr();
    g_dbg_dm_torq[i] = motor->torq();
    g_dbg_dm_angle[i] = motor->angle();
    g_dbg_dm_input[i] = motor->getInput();
    g_dbg_dm_rotor_temp[i] = dm->rotor_temp();
    g_dbg_dm_mos_temp[i] = dm->mos_temp();
    g_dbg_dm_enabled[i] = dm->is_enabled() ? 1u : 0u;
    g_dbg_dm_status_code[i] = static_cast<uint8_t>(dm->status_code());
  }

  /* 3. 喂独立看门狗（1ms 周期，600ms 超时；调试期可置 DEBUG_NO_IWDG=1 关闭） */
#if DEBUG_NO_IWDG == 0
  (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}
