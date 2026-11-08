/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :底盘通讯任务：电机帧/双板帧 CAN 收发 + 遥控 DBUS 接收 + IWDG 喂狗
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *  V1.0.0      2026-10-21      Lead            1. CAN 收发通道：0x200 单帧发送、
 *                                                CAN1/CAN2 滤波器与中断启动
 *  V1.1.0      2026-11-07      Agent B         1. WP-09/10/15/17：DBUS 18 字节空闲+
 *                                                DMA 接收、C2G 双板帧每周期发送、
 *                                                IWDG 1 ms 链末喂狗
 *******************************************************************************
 * @attention :
 *  - CommTask() 运行在 TIM6 1 ms 中断里，由 main_task.cpp 的 TIM6 回调调用；
 *    每周期必发电机帧与 C2G 帧（即使全 0），电调/云台板据此判在线。
 *  - 发送顺序保持「先算后发」：MainTask（IMU/状态机）→ CommTask。
 *  - 接收解码全部在中断回调里：CAN 帧由 src/HW_can.cpp 按 StdId 分发
 *    （电机 decode / G2C 双板帧 decode），本文件只负责 DBUS 串口接收；
 *    不要在 CommTask 里轮询接收。
 *  - DBUS：USART3 空闲 + DMA 接收 18 字节。Core/Src/usart.c 中 RX DMA 配置为
 *    CIRCULAR，TC 时 HAL 回调 Size == kRcRxDataLen（18），返回后自动继续接收，
 *    无需重启；若改为 NORMAL 模式，必须在此回调末尾重新调用
 *    HAL_UARTEx_ReceiveToIdle_DMA()。
 *  - IWDG 已在 main.c 的 MX_IWDG_Init() 启动（契约 §8.2），只能在本任务末尾喂狗；
 *    DEBUG_NO_IWDG == 1 时停喂（仅限台架 + 调试器，交付态必须为 0）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "config.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_comm.hpp"
#include "ins_motor.hpp"
#include "iwdg.h"
#include "usart.h"

/* Private constants ---------------------------------------------------------*/
namespace
{
/* 轮电机数量（索引顺序 LF、LB、RB、RF，见 config.hpp） */
constexpr int kWheelNum = config::kWheelNum;
/* DBUS 单帧长度（18 字节，见 inc/DT7.hpp） */
constexpr uint16_t kRcRxLen = hello_world::remote_control::kRcRxDataLen;

/* DBUS 接收缓冲（DMA 循环写入；回调里仅在 Size==18 时解码，避免半帧出错） */
uint8_t g_rc_rx_buf[kRcRxLen];
}  // namespace

/* DBUS / 双板帧调试计数（Ozone Live Watch 可直接观察） */
volatile uint32_t g_dbg_dbus_rx_ok = 0;   ///< 成功解包的 DBUS 帧数
volatile uint32_t g_dbg_dbus_rx_bad = 0;  ///< Size 异常（半帧/错帧）次数
volatile uint32_t g_dbg_c2g_tx_cnt = 0;   ///< C2G 双板帧发送次数
/* 固件标识（2026-11-08 DBUS 修复）：Ozone 里确认已烧录本次固件。
 * 注意：必须 used+retain，否则未引用的全局会被 --gc-sections 删掉，Ozone 会把它
 * 解析到地址 0（读到 0x20020000 = 初始栈顶），从而"看起来烧错了固件"。
 * V1.2.2 起每次改接收逻辑都换 tag，用来区分板内固件与 ELF 是否一致。 */
__attribute__((used, retain)) volatile uint32_t g_dbg_fw_tag = 0x20261109u;
volatile uint8_t g_dbg_dbus_last_evt = 0;  ///< 最近一次 UART 事件类型 0=HT 1=TC 2=IDLE
volatile uint16_t g_dbg_dbus_last_size = 0; ///< 最近一次 UART 事件长度（正常 TC=18）
/* 接收事件统计：判断帧对齐是否收敛；稳态下 TC 约 +70/秒（帧周期 ~14 ms），
 * 上电最初 1~2 帧可能出现 0<size<18 的 IDLE（用于对齐），之后不应再增长。 */
volatile uint32_t g_dbg_dbus_ht_cnt = 0;   ///< 半传输事件次数（被忽略，不应影响解码）
volatile uint32_t g_dbg_dbus_tc_cnt = 0;   ///< 传输完成事件次数
volatile uint32_t g_dbg_dbus_idle_cnt = 0; ///< 总线空闲事件次数（对齐阶段用）
/* 最近 8 次 UART 事件日志，每项 = (evt << 8) | size；用于回放启动阶段的事件序列。
 * 读法：g_dbg_dbus_evt_log[g_dbg_dbus_evt_idx] 是最旧一项，沿数组顺序递增。 */
volatile uint16_t g_dbg_dbus_evt_log[8] = {0};
volatile uint8_t g_dbg_dbus_evt_idx = 0;   ///< 下一个写入位置（环形）
/* 便于 Ozone 单变量观察的标量摘录（数组在 Ozone 里展开不方便时用） */
volatile uint8_t g_dbg_dbus_raw5 = 0;    ///< g_dbg_dbus_raw[5]（拨杆位所在字节）
volatile uint8_t g_dbg_dbus_raw16 = 0;   ///< g_dbg_dbus_raw[16]（拨轮低字节）
volatile uint8_t g_dbg_dbus_raw17 = 0;   ///< g_dbg_dbus_raw[17]（拨轮高字节）
volatile uint16_t g_dbg_dbus_sum = 0;    ///< 当前帧 18 字节按字节求和（帧内容变化即变）
volatile uint16_t g_dbg_dbus_chg_sum = 0; ///< g_dbg_dbus_chg 全数组求和（>0 表示有字节变过）

/* DBUS 原始解码镜像（Ozone Live Watch 用）：拨杆/摇杆"可读性"自检。
 * 拨杆取值：1=上、2=下、3=中（见 inc/DT7.hpp 的 SwitchState）；
 * 摇杆/拨轮取值 [-1, 1]，回中约 0。每收到一帧刷新一次（约 14 ms）。 */
volatile uint8_t g_dbg_rc_online = 0;   ///< IsRemoteOnline() 镜像：1=100 ms 内收到过帧
volatile uint8_t g_dbg_rc_l_switch = 0; ///< 左拨杆 1=上 2=下 3=中
volatile uint8_t g_dbg_rc_r_switch = 0; ///< 右拨杆 1=上 2=下 3=中
volatile float g_dbg_rc_lv = 0.0f;      ///< 左摇杆竖直 [-1,1]
volatile float g_dbg_rc_lh = 0.0f;      ///< 左摇杆水平 [-1,1]
volatile float g_dbg_rc_rv = 0.0f;      ///< 右摇杆竖直 [-1,1]
volatile float g_dbg_rc_rh = 0.0f;      ///< 右摇杆水平 [-1,1]
volatile float g_dbg_rc_wheel = 0.0f;   ///< 拨轮 [-1,1]

/* DBUS 原始帧镜像 + 位变化掩码（协议排查用，Ozone 可直接看数组）：
 *  - g_dbg_dbus_raw[0..17]：最近一帧的原始 18 字节；
 *  - g_dbg_dbus_chg[0..17]：按位累计"曾经变化过"的掩码（每帧与上一帧异或后
 *    按位或，上电清零）。用法：上电后只拨动某一个控件几秒，再观察哪些字节的
 *    掩码非零，就能确认该控件的数据到底落在帧的哪个字节/哪些位上。 */
volatile uint8_t g_dbg_dbus_raw[kRcRxLen] = {0};
volatile uint8_t g_dbg_dbus_chg[kRcRxLen] = {0};
/* 对齐/回中校验：4 个通道值（ch0..ch3）相对 1024 的绝对偏差之和。
 * 所有摇杆回中、拨杆中位时应接近 0（几十以内）；若明显偏大，说明帧错位、
 * 通道映射不符或遥控器未校准——这是判断"帧是否真的对齐"最快的单值指标。 */
volatile uint16_t g_dbg_dbus_center_err = 0;

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void) {
  /* 1. 先触发 Instance 层实例化：4 个轮电机 + 遥控解码器（保证首次构造发生在
   *    开接收中断之前，中断里只做纯读写）。 */
  for (int i = 0; i < kWheelNum; ++i) {
    (void)GetMotorWheel(i);
  }
  (void)GetRemoteControl();
  (void)GetGimbalChassisComm();

  /* 固件标识/原始帧镜像的"已初始化"标记：
   *  - g_dbg_fw_tag 写回一次，保证它被引用而不会被 --gc-sections 删除；
   *  - g_dbg_dbus_raw 预填 0xEE：若 Ozone 里看到 0xEE 说明接收回调没有写这块内存
   *    （符号地址不匹配/回调没跑），若看到真实字节说明链路正常。 */
  g_dbg_fw_tag = 0x20261109u;
  for (uint16_t i = 0; i < kRcRxLen; ++i) {
    g_dbg_dbus_raw[i] = 0xEEu;
  }

  /* 2. CAN 接收滤波器（全收：CAN1→FIFO0，CAN2→FIFO1，契约 §4.2） */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  /* 3. 启动 CAN 外设并打开接收中断。HAL_CAN_Init 后外设仍处于初始化模式，
   *    不调用 HAL_CAN_Start 则永远无法收发。 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* 除接收外再打开错误类中断（ERRIE + 细分标志），由 HAL_CAN_IRQHandler 汇总后
   * 调用 HW_can.cpp 里的 HAL_CAN_ErrorCallback，供 bus-off/ACK 错误等故障诊断。 */
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                      CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    Error_Handler();
  }

  /* CAN2 承载双板通讯帧（契约 §4.2）：G2C 0x211 由 HW_can.cpp 的 FIFO1 回调解码。 */
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(
          &hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR |
                      CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                      CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    Error_Handler();
  }

  /* 4. 启动遥控器 DBUS 接收：USART3 空闲 + DMA（CIRCULAR），收满 18 字节由
   *    HAL_UARTEx_RxEventCallback 解包。启动失败直接进 Error_Handler（无遥控时
   *    状态机永远无法进 Working，导致整车不可用，属致命初始化错误）。 */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc_rx_buf, kRcRxLen) != HAL_OK) {
    Error_Handler();
  }

  /* IWDG 已在 main.c 的 MX_IWDG_Init() 启动（契约 §8.2）：MainTaskInit 末尾喂一次，
   * 之后每 1 ms 在 CommTask() 末尾刷新。 */
}

void CommTask(void) {
  /* 1. M3508 控制帧 0x200：电调 ID 1~4 共用同一帧，每个电机 encode() 只写自己的
   *    2 字节，因此必须让 4 个电机依次 encode 到同一个 8 字节缓冲区，最后只发一帧
   *    （契约 §8.1）。每周期都发（输入为 0 也发），保证电调持续收到最新指令。 */
  uint8_t tx_data[8] = {0};
  for (int i = 0; i < kWheelNum; ++i) {
    hello_world::motor::Motor *motor = GetMotorWheel(i);
    if (motor == nullptr) {
      continue;
    }
    (void)motor->encode(tx_data);
  }
  CAN_Send_Msg(&hcan1, tx_data, config::kCanIdMotorTx, 8);

  /* 2. 双板通讯帧 C2G（0x210）：语义字段（状态/急停/模式/IMU yaw/云台目标角）由
   *    Robot::update() 每 1 ms 刷新，这里只 encode + 经 CAN2 发出。每周期必发，
   *    云台板用 isOffline() 判本板在线（契约 §3.2/§7.5）。 */
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    uint8_t comm_tx_data[8] = {0};
    comm->encode(comm_tx_data);
    CAN_Send_Msg(&hcan2, comm_tx_data, config::kCanIdC2G, 8);
    g_dbg_c2g_tx_cnt += 1;
  }

  /* 3. 遥控在线状态镜像（Ozone 直读；>100 ms 无帧即为 0） */
  g_dbg_rc_online =
      hello_world::remote_control::IsRemoteOnline() ? 1u : 0u;

#if DEBUG_NO_IWDG == 0
  /* 3. 契约 §8.2：在 1 ms 任务链末尾刷新 IWDG。本函数运行在 TIM6 中断里，
   *    若中断卡死/任务链停摆则会停止喂狗 → 底盘板 ~300 ms 后复位。 */
  (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief   UART 接收事件回调（DBUS 遥控器解包）
 * @param   huart: UART 句柄
 * @param   Size:  本次事件对应的接收字节数
 * @retval  None
 * @note    V1.2.1 起 DBUS 接收改为“非循环 DMA + 按帧边界重启”（解决帧窗口
 *          未对齐导致的字段整体旋转：表现如“拨左拨杆时轮速变化、右拨杆/
 *          拨轮无反应”）：
 *            - USART3_RX 的 DMA 改为 DMA_NORMAL（见 usart.c）；
 *            - HT 半传输事件直接忽略，它不是帧边界，重启会破坏对齐；
 *            - TC / IDLE 事件才是帧边界：
 *                Size == 18 → 完整一帧，镜像 + 解码；
 *                Size != 18 → 上次重启落在帧中间，丢弃该段（bad++）；
 *              随后 AbortReceive + 重新 ReceiveToIdle_DMA，使下一帧从
 *              g_rc_rx_buf[0] 开始，最多 2 帧即可完成对齐。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size) {
  if (huart != &huart3) {
    return;
  }

  /* 事件诊断镜像（Ozone）：0=HT 1=TC 2=IDLE */
  g_dbg_dbus_last_evt = static_cast<uint8_t>(huart->RxEventType);
  g_dbg_dbus_last_size = Size;

  /* 事件统计 + 最近 8 次事件日志（观察启动阶段事件序列，判断帧对齐是否收敛） */
  if (huart->RxEventType == HAL_UART_RXEVENT_HT) {
    g_dbg_dbus_ht_cnt += 1u;
  } else if (huart->RxEventType == HAL_UART_RXEVENT_TC) {
    g_dbg_dbus_tc_cnt += 1u;
  } else {
    g_dbg_dbus_idle_cnt += 1u;
  }
  g_dbg_dbus_evt_log[g_dbg_dbus_evt_idx] = static_cast<uint16_t>(
      (static_cast<uint16_t>(huart->RxEventType) << 8) | (Size & 0xFFu));
  g_dbg_dbus_evt_idx = static_cast<uint8_t>((g_dbg_dbus_evt_idx + 1u) & 0x07u);

  /* 半传输不是帧边界：忽略，且不重启接收（重启会破坏帧对齐）。 */
  if (huart->RxEventType == HAL_UART_RXEVENT_HT) {
    return;
  }

  if (Size == kRcRxLen) {
    hello_world::remote_control::DT7 *remote = GetRemoteControl();
    if (remote != nullptr) {
      /* 原始帧镜像 + 位变化掩码（协议排查：拨某个控件时哪些位在变） */
      static uint8_t prev_raw[kRcRxLen] = {0};
      uint16_t sum = 0;
      uint16_t chg_sum = 0;
      for (uint16_t i = 0; i < kRcRxLen; ++i) {
        g_dbg_dbus_raw[i] = g_rc_rx_buf[i];
        g_dbg_dbus_chg[i] |=
            static_cast<uint8_t>(g_rc_rx_buf[i] ^ prev_raw[i]);
        prev_raw[i] = g_rc_rx_buf[i];
        sum = static_cast<uint16_t>(sum + g_rc_rx_buf[i]);
        chg_sum = static_cast<uint16_t>(chg_sum + g_dbg_dbus_chg[i]);
      }
      g_dbg_dbus_raw5 = g_rc_rx_buf[5];
      g_dbg_dbus_raw16 = g_rc_rx_buf[16];
      g_dbg_dbus_raw17 = g_rc_rx_buf[17];
      g_dbg_dbus_sum = sum;
      g_dbg_dbus_chg_sum = chg_sum;

      /* 4 个 11 bit 通道相对中位 1024 的偏差和：所有摇杆回中时应接近 0，
       * 这是判断"帧是否对齐 / 通道映射是否标准"最快的单值指标。 */
      {
        const uint16_t ch0 = static_cast<uint16_t>(
            (g_rc_rx_buf[0] | (g_rc_rx_buf[1] << 8)) & 0x07FFu);
        const uint16_t ch1 = static_cast<uint16_t>(
            ((g_rc_rx_buf[1] >> 3) | (g_rc_rx_buf[2] << 5)) & 0x07FFu);
        const uint16_t ch2 = static_cast<uint16_t>(
            ((g_rc_rx_buf[2] >> 6) | (g_rc_rx_buf[3] << 2) |
             (g_rc_rx_buf[4] << 10)) &
            0x07FFu);
        const uint16_t ch3 = static_cast<uint16_t>(
            ((g_rc_rx_buf[4] >> 1) | (g_rc_rx_buf[5] << 7)) & 0x07FFu);
        const uint16_t dev0 =
            static_cast<uint16_t>(ch0 > 1024u ? ch0 - 1024u : 1024u - ch0);
        const uint16_t dev1 =
            static_cast<uint16_t>(ch1 > 1024u ? ch1 - 1024u : 1024u - ch1);
        const uint16_t dev2 =
            static_cast<uint16_t>(ch2 > 1024u ? ch2 - 1024u : 1024u - ch2);
        const uint16_t dev3 =
            static_cast<uint16_t>(ch3 > 1024u ? ch3 - 1024u : 1024u - ch3);
        g_dbg_dbus_center_err =
            static_cast<uint16_t>(dev0 + dev1 + dev2 + dev3);
      }

      remote->decode(g_rc_rx_buf);
      /* 镜像解码结果（Ozone Live Watch，拨杆/摇杆可读性自检） */
      g_dbg_rc_l_switch = static_cast<uint8_t>(remote->rc_l_switch());
      g_dbg_rc_r_switch = static_cast<uint8_t>(remote->rc_r_switch());
      g_dbg_rc_lv = remote->rc_lv();
      g_dbg_rc_lh = remote->rc_lh();
      g_dbg_rc_rv = remote->rc_rv();
      g_dbg_rc_rh = remote->rc_rh();
      g_dbg_rc_wheel = remote->rc_wheel();
      /* 刷新遥控在线时间戳（>100 ms 未刷新 → Robot 判遥控离线并回 Dead） */
      hello_world::remote_control::NotifyRemoteRx();
      g_dbg_dbus_rx_ok += 1;
    }
  } else {
    /* 非整帧：上次重启落在帧中间（或丢包），丢弃并重新对齐。 */
    g_dbg_dbus_rx_bad += 1;
  }

  /* 关键：从当前帧边界重启接收，保证下一帧从 g_rc_rx_buf[0] 开始。 */
  (void)HAL_UART_AbortReceive(&huart3);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc_rx_buf, kRcRxLen);
}
