/**
 *******************************************************************************
 * @file      : motor_DaMiao.hpp
 * @brief     : 达妙电机类
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2024-07-11      Caikunzhen      1. 完成正式版
 *  V1.1.0      2025-02-17      Jinletian       1. 重构代码，整合不同型号达妙电机
 *  V1.2.0      2025-03-31      Jinletian       1. 支持控制模式切换
 *  V2.0.0      2026-09-18      Hello World     1. 改装版：去通信耦合，暴露纯字节接口
 *******************************************************************************
 * @attention :
 *  1. 请先查看 motor_base.hpp 中的注意事项。
 *  2. 电机 ID 范围为 0x01~0x0F，每个电机 ID 对应一个发送报文。
 *  3. 可使用的输入类型为 InputType::kRaw、InputType::kTorq、InputType::kCurr 和
 *  InputType::kCmd。其中要注意在 InputType::kRaw 中输入值恒为正，且不具有大小比较关
 *  系，详见对应的电机说明手册。InputType::kCmd 主要用于发送电机失能使能命令。
 *  4. 电机的命令优先级高于设置的电机输入，只有当电机没有需要发送的命令时，才会发送设置
 *  的电机输入。
 *  5. 使用电机前需要使用上位机对电机进行配置，其中 CAN ID 为 ID，Master ID 为 ID + 0x10，
 *  PMAX 为 3.141593，VMAX 和 TMAX 需要根据电机具体型号设置：
 *  DM-J4310: VMAX 为 21，TMAX 为 7.5
 *  6. 该电机上电时处于失能状态，需要通过发送指令使能电机，否则电机不会工作，当电机初始
 *  化中 auto_enable 为 true 时，当电机反馈表明电机失能时会自动使能电机，同时如果需要
 *  电机处于失能状态，则需要持续输入失能命令。若 auto_enable 为 false 时，需要手动使能
 *  电机。同时，当电机处于错误状态时，会自动清除错误并重启电机。上电后会一直发送使能指令，
 *  直到电机反馈使能成功。
 *  7. 电机为一发一收的通信方式，不给电机发送指令时电机不会反馈数据。
 *  8. 可额外获得的数据为转子温度 rotor_temp、MOS管温度 mos_temp 和状态码 status_code
 *  9. 达妙电机可以使用 MIT 模式进行控制，输入期望位置、期望速度、位置环比例系数、速度环
 *  比例系数和前馈力矩即可实现控制，相关信息详见达妙驱动控制协议。注意：使用 MIT 模式控制
 *  位置时电机内部不进行过零处理，因此无法控制电机多圈旋转。
 *  10. 达妙电机可以使用速度模式进行控制，使用前需要将 enable_mode_switch 设置为 true。
 *  11. 改装版中 decode/encode 为纯字节接口：新人需在自己的 CAN 层根据反馈 ID 判断报文
 *     归属后再调用 decode，并在 encode 后通过 tx_id() 获取本次应使用的发送 CAN ID，
 *     将 encode 出的 8 字节填入对应 CAN 帧发出（RX_ID 恒为 id + 0x10）。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DAMIAO_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DAMIAO_HPP_

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

#pragma region 达妙基类
class DaMiao : public Motor
{
 public:
  enum StatusCode : uint8_t {
    kMotorDisabled = 0x0,  ///* 电机失能
    kMotorEnabled = 0x1,   ///* 电机使能
    kOverVolt = 0x8,       ///* 过压
    kUnderVolt = 0x9,      ///* 欠压
    kOverCurr = 0xA,       ///* 过流
    kMosOverTemp = 0xB,    ///* MOS过温
    kCoilOverTemp = 0xC,   ///* 电机线圈过温
    kCommLoss = 0xD,       ///* 通信丢失
    kOverload = 0xE,       ///* 过载
  };

  enum Cmd : uint8_t {
    kNone = 0,         ///* 无指令
    kClearErr = 0xFB,  ///* 清除错误
    kEnable = 0xFC,    ///* 电机使能
    kDisable = 0xFD,   ///* 电机失能
    kSaveZero = 0xFE,  ///* 保存零点
  };

  /**
   * @brief      电机控制模式
   * @note        目前只支持 MIT 模式和速度模式，如有需求可后续添加
   */
  enum CtrlMode : uint8_t {
    kUnknown = 0,  ///* 未知模式
    kMIT = 1,      ///* MIT 模式
    kVel = 3,      ///* 速度模式
  };

  /**
   * @brief      寄存器列表 具体参考达妙手册
   * @note       RW: 可读写，RO: 只读
   */
  enum Reg : uint8_t {
    UV_Value = 0,    ///* 低压保护值 RW float
    KT_Value = 1,    ///* 扭矩系数 RW float
    OT_Value = 2,    ///* 过温保护值 RW float
    OC_Value = 3,    ///* 过流保护值 RW float
    ACC = 4,         ///* 加速度 RW float
    DEC = 5,         ///* 减速度 RW float
    MAX_SPD = 6,     ///* 最大速度 RW float
    MST_ID = 7,      ///* 反馈 ID RW uint32_t
    ESC_ID = 8,      ///* 接收 ID RW uint32_t
    TIMEOUT = 9,     ///* 超时警报时间 RW uint32_t
    CTRL_MODE = 10,  ///* 控制模式 RW uint32_t
    Damp = 11,       ///* 电机粘滞系数 RO float
    Inertia = 12,    ///* 电机转动惯量 RO float
    hw_ver = 13,     ///* 保留 RO uint32_t
    sw_ver = 14,     ///* 软件版本号  RO uint32_t
    SN = 15,         ///* 保留 RO uint32_t
    NPP = 16,        ///* 电机极对数 RO uint32_t
    Rs = 17,         ///* 电机相电阻 RO float
    LS = 18,         ///* 电机相电感 RO float
    Flux = 19,       ///* 电机磁链值 RO float
    Gr = 20,         ///* 齿轮减速比 RO float
    PMAX = 21,       ///* 位置映射范围 RW float
    VMAX = 22,       ///* 速度映射范围 RW float
    TMAX = 23,       ///* 扭矩映射范围 RW float
    I_BW = 24,       ///* 电流环控制带宽 RW float
    KP_ASR = 25,     ///* 速度环比例系数 RW float
    KI_ASR = 26,     ///* 速度环积分系数 RW float
    KP_APR = 27,     ///* 位置环比例系数 RW float
    KI_APR = 28,     ///* 位置环积分系数 RW float
    OV_Value = 29,   ///* 过压保护值 RW float
    GREF = 30,       ///* 齿轮力矩效率 RW float
    Deta = 31,       ///* 速度环阻尼系数 RW float
    V_BW = 32,       ///* 速度环滤波带宽 RW float
    IQ_c1 = 33,      ///* 电流环增强系数 RW float
    VL_c1 = 34,      ///* 速度环增强系数 RW float
    can_br = 35,     ///* CAN 波特率代码 RW uint32_t
    sub_ver = 36,    ///* 子版本号 RO uint32_t
    u_off = 50,      ///* u相偏移 RO float
    v_off = 51,      ///* v相偏移 RO float
    k1 = 52,         ///* 补偿因子1 RO float
    k2 = 53,         ///* 补偿因子2 RO float
    m_off = 54,      ///* 角度偏移 RO float
    dir = 55,        ///* 方向 RO uint32_t
    p_m = 80,        ///* 电机位置 RO float
    xout = 81,       ///* 输出轴位置 RO float
  };

  DaMiao(void) = default;
  explicit DaMiao(uint8_t id,
                  const OptionalParams &opt = OptionalParams(),
                  bool auto_enable = true,
                  bool enable_mode_switch = false);
  DaMiao(const DaMiao &) = default;
  DaMiao &operator=(const DaMiao &other);
  DaMiao(DaMiao &&other);
  DaMiao &operator=(DaMiao &&other);

  virtual ~DaMiao(void) = default;

  /* 配置方法 */

  /**
   * @brief       DaMiao 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，范围 0x01 ~ 0x0F
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @param        enable_mode_switch: 电机模式切换
   * @retval       None
   * @note        老版本（v1.1以前）电机需将 auto_enable 设置为 false ，并需每隔一段
   *              时间通手动发送使能指令，以确保电机处于使能状态
   */
  void init(uint8_t id,
            const OptionalParams &opt = OptionalParams(),
            bool auto_enable = true,
            bool enable_mode_switch = false);

  /* 方法重载 */

  /**
   * @brief       将电调发回的 8 字节 CAN 反馈报文进行解包
   * @param        data: 电调发回的 8 字节 CAN 报文
   * @retval       是否解包成功
   * @note        新人需先在自己的 CAN 层根据反馈 ID（id + 0x10）判断报文归属，再调用此接口
   */
  virtual bool decode(const uint8_t data[8]) override;

  /**
   * @brief       将要发给电调的期望输入编码为 8 字节 CAN 控制报文
   * @param        data: 输出的 8 字节 CAN 报文缓冲区
   * @retval       是否编码成功
   * @note        新人需在 encode 后通过 tx_id() 获取本次应使用的发送 CAN ID，
   *              将 data 填入对应 CAN 帧发出
   */
  virtual bool encode(uint8_t data[8]) override;

  /**
   * @brief       将原始报文内容转换为输出端力矩
   * @param        raw: 原始报文数值
   * @retval       原始报文对应的输出端力矩值，单位：N·m
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float raw2Torq(float raw) const override;

  /**
   * @brief       将输出端力矩转换为原始报文内容
   * @param        torq: 输出端力矩值，单位：N·m
   * @retval       输出端力矩值对应的原始报文
   * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过 motor_info
   *              方法获取电机信息结构体，查看其中的 raw_mapping_type 变量
   */
  virtual float torq2Raw(float torq) const override;

  /**
   * @brief       设定发给电调的期望值
   * @param        input: 发给电调的期望值，当输入类型为 InputType::kCmd 时，可选值
   *               为：
   *   @arg        DaMiao::Cmd::kCmdEnable: 使能电机
   *   @arg        DaMiao::Cmd::kCmdDisable: 失能电机
   *   @arg        DaMiao::Cmd::kCmdClearErr: 清除错误
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用 get_input_type
   *              方法查看
   *              2. 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通过
   *              getInput 方法查看
   */
  virtual Status setInput(float input) override;

  /**
   * @brief       设置点击的输入类型
   * @param        input_type: 期望输入类型，可选值只有：
   *   @arg        InputType::kTorq: 输出端力矩输入
   * @retval       设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设置成功
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        None
   */
  virtual Status set_input_type(InputType input_type) override;

  /* 达妙电机指令 */

  // 使能电机
  void enable(void) { comm_data_.wait_to_handle_cmd = kEnable; }

  // 失能电机
  void disable(void) { comm_data_.wait_to_handle_cmd = kDisable; }

  // 清除错误
  void clearErr(void) { comm_data_.wait_to_handle_cmd = kClearErr; }

  // 保存零点
  void saveZero(void) { comm_data_.wait_to_handle_cmd = kSaveZero; }

  /**
   * @brief       MIT 模式控制函数
   * @param       pos_des: 期望位置，单位：rad
   * @param       vel_des: 期望速度，单位：rad/s
   * @param       kp: 位置环比例系数，范围 [0, 500]
   * @param       kd: 速度环比例系数，范围 [0, 5]
   * @param       torq: 前馈力矩，单位：N·m
   * @retval      设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   * @note        1.根据 MIT 模式可以衍生出多种控制模式，如 kp = 0 且 kd 不为 0 时，给定
   *              v_des 即可实现匀速转动; kp = 0 且 kd = 0，给定 torq 即可实现给定扭矩输出，
   *              此时效果等同于力矩输入下的 setInput(torq)
   *              2. 对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控。
   *              电机内部不进行过零处理，因此无法控制电机多圈旋转。
   */
  Status setMitInput(float pos_des, float vel_des, float kp, float kd, float torq);

  /**
   * @brief      速度模式控制函数
   * @param       vel_input: 期望速度，单位：rad/s
   * @retval      设置状态，可能的返回值有：
   *   @arg        Status::kOk: 设定成功
   *   @arg        Status::kInputValueOverflow: 设定值超出范围
   *   @arg        Status::kInputTypeError: 输入类型错误
   * @note        使用该函数必须开启 enable_mode_switch_，否则会返回错误
   */
  Status setVelInput(float vel_input);

  /* 数据修改与获取 */

  // 电机转子温度
  uint8_t rotor_temp(void) const { return dm_status_.rotor_temp; }

  // 电机 MOS 管温度
  uint8_t mos_temp(void) const { return dm_status_.mos_temp; }

  // 电机是否使能
  bool is_enabled(void) const { return dm_status_.is_enabled; }

  // 电机当前控制模式
  CtrlMode current_ctrl_mode(void) const { return dm_status_.current_ctrl_mode; }

  // 电机当前状态码
  StatusCode status_code(void) const { return dm_status_.status_code; }

  // 电机是否正在进行寄存器读写，等待应答
  bool wait_for_ack(void) const { return dm_status_.wait_for_ack; }

  // 最近一次 encode 后应使用的发送 CAN ID（由 encode 内部更新，供新人 CAN 层取用）
  uint32_t tx_id(void) const { return tx_id_; }

  // 电机是否需要模式切换
  bool needModeSwitch(void) const
  {
    return enable_mode_switch_ &&
           dm_status_.current_ctrl_mode != comm_data_.target_ctrl_mode;
  }

  /**
   * @brief       根据 id 获取对应的发送 ID
   * @param        id: 电机 ID，范围 0x01 ~ 0x0F
   * @retval       对应的发送 ID
   * @note        MIT 模式发送 ID 为电机 ID 本身；速度模式为 0x200 + id；
   *              寄存器读写命令为 0x7FF
   */
  static uint32_t GetTxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 0x0F, "Error id: %d", id);
#pragma endregion

    return kMitTx0_ + id;
  }

  /**
   * @brief       根据 id 获取对应的接收 ID
   * @param        id: 电机 ID，范围 0x01 ~ 0x0F
   * @retval       对应的接收 ID
   * @note        接收 ID 恒为 id + 0x10
   */
  static uint32_t GetRxId(uint8_t id)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(1 <= id && id <= 0x0F, "Error id: %d", id);
#pragma endregion

    return kRx0_ + id;
  }

  virtual float getMaxVel(void) const = 0;
  virtual float getMaxTorq(void) const = 0;

 protected:
  /**
   * @brief       根据给定的范围和位数，将 uint 转换为 float
   * @param        x_uint: 待转换的 uint
   * @param        x_max: 给定范围的最大值
   * @param        x_min: 给定范围的最小值
   * @param        bits: 整形的位数
   * @retval       转换后的 float 变量
   * @note        该函数仅供达妙电机数据处理用
   */
  static float Uint2Float(
      uint16_t x_int, float x_max, float x_min, uint8_t bits)
  {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
  }

  /**
   * @brief       根据给定的范围和位数，将 float 转换为 uint
   * @param        x_float: 待转换的 float
   * @param        x_max: 给定范围的最大值
   * @param        x_min: 给定范围的最小值
   * @param        bits: 整形的位数
   * @retval       转换后的 uint 变量
   * @note        该函数仅供达妙电机数据处理用
   */
  static inline uint16_t Float2Uint(
      float x_float, float x_max, float x_min, uint8_t bits)
  {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
  }

  /**
   * @brief       判断给定的寄存器地址是否对应 uint32_t 类型的数据
   * @param        reg: 寄存器地址
   * @retval       是否对应 uint32_t 类型的数据
   * @note        该函数仅供达妙电机数据处理用
   */
  static bool is_uint(Reg reg)
  {
    return (7 <= reg && reg <= 10) ||
           (13 <= reg && reg <= 16) ||
           (35 <= reg && reg <= 36);
  }

  bool decodeParam(const uint8_t *data);
  bool decodeFdb(const uint8_t *data);

  void encodeCmd(uint8_t *data, Cmd cmd);
  void encodeReadParam(uint8_t *data, Reg reg);
  void encodeWriteParam(uint8_t *data, Reg reg, float value);
  void encodeWriteParam(uint8_t *data, Reg reg, uint32_t value);
  void encodeMIT(uint8_t *data);
  void encodeVel(uint8_t *data);

  /* 附加功能开关 */
  bool auto_enable_ = true;          ///* 电机自动使能
  bool enable_mode_switch_ = false;  ///* 是否允许切换模式

  /* 电机状态 */
  struct DMStatus {
    uint8_t rotor_temp = 0;                               ///* 电机转子温度
    uint8_t mos_temp = 0;                                 ///* 电机MOS管温度
    bool is_enabled = false;                              ///* 电机是否使能
    CtrlMode current_ctrl_mode = CtrlMode::kUnknown;      ///* 当前控制模式
    StatusCode status_code = StatusCode::kMotorDisabled;  ///* 状态码
    bool wait_for_ack = false;                            ///* 是否等待应答
  } dm_status_;

  /* 电机通信 */
  struct CommData {
    Cmd wait_to_handle_cmd = kNone;  ///* 待处理的指令

    CtrlMode target_ctrl_mode = CtrlMode::kMIT;  ///* 目标控制模式

    float pos_des = 0.0f;  ///* MIT 模式期望位置
    float vel_des = 0.0f;  ///* MIT 模式期望速度
    float kp = 0.0f;       ///* MIT 模式位置环比例系数
    float kd = 0.0f;       ///* MIT 模式速度环比例系数

    float vel_input = 0.0f;  ///* 速度模式输入
  } comm_data_;

  /* 最近一次 encode 后应使用的发送 CAN ID */
  uint32_t tx_id_ = 0;

  static constexpr uint32_t kMitTx0_ = 0x00;
  static constexpr uint32_t kRx0_ = 0x10;
  static constexpr uint32_t kVelTx0_ = 0x200;
  static constexpr uint32_t kCmdTx0_ = 0x7FF;

  static constexpr uint8_t kAngleBits_ = 16;
  static constexpr uint8_t kVelBits_ = 12;
  static constexpr uint8_t kKpBits_ = 12;
  static constexpr uint8_t kKdBits_ = 12;
  static constexpr uint8_t kTorqBits_ = 12;
  static constexpr float kMaxAngle_ = 3.141593f;
  static constexpr float kMaxKp_ = 500.0f;
  static constexpr float kMaxKd_ = 5.0f;
};
#pragma endregion

#pragma region DM-J4310 电机
static const MotorBaseInfo kDM_J4310MotorBaseInfo{
    .raw_input_lim = kInvalidValue,
    .torq_input_lim = 7.0f,
    .curr_input_lim = kInvalidValue,
    .torq_const = kInvalidValue,
    .redu_rat = 1.0f,  ///* DM-J4310 为输出端编码，且减速器（10:1）难以拆卸
    .angle_rat = kInvalidValue,
    .vel_rat = kInvalidValue,
    .curr_rat = kInvalidValue,
    .torq_rat = kInvalidValue,
    .cross_0_value = static_cast<uint16_t>(kInvalidValue),
    .raw_mapping_type = RawMappingType::kTorq,
};

class DM_J4310 : public DaMiao
{
 public:
  /**
   * @brief       默认构造函数
   * @retval       None
   * @note        使用该方式实例化对象时，需要在外部调用 init() 函数进行初始化
   */
  DM_J4310(void) = default;

  /**
   * @brief       DM_J4310 初始化
   * @param        id: 电机 ID，0x01 ~ 0x0F
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @param        enable_mode_switch: 电机模式切换
   * @retval       None
   * @note        老版本（v1.1以前）电机需将 auto_enable 设置为 false ，并需每隔一段
   *              时间通手动发送使能指令，以确保电机处于使能状态
   */
  explicit DM_J4310(uint8_t id,
                    const OptionalParams &opt = OptionalParams(),
                    bool auto_enable = true,
                    bool enable_mode_switch = false)
      : DaMiao(id, opt, auto_enable, enable_mode_switch)
  {
    motor_info_ = kDM_J4310MotorBaseInfo;
  }
  DM_J4310(const DM_J4310 &) = default;
  DM_J4310 &operator=(const DM_J4310 &other)
  {
    if (this != &other) {
      DaMiao::operator=(other);
    }
    return *this;
  }
  DM_J4310(DM_J4310 &&other) : DaMiao(std::move(other)) {}
  DM_J4310 &operator=(DM_J4310 &&other) noexcept
  {
    if (this != &other) {
      DaMiao::operator=(std::move(other));
    }
    return *this;
  }

  /**
   * @brief       DM_J4310 初始化，使用默认构造函数后请务必调用此函数
   * @param        id: 电机 ID，0x01 ~ 0x0F
   * @param        opt: 电机可选配置参数
   * @param        auto_enable: 电机自动使能
   * @param        enable_mode_switch: 电机模式切换
   * @retval       None
   * @note        老版本（v1.1以前）电机需将 auto_enable 设置为 false ，并需每隔一段
   *              时间通手动发送使能指令，以确保电机处于使能状态
   */
  void init(uint8_t id, const OptionalParams &opt, bool auto_enable, bool enable_mode_switch)
  {
    motor_info_ = kDM_J4310MotorBaseInfo;
    DaMiao::init(id, opt, auto_enable, enable_mode_switch);
  }

 protected:
  float getMaxVel(void) const override { return kMaxVel; }
  float getMaxTorq(void) const override { return kMaxTorq; }

  static constexpr float kMaxVel = 21.0f;
  static constexpr float kMaxTorq = 7.5f;
};
#pragma endregion

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DAMIAO_HPP_ */
