/**
 *******************************************************************************
 * @file      : motor_DaMiao.cpp
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
 *  比例系数和前馈力矩即可实现控制，相关信息详见达妙驱动控制协议。注意：使用 MIT模式控制
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
/* Includes ------------------------------------------------------------------*/
#include "motor_DaMiao.hpp"

#include <string.h>

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

HW_OPTIMIZE_O2_START

/* Exported function definitions ---------------------------------------------*/

DaMiao::DaMiao(uint8_t id,
               const OptionalParams& opt,
               bool auto_enable,
               bool enable_mode_switch)
    : Motor()
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(0x01 <= id && id <= 0x0F, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kTorq,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
#pragma endregion

  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }

  motor_info_.raw_input_lim = kInvalidValue;
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = kInvalidValue;

  auto_enable_ = auto_enable;
  enable_mode_switch_ = enable_mode_switch;

  /* 如果禁用模式切换，则默认为 MIT 模式*/
  if (!enable_mode_switch) {
    dm_status_.current_ctrl_mode = CtrlMode::kMIT;
  }
}

DaMiao& DaMiao::operator=(const DaMiao& other)
{
  if (this != &other) {
    Motor::operator=(other);

    auto_enable_ = other.auto_enable_;
    enable_mode_switch_ = other.enable_mode_switch_;

    dm_status_ = other.dm_status_;
    comm_data_ = other.comm_data_;
    tx_id_ = other.tx_id_;
  }

  return *this;
}

DaMiao::DaMiao(DaMiao&& other) : Motor(std::move(other))
{
  if (this != &other) {
    auto_enable_ = other.auto_enable_;
    enable_mode_switch_ = other.enable_mode_switch_;

    dm_status_ = other.dm_status_;
    comm_data_ = other.comm_data_;
    tx_id_ = other.tx_id_;
  }
}

DaMiao& DaMiao::operator=(DaMiao&& other)
{
  if (this != &other) {
    Motor::operator=(std::move(other));

    auto_enable_ = other.auto_enable_;
    enable_mode_switch_ = other.enable_mode_switch_;

    dm_status_ = other.dm_status_;
    comm_data_ = other.comm_data_;
    tx_id_ = other.tx_id_;
  }

  return *this;
}

bool DaMiao::decode(const uint8_t data[8])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  bool decode_res = false;

  /* 判断反馈帧类型 */
  if (dm_status_.wait_for_ack) {
    // 上一帧发送寄存器读写指令，解码应答帧
    decode_res = decodeParam(data);
  } else {
    // 上一帧发送电机控制指令，解码反馈帧
    decode_res = decodeFdb(data);
  }

  return decode_res;
}

bool DaMiao::decodeFdb(const uint8_t* data)
{
  uint16_t raw_angle = static_cast<uint16_t>((data[1] << 8) | data[2]);
  float angle = Uint2Float(raw_angle, kMaxAngle_, -kMaxAngle_, kAngleBits_);
  float last_angle = Uint2Float(
      last_raw_angle_, kMaxAngle_, -kMaxAngle_, kAngleBits_);

  /* 判断是否旋转了一圈 */
  float round = round_;
  float delta_angle = angle - last_angle;
  if (fabsf(delta_angle) > 2 * PI * kCross0ValueThres) {
    delta_angle < 0 ? round++ : round--;

    /* 避免圈数溢出 */
    if (motor_info_.angle_range != AngleRange::kNegInfToPosInf) {
      round = fmodf(round, motor_info_.redu_rat);
    }
  }

  float actual_ang =
      static_cast<float>(motor_info_.dir) * (angle + round * 2 * PI) /
      motor_info_.redu_rat;
  float raw = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

  /* 统一对电机状态赋值 */
  round_ = round;
  actual_angle_ = normAngle(actual_ang);
  angle_ = normAngle(actual_ang - motor_info_.angle_offset);
  vel_raw_ = static_cast<float>(motor_info_.dir) *
             Uint2Float(
                 static_cast<int16_t>((data[3] << 4) | (data[4] >> 4)),
                 getMaxVel(), -getMaxVel(), kVelBits_) /
             motor_info_.redu_rat;
  vel_ = calcVel();
  torq_ = static_cast<float>(motor_info_.dir) * raw2Torq(raw);
  curr_ = static_cast<float>(motor_info_.dir) * raw2Curr(raw);
  dm_status_.mos_temp = data[6];
  dm_status_.rotor_temp = data[7];
  dm_status_.status_code = static_cast<StatusCode>(data[0] >> 4);
  last_raw_angle_ = raw_angle;

  if (dm_status_.status_code == StatusCode::kMotorEnabled) {
    dm_status_.is_enabled = true;
  } else if (dm_status_.status_code == StatusCode::kMotorDisabled) {
    dm_status_.is_enabled = false;
  } else {
    comm_data_.wait_to_handle_cmd = kClearErr;
  }
  return true;
}

bool DaMiao::decodeParam(const uint8_t* data)
{
  if (data[0] != motor_info_.id) {  // ID 不匹配
    return false;
  }

  if (data[2] != 0x33 && data[2] != 0x55) {  // 应答帧不匹配
    return false;
  }

  Reg reg = static_cast<Reg>(data[3]);
  uint32_t value_uint = 0;
  float value_float = 0;

  if (is_uint(reg)) {
    memcpy(&value_uint, data + 4, sizeof(uint32_t));
  } else {
    memcpy(&value_float, data + 4, sizeof(float));
  }

  /* 更新当前控制模式 */
  if (reg == Reg::CTRL_MODE) {
    dm_status_.current_ctrl_mode = static_cast<CtrlMode>(value_uint);
  }

  return true;
}

bool DaMiao::encode(uint8_t data[8])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(data, "data is nullptr");
#pragma endregion

  /* 自动使能 */
  if (!dm_status_.is_enabled &&
      auto_enable_ &&
      comm_data_.wait_to_handle_cmd == kNone) {
    comm_data_.wait_to_handle_cmd = kEnable;
  }

  /* 数据编码，优先级如下：
   * 1. 电机指令（使能、失能、清除错误、重置零点）
   * 2. 寄存器读写
   * 3. MIT输入、速度输入
   */
  if (comm_data_.wait_to_handle_cmd != kNone) {
    encodeCmd(data, comm_data_.wait_to_handle_cmd);
    comm_data_.wait_to_handle_cmd = kNone;
  } else if (needModeSwitch()) {
    encodeWriteParam(
        data, CTRL_MODE,
        static_cast<uint32_t>(comm_data_.target_ctrl_mode));
  } else {
    switch (dm_status_.current_ctrl_mode) {
      case CtrlMode::kMIT:
        encodeMIT(data);
        break;
      case CtrlMode::kVel:
        encodeVel(data);
        break;
      default:
        return false;
    }
  }

  return true;
}

void DaMiao::encodeCmd(uint8_t* data, Cmd cmd)
{
  tx_id_ = kMitTx0_ + motor_info_.id;
  dm_status_.wait_for_ack = false;
  memset(data, 0xFF, sizeof(uint8_t) * 7);
  data[7] = cmd;
}

void DaMiao::encodeReadParam(uint8_t* data, Reg reg)
{
  tx_id_ = kCmdTx0_;
  dm_status_.wait_for_ack = true;
  data[0] = motor_info_.id;
  data[1] = 0;
  data[2] = 0x33;
  data[3] = reg;
}

void DaMiao::encodeWriteParam(uint8_t* data, Reg reg, float value)
{
  tx_id_ = kCmdTx0_;
  dm_status_.wait_for_ack = true;
  data[0] = motor_info_.id;
  data[1] = 0;
  data[2] = 0x55;
  data[3] = reg;
  memcpy(data + 4, &value, sizeof(float));
}

void DaMiao::encodeWriteParam(uint8_t* data, Reg reg, uint32_t value)
{
  tx_id_ = kCmdTx0_;
  dm_status_.wait_for_ack = true;
  data[0] = motor_info_.id;
  data[1] = 0;
  data[2] = 0x55;
  data[3] = reg;
  memcpy(data + 4, &value, sizeof(uint32_t));
}

void DaMiao::encodeMIT(uint8_t* data)
{
  tx_id_ = kMitTx0_ + motor_info_.id;
  dm_status_.wait_for_ack = false;
  float pos_actual_des = NormPeriodData(
      -kMaxAngle_, kMaxAngle_,
      static_cast<float>(motor_info_.dir) * comm_data_.pos_des +
          motor_info_.angle_offset);
  float vel_actual_des = static_cast<float>(motor_info_.dir) * comm_data_.vel_des;
  uint16_t pos_input =
      Float2Uint(pos_actual_des,
                 kMaxAngle_, -kMaxAngle_, kAngleBits_);
  uint16_t vel_input =
      Float2Uint(vel_actual_des,
                 getMaxVel(), -getMaxVel(), kVelBits_);
  uint16_t kp_input =
      Float2Uint(comm_data_.kp, kMaxKp_, 0.0f, kKpBits_);
  uint16_t kd_input =
      Float2Uint(comm_data_.kd, kMaxKd_, 0.0f, kKdBits_);
  uint16_t torq_input =
      torq2Raw(static_cast<float>(motor_info_.dir) * torq_input_);

  data[0] = (pos_input >> 8);
  data[1] = pos_input;
  data[2] = (vel_input >> 4);
  data[3] = ((vel_input & 0xF) << 4) | (kp_input >> 8);
  data[4] = kp_input;
  data[5] = (kd_input >> 4);
  data[6] = ((kd_input & 0xF) << 4) | (torq_input >> 8);
  data[7] = torq_input;
}

void DaMiao::encodeVel(uint8_t* data)
{
  dm_status_.wait_for_ack = false;
  tx_id_ = kVelTx0_ + motor_info_.id;
  float vel_actual_des = static_cast<float>(motor_info_.dir) * comm_data_.vel_input;
  float vel_input = Bound(vel_actual_des, -getMaxVel(), getMaxVel());
  memcpy(data, &vel_input, sizeof(float));
}

float DaMiao::raw2Torq(float raw) const
{
  return Uint2Float(raw, getMaxTorq(), -getMaxTorq(), kTorqBits_) *
         motor_info_.redu_rat;
}

float DaMiao::torq2Raw(float torq) const
{
  return Float2Uint(
      torq / motor_info_.redu_rat, getMaxTorq(), -getMaxTorq(), kTorqBits_);
}

Status DaMiao::setInput(float input)
{
  /* 只使用 MIT 模式的力矩部分 */
  comm_data_.target_ctrl_mode = CtrlMode::kMIT;
  comm_data_.pos_des = 0;
  comm_data_.vel_des = 0;
  comm_data_.kp = 0;
  comm_data_.kd = 0;

  torq_input_ =
      Bound(input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
  raw_input_ = torq2Raw(torq_input_);
  if (fabsf(torq_input_) < fabsf(input)) {
    return Status::kInputValueOverflow;
  } else {
    return Status::kOk;
  }
}

Status DaMiao::set_input_type(InputType input_type)
{
  if (input_type == InputType::kTorq) {
    motor_info_.input_type = input_type;
    return Status::kOk;
  } else {
    return Status::kInputTypeError;
  }
}

Status DaMiao::setMitInput(float pos_des, float vel_des, float kp, float kd, float torq)
{
  comm_data_.target_ctrl_mode = CtrlMode::kMIT;
  comm_data_.pos_des = NormPeriodData(-kMaxAngle_, kMaxAngle_, pos_des);
  comm_data_.vel_des = Bound(vel_des, -getMaxVel(), getMaxVel());
  comm_data_.kp = Bound(kp, 0.0f, kMaxKp_);
  comm_data_.kd = Bound(kd, 0.0f, kMaxKd_);
  torq_input_ =
      Bound(torq, -motor_info_.torq_input_lim, motor_info_.torq_input_lim);
  raw_input_ = torq2Raw(torq_input_);

  if (fabsf(comm_data_.vel_des) < fabsf(vel_des)) {
    return Status::kInputValueOverflow;
  } else if (IsInRange(kp, 0.0f, kMaxKp_)) {
    return Status::kInputValueOverflow;
  } else if (IsInRange(kd, 0.0f, kMaxKd_)) {
    return Status::kInputValueOverflow;
  } else if (fabsf(torq_input_) < fabsf(torq)) {
    return Status::kInputValueOverflow;
  } else {
    return Status::kOk;
  }
}

Status DaMiao::setVelInput(float vel_input)
{
  if (enable_mode_switch_) {
    comm_data_.target_ctrl_mode = CtrlMode::kVel;
    comm_data_.vel_input = Bound(vel_input, -getMaxVel(), getMaxVel());
    if (fabsf(comm_data_.vel_input) < fabsf(vel_input)) {
      return Status::kInputValueOverflow;
    } else {
      return Status::kOk;
    }
  } else {
    return Status::kInputTypeError;
  }
}

void DaMiao::init(uint8_t id,
                  const OptionalParams& opt,
                  bool auto_enable,
                  bool enable_mode_switch)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 0x0F, "Error id: %d", id);
  HW_ASSERT(opt.dir == kDirFwd || opt.dir == kDirRev,
            "Error dir: %d", opt.dir);
  HW_ASSERT(opt.angle_range == AngleRange::k0To2Pi ||
                opt.angle_range == AngleRange::kNegInfToPosInf ||
                opt.angle_range == AngleRange::kNegPiToPosPi,
            "Error angle range: %d", opt.angle_range);
  HW_ASSERT(opt.input_type == InputType::kTorq,
            "Error input type: %d", opt.input_type);
  HW_ASSERT(opt.ex_redu_rat > 0,
            "Error external reduction ration: %f", opt.ex_redu_rat);
  HW_ASSERT(opt.max_torq_input_lim > 0,
            "Error max torque input limit: %f", opt.max_torq_input_lim);
  HW_ASSERT(opt.max_curr_input_lim > 0,
            "Error max current input limit: %f", opt.max_curr_input_lim);
#pragma endregion

  motor_info_.id = id;

  motor_info_.dir = opt.dir;
  motor_info_.angle_range = opt.angle_range;
  motor_info_.input_type = opt.input_type;
  motor_info_.angle_offset = opt.angle_offset;

  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (opt.remove_build_in_reducer) {
    motor_info_.redu_rat = opt.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= opt.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (opt.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = std::min(max_torq_input, opt.max_torq_input_lim);
  }

  motor_info_.raw_input_lim = kInvalidValue;
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = kInvalidValue;

  auto_enable_ = auto_enable;
  enable_mode_switch_ = enable_mode_switch;

  /* 如果禁用模式切换，则默认为 MIT 模式*/
  if (!enable_mode_switch) {
    dm_status_.current_ctrl_mode = CtrlMode::kMIT;
  }

  angle_ = 0.0f;
  vel_ = 0.0f;
  vel_raw_ = 0.0f;
  torq_ = 0.0f;
  curr_ = 0.0f;
  round_ = 0.0f;
  actual_angle_ = 0.0f;

  raw_input_ = 0;
  torq_input_ = 0;
  curr_input_ = 0;

  tx_id_ = 0;

  dm_status_ = DMStatus();
  comm_data_ = CommData();
}

/* Private function definitions ----------------------------------------------*/
HW_OPTIMIZE_O2_END
}  // namespace motor
}  // namespace hello_world
