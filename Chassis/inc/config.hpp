/*
 * @Author: hjlonglong 356794146@qq.com
 * @Date: 2026-10-24 19:05:02
 * @LastEditors: hjlonglong 356794146@qq.com
 * @LastEditTime: 2026-11-11 20:22:34
 * @FilePath: \Chassis\inc\config.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :config.hpp
 * @brief     :底盘板硬件/控制参数唯一来源（WP-03 建立，契约 §5.2）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      Lead            1. 初版：两板通用参数 + 底盘板参数
 *  V1.1.0      2026-11-07      Agent B         1. WP-12/14/17：新增 O 型全向轮几何/逆解
 *                                                常量（四角位置、安装角、方向符号、δ 符号）、
 *                                                回中阈值/模式去抖、跟随环微分低通、
 *                                                C2G v1.1 云台目标生成速率与 pitch 软限位
 *******************************************************************************
 * @attention :
 *  - 所有 magic number 必须放这里；改参数只改本文件（禁止在业务代码里散落常量）。
 *  - 标 TODO(硬件) 的项为默认假设，须按 docs/hardware_table.md 实测后更新；
 *    标 TODO(调参) 的项在实车调试阶段整定。
 *  - 改动本文件任何参数：同步 docs/hardware_table.md 并通知全组（契约 §0.3）。
 *  - 单位约定：长度 m、角度 rad、速度 m/s、角速度 rad/s、电流 A、力矩 N·m、时间 ms。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONFIG_HPP_
#define CONFIG_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace config
{
/* ============================ 两板通用（与 Gimbal/inc/config.hpp 必须完全一致）============================ */
inline constexpr float kTs = 0.001f;                 ///< s，控制周期（TIM6 1 ms）
inline constexpr uint32_t kCommOfflineThresMs = 100; ///< ms，双板帧离线阈值（契约 §3.2）
inline constexpr uint32_t kRemoteOfflineThresMs = 100; ///< ms，遥控器离线阈值
inline constexpr uint32_t kMotorOfflineThresMs = 100;  ///< ms，电机反馈离线阈值
inline constexpr float kStickDeadZone = 0.05f;         ///< 归一化摇杆死区
inline constexpr uint32_t kStateDebounceMs = 100;      ///< ms，状态迁移去抖
inline constexpr uint32_t kResurrectionHoldMs = 200;   ///< ms，Dead→Working 保持时间

// 双板帧 CAN ID（契约 §4.1：两板必须完全一致，改 ID = 改契约）
inline constexpr uint32_t kCanIdC2G = 0x210u;  ///< 底盘 → 云台
inline constexpr uint32_t kCanIdG2C = 0x211u;  ///< 云台 → 底盘
// 双板通讯总线（2026-11-08 实测：两板 CAN1 物理相连；原契约 §4.2 的 CAN2 假设作废）
inline constexpr uint8_t kCanBusInterboard = 1u;  ///< 1=CAN1 2=CAN2（两板必须一致）
/* 双板帧发送周期（ms）。两板 CAN1 连成一条总线，已挂 4×M3508 反馈(1 kHz)+0x200
 * (1 kHz)+云台 yaw 帧，约 7 帧/ms（~80% 负载）。若双板帧也按 1 ms 发，总数 9 帧/ms
 * 会超过 1 Mbps 容量；故降为 10 ms（100 Hz，状态机/跟随延迟可接受）。
 * TODO(硬件)：若把 yaw 电机改接到云台板 CAN2（与 pitch 同总线），可改回 1。
 * 两板必须一致（契约 §5.2 通用段）。 */
inline constexpr uint32_t kInterboardPeriodMs = 1;  ///< 双板帧周期：CAN1 仅双板帧时 1 ms（2026-11-09 拓扑修正后负载低）；若同总线还有电机等节点可调 10

/* ============================ 底盘板专用 ============================ */
/* ---- CAN 总线分配（2026-11-08 实物通讯拓扑核对）----
 *  - 两板 CAN1 相连：既走本板电机，也走双板通讯帧（C2G/G2C）；
 *  - 底盘板 CAN2：暂未使用（预留）。 */
inline constexpr uint8_t kCanBusChassisMotor = 2u;  ///< 底盘 4×M3508 所在 CAN（2026-11-09 实测：底盘板 CAN2）
inline constexpr uint32_t kCanIdMotorTx = 0x200u;       ///< M3508 控制帧（含电调 1~4）
inline constexpr uint32_t kCanIdMotorFdbBase = 0x201u;  ///< 反馈帧 = base + 电调 ID

/* ---- 轮电机（M3508；索引顺序 LF, LB, RB, RF）---- */
inline constexpr uint8_t kWheelCanId[4] = {1u, 2u, 3u, 4u};  ///< TODO(硬件): 电调 ID 与接线核对
inline constexpr int8_t kWheelDir[4] = {1, 1, 1, 1};          ///< TODO(硬件): 正方向标定（前进全正）
inline constexpr float kMaxWheelCurrent = 10.0f;              ///< A，调试期电流限幅
inline constexpr float kMaxWheelRadS = 800.0f;                ///< rad/s，轮速限幅

/* ---- 轮电机索引数量（各模块统一用该常量，避免魔法数）---- */
inline constexpr int kWheelNum = 4;  ///< 轮电机数量（索引 LF, LB, RB, RF）

/* ---- 底盘几何（v1.1：O 型全向轮，四轮驱动方向沿以车体中心为圆心的切线）----
 * 默认假设（TODO(硬件)）：轮心位于四角 (kHalfWheelbase, kHalfTrack)（x 前 / y 左），
 * 驱动方向与车体 x 轴夹角 α：LF=135°、LB=225°、RB=315°、RF=45°（切线方向）。
 * 若实车为其它安装方式，只改下面四角数组即可（逆解公式为通用形式，见
 * Chassis/inc/chassis_ik_solver.hpp）；角度符号/方向由 kWheelSign 吸收。 */
inline constexpr float kPi = 3.14159265358979f;  ///< 圆周率（本板局部常量）
inline constexpr float kHalfWheelbase = 0.19f;   ///< m，轮心到中心 x 向投影（TODO(硬件): 卷尺实测）
inline constexpr float kHalfTrack = 0.19f;       ///< m，轮心到中心 y 向投影（TODO(硬件): 卷尺实测）
inline constexpr float kWheelRadius = 0.076f;    ///< m，RM 标准 76 mm 轮（⚠️ 默认假设）
/* 四轮轮心位置（车体系，x 前 y 左），索引顺序 LF, LB, RB, RF */
inline constexpr float kWheelPosX[4] = {kHalfWheelbase, -kHalfWheelbase,
                                        -kHalfWheelbase, kHalfWheelbase};
inline constexpr float kWheelPosY[4] = {kHalfTrack, kHalfTrack,
                                        -kHalfTrack, -kHalfTrack};
/* 四轮驱动方向角 α（车体系，从 +x 逆时针量，rad），默认 O 型切线布置 */
inline constexpr float kWheelMountAngleRad[4] = {3.0f * kPi / 4.0f, 5.0f * kPi / 4.0f,
                                                 7.0f * kPi / 4.0f, kPi / 4.0f};
/* 每轮逆解输出符号（±1，TODO(硬件): 单轮正电流观察车体等效方向后标定；与 kWheelDir 分工：
 * kWheelDir 校正电机电气正方向，kWheelSign 校正轮系安装方向，二者不要同时翻同一轮） */
inline constexpr int8_t kWheelSign[4] = {1, 1, 1, 1};
/* δ 符号：δ = wrapPi(kDeltaSign * G2C.yaw_joint)，定义为【云台世界朝向 − 车头世界朝向】。
 * 2026-11-12 实车修正：IK 旋转变换已改为标准 R(δ)（车体系 = R(δ)·云台系），不再按
 * 契约旧公式（R(−δ)）实现；若实车仍反向，只改本符号（TODO(硬件)）。 */
inline constexpr int8_t kDeltaSign = 1;

/* ---- 麦轮兼容常量（骨架遗留，供对照/回退；v1.1 O 型轮逆解不再使用）---- */
inline constexpr float kMecanumK = kHalfWheelbase + kHalfTrack;  ///< L+W，麦轮旋转项
inline constexpr int8_t kRotationSign = 1;           ///< TODO(硬件): 麦轮逆解自转项符号

/* ---- 状态机/遥控输入（v1.1）---- */
inline constexpr float kStickCenterTol = 0.1f;    ///< 摇杆回中判定阈值（契约 §7.1：|摇杆|<0.1）
inline constexpr uint32_t kModeDebounceMs = 30;   ///< ms，左拨杆模式切换去抖（契约 §7.2 小陀螺 ≥30 ms）
/* 拨轮死区（2026-11-12 实车：拨轮回中附近抖动 → 自转指令抖）。
 * 归一化输入 [-1,1]；|x| <= kWheelDeadZone 输出 0，死区外线性重标定到满量程
 * （( |x|-dz )/(1-dz)），保证手感与最大转速不变。0 表示关闭；实车可调范围 0.05~0.15。 */
inline constexpr float kWheelDeadZone = 0.10f;

/* ---- 速度限幅（云台系）---- */
inline constexpr float kMaxVxVy = 1.5f;         ///< m/s，平移速度限幅
inline constexpr float kMaxW = 6.0f;            ///< rad/s，分离模式拨轮自转限幅
inline constexpr float kSpinWMax = 6.0f;        ///< rad/s，小陀螺自转限幅
/* 跟随环输出限幅（2026-11-11 提高：原 3 → 6，允许快速追上云台；与自转限幅同量级） */
inline constexpr float kFollowWMax = 6.0f;      ///< rad/s，跟随环输出限幅
/* 跟随死区（2026-11-11 收紧：原 0.035 → 0.02 rad≈1.1°，跟随后残差更小） */
inline constexpr float kFollowDeadBandRad = 0.02f;  ///< rad，跟随死区

/* ---- 轮速环 PID 初值（位置式，输入 rad/s 误差，输出电流 A）---- */
inline constexpr float kPidWheelKp = 1.2f;        ///< TODO(调参)
inline constexpr float kPidWheelKi = 0.8f;        ///< TODO(调参)
inline constexpr float kPidWheelKd = 0.0f;        ///< TODO(调参)
inline constexpr float kPidWheelIntegMax = kMaxWheelCurrent;  ///< 积分限幅

/* ---- 跟随环 PD（2026-11-11 重调：激进 PD，去掉积分）----
 * 输入 δ（云台世界朝向 − 车头世界朝向，rad），输出自转 w（rad/s），符号见 robot.cpp。
 * 原 Kp=1.2、Ki=0.8 的问题：云台快转时积分项累积（w 指令长时间偏大），δ 过零后
 * 积分来不及释放 → 底盘“冲过头再回正”（用户实测向右超调）。改为 PD：
 *   - 去掉积分（Ki=0）：消除过冲根因；
 *   - Kp 1.2 → 6.0：δ=15°(0.26rad) 就输出 ~1.6 rad/s，响应明显变快；
 *   - Kd 0 → 0.25：对 δ 测量值微分提供阻尼（chassis 超速追上时自动刹车），
 *     配合 d_lpf_alpha=0.5 滤掉编码器量化噪声；
 *   - 限幅 6 rad/s：δ=90° 时 P 项 9.4 被限到 6，全速追赶。
 * 若实车仍偏软/偏硬，优先调 kPidFollowKp（4~10），再微调 Kd（0.1~0.4），Ki 建议保持 0。 */
inline constexpr float kPidFollowKp = 6.0f;       ///< TODO(调参)
inline constexpr float kPidFollowKi = 0.0f;       ///< 建议保持 0（积分会导致跟随过冲）
inline constexpr float kPidFollowKd = 0.25f;      ///< 阻尼项（对 δ 微分，带低通）
inline constexpr float kPidFollowIntegMax = kFollowWMax;
inline constexpr float kPidFollowDlpfAlpha = 0.5f; ///< 跟随环微分一阶低通强度 [0,1]（越大越平滑）

/* ---- C2G v1.1 云台目标生成（契约 §3.2：底盘板把遥控换算成 gimbal_yaw_target /
 * gimbal_pitch_target；云台板消费）---- */
inline constexpr float kGimbalYawRateMax = 2.5f;    ///< rad/s，云台 yaw 目标角速率上限（TODO(调参)）
inline constexpr float kGimbalPitchRateMax = 1.5f;  ///< rad/s，云台 pitch 目标角速率上限（TODO(调参)）
inline constexpr float kGimbalPitchMin = -0.35f;    ///< rad，pitch 软限位下限（TODO(硬件): 与云台板 config 对齐）
inline constexpr float kGimbalPitchMax = 0.55f;     ///< rad，pitch 软限位上限（TODO(硬件): 与云台板 config 对齐）

/* ---- TEST_MODE 1：单轮台架扫掠（仅车轮离地调试；交付前 TEST_MODE 必须为 0）---- */
inline constexpr int kTestWheelIdx = 0;                  ///< 被测轮索引（0=LF,1=LB,2=RB,3=RF）
inline constexpr float kTestWheelCurrentLimit = 2.0f;    ///< A，台架测试电流限幅（≤2 A，安全）
inline constexpr float kTestWheelSweepMaxRadS = 40.0f;   ///< rad/s，扫掠目标幅值
inline constexpr float kTestWheelSweepRateRadS2 = 80.0f; ///< rad/s²，扫掠斜率（0.5 s 到峰值）
inline constexpr uint32_t kTestWheelStartDelayMs = 1000; ///< ms，上电后等待时间（便于观察）
inline constexpr uint32_t kTestWheelDurationMs = 15000;  ///< ms，扫掠总时长，到时目标归零

/* ---- 调试开关（交付前必须为 0，见工具 README §4/§9.3）---- */
#ifndef DEBUG_NO_IWDG
#define DEBUG_NO_IWDG 0  ///< 1 = 调试期临时关闭喂狗/复位（仅限台架 + Ozone 调试）
#endif
#ifndef TEST_MODE
#define TEST_MODE 0  ///< 1 = 开环/单项调试模式（交付前必须为 0）
#endif

}  // namespace config

#endif /* CONFIG_HPP_ */
