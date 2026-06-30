/**
 * @file    BoardLink.cpp
 * @brief   双板 CAN 通信模块 (底盘板 ? 云台板)
 *
 *          本文件集中管理所有板间通信的收发逻辑:
 *          - TX: 底盘板向云台板发送裁判系统数据、大Yaw相对角度
 *          - RX: 接收云台板下发的速度指令、IMU、模式等
 *          所有板间通信 CAN ID 在此统一定义, 外部模块不应直接处理板间帧。
 *
 * @author  Nas (refactored 2026-05)
 */
#include "BoardLink.hpp"

#include "Auto_control.hpp"
#include "CAN_receive_send.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"

#include "dm_imu.hpp"
#include "motor.hpp"
#include "User_math.hpp"

#include <cstring>

namespace
{

/* ============ CAN ID 定义 ============ */
enum class BoardLinkCanId : uint16_t
{
    ChassisSpeedXY      = 0x101,   // 云台→底盘: vx, vy
    ChassisSpeedRYaw    = 0x102,   // 云台→底盘: vr, yaw目标
    ChassisPowerLimit   = 0x103,   // 云台→底盘: 功率上限
    ChassisMode         = 0x104,   // 云台→底盘: 控制模式 + 底盘模式
    ChassisImuAttitude  = 0x105,   // 云台→底盘: pitch + yaw_cnt
    ChassisImuGyro      = 0x106,   // 云台→底盘: gyro[0], gyro[2]
    GimbalRelativeAngle = 0x107,   // 底盘→云台: 大Yaw相对角 + 速度
    ShootTriggerMode    = 0x108,   // 云台→底盘: 射击模式 + 自瞄模式
    SaltationMode       = 0x109,   // 云台→底盘: 跳变/扫描参数
    RefereeData1        = 0x091,   // 底盘→云台: HP + 比赛状态
    RefereeData2        = 0x092,   // 底盘→云台: 弹量 + 前哨站/基地血量
    RefereeData3        = 0x093,   // 底盘→云台: RFID + 初速
};

constexpr uint8_t kFrameLen        = 8;   // CAN 帧数据长度
constexpr uint8_t kSchedulePeriod  = 20;  // TX 调度周期 (tick数)

/**
 * @brief  BoardLinkCanId 枚举 → 原始 uint16_t CAN ID
 */
inline uint16_t ToRawId(BoardLinkCanId id)
{
    return static_cast<uint16_t>(id);
}

/**
 * @brief  判断接收到的CAN ID是否匹配指定的 BoardLinkCanId
 * @param  raw_id  接收到的原始CAN帧ID
 * @param  id      要匹配的 BoardLinkCanId 枚举值
 * @return true=匹配, false=不匹配
 */
inline bool MatchId(uint16_t raw_id, BoardLinkCanId id)
{
    return raw_id == ToRawId(id);
}

/* ============ 序列化工具 ============ */
inline void StoreFloat (float    v, uint8_t *p) { float_to_bytes(v, p); }
inline void StoreUint16(uint16_t v, uint8_t *p) { uint16_to_bytes(v, p); }
inline void StoreUint8 (uint8_t  v, uint8_t *p) { uint8_to_bytes(v, p); }

inline float    LoadFloat (uint8_t *p) { return bytes_to_float(p); }
inline uint16_t LoadUint16(uint8_t *p) { return bytes_to_uint16(p); }
inline uint8_t  LoadUint8 (uint8_t *p) { return bytes_to_uint8(p); }

/* ============================================================
 *  TX — 打包函数 (底盘板 → 云台板)
 * ============================================================ */

// 裁判系统数据包1: 当前/最大HP + 比赛阶段信息
void Pack_RefereeData1(uint8_t data[kFrameLen])
{
    StoreUint16(robot_status.current_HP,       &data[0]);
    StoreUint16(robot_status.maximum_HP,       &data[2]);
    StoreUint8 (game_status.game_type,         &data[4]);
    StoreUint8 (game_status.game_progress,     &data[5]);
    StoreUint16(game_status.stage_remain_time, &data[6]);
}

// 裁判系统数据包2: 17mm弹量 + 友方前哨站/基地血量 + 射频
void Pack_RefereeData2(uint8_t data[kFrameLen])
{
    StoreUint16(Referee_data.projectile_allowance_17mm, &data[0]);
    StoreUint16(game_robot_HP.ally_outpost_HP,          &data[2]);
    StoreUint16(game_robot_HP.ally_base_HP,             &data[4]);
    StoreUint8 (Referee_data.Launching_Frequency,       &data[6]);
}

// 裁判系统数据包3: RFID状态 + 弹丸初速
void Pack_RefereeData3(uint8_t data[kFrameLen])
{
    float_to_bytes(rfid_status.rfid_status, &data[0]);
    StoreFloat(Referee_data.Initial_SPEED,  &data[4]);
}

/* ============================================================
 *  RX — 解包函数 (云台板 → 底盘板)
 * ============================================================ */

// 底盘平移速度: vx(取反适配坐标系), vy(取反适配坐标系)
static void Receive_XY_Speed(uint8_t data[kFrameLen])
{
    chassis.vx = -LoadFloat(&data[0]);
    chassis.vy = -LoadFloat(&data[4]);
}

// 底盘旋转速度 + 云台Yaw目标角
static float speed_yaw;
static void Receive_R_Yaw(uint8_t data[kFrameLen])
{
    chassis.vr = LoadFloat(&data[0]);
    speed_yaw   = LoadFloat(&data[4]);
    Global.Gimbal.input.yaw = speed_yaw;
}

// 控制模式 + 底盘运动模式
static void Receive_Control_Mode(uint8_t data[kFrameLen])
{
    Global.Control.mode = (control_mode_e)(int)LoadFloat(&data[0]);
    Global.Chassis.mode = (chassis_mode_e)(int)LoadFloat(&data[4]);
}

// IMU姿态: pitch + yaw累计角
static void Receive_IMU_Attitude(uint8_t data[kFrameLen])
{
    imu.pitch   = LoadFloat(&data[0]);
    imu.yaw_cnt = LoadFloat(&data[4]);
}

// IMU角速度: gyro[0](roll轴) + gyro[2](yaw轴)
static void Receive_IMU_Gyro(uint8_t data[kFrameLen])
{
    imu.gyro[0] = LoadFloat(&data[0]);
    imu.gyro[2] = LoadFloat(&data[4]);
}

// 跳变/扫描参数: 云台规划速度 + 对齐标志 + 小电脑模式 + 扫描标志 + 区域码
static void Receive_Saltation_Mode(uint8_t data[kFrameLen])
{
    gimbal.yaw.planning_spd = LoadFloat(&data[0]);
    chassis.is_aligning     = LoadUint8(&data[4]);
    fromMINIPC.mode         = LoadUint8(&data[5]);
    Auto_data.is_scaning    = LoadUint8(&data[6]);
    chassis.region_code     = LoadUint8(&data[7]);
}

// 射击触发模式 + 自瞄模式
static void Receive_Trigger_Mode(uint8_t data[kFrameLen])
{
    Global.Shoot.trigger_mode = (trigger_mode_e)(int)LoadFloat(&data[0]);
    Global.Auto.mode          = (auto_mode_e)(int)LoadFloat(&data[4]);
}

/* ============================================================
 *  帧调度表
 *  TX: 分频+相位实现错峰发送, 避免总线拥塞
 *  RX: ID → 处理函数映射
 * ============================================================ */

struct TxFrame
{
    BoardLinkCanId id;
    void (*pack)(uint8_t[kFrameLen]);
    uint8_t divider;  // 分频系数 (1=每tick发送, 20=每20tick发送一次)
    uint8_t phase;    // 相位偏移 (错开各帧发送时刻)
};

struct RxFrame
{
    BoardLinkCanId id;
    void (*handle)(uint8_t[kFrameLen]);
};

// TX 发送表: 相对角度每tick发送, 裁判系统数据20tick一轮错峰发送
constexpr TxFrame kTxFrames[] = {
    { BoardLinkCanId::GimbalRelativeAngle, Pack_RelativeAngle,  1,  0 },
    { BoardLinkCanId::RefereeData1,        Pack_RefereeData1,  20,  2 },
    { BoardLinkCanId::RefereeData2,        Pack_RefereeData2,  20,  9 },
    { BoardLinkCanId::RefereeData3,        Pack_RefereeData3,  20, 16 },
};

// RX 接收表: 云台板下发的所有帧
constexpr RxFrame kRxFrames[] = {
    { BoardLinkCanId::ChassisSpeedXY,     Receive_XY_Speed },
    { BoardLinkCanId::ChassisSpeedRYaw,   Receive_R_Yaw },
    { BoardLinkCanId::ChassisImuAttitude, Receive_IMU_Attitude },
    { BoardLinkCanId::ChassisImuGyro,     Receive_IMU_Gyro },
    { BoardLinkCanId::ChassisMode,        Receive_Control_Mode },
    { BoardLinkCanId::ShootTriggerMode,   Receive_Trigger_Mode },
    { BoardLinkCanId::SaltationMode,      Receive_Saltation_Mode },
};

uint8_t g_tick = 0;  // TX 调度计数器

} // anonymous namespace

/* ============================================================
 *  extern "C" 公开函数 — 供 freertos.c / CAN中断调用
 * ============================================================ */

/**
 * @brief  打包大Yaw相对角度 + 当前角速度
 *         每个控制周期由 TX 调度自动调用
 */
extern "C" void Pack_RelativeAngle(uint8_t data[8])
{
    // 从DM4310编码器获取相对角度 (减去零点)
    chassis.relative_angle = DMMotor_GetData(BIGYAWMotor).motor_data.para.angle_cnt - BIG_YAW_ZERO;
    float_to_bytes(chassis.relative_angle, &data[0]);
    float_to_bytes(gimbal.yaw.spd_now,     &data[4]);
}

/* ============================================================
 *  公开接口实现
 * ============================================================ */

/**
 * @brief  初始化板间通信模块, 重置调度计数器
 */
void BoardLink_Init(void)
{
    g_tick = 0;
}

/**
 * @brief  TX 发送步进 — 每次控制周期调用一次
 *         根据分频+相位决定本tick发送哪些帧
 */
void BoardLink_TxStep(void)
{
    uint8_t data[kFrameLen];

    for (uint8_t i = 0; i < sizeof(kTxFrames) / sizeof(kTxFrames[0]); i++)
    {
        const TxFrame &frame = kTxFrames[i];
        if (g_tick % frame.divider != frame.phase)
            continue;

        memset(data, 0, sizeof(data));
        frame.pack(data);
        Fdcanx_SendData(Get_CanHandle(1), ToRawId(frame.id), data, kFrameLen);
    }

    g_tick++;
    if (g_tick >= kSchedulePeriod)
        g_tick = 0;
}

/**
 * @brief  RX 接收分发 — 在CAN接收中断中调用
 *         遍历 kRxFrames 路由表, 用 MatchId 匹配 ID 并调用对应处理函数
 * @param  hfdcan  CAN外设句柄 (保留, 便于多CAN扩展)
 * @param  id      接收到的CAN帧ID
 * @param  data    8字节数据
 * @return 1=已处理(板间通信帧), 0=未匹配(交给其他模块处理)
 */
uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8])
{
    (void)hfdcan;

    for (uint8_t i = 0; i < sizeof(kRxFrames) / sizeof(kRxFrames[0]); i++)
    {
        if (MatchId(id, kRxFrames[i].id))
        {
            kRxFrames[i].handle(data);
            return BOARD_LINK_RX_HANDLED;
        }
    }

    return BOARD_LINK_RX_UNHANDLED;
}
