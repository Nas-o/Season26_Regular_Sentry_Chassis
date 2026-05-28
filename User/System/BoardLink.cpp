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
enum class BoardLinkCanId : uint16_t
{
    ChassisSpeedXY = 0x101,
    ChassisSpeedRYaw = 0x102,
    ChassisPowerLimit = 0x103,
    ChassisMode = 0x104,
    ChassisImuAttitude = 0x105,
    ChassisImuGyro = 0x106,
    GimbalRelativeAngle = 0x107,
    ShootTriggerMode = 0x108,
    RefereeData1 = 0x091,
    RefereeData2 = 0x092,
    RefereeData3 = 0x093,
    SaltationMode = 0x109,
};

constexpr uint8_t kCanFrameLength = 8;
constexpr uint8_t kSchedulePeriodTick = 20;

struct SchedulerState
{
    uint8_t tick = 0;
};

inline bool ShouldForceTriggerResend(BoardLinkCanId) { return false; }
inline bool ShouldForceModeResend(BoardLinkCanId) { return false; }
inline void ConsumeFastResend(BoardLinkCanId) {}

struct TxFrame
{
    BoardLinkCanId id {};
    void (*pack)(uint8_t data[kCanFrameLength]) = nullptr;
    uint8_t divider = 1;
    uint8_t phase = 0;
};

struct RxFrame
{
    BoardLinkCanId id {};
    void (*handle)(uint8_t data[kCanFrameLength]) = nullptr;
};


inline uint16_t ToRawId(BoardLinkCanId id)
{
    return static_cast<uint16_t>(id);
}

inline bool MatchId(uint16_t raw_id, BoardLinkCanId id)
{
    return raw_id == ToRawId(id);
}

inline void StoreFloat(float value, uint8_t *bytes)
{
    float_to_bytes(value, bytes);
}

inline void StoreUint16(float value, uint8_t *bytes)
{
    uint16_to_bytes(value, bytes);
}

inline void StoreUint8(uint8_t value, uint8_t *bytes)
{
    uint8_to_bytes(value, bytes);
}

inline float LoadFloat(uint8_t *bytes)
{
    return bytes_to_float(bytes);
}

inline uint16_t LoadUint16(uint8_t *bytes)
{
    return bytes_to_uint16(bytes);
}

inline uint8_t LoadUint8(uint8_t *bytes)
{
    return bytes_to_uint8(bytes);
}

void Pack_RefereeData1(uint8_t data[kCanFrameLength])
{
    StoreUint16(robot_status.current_HP,         &data[0]);
    StoreUint16(robot_status.maximum_HP,         &data[2]);
    StoreUint8 (game_status.game_type,           &data[4]);
    StoreUint8 (game_status.game_progress,       &data[5]);
    StoreUint16(game_status.stage_remain_time,   &data[6]);
}

void Pack_RefereeData2(uint8_t data[kCanFrameLength])
{
    StoreUint16(Referee_data.projectile_allowance_17mm, &data[0]);
    StoreUint16(game_robot_HP.ally_outpost_HP,          &data[2]);
    StoreUint16(game_robot_HP.ally_base_HP,             &data[4]);
    StoreUint8 (Referee_data.Launching_Frequency,       &data[6]);
}

void Pack_RefereeData3(uint8_t data[kCanFrameLength])
{
    StoreFloat (rfid_status.rfid_status,    &data[0]);
    StoreFloat (Referee_data.Initial_SPEED, &data[4]);
}

void Pack_RelativeAngle(uint8_t data[kCanFrameLength])
{
    Chassis.relative_angle = DMMotor_GetData(BIGYAWMotor).motor_data.para.angle_cnt - BIG_YAW_ZERO;
    StoreFloat (Chassis.relative_angle, &data[0]);
    StoreFloat (Gimbal.big_yaw.big_yaw_speed_now, &data[4]);
}

static void Receive_XY_Speed(uint8_t data[kCanFrameLength])
{
    X_speed = -LoadFloat(&data[0]);
    Y_speed = -LoadFloat(&data[4]);
}

static float speed_yaw;
static void Receive_R_Yaw(uint8_t data[kCanFrameLength])
{
    R_speed = LoadFloat(&data[0]);
    speed_yaw = LoadFloat(&data[4]);
    Global.Gimbal.input.yaw = speed_yaw; // µ¥Î» rad
}

static void Receive_Control_Mode(uint8_t data[kCanFrameLength])
{
    Global.Control.mode = (enum control_mode_e)(int)LoadFloat(&data[0]);
    Global.Chassis.mode = (enum chassis_mode_e)(int)LoadFloat(&data[4]);
}

static void Receive_IMU_Attitude(uint8_t data[kCanFrameLength])
{
    imu.pitch   = LoadFloat(&data[0]);
    imu.yaw_cnt = LoadFloat(&data[4]);
}

static void Receive_IMU_Gyro(uint8_t data[kCanFrameLength])
{
    imu.gyro[0] = LoadFloat(&data[0]);
    imu.gyro[2] = LoadFloat(&data[4]);
}

static void Receive_Saltation_Mode(uint8_t data[kCanFrameLength])
{
    Gimbal.big_yaw.planning_speed = LoadFloat(&data[0]);
    Chassis.is_aligning           = LoadUint8(&data[4]);
    fromMINIPC.mode               = LoadUint8(&data[5]);
    Auto_data.is_scaning          = LoadUint8(&data[6]);
    Chassis.region_code           = LoadUint8(&data[7]);
}

static void Receive_Trigger_Mode(uint8_t data[kCanFrameLength])
{
    Global.Shoot.trigger_mode = (enum trigger_mode_e)(int)LoadFloat(&data[0]);   
    Global.Auto.mode          = (enum auto_mode_e)(int)LoadFloat(&data[4]);
}
const TxFrame kTxFrames[] = {
    {BoardLinkCanId::GimbalRelativeAngle, Pack_RelativeAngle, 1, 0},
    {BoardLinkCanId::RefereeData1, Pack_RefereeData1, 20, 2},
    {BoardLinkCanId::RefereeData2, Pack_RefereeData2, 20, 9},
    {BoardLinkCanId::RefereeData3, Pack_RefereeData3, 20, 16},
};

const RxFrame kRxFrames[] = {
    {BoardLinkCanId::ChassisSpeedXY, Receive_XY_Speed},
    {BoardLinkCanId::ChassisSpeedRYaw, Receive_R_Yaw},
    {BoardLinkCanId::ChassisImuAttitude, Receive_IMU_Attitude},
    {BoardLinkCanId::ChassisImuGyro, Receive_IMU_Gyro},
    {BoardLinkCanId::ChassisMode, Receive_Control_Mode},
    {BoardLinkCanId::ShootTriggerMode, Receive_Trigger_Mode},
    {BoardLinkCanId::SaltationMode, Receive_Saltation_Mode},
};

SchedulerState g_scheduler {};

}

void BoardLink_Init(void)
{
    g_scheduler = SchedulerState{};
}

void BoardLink_TxStep(void)
{

    uint8_t data[kCanFrameLength];
    for (uint8_t i = 0; i < sizeof(kTxFrames) / sizeof(kTxFrames[0]); i++)
    {
        const TxFrame &frame = kTxFrames[i];
        uint8_t should_send = (g_scheduler.tick % frame.divider == frame.phase);

        if (ShouldForceTriggerResend(frame.id) || ShouldForceModeResend(frame.id))
        {
            should_send = 1;
        }

        if (should_send == 0)
        {
            continue;
        }

        memset(data, 0, sizeof(data));
        frame.pack(data);
        if (Fdcanx_SendData(Get_CanHandle(1), ToRawId(frame.id), data, kCanFrameLength) == 0)
        {
            ConsumeFastResend(frame.id);
        }
    }

    g_scheduler.tick++;
    if (g_scheduler.tick >= kSchedulePeriodTick)
    {
        g_scheduler.tick = 0;
    }
}

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
