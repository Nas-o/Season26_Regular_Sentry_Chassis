/**
 * @file Auto_control.cpp
 * @author Nas (1319621819@qq.com)
 * @brief 自瞄控制器
 * @version 0.1
 * @date 2026-06-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Auto_control.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "Chassis.hpp"
#include "BoardLink.hpp"

#include "IMU_updata.hpp"
#include "dm_imu.hpp"
#include "referee_system.hpp"
#include "UART_data_txrx.hpp"
#include "USB_VirCom.hpp"

#include "CRC8_CRC16.hpp"
#include "string.h"

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;
STM32ROS_data_t stm32send_1;
Sentry_cmd_t Sentry_cmd_1;
Auto_Plan_t Auto_data;
uint8_t Debug_Sentry_Revive_State = 0;
uint32_t Debug_Sentry_Tx_Count = 0;

uint8_t data[128];
uint8_t rx_data[100];


void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

void decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len)
{
    memcpy(target, buff, len);
}

#if(AUTO_CSU == 1)

/**
 * @brief 向上位机发送自瞄相关数据
 * 
 * @param yaw yaw轴当前角度（弧度）
 * @param pitch pitch轴当前角度（弧度）
 * @param omega yaw轴当前角速度（rad/s）
 */
 void STM32_to_MINIPC()
{
    toMINIPC.header = 0xff;
    toMINIPC.ender = 0x0d;
    toMINIPC.mode = 0;
    toMINIPC.roll = imu.roll / RAD_TO_DEG;
    toMINIPC.pitch = imu.pitch / RAD_TO_DEG;
	toMINIPC.yaw = imu.yaw / RAD_TO_DEG;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    Global.Auto.input.shoot_pitch = fromMINIPC.pitch;
	Global.Auto.input.shoot_yaw = fromMINIPC.yaw;
	Global.Auto.input.fire = fromMINIPC.fire;
} 

#endif

#if(AUTO_TJU == 1)

void STM32_to_MINIPC(float yaw,float pitch,float omega)
{
    toMINIPC.FrameHeader.sof = 0xA5;
    toMINIPC.FrameHeader.crc8 = 0x00;
    toMINIPC.To_minipc_data.curr_pitch = pitch;//IMU_data.AHRS.pitch;
    toMINIPC.To_minipc_data.curr_yaw = yaw;//IMU_data.AHRS.yaw;
    toMINIPC.To_minipc_data.curr_omega = omega;//cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[1];
    toMINIPC.To_minipc_data.autoaim = 1;
    if (Referee_data.robot_id >= 100)
        toMINIPC.To_minipc_data.enemy_color = 1;
    else
        toMINIPC.To_minipc_data.enemy_color = 0;
    toMINIPC.To_minipc_data.state = 0;
    toMINIPC.FrameTailer.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.FrameHeader.sof, 17, 0xffff);
    toMINIPC.enter = 0x0A;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    // VirCom_send(data, sizeof(STM32_data_t));
    UART_SendData(UART1_data, data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    if (fabs(fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw) > PI / 2.0f) // 过零点处理
    {
        if (fromMINIPC.from_minipc_data.shoot_yaw > PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw -= 2 * PI;
        else if (fromMINIPC.from_minipc_data.shoot_yaw < -PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw += 2 * PI;
    }
    Global.Auto.input.shoot_pitch = fromMINIPC.from_minipc_data.shoot_pitch - IMU_data.AHRS.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw;
    Global.Auto.input.fire = fromMINIPC.from_minipc_data.fire;
    Global.Auto.input.target_id = fromMINIPC.from_minipc_data.target_id;
    if(fromMINIPC.from_minipc_data.shoot_pitch==0&&fromMINIPC.from_minipc_data.shoot_yaw==0)
        Global.Auto.input.fire = -1;
}


#endif

#if(AUTO_TongJi == 1)

void STM32_to_MINIPC()
{
    toMINIPC.header[0] = 'S';
    toMINIPC.header[1] = 'P';
    toMINIPC.mode = MODE_SMALL_BUFF;
    toMINIPC.yaw = IMU_data.AHRS.yaw;
    toMINIPC.pitch = IMU_data.AHRS.pitch;
    toMINIPC.yaw_vel = IMU_data.gyro[2];
    toMINIPC.pitch_vel = IMU_data.gyro[0];
    toMINIPC.q[0] = IMU_data.AHRS.q[0];
    toMINIPC.q[1] = IMU_data.AHRS.q[1];
    toMINIPC.q[2] = IMU_data.AHRS.q[2];
    toMINIPC.q[3] = IMU_data.AHRS.q[3];
    toMINIPC.bullet_speed = Referee_data.Initial_SPEED;
    toMINIPC.bullet_count = Referee_data.Launching_Frequency;
    toMINIPC.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.header,41,0xFFFF);
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    Vircom_Send(data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    Global.Auto.input.shoot_pitch = fromMINIPC.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.yaw;
}
#endif

uint32_t Get_Sentry_Posture_From_Aligning(uint8_t aligning)
{
    if (aligning >= 1 && aligning <= 3)
        return aligning;
    return 3;
}

void Decision_Transmission(Sentry_cmd_t *sentry_cmd_info, uint8_t *tx_buffer)
{
    // 使用 uint8_t 指针直接操作缓冲区
    uint8_t *frame = tx_buffer;
    static uint8_t s_seq = 0;
    // 根据协议文档，哨兵自主决策指令(0x0120)需要发送给裁判系统服务器(0x8080)
    // 因此必须使用 机器人间交互数据(0x0301) 进行封装
    // 数据段包含: 交互数据头(6字节) + 决策数据(4字节) = 10字节
    uint16_t data_length = 10;
    //  填充帧头 (5 bytes)
    frame[0] = 0xA5; // SOF
    frame[1] = data_length & 0xFF;    
    frame[2] = (data_length >> 8) & 0xFF; 
    frame[3] = s_seq++; 
    // 计算并填充 Header CRC8 (写入 frame[4])
    // 校验范围: frame[0] ~ frame[3] (4 bytes)
    append_CRC8_check_sum(frame, 5);
    // 填充命令码 CmdID (2 bytes) 
    // 0x0301 机器人间交互
    frame[5] = 0x01; 
    frame[6] = 0x03; 
    // 填充数据段 (10 bytes)
    //交互数据头 (6 bytes)
    // data_cmd_id = 0x0120 (哨兵自主决策)
    frame[7] = 0x20;
    frame[8] = 0x01;
    // sender_id = robot_status.robot_id (本机ID)
    frame[9] = robot_status.robot_id & 0xFF;
    frame[10] = (robot_status.robot_id >> 8) & 0xFF;
    // receiver_id = 0x8080 (裁判系统服务器)
    frame[11] = 0x80;
    frame[12] = 0x80;
    // 3.2 哨兵决策数据 (4 bytes)
    uint32_t decision_data = 0;
    // bit 0: 哨兵机器人是否确认复活
    if (sentry_cmd_info->check_revive_status)
        decision_data |= (1 << 0); 
    // bit 1: 哨兵机器人是否想要兑换立即复活
    if (sentry_cmd_info->check_exchange_revive_status)
        decision_data |= (1 << 1);
    // bit 2-12: 哨兵想要兑换的发弹量值 (11 bits)
    decision_data |= (sentry_cmd_info->request_increase_ammo_number & 0x7FF) << 2;
    // bit 13-16: 哨兵想要远程兑换发弹量的请求次数 (4 bits)
    decision_data |= (sentry_cmd_info->request_exchange_ammo_time & 0x0F) << 13;
    // bit 17-20: 哨兵想要远程兑换血量的请求次数 (4 bits)
    decision_data |= (sentry_cmd_info->request_HP_time & 0x0F) << 17;
    // bit 21-22: 哨兵修改当前姿态指令 (2 bits)
    decision_data |= (sentry_cmd_info->sentry_posture & 0x03) << 21;
    // bit 23: 能量机关激活确认
    if (sentry_cmd_info->activate_energy_mechanism)
    decision_data |= (1 << 23);
    // bit 21-31: 保留 (默认为0)
    // 将 32位 数据按小端序写入 frame[13]~frame[16]
    memcpy(&frame[13], &decision_data, 4);
    // 计算并追加 Frame CRC16 (2 bytes)
    // 校验范围: frame[0] ~ frame[16] (17 bytes)
    // 写入位置: frame[17], frame[18]
    // 总长度: 19 bytes
    append_CRC16_check_sum(frame, 19);

    // 发送数据 
    if (HAL_UART_Transmit_DMA(&huart1, frame, 19) == HAL_OK)
    {
        Debug_Sentry_Tx_Count++;
        if (sentry_cmd_info->check_revive_status != 0 || sentry_cmd_info->check_exchange_revive_status != 0)
        {
            Debug_Sentry_Revive_State = 1; // 正在发送复活指令
        }
        else
        {
            Debug_Sentry_Revive_State = 0; // 未发送复活指令
        }
    }
}


void Auto_Control()
{

    Gimbal_SetYawAngle((180.0 / 3.14159265358979323846) * imu.yaw_cnt + (180.0 / 3.14159265358979323846) * (Global.Auto.input.shoot_yaw));
    Gimbal_SetPitchAngle((180.0 / 3.14159265358979323846) * imu.pitch + (180.0 / 3.14159265358979323846) * (Global.Auto.input.shoot_pitch));

}

// 只填充 data，不发送，不声明自己的 buffer
namespace {

void Pack_RefereeData1(uint8_t data[8])
{
    uint16_to_bytes(robot_status.current_HP,         &data[0]);
    uint16_to_bytes(robot_status.maximum_HP,         &data[2]);
    uint8_to_bytes (game_status.game_type,           &data[4]);
    uint8_to_bytes (game_status.game_progress,       &data[5]);
    uint16_to_bytes(game_status.stage_remain_time,   &data[6]);
}

void Pack_RefereeData2(uint8_t data[8])
{
    uint16_to_bytes(Referee_data.projectile_allowance_17mm, &data[0]);
    uint16_to_bytes(game_robot_HP.ally_outpost_HP,          &data[2]);
    uint16_to_bytes(game_robot_HP.ally_base_HP,             &data[4]);
    uint8_to_bytes (Referee_data.Launching_Frequency,       &data[6]);
}

void Pack_RefereeData3(uint8_t data[8])
{
    float_to_bytes(rfid_status.rfid_status,    &data[0]);
    float_to_bytes(Referee_data.Initial_SPEED, &data[4]);
} 


typedef struct {
    uint16_t    id;
    CanTxPack_t pack;
    uint8_t     divider;
    uint8_t     phase;
} GimbalTxEntry_t;

const GimbalTxEntry_t GimbalTxTable[] = {
    { CAN_ID_GIMBAL_RELATIVE_ANGLE, Pack_RelativeAngle,  1,  0 },
    { CAN_ID_REFEREE_DATA_1,        Pack_RefereeData1,  20,  2 },  // 50Hz, phase=2ms
    { CAN_ID_REFEREE_DATA_2,        Pack_RefereeData2,  20,  9 },  // 50Hz, phase=9ms
    { CAN_ID_REFEREE_DATA_3,        Pack_RefereeData3,  20, 16 },  // 50Hz, phase=16ms
};

} // anonymous namespace

void Gimbal_CAN_SendAll(void)
{
    static uint8_t s_tick = 0;
    uint8_t buf[8];

    for (uint8_t i = 0; i < sizeof(GimbalTxTable)/sizeof(GimbalTxTable[0]); i++) {
        if (s_tick % GimbalTxTable[i].divider != GimbalTxTable[i].phase) {
            continue;
        }

        memset(buf, 0, sizeof(buf));
        GimbalTxTable[i].pack(buf);
        Fdcanx_SendData(&hfdcan2, GimbalTxTable[i].id, buf, 8);
    }

    if (++s_tick >= 20) s_tick = 0;
}
