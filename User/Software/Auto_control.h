/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-24 04:16:18
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Auto_control.h
 */
#ifndef __AUTO_CONTROL_H__
#define __AUTO_CONTROL_H__

#include "stdint.h"
#include "referee_system.h"
#include "robot_param.h"

#if(AUTO_TJU == 1)

#pragma pack(1)

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float curr_yaw;
        float curr_pitch;
        float curr_omega;
        uint8_t state;// state 0是打车 1是打前哨站 2是打小符 3是打符
        uint8_t autoaim; // autoaim那个0是不用自瞄 1是开自瞄
        uint8_t enemy_color;// 0为蓝色，1为红色
    } To_minipc_data; // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;//2
    uint8_t enter;//1
} STM32_data_t;

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float shoot_yaw;
        float shoot_pitch;
        uint8_t fire;      // 发弹信号
        uint8_t target_id; // 目标ID,UI显示用
    } from_minipc_data;    // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;
} MINIPC_data_t;

#pragma pack(4)

void STM32_to_MINIPC(float yaw,float pitch,float omega);

#endif

#if(AUTO_CSU == 1)
#pragma pack(1)


typedef struct
{
    uint8_t header;   
    uint8_t mode;     
    float roll;       
    float pitch;
    float yaw;
    uint8_t fill;
    uint8_t ender;    
} __attribute__((packed))STM32_data_t;

typedef struct
{
    uint8_t header;   
    uint8_t fire;     
    float pitch;
    float yaw;
    float distence;   
    uint8_t ender;    
} __attribute__((packed))MINIPC_data_t;
#pragma pack(4)

void STM32_to_MINIPC();

#endif

#if(AUTO_TongJi == 1)
#pragma pack(1)

/**
 * @brief 机器人运行模式 (下位机 -> 上位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum {
    MODE_IDLE       = 0, // 空闲
    MODE_AUTO_AIM   = 1, // 自瞄
    MODE_SMALL_BUFF = 2, // 小符
    MODE_BIG_BUFF   = 3, // 大符
    MODE_OUTPOST    = 4  // 前哨站 (保留)
} RobotMode_e;


/**
 * @brief 视觉控制模式 (上位机 -> 下位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum {
    CTRL_NO_CONTROL     = 0, // 不控制
    CTRL_AIM_ONLY       = 1, // 控制云台但不开火
    CTRL_AIM_AND_FIRE   = 2  // 控制云台且开火
} VisionControlMode_e;


/**
 * @brief 上位机发送给STM32的数据 (MiniPC -> STM32)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct {
    uint8_t header[2] ;   // 帧头 (固定值 {'S', 'P'})

    uint8_t mode;        // 控制模式 (VisionControlMode_e)

    float yaw;           // 偏航角 (rad)
    float yaw_vel;       // 偏航角速度 (rad/s)
    float yaw_acc;       // 偏航角加速度 (rad/s^2)

    float pitch;         // 俯仰角 (rad)
    float pitch_vel;     // 俯仰角速度 (rad/s)
    float pitch_acc;     // 俯仰角加速度 (rad/s^2) 

    uint16_t crc16;      // CRC16 校验
} MINIPC_data_t;

/**
 * @brief STM32发送给上位机的数据 (STM32 -> MiniPC)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct {
    uint8_t header[2];   // 帧头 (固定值 {'S', 'P'})

    uint8_t mode;        // 当前模式 (RobotMode_e)

    // --- IMU 姿态 (四元数) ---
    float q[4];          // w, x, y, z 顺序

    // --- 云台状态 ---
    float yaw;           // 当前偏航角 (rad)
    float yaw_vel;       // 当前偏航角速度 (rad/s)
    float pitch;         // 当前俯仰角 (rad)
    float pitch_vel;     // 当前俯仰角速度 (rad/s)

    // --- 射击状态 ---
    float bullet_speed;      // 弹速 (m/s)
    uint16_t bullet_count;   // 子弹累计发送次数 (用于统计/丢包检测)

    uint16_t crc16;      // CRC16 校验
} STM32_data_t;

void STM32_to_MINIPC();
#pragma pack(4)
#endif

#pragma pack(1)

typedef struct
{
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t remain_hp;
    uint16_t max_hp;
    uint16_t stage_remain_time;
    uint16_t bullet_remaining_num_17mm;
    uint16_t outpost_hp;
    uint16_t base_hp;
    uint32_t rfid_status;
} STM32ROS_data_t;


typedef struct
{
    uint8_t header;
    uint8_t checksum;
    float x_speed;
    float y_speed;
    float rotate;
    float yaw_speed;
    uint8_t running_state;
} Navigation_data_t;

typedef struct                                       //哨兵自主决策信息
{
    uint16_t header;                                 // 帧头
    uint16_t sender;                                 // 发送端
    uint16_t receiver;                               // 接收端

    uint32_t check_revive_status : 1;                // 检查复活状态
    uint32_t check_exchange_revive_status : 1;       // 检查云台手让哨兵是否复活
    uint32_t request_increase_ammo_number : 11;      // 请求云台手补弹丸数量
    uint32_t request_exchange_ammo_time : 4;         // 请求交换弹丸时间
    uint32_t request_HP_time : 4;                    // 请求加血时间
    uint32_t sentry_posture : 2;                     // 哨兵姿态 (1进攻, 2防御, 3移动)
    uint32_t activate_energy_mechanism : 1;          // 确认能量机关激活状态 (1为确认)
    uint32_t reserved : 8;                           //保留位  
    uint16_t CRC_16;
}Sentry_cmd_t;

typedef struct 
{
    uint8_t frame_header;
    uint16_t length;
    uint8_t sequence;
    uint8_t CRC8;
    uint8_t data[121];
    referee_cmd_id_t referee_cmd_id;
}Referee_UART_data_t;

/**
 * @brief 自瞄规划数据
 * 
 */
typedef struct 
{
    float target_yaw;
    float target_pitch;
    uint8_t is_scaning;
}Auto_Plan_t;
#pragma pack(4)

void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len);
void Auto_Control();
void MINIPC_to_STM32();
void decodeNAVdata(Navigation_data_t *target, unsigned char buff[], unsigned int len);
void Decision_Transmission(Sentry_cmd_t *sentry_cmd_info, uint8_t *tx_buffer);
uint32_t Get_Sentry_Posture_From_Aligning(uint8_t aligning);
extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;;
extern Navigation_data_t Navigation_receive_1;
extern STM32ROS_data_t stm32send_1;
extern int Navigation_online;
extern Sentry_cmd_t Sentry_cmd_1;
extern Auto_Plan_t Auto_data;

#endif