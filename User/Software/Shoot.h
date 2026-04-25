/*
 * @Date: 2025-10-27 21:03:17
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-04-25 23:58:45
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Shoot.h
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "CAN_receive_send.h"
#include "pid.h"

#include "motor.h"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,// 开始反转值
    SHOOT_SPEED_CLOSE = 0,    // 停止速度值
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,  // 正常工作值
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG, // 退弹低速值
};

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,
    TRIGGER_SPEED_AIM_HIGH = 7680,
    TRIGGER_SPEED_AIM_MID = 2400,
    TRIGGER_SPEED_AIM_LOW = 1200,
};

/* 热量控制状态机 */
typedef enum {
    HEAT_BYPASS = 0, // 未接收到裁判热量数据时旁路热量控制
    HEAT_BOOST,
    HEAT_TRACK,
    HEAT_KEEPALIVE,
    HEAT_GUARD,
} trigger_heat_state_e;

/* 热量控制参数 */
#define HEAT_VALID_LIMIT_MIN         1.0f            // Heat_Limit 的最小有效值，小于该值时认为裁判热量上限无效
#define HEAT_VALID_COOLING_MIN       1.0f            // CooLing_Value 的最小有效值，小于该值时认为裁判冷却数据无效
#define HEAT_RESERVE_MARGIN          15.0f           // 预留热量安全余量，实际控制时使用 safe_margin = Heat_Limit - Reserve - Barrel_Heat
#define HEAT_BOOST_ENTER_FACTOR      4.0f            // 进入 BOOST 的阈值系数：safe_margin >= 4 * cooling_value
#define HEAT_BOOST_EXIT_FACTOR       (10.0f / 3.0f)  // 退出 BOOST 的阈值系数，用于形成 BOOST -> TRACK 回差
#define HEAT_TRACK_ENTER_FACTOR      2.0f            // BYPASS 首次归位时进入 TRACK 的阈值系数
#define HEAT_KEEP_EXIT_FACTOR        (5.0f / 3.0f)   // TRACK -> KEEPALIVE 的阈值系数，safe_margin 低于该值时进入保活区
#define HEAT_KEEP_ENTER_FACTOR       1.0f            // BYPASS 首次归位时进入 KEEPALIVE 的阈值系数
#define HEAT_TRACK_RETURN_FACTOR     (7.0f / 3.0f)   // KEEPALIVE -> TRACK 的返回阈值系数，同时也是 TRACK 平滑段的下边界
#define HEAT_GUARD_ENTER_FACTOR      0.50f           // KEEPALIVE -> GUARD 的阈值系数，进入极低速保底区
#define HEAT_GUARD_EXIT_FACTOR       1.00f           // GUARD -> KEEPALIVE 的返回阈值系数，形成 GUARD 回差
#define HEAT_KEEP_SPEED_RATIO        0.20f           // KEEPALIVE 目标速度占基础速度的比例
#define HEAT_KEEP_SPEED_MIN          3840.0f          // KEEPALIVE 最小速度，防止高热量区转速过低导致拨盘停滞
#define HEAT_KEEP_SPEED_MAX          5760.0f         // KEEPALIVE 最大速度，限制高基础速度模式下的热量继续上冲
#define HEAT_GUARD_SPEED_RATIO       0.10f           // GUARD 目标速度占基础速度的比例
#define HEAT_GUARD_SPEED_MIN         2400.0f          // GUARD 最小速度，保证逼近热量边界时拨盘仍持续转动
#define HEAT_GUARD_SPEED_MAX         3480.0f          // GUARD 最大速度，防止极限热量区仍输出过高转速

/* 内部调用 */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

/*内部数据类型*/
typedef struct
{
    /*-------PID-------*/
    pid_t shoot_L_speed_pid;
    pid_t shoot_R_speed_pid;
    pid_t trigger_speed_pid;

    /*-------状态量-------*/
    float shoot_speed_L_now;
    float shoot_speed_R_now;
    float trigger_speed_now;
    float trigger_current_now;

    /*-------目标量-------*/
    float trigger_speed_base;
    float trigger_speed_set;
    enum shoot_speed_e shoot_speed_set;
    float current[3];

    /*-------热量控制-------*/
    uint8_t heat_control_enabled;
    trigger_heat_state_e heat_state;

}Shoot_t;

/* 外部调用 */
void Shoot_Init();
void Shoot_Tasks();
#endif 

