/**
 * @file    Shoot.hpp
 * @brief   发射机构控制器
 * @author  Nas (refactored 2026-05)
 */
#ifndef __SHOOT_H__
#define __SHOOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* C-compatible API */
void Shoot_Init(void);
void Shoot_Tasks(void);

#ifdef __cplusplus
}
#endif

/* =================== C++ Only =================== */
#ifdef __cplusplus

#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "motor.hpp"

/* 摩擦轮速度枚举 */
enum shoot_speed_e
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,
    SHOOT_SPEED_CLOSE = 0,
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG,
};

/* 拨弹电机速度枚举 */
enum trigger_speed_e
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

/* 单发限位参数 */
#define A_BULLET_ECD 85599
#define TRIGGER_ONE_BULLET_ECD   ((int32_t)A_BULLET_ECD)
#define TRIGGER_EXTRA_ECD        (150)
#define SINGLE_DONE_ECD_TH     1200
#define STOP_FIRE_TIMEOUT_MS     2000U
#define TRIGGER_ANGLE_SPEED_MAX  6000.0f

/* 电机宏 */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

/* 停火状态机 */
enum stop_fire_state_e {
    STOP_FIRE_DISABLE = 0,
    STOP_FIRE_ACTIVE,
    STOP_FIRE_PROCESS,
};

/* 拨弹控制模式 */
enum trigger_ctrl_mode_e {
    TRIGGER_CTRL_SPEED = 0,
    TRIGGER_CTRL_ANGLE,
};

/* ==================================================================
 * Shooter — 发射机构控制器类
 * ================================================================== */
class Shooter {
public:
    /* --- 公开状态 --- */
    float shoot_speed_L_now = 0;
    float shoot_speed_R_now = 0;
    float trigger_speed_now = 0;
    float trigger_current_now = 0;

    float trigger_speed_base = 0;
    float trigger_speed_set  = 0;
    shoot_speed_e shoot_speed_set = SHOOT_SPEED_CLOSE;
    float current[3] = {};

    int32_t trigger_ecd_now    = 0;
    int32_t trigger_target_ecd = 0;
    uint32_t stop_fire_tick    = 0;
    uint8_t last_trigger_mode  = 0;
    trigger_ctrl_mode_e trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
    stop_fire_state_e   stop_fire_state   = STOP_FIRE_DISABLE;

    /* 公开方法 */
    void Init();
    void Tasks();

private:
    /* PID */
    pid_t shoot_L_speed_pid_{};
    pid_t shoot_R_speed_pid_{};
    pid_t trigger_speed_pid_{};
    pid_t trigger_angle_pid_{};

    /* 内部方法 */
    void Updater();
    void Calculator();
    void Controller();
    void StopFireFSM();

    /* 工具 */
    static float GetBaseSpeed(uint8_t trigger_mode);
    static uint8_t IsSpeedMode(uint8_t mode);
};

extern Shooter shooter;

#endif /* __cplusplus */
#endif /* __SHOOT_H__ */