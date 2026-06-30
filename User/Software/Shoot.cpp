/**
 * @file Shoot.cpp
 * @author Nas (1319621819@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <stdlib.h>
#include "Shoot.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "referee_system.hpp"
#include "HeatControl.hpp"

robot::shoot::Shooter shooter;

namespace robot::shoot {

namespace {

/* ============ 工具函数 ============ */

float absFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float clampFloat(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

} // anonymous namespace

/* ============================================================
 *  Shooter 类实现
 * ============================================================ */

float Shooter::getBaseSpeed(uint8_t trigger_mode)
{
    switch (trigger_mode)
    {
    case HIGH:              return (float)TRIGGER_SPEED_HIGH;
    case MID:               return (float)TRIGGER_SPEED_MID;
    case LOW:               return (float)TRIGGER_SPEED_LOW;
    case DEBUG_TRIGGER:     return (float)TRIGGER_SPEED_DEBUG;
    case ONLY_AIM_HIGH:     return (float)TRIGGER_SPEED_AIM_HIGH;
    case ONLY_AIM_MID:      return (float)TRIGGER_SPEED_AIM_MID;
    case ONLY_AIM_LOW:      return (float)TRIGGER_CLOSE;
    case TRIGGER_CLOSE:
    default:                return 0.0f;
    }
}

uint8_t Shooter::isSpeedMode(uint8_t mode)
{
    switch (mode)
    {
    case HIGH:
    case MID:
    case LOW:
    case DEBUG_TRIGGER:
    case ONLY_AIM_HIGH:
    case ONLY_AIM_MID:
    case ONLY_AIM_LOW:
        return 1;
    default:
        return 0;
    }
}

void Shooter::stopFireFsm()
{
    if (stop_fire_state == STOP_FIRE_ACTIVE)
    {
        uint32_t elapsed = HAL_GetTick() - stop_fire_tick;
        int32_t ecd_err = trigger_target_ecd - trigger_ecd_now;
        if (ecd_err < 0) ecd_err = -ecd_err;

        if (ecd_err <= kSingleDoneEcdTh || elapsed >= kStopFireTimeoutMs)
        {
            stop_fire_state = STOP_FIRE_PROCESS;
        }
    }

    if (stop_fire_state == STOP_FIRE_PROCESS)
    {
        trigger_target_ecd = trigger_ecd_now;
        stop_fire_state = STOP_FIRE_DISABLE;
    }
}

void Shooter::init()
{
    TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);
    PID_Set(&trigger_speed_pid_, 8, 0, 3, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    PID_Set(&trigger_angle_pid_, 0.20f, 0.0f, 0.0f, 0.0f, kTriggerAngleSpeedMax, 0.0f);
    trigger_speed_base = 0.0f;
    trigger_speed_set = 0.0f;

    HeatControl_Init();

    trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
    stop_fire_state = STOP_FIRE_DISABLE;
    trigger_target_ecd = 0;
    stop_fire_tick = 0;
}

void Shooter::updater()
{
    uint8_t mode = Global.Shoot.trigger_mode;
    trigger_speed_now = TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm;
    trigger_current_now = TriggerMotor_get_data(TRIGGER_MOTOR).given_current;
    trigger_ecd_now = (int32_t)TriggerMotor_get_data(TRIGGER_MOTOR).ecd_cnt;

    if (mode == SINGLE && last_trigger_mode != SINGLE)
    {
        trigger_ctrl_mode = TRIGGER_CTRL_ANGLE;
        stop_fire_state = STOP_FIRE_ACTIVE;
        stop_fire_tick = HAL_GetTick();
        trigger_target_ecd = trigger_ecd_now
                           + kTriggerOneBulletEcd
                           + kTriggerExtraEcd;
    }
    else if (isSpeedMode(mode))
    {
        trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
        stop_fire_state = STOP_FIRE_DISABLE;
    }

    stopFireFsm();

    last_trigger_mode = mode;
    trigger_speed_base = getBaseSpeed(Global.Shoot.trigger_mode);

    HeatControl_Update();
}

void Shooter::calculator()
{
    if (trigger_ctrl_mode == TRIGGER_CTRL_ANGLE)
    {
        float speed_ref = PID_Cal(&trigger_angle_pid_,
                                  (float)trigger_ecd_now,
                                  (float)trigger_target_ecd);
        speed_ref = clampFloat(speed_ref,
                                     -kTriggerAngleSpeedMax,
                                     kTriggerAngleSpeedMax);
        trigger_speed_set = speed_ref;
        current[2] = PID_Cal(&trigger_speed_pid_,
                             trigger_speed_now, speed_ref);
        return;
    }

    trigger_speed_set = HeatControl_GetSpeed(trigger_speed_base);

    /* 堵转反转脱困 */
    static int s_trigger_kill_cnt = 0;
    if (Global.Shoot.trigger_mode == TRIGGER_CLOSE &&
        absFloat(trigger_speed_set) < 1.0f)
    {
        s_trigger_kill_cnt = 0;
    }
    else if (absFloat(trigger_current_now) > 10000.0f)
    {
        s_trigger_kill_cnt = 50;
    }

    if (s_trigger_kill_cnt > 1)
    {
        current[2] = PID_Cal(&trigger_speed_pid_, trigger_speed_now, -3000.0f);
        s_trigger_kill_cnt--;
    }
    else
    {
        current[2] = PID_Cal(&trigger_speed_pid_, trigger_speed_now, trigger_speed_set);
    }
}

void Shooter::controller()
{
    SHOOTMotor_set(current[2], TRIGGER_MOTOR);
}

void Shooter::tasks()
{
#if (USE_SHOOT != 0)
    updater();
    calculator();
    controller();
#endif
}

} // namespace robot::shoot

/* =================== extern "C" 接口 =================== */

extern "C" void Shoot_Init(void)
{
    shooter.init();
}

extern "C" void Shoot_Tasks(void)
{
    shooter.tasks();
}
