/**
 * @file    Shoot.cpp
 * @brief   发射机构控制器实现
 * @author  Nas (refactored 2026-05)
 */
#include <stdlib.h>
#include "Shoot.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "referee_system.hpp"
#include "HeatControl.hpp"

Shooter shooter;

/* ============ 工具函数 ============ */

static float Shoot_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Shoot_ClampFloat(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/* ============================================================
 *  Shooter 类实现
 * ============================================================ */

float Shooter::GetBaseSpeed(uint8_t trigger_mode)
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

uint8_t Shooter::IsSpeedMode(uint8_t mode)
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

void Shooter::StopFireFSM()
{
    if (stop_fire_state == STOP_FIRE_ACTIVE)
    {
        uint32_t elapsed = HAL_GetTick() - stop_fire_tick;
        int32_t ecd_err = trigger_target_ecd - trigger_ecd_now;
        if (ecd_err < 0) ecd_err = -ecd_err;

        if (ecd_err <= SINGLE_DONE_ECD_TH || elapsed >= STOP_FIRE_TIMEOUT_MS)
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

void Shooter::Init()
{
    TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);
    PID_Set(&trigger_speed_pid_, 8, 0, 3, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    PID_Set(&trigger_angle_pid_, 0.20f, 0.0f, 0.0f, 0.0f, TRIGGER_ANGLE_SPEED_MAX, 0.0f);
    trigger_speed_base = 0.0f;
    trigger_speed_set = 0.0f;

    HeatControl_Init();

    trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
    stop_fire_state = STOP_FIRE_DISABLE;
    trigger_target_ecd = 0;
    stop_fire_tick = 0;
}

void Shooter::Updater()
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
                           + TRIGGER_ONE_BULLET_ECD
                           + TRIGGER_EXTRA_ECD;
    }
    else if (IsSpeedMode(mode))
    {
        trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
        stop_fire_state = STOP_FIRE_DISABLE;
    }

    StopFireFSM();

    last_trigger_mode = mode;
    trigger_speed_base = GetBaseSpeed(Global.Shoot.trigger_mode);

    HeatControl_Update();
}

void Shooter::Calculator()
{
    if (trigger_ctrl_mode == TRIGGER_CTRL_ANGLE)
    {
        float speed_ref = PID_Cal(&trigger_angle_pid_,
                                  (float)trigger_ecd_now,
                                  (float)trigger_target_ecd);
        speed_ref = Shoot_ClampFloat(speed_ref,
                                     -TRIGGER_ANGLE_SPEED_MAX,
                                     TRIGGER_ANGLE_SPEED_MAX);
        trigger_speed_set = speed_ref;
        current[2] = PID_Cal(&trigger_speed_pid_,
                             trigger_speed_now, speed_ref);
        return;
    }

    trigger_speed_set = HeatControl_GetSpeed(trigger_speed_base);

    /* 堵转反转脱困 */
    static int trigger_kill_cnt = 0;
    if (Global.Shoot.trigger_mode == TRIGGER_CLOSE &&
        Shoot_AbsFloat(trigger_speed_set) < 1.0f)
    {
        trigger_kill_cnt = 0;
    }
    else if (Shoot_AbsFloat(trigger_current_now) > 10000.0f)
    {
        trigger_kill_cnt = 50;
    }

    if (trigger_kill_cnt > 1)
    {
        current[2] = PID_Cal(&trigger_speed_pid_, trigger_speed_now, -3000.0f);
        trigger_kill_cnt--;
    }
    else
    {
        current[2] = PID_Cal(&trigger_speed_pid_, trigger_speed_now, trigger_speed_set);
    }
}

void Shooter::Controller()
{
    SHOOTMotor_set(current[2], TRIGGER_MOTOR);
}

void Shooter::Tasks()
{
#if (USE_SHOOT != 0)
    Updater();
    Calculator();
    Controller();
#endif
}

/* =================== extern "C" 接口 =================== */

extern "C" void Shoot_Init(void)
{
    shooter.Init();
}

extern "C" void Shoot_Tasks(void)
{
    shooter.Tasks();
}
