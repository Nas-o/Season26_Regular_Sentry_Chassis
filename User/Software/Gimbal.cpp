/**
* @file Gimbal.cpp
 * @author Nas (1319621819@qq.com)
 * @brief 云台控制器
 * @version 0.1
 * @date 2026-06-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <cmath>
#include "Gimbal.hpp"
#include "Chassis.hpp"
#include "Global_status.hpp"
#include "Auto_control.hpp"
#include "User_math.hpp"
#include "remote_control.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"

robot::gimbal::Gimbal gimbal;

namespace robot::gimbal 
{

/* ============================================================
 *  Gimbal 类实现
 * ============================================================ */

    void Gimbal::init()
    {
        initMotor();
    }

    void Gimbal::initMotor()
    {
        /* 注册 大Yaw 电机 */
        DMMotor_Init(DM_4310, BIGYAWMotor);

        /* Yaw 速度环 PID */
        PID_Set(&yaw.spd_pid,
                0.024f, 0.0f, 0.0f, 0.0f,
                7.0f, 100);

        /* Yaw 位置环 PID */
        PID_Set(&yaw.pos_pid,
                6.0f, 0.0f, 1.0f, 0.0f,
                500.0f, 100);

        /* 设置大Yaw编码器零点 */
        DMMotor_SetZero(BIG_YAW_ZERO, BIGYAWMotor);
    }

    void Gimbal::update()
    {
        /* ---- 状态量更新 ---- */
        float cos_pitch = cosf(degree2rad(imu.pitch));
        float sin_pitch = sinf(degree2rad(imu.pitch));
        yaw.spd_now = cos_pitch * rad2degree(imu.gyro[2])
                    - sin_pitch * rad2degree(imu.gyro[0]);

        yaw.pos_now = imu.yaw_cnt;

        /* ---- 模式切换检测 ---- */
        uint8_t mode_changed    = (last_mode_ != (int)Global.Chassis.mode);
        uint8_t leaving_disable = (last_ctrl_mode_ == LOCK && Global.Control.mode != LOCK);

        if (mode_changed || leaving_disable)
        {
            yaw_offset_ = yaw.pos_now - Global.Gimbal.input.yaw;
        }
        last_ctrl_mode_ = Global.Control.mode;
        last_mode_ = Global.Chassis.mode;

        /* ---- 目标量计算 ---- */
        if (Global.Control.mode == LOCK || Global.Chassis.mode == FLOW_Chassis)
        {
            yaw.pos_set = yaw.pos_now;
        }
        else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0)
        {
            float current_relative = ::chassis.relative_angle;
            float target_yaw = Global.Gimbal.input.yaw;
            yaw.pos_set = yaw.pos_now + (target_yaw - current_relative);
        }
        else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 1)
        {
            yaw.pos_set = yaw.pos_now;
        }
        else
        {
            yaw.pos_set = Global.Gimbal.input.yaw + yaw_offset_;
        }
    }

    void Gimbal::calculator()
    {
        /* FLOW_Chassis模式: 泄力 */
        if (Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode != CAR)
        {
            yaw.spd_set = 0.0f;
            yaw.current = 0.0f;
            return;
        }

        /* Navigation 导航模式 */
        if (Global.Chassis.mode == Navigation)
        {
            if (Auto_data.is_scaning == 0)
            {
                yaw.spd_set = PID_Cal(&yaw.pos_pid, yaw.pos_now, yaw.pos_set);
            }
            else if (::chassis.region_code == 5 && !::chassis.alignment_done)
            {
                yaw.pos_set = yaw.pos_now + yaw.planning_spd;
                yaw.spd_set = PID_Cal(&yaw.pos_pid, yaw.pos_now, yaw.pos_set);
            }
            else
            {
                yaw.spd_set = 120.0f;
            }

            yaw.current = PID_Cal(&yaw.spd_pid, yaw.spd_now, yaw.spd_set);
            return;
        }

        /* 默认: 位置 + 速度 双环 */
        yaw.spd_set = PID_Cal(&yaw.pos_pid, yaw.pos_now, yaw.pos_set);
        yaw.current = PID_Cal(&yaw.spd_pid, yaw.spd_now, yaw.spd_set);
    }

    void Gimbal::controller()
    {
        float tor = 0.0f;
        if (Global.Control.mode != LOCK)
        {
            tor = yaw.current;
        }
        DMMotor_Set(BIGYAWMotor, 0, 0, tor, 0, 0);
    }

    void Gimbal::setPitch(float angle)
    {
        if (angle < PITCHI_MIN_ANGLE)
            angle = PITCHI_MIN_ANGLE;
        if (angle > PITCHI_MAX_ANGLE)
            angle = PITCHI_MAX_ANGLE;
        Global.Gimbal.input.pitch = angle;
    }

    void Gimbal::setYaw(float angle)
    {
        Global.Gimbal.input.yaw = angle;
    }

    void Gimbal::tasks()
    {
    #if (USE_GIMBAL != 0)
        update();
        calculator();
        controller();
    #endif
    }

} // namespace robot::gimbal

/* =================== extern "C" 接口 =================== */

extern "C" void Gimbal_Init(void)
{
    gimbal.init();
}

extern "C" void Gimbal_Tasks(void)
{
    gimbal.tasks();
}

extern "C" void Gimbal_SetPitchAngle(float a)
{
    robot::gimbal::Gimbal::setPitch(a);
}

extern "C" void Gimbal_SetYawAngle(float a)
{
    robot::gimbal::Gimbal::setYaw(a);
}
