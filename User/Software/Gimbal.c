/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-05 19:41:06
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Gimbal.c
 */
/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       gimbal.c/h
  * @brief      云台控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建云台代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#include "Gimbal.h"
#include "Chassis.h"
#include "Global_status.h"
#include "Auto_control.h"

#include "User_math.h"
#include "remote_control.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "motor.h"
Gimbal_t Gimbal;



/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    DMMotor_Init(DM_4310, BIGYAWMotor);

    PID_Set(&Gimbal.big_yaw.big_yaw_speed_pid, 0.021f, 0.0f, 0.0f, 0.0f, 7.0, 100);
    PID_Set(&Gimbal.big_yaw.big_yaw_location_pid, 6.0f, 0.0f, 1.0f, 0.0f, 500.0, 100);

    DMMotor_SetZero(BIG_YAW_ZERO, BIGYAWMotor);
}



/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Gimbal_Updater()
{
    static int last_chassis_mode = FLOW_Chassis;
    static float yaw_offset = 0.0f;
    static int last_Global_Control_mode = 0;

    /*------状态量更新------*/
    Gimbal.big_yaw.big_yaw_speed_now = (cos(degree2rad(imu.pitch)) * rad2degree(imu.gyro[2]) - sin(degree2rad(imu.pitch)) * rad2degree(imu.gyro[0]));
    Gimbal.big_yaw.big_yaw_location_now = imu.yaw_cnt;

    /*------模式切换处理------*/
    uint8_t mode_changed    = (last_chassis_mode != (int)Global.Chassis.mode);
    uint8_t leaving_disable = (last_Global_Control_mode == LOCK && Global.Control.mode != LOCK);
    if (mode_changed || leaving_disable)
    {
        yaw_offset = Gimbal.big_yaw.big_yaw_location_now - Global.Gimbal.input.yaw;
    }
    last_Global_Control_mode = Global.Control.mode;
    last_chassis_mode = Global.Chassis.mode;

    /*------目标量更新------*/
    if (Global.Control.mode == LOCK || Global.Chassis.mode == FLOW_Chassis)
    {
        // 泄力 / FLOW_Chassis：目标跟随当前位置
        Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now;
    }
    else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0)
    {
        // 自瞄锁定：上板发的是编码器坐标系目标，转成IMU世界角
        float current_encoder = Chassis.relative_angle;
        float target_encoder  = Global.Gimbal.input.yaw;
        float delta_encoder   = target_encoder - current_encoder;
        Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now + delta_encoder;
    }
    else if (Global.Chassis.mode == Navigation)
    {
        // Navigation + 扫描：目标跟随当前位置
        Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now;
    }
    else
    {
        // 其他模式（FLOW_Gimbal, SPIN等）：位置控制
        Gimbal.big_yaw.big_yaw_location_set = Global.Gimbal.input.yaw + yaw_offset;
    }
}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算.
 * 
 * @param          none
 * @retval         none
 */
void Gimbal_Calculator()
{
    // FLOW_Chassis: 大yaw泄力
    if (Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode != CAR)
    {
        Gimbal.big_yaw.big_yaw_speed_set = 0.0f;
        Gimbal.big_yaw.current = 0.0f;
        return;
    }

    // Navigation: 大yaw跟随小yaw / 扫描 / 颠簸路段锁定方向
    if (Global.Chassis.mode == Navigation)
    {
        if (Auto_data.is_scaning == 0)
        {
            // 导航锁定目标：大yaw位置+速度双环跟随小yaw方向
            Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        }
        else if (Chassis.region_code == 5 && !Chassis.alignment_done)
        {
            // 颠簸路段未对齐：大yaw位置环锁定方向
            Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.big_yaw_location_now + Gimbal.big_yaw.planning_speed;
            Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        }
        else if (Chassis.region_code != 5 && Gimbal.big_yaw.planning_speed != 0)
        {
            Gimbal.big_yaw.big_yaw_location_set = Gimbal.big_yaw.planning_speed;
            Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        }
        else
        {
            // 正常导航 或 对齐已完成：速度环扫描
            Gimbal.big_yaw.big_yaw_speed_set = 160.0f;
        }
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
        return;
    }

    // 其他模式：位置+速度双环PID
    Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
    Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none 
 * @retval         none
 */
void Gimbal_Controller()
{
    if(Global.Control.mode != LOCK)
    {
        DMMotor_Set(BIGYAWMotor,
                 0,
                 0,
                 Gimbal.big_yaw.current,
                 0,
                 0);
    }
    else
    {
        DMMotor_Set(BIGYAWMotor,
                 0,
                 0,
                 0,
                 0,
                 0);
    }
       
}


/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
	// 云台数据更新
	Gimbal_Updater();
    // 云台控制解算
    Gimbal_Calculator();
	// 云台电机控制
	Gimbal_Controller();
#endif
}


/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**          
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台eYAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

void Pack_RelativeAngle(uint8_t data[8])
{
    static uint8_t seq = 0;

    Chassis.relative_angle = DMMotor_GetData(BIGYAWMotor).motor_data.para.angle_cnt - BIG_YAW_ZERO;
    float_to_bytes(Chassis.relative_angle, &data[0]);
    data[4] = seq++;
    data[5] = 0xA5u;
    data[6] = 0x5Au;
    data[7] = 0;
}