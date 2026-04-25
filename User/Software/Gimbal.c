/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-04-26 00:06:29
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

#include "User_math.h"
#include "remote_control.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "motor.h"
#include "Auto_control.h"
Gimbal_t Gimbal;



/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化
    DMMotor_Init(DM_4310, BIGYAWMotor);


    /*PID速度环初始化*/ 
    // 遥控
    PID_Set(&Gimbal.big_yaw.big_yaw_speed_pid, 0.021f, 0.0f, 0.0f, 0.0f, 10.0, 1000);
    //自瞄
    PID_Set(&Gimbal.big_yaw.big_yaw_auto_speed_pid, 0.021f, 0.0f, 0.0f, 0.0f, 10, 1000);
    /*PID位置环初始化*/
    // 遥控
    PID_Set(&Gimbal.big_yaw.big_yaw_location_pid, 12.0f, 0.0f, 1.0f, 0.0f, 500.0, 100);
    //自瞄
    PID_Set(&Gimbal.big_yaw.big_yaw_auto_location_pid, 12.0f, 0.0f, 1.0f, 0.0f, 500, 100);
    // 云台零点初始化
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
    static float last_Global_Control_mode = 0.0f;
    static uint8_t last_is_scaning = 1;

    /*------状态量更新------*/
    //速度
    Gimbal.big_yaw.big_yaw_speed_now = (cos(degree2rad(imu.pitch)) * rad2degree(imu.gyro[2]) - sin(degree2rad(imu.pitch)) * rad2degree(imu.gyro[0]));
    //位置
    Gimbal.big_yaw.big_yaw_location_now = imu.yaw_cnt; 

    /*------模式切换处理------*/
    // 通用规则：任何底盘模式切换 或 泄力退出 → 保持大yaw当前位置，防止回零点
    // 例外：进入导航模式、进入自瞄模式(大yaw跟随小yaw)
    uint8_t mode_changed    = (last_chassis_mode != (int)Global.Chassis.mode);
    uint8_t leaving_disable = (last_Global_Control_mode == 2 && Global.Control.mode != 2);
    if (mode_changed || leaving_disable)
    {
        // 用偏移量锁定当前位置，避免CAN覆盖导致回零
        // location_set = CAN_target + offset = CAN_target + (current - CAN_target) = current
        yaw_offset = Gimbal.big_yaw.big_yaw_location_now - Global.Gimbal.input.yaw;
    }
    last_Global_Control_mode = Global.Control.mode;

    // Navigation 内部 is_scaning 切换过渡：做一次位置同步，防止速度/位置控制硬切
    if (Global.Chassis.mode == Navigation && last_is_scaning != Auto_data.is_scaning)
    {
        Global.Gimbal.input.yaw = Gimbal.big_yaw.big_yaw_location_now;
        yaw_offset = 0.0f;
    }
    last_is_scaning = Auto_data.is_scaning;
    last_chassis_mode = Global.Chassis.mode;

    /*------目标量更新------*/
    if (Global.Control.mode == 2 || Global.Chassis.mode == FLOW_Chassis)
    {
        // 泄力 或 FLOW_Chassis：目标跟随当前位置
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
        // Navigation 扫描：位置跟随当前
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
    // FLOW_Chassis: 大yaw泄力，清零电流并跳过PID（防止积分饱和）
    if (Global.Chassis.mode == FLOW_Chassis)
    {
        Gimbal.big_yaw.big_yaw_speed_set = 0.0f;
        Gimbal.big_yaw.current = 0.0f;
        return;
    }

    uint8_t nav_auto_lock = (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0);
    if (nav_auto_lock)
    {
        // 导航 + 自瞄锁定：大yaw位置环追目标
        Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        // 非自瞄（含导航扫描模式 is_scaning==1）
        switch(Global.Chassis.mode)
        {
            case Navigation :
            /* case FLOW_Chassis : */ 
                Gimbal.big_yaw.big_yaw_speed_set = 80.0f;
                if(Chassis.is_aligning == 5)
                {
                    Gimbal.big_yaw.big_yaw_speed_set = Gimbal.big_yaw.planning_speed;
                }
            break;
            default :
                Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        }
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set); 
        if (Global.Auto.input.Auto_control_online > 0)
            Global.Auto.input.Auto_control_online--;
    }
    else
    { 
        // 自瞄
        Gimbal.big_yaw.big_yaw_speed_set = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_location_pid, Gimbal.big_yaw.big_yaw_location_now, Gimbal.big_yaw.big_yaw_location_set);
        Gimbal.big_yaw.current = PID_Cal(&Gimbal.big_yaw.big_yaw_auto_speed_pid, Gimbal.big_yaw.big_yaw_speed_now, Gimbal.big_yaw.big_yaw_speed_set);
        Global.Auto.input.Auto_control_online--;
    }
}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none 
 * @retval         none
 */
void Gimbal_Controller()
{
    if(Global.Control.mode != 1)
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