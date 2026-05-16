/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-01 00:07:25
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Chassis.h
 */

/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       chassis.c/h
  * @brief      底盘控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.10.31       Wang Zihao       1.重新构建底盘代码结构
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "CAN_receive_send.h"

#include "pid.h"
#include "User_math.h"
#include "ramp_generator.h"

#include "motor.h"


/*电机配置*/

#define CHASSISMotor_init(type, id)  DJIMotor_Init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_Set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_GetData(id)

/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT


// 轮子向前时6020编码器的值
#define X_AXIS_ECD_FL 918                    
#define X_AXIS_ECD_FR 4426
#define X_AXIS_ECD_BL 3020
#define X_AXIS_ECD_BR 7746

/* 功率限制模型参数 (需根据实际底盘辨识调整) */
// P = k0*I*w + k1*|w| + k2*(k0*I)^2 + k3

/*   1. 断开超级电容电源（物理断开或软件禁用）
  2. 将所有电机失能（输出电流设为0）
  3. 等待系统稳定（约5-10秒）
  4. 从裁判系统读取底盘实时功率
  5. 取10秒内的平均值 → 这就是 k3 */

// 轮向电机功率模型参数（当前轮电机减速比为 15.8:1，建议实车重新辨识）
#define WHEEL_K0 0.214f
#define WHEEL_K1 0.132f
#define WHEEL_K2 3.47f
#define CHASSIS_TOTAL_K3 7.6f
#define WHEEL_K3 (CHASSIS_TOTAL_K3 * 0.5f)

// 舵向电机 (GM6020)
#define STEER_K0 0.741f
#define STEER_K1 0.005f
#define STEER_K2 12.98f
#define STEER_K3 (CHASSIS_TOTAL_K3 * 0.5f)

/* 功率限制器结构体 */
typedef struct
{
    // 模型参数
    float k0, k1, k2, k3;
    
    // 状态量
    float limit_power_w;     // 当前生效的功率上限 (裁判限制 + 电容补偿)
    float predict_power; // 预测的总功率
    
    // 调试/监控用
    float scaling_factor;    // 电流缩放系数 (0.0 ~ 1.0)
    
} PowerLimiter_t;

/*内部数据类型*/
typedef struct
{
    /**************************以下是轮向电机结构体定义***************************/

    struct
    {
        pid_t chassis_speed_pid_forward_FL;
        float V_FL;
        float opposite_direction_FL;
        float target_velocity_FL, current_velocity_FL;
        int16_t wheel_current_FL;
    } forward_FL;
    struct
    {
        pid_t chassis_speed_pid_forward_FR;       
        float V_FR;
        float opposite_direction_FR;
        float target_velocity_FR, current_velocity_FR;
        int16_t wheel_current_FR;
    } forward_FR;
    struct
    {
        pid_t chassis_speed_pid_forward_BL;
        float V_BL;
        float opposite_direction_BL;
        float target_velocity_BL, current_velocity_BL;
        int16_t wheel_current_BL;
    } forward_BL;
    struct
    {
        pid_t chassis_speed_pid_forward_BR;
        float V_BR;
        float opposite_direction_BR;
        float target_velocity_BR, current_velocity_BR;
        int16_t wheel_current_BR;
    } forward_BR;

    /**************************以下是舵向电机结构体定义***************************/


    struct
    {
        pid_t chassis_location_pid_turn_FL;
        pid_t chassis_speed_pid_turn_FL;
        float set, now;
        float set_turn_FL_speed;
        float target_angle_turn_FL;
        float set_angle_FL;
        float set_ECD_FL;
        int16_t current_steer_FL;
    } turn_FL;
    struct
    {
        pid_t chassis_location_pid_turn_FR;
        pid_t chassis_speed_pid_turn_FR;
        float set, now;
        float set_turn_FR_speed;
        float target_angle_turn_FR;
        float set_angle_FR;
        float set_ECD_FR;
        int16_t current_steer_FR;
    } turn_FR;
    struct
    {
        pid_t chassis_location_pid_turn_BL;
        pid_t chassis_speed_pid_turn_BL;
        float set, now;
        float set_turn_BL_speed;
        float target_angle_turn_BL;
        float set_angle_BL;
        float set_ECD_BL;
        int16_t current_steer_BL;
    } turn_BL;
    struct
    {
        pid_t chassis_location_pid_turn_BR;
        pid_t chassis_speed_pid_turn_BR;
        float set, now;
        float set_turn_BR_speed;
        float target_angle_turn_BR;
        float set_angle_BR;
        float set_ECD_BR;
        int16_t current_steer_BR;
    } turn_BR;
    struct
    {
        float x, y, r, yaw;
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s
    } speed;

    struct
    {
        float x, y, r, big_yaw;
    } speed_RC;

    struct
    {
        float now_x, now_y, now_r;
        float max_x, max_y, max_r; // m/s^2
    } acc;

    struct
    {
        int refresh_interval;
        int smaller_than_2_count;
        int valve;
    }random;

    pid_t chassis_follow_pid;
    pid_t chassis_heading_pid;   // 底盘航向闭环PID（用于颠簸路段自主对齐）
    float chassis_heading_target; // 底盘目标航向角（世界坐标系，由上位机设定）
    uint8_t is_open_cap;
    float relative_angle;
    float chassis_saltation_mode;
    uint8_t is_aligning;
    uint8_t alignment_done;   // 颠簸路段一次性对齐完成标志
    uint8_t region_code;      //指定区域
    PowerLimiter_t limiter_wheel; // 轮向限制器
    PowerLimiter_t limiter_steer; // 舵向限制器
    
}Chassis_t;
extern Chassis_t Chassis;
extern float flag_lock;
extern float R_speed;
/*外部调用*/
void Chassis_Init();
void Chassis_Tasks();
void Chassis_SetX(float x);
void Chassis_SetY(float y);
void Chassis_SetR(float r);
void Chassis_SetAccel(float acc);
void Rudder_Angle_Calculation(float x, float y, float w);
void Nearby_Transposition();
void Chassis_Calculator(float vx,float vy,float vw);
float Random_Spin(float min, float max);
float Generate_Random_Float(float min, float max);
/* void Chassis_PowerLimit_Update(float max_power); */
uint8_t Chassis_CAN_Dispatch(uint16_t id, uint8_t data[8]);

#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif
