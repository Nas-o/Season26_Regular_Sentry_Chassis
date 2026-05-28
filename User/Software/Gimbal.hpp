/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-12-25 19:37:17
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-24 04:19:25
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Gimbal.h
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

#ifndef __GIMBAL_DIRECT_H__
#define __GIMBAL_DIRECT_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "CAN_receive_send.hpp"
#include "pid.hpp"
#include "ramp_generator.hpp"

#include "motor.hpp"


/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT MAX_CURRENT


/*内部数据类型*/
typedef struct
{

  struct 
  {
    pid_t pitch_speed_pid;
    pid_t pitch_location_pid;
    pid_t pitch_auto_speed_pid;
    pid_t pitch_auto_location_pid;
    float pitch_speed_now;
    float pitch_location_now;
    float pitch_speed_set;
    float pitch_location_set;
    float current;
    int turnover_pitch;
  }pitch;

  struct
  {
    pid_t small_yaw_speed_pid;
    pid_t small_yaw_location_pid;
    pid_t small_yaw_auto_speed_pid;
    pid_t small_yaw_auto_location_pid;
    float small_yaw_speed_now;
    float small_yaw_location_now;
    float small_yaw_speed_set;
    float small_yaw_location_set;
    float current;
    int turnover_yaw;
  }small_yaw;

  struct 
  {
    pid_t big_yaw_speed_pid;
    pid_t big_yaw_location_pid;
    pid_t big_yaw_auto_speed_pid;
    pid_t big_yaw_auto_location_pid;
    float big_yaw_speed_now;
    float big_yaw_location_now;
    float big_yaw_speed_set;
    float big_yaw_location_set;
    float current;
    float tor;
    float planning_speed;
  }big_yaw;  
    // 斜坡
    RampGenerator pitch_ramp;
    RampGenerator yaw_ramp;

}Gimbal_t;

void Gimbal_Init();
void Gimbal_Tasks();
void Gimbal_SetPitchAngle(float angle);
void Gimbal_SetYawAngle(float angle);
void Pack_RelativeAngle(uint8_t data[8]);
extern Gimbal_t Gimbal;

#ifdef __cplusplus
}
#endif
#endif