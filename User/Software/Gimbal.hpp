/**
 * @file    Gimbal.hpp
 * @brief   云台控制器 (大Yaw轴 DM4310)
 * @author  Nas (refactored 2026-05)
 */
#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* C-compatible API */
void Gimbal_Init(void);
void Gimbal_Tasks(void);
void Gimbal_SetPitchAngle(float angle);
void Gimbal_SetYawAngle(float angle);

#ifdef __cplusplus
}
#endif

/* =================== C++ Only =================== */
#ifdef __cplusplus

#include "pid.hpp"
#include "ramp_generator.hpp"
#include "motor.hpp"
#include "CAN_receive_send.hpp"

/* 单轴控制数据 */
struct AxisCtrl {
    pid_t spd_pid{};            // 速度环
    pid_t pos_pid{};            // 位置环
    pid_t auto_spd_pid{};       // 自瞄速度环
    pid_t auto_pos_pid{};       // 自瞄位置环
    float spd_now    = 0;       // 当前速度
    float pos_now    = 0;       // 当前位置
    float spd_set    = 0;       // 速度目标
    float pos_set    = 0;       // 位置目标
    float current    = 0;       // 输出电流/力矩
};

/* 云台控制器 */
struct GimbalCtrl {
    AxisCtrl pitch;
    AxisCtrl small_yaw;

    struct {
        pid_t spd_pid{};
        pid_t pos_pid{};
        pid_t auto_spd_pid{};
        pid_t auto_pos_pid{};
        float spd_now       = 0;
        float pos_now       = 0;
        float spd_set       = 0;
        float pos_set       = 0;
        float current       = 0;
        float torque        = 0;
        float planning_spd  = 0;    // 导航扫描速度
    } yaw;

    RampGenerator pitch_ramp{};
    RampGenerator yaw_ramp{};
};

extern GimbalCtrl gimbal;

#endif /* __cplusplus */
#endif /* __GIMBAL_H__ */
