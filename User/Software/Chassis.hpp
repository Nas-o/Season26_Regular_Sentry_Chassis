/**
 * @file    Chassis.hpp
 * @brief   舵轮底盘控制器 (Swerve Drive Controller)
 * @author  Nas (refactored 2026-05)
 * @note    使用 class 封装底盘状态与控制逻辑
 */
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* ======== C-compatible API (freertos.c 调用) ======== */
void Chassis_Init(void);
void Chassis_Tasks(void);
uint8_t Chassis_GetAligning(void);

#ifdef __cplusplus
}
#endif

/* =================== C++ Only Section =================== */
#ifdef __cplusplus

#include <array>
#include "pid.hpp"
#include "User_math.hpp"
#include "ramp_generator.hpp"
#include "motor.hpp"
#include "CAN_receive_send.hpp"
#include "PowerControl.hpp"

/* 轮位索引 */
enum WheelPos : uint8_t { FL = 0, FR, BL, BR, WHEEL_NUM = 4 };

/* ---- 电机配置宏 ---- */
#define CHASSIS_MAX_CURRENT MAX_CURRENT

/* ---- 驱动电机 (M3508) ---- */
struct DriveUnit {
    pid_t   pid{};
    float   target_velocity   = 0;    // 目标线速度 m/s
    float   current_velocity  = 0;    // 当前线速度 m/s
    float   direction         = 1.0f; // 就近换向乘数 (+1/-1)
    int16_t current           = 0;    // 输出电流
};

/* ---- 舵向电机 (GM6020) ---- */
struct SteerUnit {
    pid_t   pos_pid{};          // 位置环PID
    pid_t   spd_pid{};          // 速度环PID
    float   set         = 0;    // 目标ECD
    float   now         = 0;    // 当前ECD
    float   spd_set     = 0;    // 速度环目标值
    float   target_angle = 0;   // 归一化角度偏差
    float   set_angle   = 0;    // 计算出的转角 (deg)
    float   set_ecd     = 0;    // 计算出的目标ECD
    int16_t current     = 0;    // 输出电流
};

/* ---- 单轮组合 ---- */
struct Wheel {
    DriveUnit drive;
    SteerUnit steer;
};

/* ==================================================================
 * Chassis — 舵轮底盘控制器类
 * ================================================================== */
class Chassis {
public:
    /* --- 公开状态 (供外部模块读写) --- */
    float    relative_angle = 0;    // 底盘-云台相对角
    float    heading_target = 0;    // 底盘航向目标
    uint8_t  is_aligning    = 0;
    uint8_t  alignment_done = 0;
    uint8_t  region_code    = 0;

    /* --- 轮组数据 --- */
    std::array<Wheel, WHEEL_NUM> wheel{};

    /* --- 斜坡规划器 --- */
    RampGenerator ramp_vx{}, ramp_vy{}, ramp_vw{};

    /* --- 速度输入 (由CAN接收或遥控器设置) --- */
    float vx_ = 0, vy_ = 0, vr_ = 0;

    /* 公开方法 */
    void Init();
    void Tasks();

    void SetX(float x) { vx_ = x; }
    void SetY(float y) { vy_ = y; }
    void SetR(float r);

private:
    /* 内部PID */
    pid_t follow_pid_{};
    pid_t heading_pid_{};
    pid_t power_pid_{};

    /* 随机运动状态 */
    struct { int interval = 250; int small_cnt = 0; int valve = 0; } random_state_{};

    /* 内部方法 */
    void InitMotor();
    void Update();
    void RudderCalculation(float x, float y, float w);
    void NearbyTransposition();
    float FollowUpdate();
    float HeadingAlign();
    void Calculator(float vx, float vy, float vw);
    void PowerTotal(float total_max);
    void Controller();
};

extern Chassis chassis;

#endif /* __cplusplus */
#endif /* __CHASSIS_H__ */

