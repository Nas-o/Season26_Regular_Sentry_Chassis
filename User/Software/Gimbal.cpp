/**
 * @file    Gimbal.cpp
 * @brief   云台控制器实现
 * @author  Nas (refactored 2026-05)
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

GimbalCtrl gimbal;

/* -------------------- Init -------------------- */

/**
 * @brief  初始化云台电机及PID参数
 *         DM4310 大Yaw: 位置+速度双环
 */
static void Gimbal_InitMotor()
{
    /* 注册 大Yaw 电机 */
    DMMotor_Init(DM_4310, BIGYAWMotor);

    /* Yaw 速度环 PID */
    PID_Set(&gimbal.yaw.spd_pid,
            0.024f, 0.0f, 0.0f, 0.0f,
            7.0f, 100);

    /* Yaw 位置环 PID */
    PID_Set(&gimbal.yaw.pos_pid,
            6.0f, 0.0f, 1.0f, 0.0f,
            500.0f, 100);

    /* 设置大Yaw编码器零点 */
    DMMotor_SetZero(BIG_YAW_ZERO, BIGYAWMotor);
}

/* -------------------- Update -------------------- */

/**
 * @brief  更新云台状态量与目标量
 *         - 状态量: IMU角速度投影到yaw轴 + 累计角度
 *         - 目标量: 根据当前模式决定yaw目标
 */
static void Gimbal_Update()
{
    static int last_mode = FLOW_Chassis;
    static float yaw_offset = 0.0f;
    static int last_ctrl_mode = 0;

    /* ---- 状态量更新 ---- */

    /*
     * Yaw角速度: 将IMU的陀螺仪数据投影到yaw轴
     * 公式: yaw_rate = cos(pitch)*gyro_z - sin(pitch)*gyro_x
     * 当pitch倾斜时, gyro_z不再纯粹是yaw转速, 需要补偿
     */
    float cos_pitch = cosf(degree2rad(imu.pitch));
    float sin_pitch = sinf(degree2rad(imu.pitch));
    gimbal.yaw.spd_now = cos_pitch * rad2degree(imu.gyro[2])
                       - sin_pitch * rad2degree(imu.gyro[0]);

    /* Yaw累计角度 (来自IMU融合算法) */
    gimbal.yaw.pos_now = imu.yaw_cnt;

    /* ---- 模式切换检测 ---- */
    uint8_t mode_changed    = (last_mode != (int)Global.Chassis.mode);
    uint8_t leaving_disable = (last_ctrl_mode == LOCK && Global.Control.mode != LOCK);

    /* 切换模式时重新同步yaw偏移量, 避免目标角跳变 */
    if (mode_changed || leaving_disable)
    {
        yaw_offset = gimbal.yaw.pos_now - Global.Gimbal.input.yaw;
    }
    last_ctrl_mode = Global.Control.mode;
    last_mode = Global.Chassis.mode;

    /* ---- 目标量计算 ---- */

    /* 锁定模式 或 底盘独立模式: 目标 = 当前位置 (维持不动) */
    if (Global.Control.mode == LOCK || Global.Chassis.mode == FLOW_Chassis)
    {
        gimbal.yaw.pos_set = gimbal.yaw.pos_now;
    }
    /* 导航模式 (非扫描): 根据底盘-云台相对角差值跟踪目标 */
    else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 0)
    {
        float current_relative = chassis.relative_angle;
        float target_yaw = Global.Gimbal.input.yaw;
        gimbal.yaw.pos_set = gimbal.yaw.pos_now + (target_yaw - current_relative);
    }
    /* 导航模式 (扫描中): 保持当前朝向, 由扫描逻辑接管 */
    else if (Global.Chassis.mode == Navigation && Auto_data.is_scaning == 1)
    {
        gimbal.yaw.pos_set = gimbal.yaw.pos_now;
    }
    /* 默认: 遥控器/自瞄输入 + 偏移补偿 */
    else
    {
        gimbal.yaw.pos_set = Global.Gimbal.input.yaw + yaw_offset;
    }
}

/* -------------------- Calculate -------------------- */

/**
 * @brief  云台PID计算 — 根据模式选择不同的控制策略
 *         - FLOW_Chassis: 泄力 (不输出力矩)
 *         - Navigation:   按扫描/对齐/跟踪分支控制
 *         - 默认:         位置+速度双环
 */
static void Gimbal_Calculator()
{
    /* ---- FLOW_Chassis模式: 云台泄力, 不输出力矩 ---- */
    if (Global.Chassis.mode == FLOW_Chassis && Global.Auto.mode != CAR)
    {
        gimbal.yaw.spd_set = 0.0f;
        gimbal.yaw.current = 0.0f;
        return;
    }

    /* ---- Navigation 导航模式 ---- */
    if (Global.Chassis.mode == Navigation)
    {
        /* 非扫描状态: 位置环跟踪目标角度 */
        if (Auto_data.is_scaning == 0)
        {
            gimbal.yaw.spd_set = PID_Cal(
                &gimbal.yaw.pos_pid,
                gimbal.yaw.pos_now,
                gimbal.yaw.pos_set);
        }
        /* 区域5对齐中: 以规划速度递增目标, 位置环跟踪 */
        else if (chassis.region_code == 5 && !chassis.alignment_done)
        {
            gimbal.yaw.pos_set = gimbal.yaw.pos_now + gimbal.yaw.planning_spd;
            gimbal.yaw.spd_set = PID_Cal(
                &gimbal.yaw.pos_pid,
                gimbal.yaw.pos_now,
                gimbal.yaw.pos_set);
        }
        /* 默认扫描: 固定角速度旋转 */
        else
        {
            gimbal.yaw.spd_set = 120.0f;
        }

        /* 速度环 → 力矩输出 */
        gimbal.yaw.current = PID_Cal(
            &gimbal.yaw.spd_pid,
            gimbal.yaw.spd_now,
            gimbal.yaw.spd_set);
        return;
    }

    /* ---- 默认模式: 位置 + 速度 双环控制 ---- */

    /* 位置环 → 速度目标 */
    gimbal.yaw.spd_set = PID_Cal(
        &gimbal.yaw.pos_pid,
        gimbal.yaw.pos_now,
        gimbal.yaw.pos_set);

    /* 速度环 → 力矩输出 */
    gimbal.yaw.current = PID_Cal(
        &gimbal.yaw.spd_pid,
        gimbal.yaw.spd_now,
        gimbal.yaw.spd_set);
}

/* -------------------- Control -------------------- */

/**
 * @brief  云台力矩输出
 *         锁定模式下输出0力矩, 否则输出Calculator计算结果
 */
static void Gimbal_Controller()
{
    float tor = 0.0f;
    if (Global.Control.mode != LOCK)
    {
        tor = gimbal.yaw.current;
    }
    DMMotor_Set(BIGYAWMotor, 0, 0, tor, 0, 0);
}

/* -------------------- Set -------------------- */

/**
 * @brief  设置Pitch目标角度 (带上下限限幅)
 * @param  angle  目标角度 (degree)
 */
static void Gimbal_SetPitch(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief  设置Yaw目标角度
 * @param  angle  目标角度 (degree)
 */
static void Gimbal_SetYaw(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

/* =================== extern "C" 接口 =================== */

extern "C" void Gimbal_Init(void)
{
    Gimbal_InitMotor();
}

extern "C" void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
    Gimbal_Update();
    Gimbal_Calculator();
    Gimbal_Controller();
#endif
}

extern "C" void Gimbal_SetPitchAngle(float a)
{
    Gimbal_SetPitch(a);
}

extern "C" void Gimbal_SetYawAngle(float a)
{
    Gimbal_SetYaw(a);
}
