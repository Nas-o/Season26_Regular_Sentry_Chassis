/**
 * @file    Chassis.cpp
 * @brief   舵轮底盘控制器实现
 * @author  Nas (refactored 2026-05)
 * @note    使用 class 封装底盘状态与控制逻辑
 *          4个 M3508 驱动电机 (速度环)
 *          4个 GM6020 舵向电机 (位置环+速度环)
 */
#include <cmath>
#include <cstdlib>

#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"
#include "Auto_control.hpp"
#include "remote_control.hpp"
#include "Shoot.hpp"
#include "PowerControl.hpp"

#include "referee_system.hpp"
#include "supercup.hpp"
#include "Stm32_time.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"
#include "robot_param.hpp"

/* ============ 全局实例 ============ */
Chassis chassis;

/* ============ 静态配置 ============ */
static constexpr uint16_t INIT_ECD[WHEEL_NUM]       = {918, 4426, 3020, 7746};
static constexpr float    DRIVE_SIGN[WHEEL_NUM]     = {1.0f, 1.0f, 1.0f, 1.0f};
static constexpr float    STEER_SIN_SIGN[WHEEL_NUM] = {-1, 1, -1, 1};
static constexpr float    STEER_COS_SIGN[WHEEL_NUM] = {1, 1, -1, -1};

/* 电机ID配置表 */
static constexpr std::array<DJIcan_id, WHEEL_NUM> DRIVE_ID = {
    WHEEL_MOVE_FL, WHEEL_MOVE_FR, WHEEL_MOVE_BL, WHEEL_MOVE_BR
};
static constexpr std::array<DJIcan_id, WHEEL_NUM> STEER_ID = {
    WHEEL_TURN_FL, WHEEL_TURN_FR, WHEEL_TURN_BL, WHEEL_TURN_BR
};

/* 舵轮几何 (运行时转为弧度) */
static float deta[WHEEL_NUM] = {45.0f, 45.0f, 45.0f, 45.0f};

/* ============ 工具函数 ============ */

/**
 * @brief  将角度限制到 [-180, 180] 范围
 */
static float AngleLimit(float angle)
{
    uint32_t mul = (uint32_t)(fabsf(angle) / 180.0f);
    if (angle > 180.0f)
        angle -= ((mul % 2 == 1) ? (mul + 1) : mul) * 180.0f;
    if (angle < -180.0f)
        angle += ((mul % 2 == 1) ? (mul + 1) : mul) * 180.0f;
    return angle;
}

/**
 * @brief  将 x 归一化到 [-modulus/2, modulus/2] 范围
 *         用于ECD编码器差值的周期化处理
 */
static float ModNorm(float x, float modulus)
{
    float tmp = fmodf(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
        tmp += modulus;
    return tmp - modulus / 2.0f;
}

/* ============================================================
 *  Chassis 类实现
 * ============================================================ */

void Chassis::Init()
{
    InitMotor();
}

/**
 * @brief  初始化底盘所有电机、PID参数及功率限制器
 */
void Chassis::InitMotor()
{
    /* 电机注册 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        DJIMotor_Init(DJI_M3508, DRIVE_ID[i]);   /* 驱动电机 */
        DJIMotor_Init(DJI_GM6020, STEER_ID[i]);   /* 舵向电机 */
    }

    /* 驱动电机 PID (速度环) */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        PID_Set(&wheel[i].drive.pid,
                8000.0f, 0, 0.3f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
    }

    /* 舵向电机 PID (位置环 + 速度环) */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        PID_Set(&wheel[i].steer.pos_pid,
                8.0f, 0.0f, 0.0f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
        PID_Set(&wheel[i].steer.spd_pid,
                6.0f, 0.0f, 2.0f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
    }

    /* 底盘跟随 PID */
    PID_Set(&follow_pid_,
            1.0f, 0.0f, 0.0f, 0.0f,
            120, 40);

    /* 航向对齐 PID */
    PID_Set(&heading_pid_,
            0.5f, 0.0f, 0.0f, 0.0f,
            120, 40);

    /* 功率闭环 PID */
    PID_Set(&power_pid_,
            0.04f, 0.0f, 0.01f, 0,
            0.09f, 0.09f);

    /* 功率控制模块初始化 */
    PowerControl_Init();

    /* 默认模式 */
    Global.Chassis.mode = FLOW_Chassis;

    /* 舵向初始ECD */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        wheel[i].steer.now = 0;
        wheel[i].steer.set = INIT_ECD[i];
    }

    /* deta角度转弧度 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        deta[i] = deta[i] * PI / 180.0f;
    }

    /* 随机运动参数 */
    random_state_.interval = 250;
    srand(HAL_GetTick());
}

/* -------------------- Update -------------------- */

/**
 * @brief  更新底盘传感器数据
 *         从电机编码器读取舵向角度和驱动轮速
 */
void Chassis::Update()
{
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        /* 舵向: 读取当前编码器值 */
        wheel[i].steer.now =
            DJIMotor_GetData(STEER_ID[i]).ecd;
        /* 驱动: RPM -> rad/s -> 线速度 (m/s) */
        wheel[i].drive.current_velocity =
            DJIMotor_GetData(DRIVE_ID[i]).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    }
}

/* -------------------- Rudder Angle Calculation -------------------- */

/**
 * @brief  计算四个轮子的舵向目标角度
 * @param  x  底盘vx速度分量
 * @param  y  底盘vy速度分量
 * @param  w  底盘旋转角速度
 * @note   公式: angle = atan2(y ± w*cos45, x ± w*sin45)
 *         符号由 STEER_COS_SIGN / STEER_SIN_SIGN 决定各轮位置
 */
void Chassis::RudderCalculation(float x, float y, float w)
{
    w *= WHEEL_TRACK;

    /* 全零速度时保持当前舵角, 不做计算 */
    if (x == 0 && y == 0 && w == 0)
    {
        for (int i = 0; i < WHEEL_NUM; i++)
            wheel[i].steer.set_angle = 0.0f;
    }
    else
    {
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            float vy_comp = y - STEER_COS_SIGN[i] * w * 0.707107f;
            float vx_comp = x + STEER_SIN_SIGN[i] * w * 0.707107f;
            wheel[i].steer.set_angle =
                atan2f(vy_comp, vx_comp) * 180.0f / PI;
        }
    }

    /* 角度 → ECD编码器目标值 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        wheel[i].steer.set_ecd =
            INIT_ECD[i] + wheel[i].steer.set_angle * 8192.0f / 360.0f;
        wheel[i].steer.set = (int32_t)wheel[i].steer.set_ecd;
    }
}

/* -------------------- Nearby Transposition -------------------- */

/**
 * @brief  就近转位: 避免舵向转动超过90°, 通过反转驱动方向代替
 * @note   使用余弦连续调制: direction = cos(偏差角), 在±90°边界平滑过渡
 */
void Chassis::NearbyTransposition()
{
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = wheel[i].steer;
        DriveUnit &dr = wheel[i].drive;

        /* 计算目标ECD与当前ECD的归一化偏差 (±4096范围) */
        st.target_angle = ModNorm(st.set - st.now, 8192.0f);

        /* cos值用于连续调制驱动方向, 在±90°边界平滑过渡 */
        float cos_val = cosf(st.target_angle * (2.0f * PI / 8192.0f));

        /* 偏差在±90°以内: 直接转到目标 */
        if (-2048.0f <= st.target_angle && st.target_angle <= 2048.0f)
        {
            st.set = st.target_angle + st.now;
        }
        /* 偏差超过90°: 转到反向位置, 由cos_val反转驱动方向 */
        else
        {
            st.set = ModNorm(st.target_angle + 4096.0f, 8192.0f) + st.now;
        }

        /* 驱动方向 = 配置符号 × 余弦调制 */
        dr.direction = DRIVE_SIGN[i] * cos_val;
    }
}

/* -------------------- Follow / Heading -------------------- */

/**
 * @brief  底盘跟随云台: 根据相对角差PID输出旋转速度
 * @return 底盘旋转角速度 (rad/s), 相对角≤2°时返回0
 */
float Chassis::FollowUpdate()
{
    relative_angle = AngleLimit(relative_angle);

    /* 死区: 相对角很小时不跟随, 避免抖动 */
    if (fabsf(relative_angle) <= 2.0f)
        return 0.0f;

    return -PID_Cal(&follow_pid_, relative_angle, 0.0f) * RPM_TO_RAD_S;
}

/**
 * @brief  底盘航向对齐: 根据航向误差PID输出旋转速度
 * @return 底盘旋转角速度 (rad/s), 误差<=2度时返回0
 */
float Chassis::HeadingAlign()
{
    float chassis_yaw = imu.yaw_cnt - relative_angle;
    float err = AngleLimit(heading_target - chassis_yaw);

    /* 死区: 航向误差很小时不修正 */
    if (fabsf(err) <= 2.0f)
        return 0.0f;

    return PID_Cal(&heading_pid_, 0.0f, err) * RPM_TO_RAD_S;
}

/* -------------------- Calculate (速度分解 + PID计算) -------------------- */

/**
 * @brief  根据目标速度计算各轮电流
 * @param  vx  底盘x方向速度 (m/s)
 * @param  vy  底盘y方向速度 (m/s)
 * @param  vw  底盘旋转角速度 (rad/s)
 * @note   流程: 就近转位 -> 舵向位置环 -> 驱动速度分解 -> 舵向速度环 -> 驱动速度环
 */
void Chassis::Calculator(float vx, float vy, float vw)
{
    /* 第1步: 就近转位 (避免舵轮转超90度) */
    NearbyTransposition();

    /* 第2步: 舵向位置环 -> 速度目标 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = wheel[i].steer;
        st.spd_set = PID_Cal(&st.pos_pid, st.now, st.set);
    }

    /* 第3步: 驱动速度分解 - 将底盘速度分解到各轮 */
    float w = vw * WHEEL_TRACK;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        float vx_c = vx + STEER_SIN_SIGN[i] * w * sinf(deta[i]);
        float vy_c = vy + STEER_COS_SIGN[i] * w * cosf(deta[i]);
        float v_mag = sqrtf(vx_c * vx_c + vy_c * vy_c);
        wheel[i].drive.target_velocity = v_mag * wheel[i].drive.direction;
    }

    /* 第4步: 舵向速度环 -> 电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = wheel[i].steer;
        st.current = (int16_t)PID_Cal(
            &st.spd_pid,
            DJIMotor_GetData(STEER_ID[i]).speed_rpm,
            st.spd_set);
    }

    /* 第5步: 驱动速度环 -> 电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        DriveUnit &dr = wheel[i].drive;
        dr.current = (int16_t)PID_Cal(
            &dr.pid,
            dr.current_velocity,
            dr.target_velocity);
    }
}

/* -------------------- Power Control -------------------- */

/**
 * @brief  调用 PowerControl 模块对底盘电流执行功率限幅
 * @param  total_max  本周期功率预算总额 (W)
 */
void Chassis::PowerTotal(float total_max)
{
    int16_t w_c[WHEEL_NUM], s_c[WHEEL_NUM];
    float   w_s[WHEEL_NUM], s_s[WHEEL_NUM];

    /* 拷贝当前电流和速度 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        w_c[i] = wheel[i].drive.current;
        s_c[i] = wheel[i].steer.current;
        w_s[i] = DJIMotor_GetData(DRIVE_ID[i]).round_speed * RPM_TO_RAD_S;
        s_s[i] = DJIMotor_GetData(STEER_ID[i]).speed_rpm * RPM_TO_RAD_S;
    }

    /* 调用 PowerControl 模块统一限幅 */
    PowerControl_Apply(w_c, w_s, s_c, s_s, total_max);

    /* 回写限幅后的电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        wheel[i].drive.current = w_c[i];
        wheel[i].steer.current = s_c[i];
    }
}

/* -------------------- Control -------------------- */

/**
 * @brief  底盘电流输出
 *         非锁定模式: 输出驱动+舵向电流
 *         锁定模式: 全部置零
 */
void Chassis::Controller()
{
    if (Global.Control.mode != LOCK)
    {
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            DJIMotor_Set(wheel[i].drive.current, DRIVE_ID[i]);
            DJIMotor_Set(wheel[i].steer.current, STEER_ID[i]);
        }
    }
    else
    {
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            DJIMotor_Set(0, DRIVE_ID[i]);
            DJIMotor_Set(0, STEER_ID[i]);
        }
    }
}

/* -------------------- Set -------------------- */

void Chassis::SetR(float r)
{
    Global.Chassis.input.r = r;
}

/* -------------------- Tasks (主控制循环) -------------------- */

/**
 * @brief  底盘主控制循环 (由RTOS任务周期调用)
 *         流程: 对齐判断 -> 跟随/独立模式 -> 舵角计算 -> 速度分解 -> 功率限制 -> 电流输出
 */
void Chassis::Tasks()
{
#if (USE_CHASSIS != 0)

    /* ---- 对齐逻辑 ---- */
    /* 离开区域5: 重置对齐完成标志 */
    if (region_code != 5)
        alignment_done = 0;
    /* 进入区域5 且 相对角很小: 标记对齐完成 */
    if (region_code == 5 && !alignment_done && fabsf(relative_angle) <= 5.0f)
        alignment_done = 1;

    /* ---- 判断是否需要底盘跟随云台 ---- */
    uint8_t use_follow =
        (Global.Chassis.mode == FLOW_Gimbal) ||
        (Global.Chassis.mode == Navigation
         && region_code == 5
         && !alignment_done);

    float final_r  = vr_;
    float final_vx = vx_;
    float final_vy = vy_;

    if (use_follow)
    {
        /*
         * 跟随模式: 底盘跟随云台旋转
         * 同时将世界坐标系速度旋转到底盘坐标系
         */
        final_r = FollowUpdate();
        float theta = relative_angle * (PI / 180.0f);
        float cos_t = cosf(theta);
        float sin_t = sinf(theta);
        final_vx = vx_ * cos_t - vy_ * sin_t;
        final_vy = vx_ * sin_t + vy_ * cos_t;
    }

    /* ---- 控制流水线 ---- */

    /* 1. 计算各轮舵向目标角度 */
    RudderCalculation(final_vx, final_vy, final_r);

    /* 2. 更新传感器数据 */
    Update();

    /* 3. 速度分解 + PID -> 电流 */
    Calculator(final_vx, final_vy, final_r);

    /* 4. 功率限制 (仅裁判系统有功率限制时生效) */
    if (Referee_data.Chassis_Power_Limit > 0)
    {
        float lim = PowerControl_BudgetUpdate();
        if (lim > 0.0f)
            PowerTotal(lim);
    }

    /* 5. 电流输出到电机 */
    Controller();

#endif
}

/* =================== extern "C" 接口 =================== */

extern "C" void Chassis_Init(void)
{
    chassis.Init();
}

extern "C" void Chassis_Tasks(void)
{
    chassis.Tasks();
}

extern "C" uint8_t Chassis_GetAligning(void)
{
    return chassis.is_aligning;
}
