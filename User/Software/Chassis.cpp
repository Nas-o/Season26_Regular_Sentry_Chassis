/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-07 08:28:56
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Chassis.c
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
#include "math.h"

#include "Auto_control.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"
#include "remote_control.hpp"
#include "Shoot.hpp"

#include "referee_system.hpp"
#include "supercup.hpp"
#include "Stm32_time.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"

#include "User_math.hpp"
#include "robot_param.hpp"
#include "ramp_generator.hpp"

Chassis_t Chassis ;
static fp32 deta[4] = {45.0f, 45.0f, 45.0f, 45.0f};   
RC_ctrl_t RC_data;
RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
int Last_Hp = 400;
int Hp_Time_Wait = 0;
uint32_t last_gimbal_msg_time;
float X_speed, Y_speed, R_speed;
pid_t chassis_power_pid; // 功率闭环控制pid (legacy, 保留备用)

#define POWER_ALLOC_STEER_RATIO 0.8f
#define POWER_LIMIT_REFEREE_MIN 40.0f
#define POWER_LIMIT_MIN 20.0f

/* ============================================================
 * 慢环功率预算参数 (PowerBudget)
 * 设计原则：快环开环限流保平滑，慢环修正预算保能量
 * ============================================================ */

// 第1层：裁判长期可持续基准
#define POWER_SAFETY_MARGIN 0.93f    // 5%公差 + 模型误差 → 打93折

// 第2层：超电状态管控
#define CAP_MIN_VOL         13.0f    // 超电保护电压(V)，低于此禁止放电
#define CAP_ALLOW_VOL       16.0f    // 允许放电电压阈值
#define CAP_FULL_VOL        24.0f    // 超电满电压
#define CAP_EXTRA_RATIO     0.5f     // 超电额外功率上限 = 裁判限制 * 此比例（按比例浮动）
#define CAP_RELEASE_RATE    0.5f     // 放开速率(W/次)，慢
#define CAP_TIGHTEN_RATE    3.0f     // 收紧速率(W/次)，快
#define CAP_VOLT_DROP_THRESH 0.05f   // 电压下降率阈值(V/次)

// 第3层：Buffer Energy 慢闭环
#define BUFFER_TARGET       45.0f    // 目标缓冲能量(J)
#define BUFFER_DEADBAND     5.0f     // 死区(J)：target±deadband内不动作
#define BUFFER_DANGER       15.0f    // 危险阈值(J)
#define BUFFER_DERATING_MAX 50.0f    // 最大扣减量(W)
#define BUFFER_TIGHTEN_RATE 2.0f     // 收紧速率(W/次)
#define BUFFER_RELEASE_RATE 0.3f     // 释放速率(W/次)

// 第4层：实测功率滤波保护
#define POWER_LPF_ALPHA       0.02f  // 低通滤波系数 (越小越平滑, ~50ms时间常数@1ms)
#define POWER_OVER_THRESHOLD  0.0f   // 超功率死区(W)：超出P_target才触发
#define POWER_DERATING_MAX    25.0f  // 最大扣减量(W)
#define POWER_TIGHTEN_RATE    1.5f   // 收紧速率(W/次)
#define POWER_RELEASE_RATE    0.2f   // 释放速率(W/次)

// 慢环更新周期
#define BUDGET_UPDATE_PERIOD_MS 20   // 每20ms更新一次慢环

/* 慢环状态变量 */
static float s_cap_extra_power = 0.0f;      // 超电额外功率（状态量）
static float s_buffer_derating = 0.0f;      // 缓冲能量扣减量
static float s_power_derating = 0.0f;       // 实测功率扣减量
static float s_power_measured_filt = 0.0f;  // 实测功率低通滤波值
static float s_cap_vol_last = 0.0f;         // 上次超电电压（用于算下降率）
static uint32_t s_budget_last_tick = 0;     // 上次更新时间戳

float power_limt(float FL_current, float FR_current, float BL_current, float BR_current,
                 float FL_speed, float FR_speed, float BL_speed, float BR_speed, float max_p)
{
    float current[4] = {FL_current, FR_current, BL_current, BR_current};
    float speed[4] = {FL_speed, FR_speed, BL_speed, BR_speed};
    float now_p = 0.0f;

    float a00 = 0.48872161;
    float a01 = -2.93589057e-04;
    float a10 = 5.3241338928e-05;
    float a02 = 2.70936086e-07;
    float a11 = 2.03985936e-06;
    float a20 = 2.17417767e-07;
    /*最大功率设置*/
    Supercap_SetPower(max_p - 5.0f); //
    cap.cache_energy = Referee_data.Buffer_Energy;
    if ((cap.remain_vol <= 12) || (Global.Cap.mode == Not_FULL))
    {
        max_p -= 5.0f; // 5w余量
        if (cap.remain_vol <= 10)
            max_p -= 5.0f;
        if (cap.remain_vol <= 8)
            max_p -= 5.0f;
    }
    else if (cap.remain_vol > 12)
    {
        max_p += cap.remain_vol * 8; // 12
    }
    /*估算当前功率*/
    for (int i = 0; i < 4; i++)
    {
        now_p += fabs(a00 + a01 * speed[1] + a10 * current[i] +
                      a02 * speed[i] * speed[i] +
                      a11 * speed[i] * current[i] +
                      a20 * current[i] * current[i]);
    }
    float percentage = max_p / now_p;
    if (cap.Chassis_power >= (max_p / 3.0f)) // 底盘当前功率过小不使用闭环
        percentage += PID_Cal(&chassis_power_pid, cap.Chassis_power, max_p);
    // if (Global.Chssis.input.y == 0 && Global.Chssis.input.x == 0)//急刹车
    //     percentage = 1;
    // if ((/*(-Global.Chssis.input.x > 0.5 && Vx_now < -25) || (-Global.Chssis.input.x < -0.5 && Vx_now > 25) ||*/
    //     (Global.Chassis.input.y > 0.5 && Vy_now < -25) || (Global.Chassis.input.y < -0.5 && Vy_now > 25)&&Global.Chassis.mode == FLOW))
    //     percentage = 1;

    if (percentage < 0)
        return 0.0f;
    if (percentage > 1.0f) // 防止输出过大
        return 1.0f;

    return percentage;
}

static float PowerLimit_GetRaw2AmpRatio(uint8_t is_steer)
{
    return is_steer ? (3.0f / 16384.0f) : (20.0f / 16384.0f);
}

static float PowerLimit_GetMotorPower(const PowerLimiter_t *lim, int16_t current_raw, float speed_rad_s, uint8_t is_steer)
{
    float i_real = (float)current_raw * PowerLimit_GetRaw2AmpRatio(is_steer);
    float torque = i_real * lim->k0;

    return torque * speed_rad_s +
           lim->k1 * fabsf(speed_rad_s) +
           lim->k2 * torque * torque +
           lim->k3 / 4.0f;
}
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Chassis_Init()
{
    //底盘电机初始化
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_FL);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_FR);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_BL);
    CHASSISMotor_init(DJI_M3508, WHEEL_MOVE_BR);

    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_FL);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_FR);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_BL);
    CHASSISMotor_init(DJI_GM6020, WHEEL_TURN_BR);
    
    //PID初始化
    /*PID轮向电机速度环初始化*/
    PID_Set(&Chassis.forward_FL.chassis_speed_pid_forward_FL, 8000.0f, 0, 0.3f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.forward_FR.chassis_speed_pid_forward_FR, 8000.0f, 0, 0.3f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.forward_BL.chassis_speed_pid_forward_BL, 8000.0f, 0, 0.3f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.forward_BR.chassis_speed_pid_forward_BR, 8000.0f, 0, 0.3f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    /*PID舵向电机位置环初始化*/
    PID_Set(&Chassis.turn_FL.chassis_location_pid_turn_FL,8.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_FR.chassis_location_pid_turn_FR,8.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_BL.chassis_location_pid_turn_BL,8.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_BR.chassis_location_pid_turn_BR,8.0f, 0.0f, 0.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    /*PID舵向电机速度环初始化*/
    PID_Set(&Chassis.turn_FL.chassis_speed_pid_turn_FL,6.0f, 0.0f, 2.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_FR.chassis_speed_pid_turn_FR,6.0f, 0.0f, 2.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_BL.chassis_speed_pid_turn_BL,6.0f, 0.0f, 2.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);
    PID_Set(&Chassis.turn_BR.chassis_speed_pid_turn_BR,6.0f, 0.0f, 2.0f, 0.0f, CHASSISMOTOR_MAX_CURRENT, 1500);


    /*底盘跟随PID*/
    PID_Set(&Chassis.chassis_follow_pid, 1.0f, 0.0f, 0.0f, 0.0f, 120, 40);
    /*底盘航向闭环PID（颠簸路段用）*/
    PID_Set(&Chassis.chassis_heading_pid, 0.5f, 0.0f, 0.0f, 0.0f, 120, 40);
    /*底盘功率控制pid (legacy保留)*/
    PID_Set(&chassis_power_pid, 0.04f, 0.0f, 0.01f, 0, 0.09f, 0.09f);
    
    /* 慢环功率预算状态初始化 */
    s_cap_extra_power = 0.0f;
    s_buffer_derating = 0.0f;
    s_power_derating = 0.0f;
    s_power_measured_filt = 0.0f;
    s_cap_vol_last = 0.0f;
    s_budget_last_tick = 0;

    //默认地盘跟随模式
    Global.Chassis.mode = FLOW_Chassis;
    /* 舵向电机参数初始化 */
    Chassis.turn_FL.now = 0;
    Chassis.turn_FL.set = X_AXIS_ECD_FL;
    Chassis.turn_FR.now = 0;
    Chassis.turn_FR.set = X_AXIS_ECD_FR;

    Chassis.turn_BL.now = 0;
    Chassis.turn_BL.set = X_AXIS_ECD_BL;

    Chassis.turn_BR.now = 0;
    Chassis.turn_BR.set = X_AXIS_ECD_BR;

    /* 旋转速度分解的角度deta */
    for (int i = 0; i < 4; i++)
    {
        deta[i] = deta[i] * PI / 180.0f; // 角度转弧度
    }

    Chassis.random.refresh_interval = 250;
    srand(HAL_GetTick());

/* 初始化轮向参数 (M3508) */
    Chassis.limiter_wheel.k0 = WHEEL_K0;
    Chassis.limiter_wheel.k1 = WHEEL_K1;
    Chassis.limiter_wheel.k2 = WHEEL_K2;
    Chassis.limiter_wheel.k3 = WHEEL_K3;

    /* 初始化舵向参数 (GM6020) */
    Chassis.limiter_steer.k0 = STEER_K0;
    Chassis.limiter_steer.k1 = STEER_K1;
    Chassis.limiter_steer.k2 = STEER_K2;
    Chassis.limiter_steer.k3 = STEER_K3;
}




/*-------------------- PowerLimit --------------------*/

/**
 * @brief          底盘功率限制核心算法 (Predictive Power Limiting)
 * @author         Nas (1319621819@qq.com) ----基于港科开源的适应版
 * @note           该算法基于电机动力学模型，而非简单的电流钳位。
 * 模型公式: P_total = P_mech + P_loss
 * P_total = (k0*I*w) + (k1*|w| + k2*I^2*R + k3)
 * 其中:

 * * 工作流程:
 * 1. 获取 PID 计算出的原始目标电流 (I_cmd) 和当前转速 (w)。
 * 2. 代入模型计算 "如果执行该电流，总功率会是多少" (P_predict)。
 * 3. 比较 P_predict 与 P_limit (裁判限制+电容补偿)。
 * 4. 如果超标，计算缩放系数 k = P_limit / P_predict。
 * 5. 将所有轮子的目标电流乘以 k，实现平滑降功率。
 *
 * @param[in]      max_power: 当前允许的最大输入功率 (W)
 */
/* ===================================================================================
 * 高级功率控制模块
 * (Dual-Limiter & Predictive Power Control)
 * ===================================================================================
 * 核心思想：
 * 1. 物理建模：不只看输出功率，更看重发热损耗 (I^2*R)。
 * 2. 预测控制：在电流发给电机前，先算算会不会超功率。
 * 3. 优先级仲裁：舵向电机(6020)决定航向，优先级高于轮向电机(3508)。
 * ===================================================================================
 */

/**
 * @brief          【辅助函数】预测一组电机的功率消耗
 * @author         Adapted for ADAM (Reference: SJTU-SG/HKUST)
 * @param[in]      lim:       限制器参数结构体 (包含 k0, k1, k2, k3)
 * @param[in]      currents:  4个电机的电流控制值数组 (Raw Value, e.g., 0~16384)
 * @param[in]      speeds:    4个电机的实际转速数组 (单位: rad/s)
 * @param[in]      is_steer:  是否为舵向电机 (1:是, 0:否) - 用于区分电流单位转换
 * @note           
 * 物理模型公式: P_total = P_mech + P_heat + P_const
 * P_predict = (k0 * T * w) + (k1 * |w|) + (k2 * T^2) + (k3 / 4)
 * - k0*I*w: 机械功率 (转矩*转速)
 * - k2*I^2: 焦耳热损耗 (最主要的超功率来源)
 */
void PowerLimit_Predict(PowerLimiter_t *lim, int16_t *currents, float *speeds, uint8_t is_steer)
{
    float total_p = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        float p_item = PowerLimit_GetMotorPower(lim, currents[i], speeds[i], is_steer);

        if (p_item > 0.0f)
        {
            total_p += p_item;
        }
    }

    lim->predict_power = total_p;
}

/**
 * @brief          【辅助函数】按原版舵轮思路对每个电机分别求解限幅后的电流
 * @author         Nas (1319621819@qq.com)
 * @param[in/out]  lim:          限制器结构体 (读取predict, 写入scaling)
 * @param[in]      limit_power:  分配给该组电机的功率限额 (W)
 * @param[in/out]  currents:     4个电机的电流数组 (原地修改为限制后的值)
 * @param[in]      speeds:       4个电机的实际转速数组 (单位: rad/s)
 * @param[in]      is_steer:     是否为舵向电机
 * @note           
 * 核心算法：对每个电机按分配功率求解一元二次方程，
 * 比统一缩放更接近原版 SGPowerLimitator 的行为。
 */
void PowerLimit_Apply(PowerLimiter_t *lim, float limit_power, int16_t *currents, float *speeds, uint8_t is_steer)
{
    float allocatable_power = limit_power;
    float power_sum_required = 0.0f;
    float current_sum_before = 0.0f;
    float current_sum_after = 0.0f;
    float cmd_power[4] = {0};
    float raw2amp_ratio = PowerLimit_GetRaw2AmpRatio(is_steer);

    lim->limit_power_w = limit_power;

    for (int i = 0; i < 4; i++)
    {
        cmd_power[i] = PowerLimit_GetMotorPower(lim, currents[i], speeds[i], is_steer);
        current_sum_before += fabsf((float)currents[i]);

        if (cmd_power[i] > 0.0f)
        {
            power_sum_required += cmd_power[i];
        }
        else
        {
            allocatable_power -= cmd_power[i];
        }
    }

    if (lim->predict_power <= limit_power) {
        lim->scaling_factor = 1.0f;
        return;
    }

    if (allocatable_power < 0.0f)
    {
        allocatable_power = 0.0f;
    }

    if (power_sum_required <= 0.0f)
    {
        lim->scaling_factor = 1.0f;
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        float limited_current;

        if (cmd_power[i] <= 0.0f)
        {
            current_sum_after += fabsf((float)currents[i]);
            continue;
        }

        float power_weight = cmd_power[i] / power_sum_required;
        float motor_budget = power_weight * allocatable_power;
        float w_real = speeds[i];
        float delta = w_real * w_real -
                      4.0f * lim->k2 * (lim->k1 * fabsf(w_real) + lim->k3 / 4.0f - motor_budget);
        float torque_limit;

        if (delta <= 0.0f)
        {
            torque_limit = -w_real / (2.0f * lim->k2);
        }
        else if (currents[i] >= 0)
        {
            torque_limit = (-w_real + sqrtf(delta)) / (2.0f * lim->k2);
        }
        else
        {
            torque_limit = (-w_real - sqrtf(delta)) / (2.0f * lim->k2);
        }

        limited_current = torque_limit / (lim->k0 * raw2amp_ratio);

        if (currents[i] >= 0 && limited_current > currents[i])
        {
            limited_current = currents[i];
        }
        else if (currents[i] < 0 && limited_current < currents[i])
        {
            limited_current = currents[i];
        }

        if (limited_current > CHASSISMOTOR_MAX_CURRENT)
        {
            limited_current = CHASSISMOTOR_MAX_CURRENT;
        }
        else if (limited_current < -CHASSISMOTOR_MAX_CURRENT)
        {
            limited_current = -CHASSISMOTOR_MAX_CURRENT;
        }

        currents[i] = (int16_t)limited_current;
        current_sum_after += fabsf(limited_current);
    }

    if (current_sum_before > 0.0f)
    {
        lim->scaling_factor = current_sum_after / current_sum_before;
    }
    else
    {
        lim->scaling_factor = 1.0f;
    }
}

/**
 * @brief          【主入口】双限制器功率控制逻辑
 * @author         Nas (1319621819@qq.com)
 * @param[in]      total_max_power: 总功率限制 (裁判系统限制 + 电容额外功率)
 * @note           
 * 调用流程:
 * 1. 准备数据 (指针操作)
 * 2. 预测 : 算算舵向和轮向各想吃多少功率
 * 3. 仲裁 : 舵向优先吃饱，剩下的残羹剩饭给轮向
 * 4. 限制 : 根据分配的额度，计算缩放系数并修改电流
 */
void Chassis_PowerControl_Total(float total_max_power) {
    
    /* ------------------ 1. 数据准备 (Data Preparation) ------------------ */
    // 使用指针数组，方便在一个 for 循环里处理 4 个电机，避免写 4 遍重复代码
    
    // [轮向电机] 电流指针 (指向 Chassis 结构体中的真实变量)
    int16_t *wheel_cur_ptr[] = {
        &Chassis.forward_FL.wheel_current_FL, 
        &Chassis.forward_FR.wheel_current_FR, 
        &Chassis.forward_BL.wheel_current_BL, 
        &Chassis.forward_BR.wheel_current_BR
    };
    // [轮向电机] 实际转速 (rad/s)
    // 使用减速后的出轴角速度，与功率模型中的 k0 / k1 / k2 单位保持一致
    float wheel_spd[] = {
        CHASSISMotor_get_data(WHEEL_MOVE_FL).round_speed * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_MOVE_FR).round_speed * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_MOVE_BL).round_speed * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_MOVE_BR).round_speed * RPM_TO_RAD_S
    };
    
    // [舵向电机] 电流指针
    int16_t *steer_cur_ptr[] = {
        &Chassis.turn_FL.current_steer_FL, 
        &Chassis.turn_FR.current_steer_FR, 
        &Chassis.turn_BL.current_steer_BL, 
        &Chassis.turn_BR.current_steer_BR
    };
    // [舵向电机] 实际转速 (需要将 RPM 转为 rad/s)
    // 1 rpm = 2*pi/60 rad/s ≈ 0.10472 rad/s
    float steer_spd[] = {
        CHASSISMotor_get_data(WHEEL_TURN_FL).speed_rpm * RPM_TO_RAD_S, 
        CHASSISMotor_get_data(WHEEL_TURN_FR).speed_rpm * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_TURN_BL).speed_rpm * RPM_TO_RAD_S,
        CHASSISMotor_get_data(WHEEL_TURN_BR).speed_rpm * RPM_TO_RAD_S
    };
    
    // 将结构体中的 PID 原始计算值提取到临时数组中进行处理
    int16_t w_c[4] = {*wheel_cur_ptr[0], *wheel_cur_ptr[1], *wheel_cur_ptr[2], *wheel_cur_ptr[3]};
    int16_t s_c[4] = {*steer_cur_ptr[0], *steer_cur_ptr[1], *steer_cur_ptr[2], *steer_cur_ptr[3]};

    /* ------------------ 2. 功率预测 (Prediction) ------------------ */
    // 分别预测舵向组和轮向组，如果全速执行，会消耗多少功率
    
    // 预测舵向 (is_steer = 1)
    PowerLimit_Predict(&Chassis.limiter_steer, s_c, steer_spd, 1);
    
    // 预测轮向 (is_steer = 0)
    PowerLimit_Predict(&Chassis.limiter_wheel, w_c, wheel_spd, 0);

    /* ------------------ 3. 优先级分配 (Priority Arbitration) ------------------ */
    float steer_need = Chassis.limiter_steer.predict_power;
    float steer_limit = total_max_power * POWER_ALLOC_STEER_RATIO;
    float wheel_limit = total_max_power - fminf(steer_need, steer_limit);

    if (wheel_limit < 0.0f)
    {
        wheel_limit = 0.0f;
    }

    /* ------------------ 4. 执行限制 (Execution) ------------------ */

    PowerLimit_Apply(&Chassis.limiter_steer, steer_limit, s_c, steer_spd, 1);
    PowerLimit_Apply(&Chassis.limiter_wheel, wheel_limit, w_c, wheel_spd, 0);

    /* ------------------ 5. 写入结果 (Write Back) ------------------ */
    // 将计算好并缩放过的电流值，写回 Chassis 结构体，等待 Controller 发送
    for(int i = 0; i < 4; i++) {
        *wheel_cur_ptr[i] = w_c[i];
        *steer_cur_ptr[i] = s_c[i];
    }
}

/**
 * @brief  慢环功率预算更新 (每20ms调用一次)
 * @note   快环开环限流，慢环修正功率预算
 *         - 电流输出不追实时功率抖动
 *         - 功率上限 final_limit 被 buffer/滤波功率/超电状态 慢慢收紧或放开
 * @return float 本周期的功率预算(W)
 */
static float PowerBudget_Update(void)
{
    /* ========== 时间门控：每 BUDGET_UPDATE_PERIOD_MS 更新一次 ========== */
    uint32_t now_tick = HAL_GetTick();
    static float s_last_budget = 60.0f; // 上次输出的预算，用于非更新周期直接返回
    
    if ((now_tick - s_budget_last_tick) < BUDGET_UPDATE_PERIOD_MS)
    {
        return s_last_budget; // 非更新周期，直接返回上次结果
    }
    s_budget_last_tick = now_tick;

    /* ========== 第1层：裁判限制作为长期基准 ========== */
    float P_ref = (float)Referee_data.Chassis_Power_Limit;
    
    if (P_ref <= 0.0f)
    {
        s_last_budget = 0.0f;
        return 0.0f;
    }
    if (P_ref < POWER_LIMIT_REFEREE_MIN)
    {
        P_ref = POWER_LIMIT_REFEREE_MIN;
    }
    
    // 长期可持续目标 = 裁判限制 * 安全系数
    float P_sustain = P_ref * POWER_SAFETY_MARGIN;
    
    // 通知超电板当前功率限制
    if (P_ref > 5.0f)
    {
        Supercap_SetPower((uint8_t)(P_ref - 5.0f));
    }
    else
    {
        Supercap_SetPower(0);
    }
    // 更新缓冲能量给超电通信（饱和赋值防截断）
    cap.cache_energy = (Referee_data.Buffer_Energy > 255u) ? 255u : (uint8_t)Referee_data.Buffer_Energy;

    /* ========== 第2层：超电额外功率 —— 状态量管控 ========== */
    // 核心原则：放开要慢，收紧要快；超电只负责短时爆发，不负责全局全速
    {
        float cap_vol = cap.remain_vol;
        float cap_vol_drop = s_cap_vol_last - cap_vol; // 正值=电压在下降
        float buffer_energy = (float)Referee_data.Buffer_Energy;
        
        if (cap_vol < CAP_MIN_VOL)
        {
            // 保护：电压过低，禁止额外放电
            s_cap_extra_power = 0.0f;
        }
        else if (cap_vol_drop > CAP_VOLT_DROP_THRESH)
        {
            // 超电净放电（电压在下降）→ 快速收紧
            s_cap_extra_power -= CAP_TIGHTEN_RATE;
        }
        else if (cap_vol > CAP_ALLOW_VOL && buffer_energy > BUFFER_TARGET)
        {
            // 条件好（电压高 + 缓冲充足）→ 慢慢放开
            s_cap_extra_power += CAP_RELEASE_RATE;
        }
        else if (buffer_energy < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            // 缓冲能量偏低，即使超电电压够也不能加
            s_cap_extra_power -= CAP_TIGHTEN_RATE * 0.5f;
        }
        // else: 维持当前值不变
        
        // 钳位：上限跟随裁判限制按比例浮动
        float cap_extra_max_now = P_ref * CAP_EXTRA_RATIO;
        if (s_cap_extra_power < 0.0f) s_cap_extra_power = 0.0f;
        if (s_cap_extra_power > cap_extra_max_now) s_cap_extra_power = cap_extra_max_now;
        
        s_cap_vol_last = cap_vol;
    }

    /* ========== 第3层：Buffer Energy 慢闭环 ========== */
    // Buffer 不是瞬时功率，它是"裁判系统的能量存款"
    // 只允许扣功率或释放之前的扣减，永远不反向加功率
    {
        float buffer_energy = (float)Referee_data.Buffer_Energy;
        
        if (buffer_energy < BUFFER_DANGER)
        {
            // 紧急模式：直接拉满扣减
            s_buffer_derating = BUFFER_DERATING_MAX;
        }
        else if (buffer_energy < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            // 偏低：逐步增大扣减（收紧快）
            s_buffer_derating += BUFFER_TIGHTEN_RATE;
        }
        else if (buffer_energy > BUFFER_TARGET + BUFFER_DEADBAND)
        {
            // 偏高：缓慢释放之前的扣减（放开慢）
            s_buffer_derating -= BUFFER_RELEASE_RATE;
        }
        // else: 在死区内不动作
        
        // 钳位
        if (s_buffer_derating < 0.0f) s_buffer_derating = 0.0f;
        if (s_buffer_derating > BUFFER_DERATING_MAX) s_buffer_derating = BUFFER_DERATING_MAX;
    }

    /* ========== 第4层：实测功率滤波保护 ========== */
    // 有5%公差 → 先低通滤波，再加死区，不直接PID追
    {
        float P_meas_raw = cap.Chassis_power;
        
        // 一阶低通滤波: filt += alpha * (raw - filt)
        if (P_meas_raw > 0.0f) // 只在有有效数据时更新
        {
            s_power_measured_filt += POWER_LPF_ALPHA * (P_meas_raw - s_power_measured_filt);
        }
        
        // 超出目标量
        float over_power = s_power_measured_filt - P_sustain;
        
        if (over_power > POWER_OVER_THRESHOLD)
        {
            // 超功率：增大扣减（快）
            s_power_derating += POWER_TIGHTEN_RATE;
        }
        else if (s_power_measured_filt < P_ref * 0.85f)
        {
            // 功率远低于限制：缓慢释放（慢）
            s_power_derating -= POWER_RELEASE_RATE;
        }
        // else: 在合理范围内不动作
        
        // 钳位
        if (s_power_derating < 0.0f) s_power_derating = 0.0f;
        if (s_power_derating > POWER_DERATING_MAX) s_power_derating = POWER_DERATING_MAX;
    }

    /* ========== 综合计算最终功率预算 ========== */
    float final_limit = P_sustain
                      + s_cap_extra_power
                      - s_buffer_derating
                      - s_power_derating;

    if (final_limit < POWER_LIMIT_MIN)
    {
        final_limit = POWER_LIMIT_MIN;
    }

    s_last_budget = final_limit;
    return final_limit;
}


/*-------------------- AngleLimit --------------------*/

/**
 * @brief          多圈角度限制
 * @param          angle:要化简的角度
 * @retval         angle:化简后的角度
 * @author         Nas(1319621819@qq.com)
 */
float Chassis_AngleLimit(float angle)
{
    uint32_t mul = fabs(angle) / 180.0f;
    if (angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            angle -= (mul + 1) * 180.0f;
        else // 处于180度
            angle -= mul * 180.0f;
    }
    if (angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            angle += (mul + 1) * 180.0f;
        else // 处于-180度
            angle += mul * 180.0f;
    }
    return angle;
}

/*-----------------------obtain_modulus_normalization-----------------------*/
/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 * @author         Nas(1319621819@qq.com)
 */
float Obtain_Modulus_Normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}


static float Chassis_Follow_Update(void)
{
    Chassis.relative_angle = Chassis_AngleLimit(Chassis.relative_angle);

    if (fabsf(Chassis.relative_angle) <= 2.0f)
    {
        return 0.0f;
    }

    return -PID_Cal(&Chassis.chassis_follow_pid, Chassis.relative_angle, 0.0f) * RPM_TO_RAD_S;
}

/**
 * @brief  底盘航向闭环（颠簸路段用）
 * @note   底盘航向 = imu.yaw_cnt - relative_angle
 *         通过PID输出旋转分量，让底盘自行对齐目标方向
 *         大yaw继续扫描不受影响
 * @return float 旋转分量vw (rad/s)
 */
static float Chassis_Heading_Align(void)
{
    // 底盘当前绝对航向 = 云台IMU航向 - 云台相对底盘偏角
    float chassis_yaw_now = imu.yaw_cnt - Chassis.relative_angle;
    // 航向误差（限制在±180°内）
    float heading_err = Chassis_AngleLimit(Chassis.chassis_heading_target - chassis_yaw_now);

    if (fabsf(heading_err) <= 2.0f)
    {
        return 0.0f;
    }

    return PID_Cal(&Chassis.chassis_heading_pid, 0.0f, heading_err) * RPM_TO_RAD_S;
}

/*************************Rudder_Angle_calculation****************************/
/**
 * @brief 舵向角度解算
 * 
 * @param x
 * @param y
 * @param w
 * @author         Nas(1319621819@qq.com)
 */
void Rudder_Angle_Calculation(float x, float y, float w)
{
    // 线速度
    w = w * WHEEL_TRACK;
    int16_t angle_temp[4];

    // 旋转运动
    if (x == 0 && y == 0 && w == 0)
    {
        Chassis.turn_FL.set_angle_FL = 0.0f;
        Chassis.turn_FR.set_angle_FR = 0.0f;
        Chassis.turn_BL.set_angle_BL = 0.0f;
        Chassis.turn_BR.set_angle_BR = 0.0f;
    }
    else
    {
        Chassis.turn_FL.set_angle_FL = atan2((y - w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        Chassis.turn_FR.set_angle_FR = atan2((y - w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
        Chassis.turn_BL.set_angle_BL = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        Chassis.turn_BR.set_angle_BR = atan2((y + w * 0.707107f), (x + w * 0.707107f)) * 180.0f / PI;
    }

    Chassis.turn_FL.set_ECD_FL = X_AXIS_ECD_FL + Chassis.turn_FL.set_angle_FL * 8192.0 / 360.0;
    Chassis.turn_FR.set_ECD_FR = X_AXIS_ECD_FR + Chassis.turn_FR.set_angle_FR * 8192.0 / 360.0;
    Chassis.turn_BL.set_ECD_BL = X_AXIS_ECD_BL + Chassis.turn_BL.set_angle_BL * 8192.0 / 360.0;
    Chassis.turn_BR.set_ECD_BR = X_AXIS_ECD_BR + Chassis.turn_BR.set_angle_BR * 8192.0 / 360.0;

    // 角度赋值
    //--在这里设置的需要的角度对应在编码器上的位置
    Chassis.turn_FL.set = (int32_t)Chassis.turn_FL.set_ECD_FL;
    Chassis.turn_FR.set = (int32_t)Chassis.turn_FR.set_ECD_FR;
    Chassis.turn_BL.set = (int32_t)Chassis.turn_BL.set_ECD_BL;
    Chassis.turn_BR.set = (int32_t)Chassis.turn_BR.set_ECD_BR;
}
/*----------------------------Nearby_Transposition----------------------------*/
/**
 * @brief 就近转位 (余弦连续化版本)
 * @note  用 cos(舵向误差角) 连续调制轮向速度，替代原先在±90°处的硬切换。
 *        cos(0°)=1 全速 → cos(90°)=0 停转 → cos(180°)=-1 反转，自然平滑过渡。
 *        彻底消除旋转+平移时因方向频繁翻转导致的顿挫。
 * @author Nas(1319621819@qq.com)
 */
void Nearby_Transposition()
{
    Chassis.turn_FL.target_angle_turn_FL = Obtain_Modulus_Normalization(Chassis.turn_FL.set - Chassis.turn_FL.now, 8192.0f);
    Chassis.turn_FR.target_angle_turn_FR = Obtain_Modulus_Normalization(Chassis.turn_FR.set - Chassis.turn_FR.now, 8192.0f);
    Chassis.turn_BL.target_angle_turn_BL = Obtain_Modulus_Normalization(Chassis.turn_BL.set - Chassis.turn_BL.now, 8192.0f);
    Chassis.turn_BR.target_angle_turn_BR = Obtain_Modulus_Normalization(Chassis.turn_BR.set - Chassis.turn_BR.now, 8192.0f);

    // 将 ECD 误差转换为弧度，用于余弦权重计算 (8192 ECD = 2*PI)
    float cos_FL = cosf(Chassis.turn_FL.target_angle_turn_FL * (2.0f * PI / 8192.0f));
    float cos_FR = cosf(Chassis.turn_FR.target_angle_turn_FR * (2.0f * PI / 8192.0f));
    float cos_BL = cosf(Chassis.turn_BL.target_angle_turn_BL * (2.0f * PI / 8192.0f));
    float cos_BR = cosf(Chassis.turn_BR.target_angle_turn_BR * (2.0f * PI / 8192.0f));

    // FL: 舵向就近转位 (选择最近侧旋转)
    if (-2048.0f <= Chassis.turn_FL.target_angle_turn_FL && Chassis.turn_FL.target_angle_turn_FL <= 2048.0f)
    {
        Chassis.turn_FL.set = Chassis.turn_FL.target_angle_turn_FL + Chassis.turn_FL.now;
    }
    else
    {
        Chassis.turn_FL.set = Obtain_Modulus_Normalization(Chassis.turn_FL.target_angle_turn_FL + 4096.0f, 8192.0f) + Chassis.turn_FL.now;
    }
    // 轮向: 余弦连续调制 (default_sign = 1.0f)
    Chassis.forward_FL.opposite_direction_FL = 1.0f * cos_FL;

    // FR
    if (-2048.0f <= Chassis.turn_FR.target_angle_turn_FR && Chassis.turn_FR.target_angle_turn_FR <= 2048.0f)
    {
        Chassis.turn_FR.set = Chassis.turn_FR.target_angle_turn_FR + Chassis.turn_FR.now;
    }
    else
    {
        Chassis.turn_FR.set = Obtain_Modulus_Normalization(Chassis.turn_FR.target_angle_turn_FR + 4096.0f, 8192.0f) + Chassis.turn_FR.now;
    }
    // 轮向: 余弦连续调制 (default_sign = -1.0f)
    Chassis.forward_FR.opposite_direction_FR = -1.0f * cos_FR;

    // BL
    if (-2048.0f <= Chassis.turn_BL.target_angle_turn_BL && Chassis.turn_BL.target_angle_turn_BL <= 2048.0f)
    {
        Chassis.turn_BL.set = Chassis.turn_BL.target_angle_turn_BL + Chassis.turn_BL.now;
    }
    else
    {
        Chassis.turn_BL.set = Obtain_Modulus_Normalization(Chassis.turn_BL.target_angle_turn_BL + 4096.0f, 8192.0f) + Chassis.turn_BL.now;
    }
    // 轮向: 余弦连续调制 (default_sign = 1.0f)
    Chassis.forward_BL.opposite_direction_BL = 1.0f * cos_BL;

    // BR
    if (-2048.0f <= Chassis.turn_BR.target_angle_turn_BR && Chassis.turn_BR.target_angle_turn_BR <= 2048.0f)
    {
        Chassis.turn_BR.set = Chassis.turn_BR.target_angle_turn_BR + Chassis.turn_BR.now;
    }
    else
    {
        Chassis.turn_BR.set = Obtain_Modulus_Normalization(Chassis.turn_BR.target_angle_turn_BR + 4096.0f, 8192.0f) + Chassis.turn_BR.now;
    }
    // 轮向: 余弦连续调制 (default_sign = -1.0f)
    Chassis.forward_BR.opposite_direction_BR = -1.0f * cos_BR;
}



/**************************** Random_Spin ****************************/

/**
 * @brief 任意转动
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Random_Spin(float min, float max)
{
    
    static int run_count = 0;//计数器，为0更方便看周期
	run_count++;
	float target_r;
	if (run_count >= Chassis.random.refresh_interval) // 判断周期更新
	{
		if (Chassis.random.smaller_than_2_count < 2) // 检查低于底线值的计数
		{
			Chassis.random.valve = Generate_Random_Float(min, max); // 生成随机值
			if (Chassis.random.valve < 7.0f) // 检查是否低于底线值
			{
				Chassis.random.smaller_than_2_count++;
			}
		}
		else // smaller_than_2_count >= 2，说明已经连续多次低于底线值了
		{
			Chassis.random.valve = 9.0f; // 返回固定值，强制提升速度
			Chassis.random.smaller_than_2_count = 0; // 重置标志位计数
		}
		run_count = 0; // 重置主计数器，开始新周期
	 }
}

/*********************** Generate_Random_Float ***********************/

/**
 * @brief 计算随机数
 * 
 * @param min 
 * @param max 
 * @return float 
 * @author Nas(1319621819@qq.com)
 */
float Generate_Random_Float(float min, float max)
{
	return min + ((float)rand() / (float)RAND_MAX) * (max - min);/*算式后半部分，rand()随机数在0 ~ 最大数RAND_MAX之间，
																																(float)rand() / (float)RAND_MAX) = 0.0f ~ 1.0f 
																																max - min 按照烧饼上场小陀螺速度，差值也就4~5？？
																																最后返回的值就是一个随机的值*/
}



/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */

void Chassis_Updater()
{

    /*---------------状态量更新----------------*/
    //--编码器赋值
    Chassis.turn_FL.now = CHASSISMotor_get_data(WHEEL_TURN_FL).ecd;
    Chassis.turn_FR.now = CHASSISMotor_get_data(WHEEL_TURN_FR).ecd;
    Chassis.turn_BL.now = CHASSISMotor_get_data(WHEEL_TURN_BL).ecd;
    Chassis.turn_BR.now = CHASSISMotor_get_data(WHEEL_TURN_BR).ecd;
    //--出轴线速度 (m/s) = 出轴转速(RPM) * (π/30) * 轮子半径(m)
    Chassis.forward_FL.current_velocity_FL = CHASSISMotor_get_data(WHEEL_MOVE_FL).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    Chassis.forward_FR.current_velocity_FR = CHASSISMotor_get_data(WHEEL_MOVE_FR).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    Chassis.forward_BL.current_velocity_BL = CHASSISMotor_get_data(WHEEL_MOVE_BL).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    Chassis.forward_BR.current_velocity_BR = CHASSISMotor_get_data(WHEEL_MOVE_BR).round_speed * RPM_TO_RAD_S * WHEEL_RADIUS;
    //--转速RPM赋值
    
    /*---------------目标量更新----------------*/
    //车速

}


/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */

void Chassis_Calculator(float vx,float vy,float vw)
{
    Nearby_Transposition();

    // 舵向位置环 PID
    Chassis.turn_FL.set_turn_FL_speed = PID_Cal(&Chassis.turn_FL.chassis_location_pid_turn_FL, Chassis.turn_FL.now, Chassis.turn_FL.set);
    Chassis.turn_FR.set_turn_FR_speed = PID_Cal(&Chassis.turn_FR.chassis_location_pid_turn_FR, Chassis.turn_FR.now, Chassis.turn_FR.set);
    Chassis.turn_BL.set_turn_BL_speed = PID_Cal(&Chassis.turn_BL.chassis_location_pid_turn_BL, Chassis.turn_BL.now, Chassis.turn_BL.set);
    Chassis.turn_BR.set_turn_BR_speed = PID_Cal(&Chassis.turn_BR.chassis_location_pid_turn_BR, Chassis.turn_BR.now, Chassis.turn_BR.set);

    // 计算和速度
    vw = vw * WHEEL_TRACK;
    // 分别计算四个轮子的速度大小
    Chassis.forward_FL.V_FL = sqrt((vx - vw * sinf(deta[1])) * (vx - vw * sinf(deta[1])) + (vy + vw * cosf(deta[1])) * (vy + vw * cosf(deta[1])));
    Chassis.forward_FR.V_FR = -sqrt((vx + vw * sinf(deta[0])) * (vx + vw * sinf(deta[0])) + (vy + vw * cosf(deta[0])) * (vy + vw * cosf(deta[0])));
    Chassis.forward_BL.V_BL = sqrt((vx - vw * sinf(deta[2])) * (vx - vw * sinf(deta[2])) + (vy - vw * cosf(deta[2])) * (vy - vw * cosf(deta[2])));
    Chassis.forward_BR.V_BR = -sqrt((vx + vw * sinf(deta[3])) * (vx + vw * sinf(deta[3])) + (vy - vw * cosf(deta[3])) * (vy - vw * cosf(deta[3])));


    Chassis.turn_FL.current_steer_FL = PID_Cal(&Chassis.turn_FL.chassis_speed_pid_turn_FL, 
                                     CHASSISMotor_get_data(WHEEL_TURN_FL).speed_rpm, 
                                     Chassis.turn_FL.set_turn_FL_speed);
                                     
    Chassis.turn_FR.current_steer_FR = PID_Cal(&Chassis.turn_FR.chassis_speed_pid_turn_FR, 
                                     CHASSISMotor_get_data(WHEEL_TURN_FR).speed_rpm, 
                                     Chassis.turn_FR.set_turn_FR_speed);
                                     
    Chassis.turn_BL.current_steer_BL = PID_Cal(&Chassis.turn_BL.chassis_speed_pid_turn_BL, 
                                     CHASSISMotor_get_data(WHEEL_TURN_BL).speed_rpm, 
                                     Chassis.turn_BL.set_turn_BL_speed);
                                     
    Chassis.turn_BR.current_steer_BR = PID_Cal(&Chassis.turn_BR.chassis_speed_pid_turn_BR, 
                                     CHASSISMotor_get_data(WHEEL_TURN_BR).speed_rpm, 
                                     Chassis.turn_BR.set_turn_BR_speed);


    //--确定目标速度的正负
    Chassis.forward_FL.target_velocity_FL = Chassis.forward_FL.V_FL * Chassis.forward_FL.opposite_direction_FL;
    Chassis.forward_FR.target_velocity_FR = Chassis.forward_FR.V_FR * Chassis.forward_FR.opposite_direction_FR;
    Chassis.forward_BL.target_velocity_BL = Chassis.forward_BL.V_BL * Chassis.forward_BL.opposite_direction_BL;
    Chassis.forward_BR.target_velocity_BR = Chassis.forward_BR.V_BR * Chassis.forward_BR.opposite_direction_BR;

    // 计算马达电流 (反馈和目标均为 m/s)
    Chassis.forward_FL.wheel_current_FL = PID_Cal(&Chassis.forward_FL.chassis_speed_pid_forward_FL, Chassis.forward_FL.current_velocity_FL, Chassis.forward_FL.target_velocity_FL);
    Chassis.forward_FR.wheel_current_FR = PID_Cal(&Chassis.forward_FR.chassis_speed_pid_forward_FR, Chassis.forward_FR.current_velocity_FR, Chassis.forward_FR.target_velocity_FR);
    Chassis.forward_BL.wheel_current_BL = PID_Cal(&Chassis.forward_BL.chassis_speed_pid_forward_BL, Chassis.forward_BL.current_velocity_BL, Chassis.forward_BL.target_velocity_BL);
    Chassis.forward_BR.wheel_current_BR = PID_Cal(&Chassis.forward_BR.chassis_speed_pid_forward_BR, Chassis.forward_BR.current_velocity_BR, Chassis.forward_BR.target_velocity_BR);

}


/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Chassis_Controller()
{
    if (Global.Control.mode != LOCK)
    {
    /*电流设置*/
    CHASSISMotor_set(Chassis.forward_FL.wheel_current_FL, WHEEL_MOVE_FL);
    CHASSISMotor_set(Chassis.forward_FR.wheel_current_FR, WHEEL_MOVE_FR);
    CHASSISMotor_set(Chassis.forward_BL.wheel_current_BL, WHEEL_MOVE_BL);
    CHASSISMotor_set(Chassis.forward_BR.wheel_current_BR, WHEEL_MOVE_BR);

    CHASSISMotor_set(Chassis.turn_FL.current_steer_FL, WHEEL_TURN_FL);
    CHASSISMotor_set(Chassis.turn_FR.current_steer_FR, WHEEL_TURN_FR);
    CHASSISMotor_set(Chassis.turn_BL.current_steer_BL, WHEEL_TURN_BL);
    CHASSISMotor_set(Chassis.turn_BR.current_steer_BR, WHEEL_TURN_BR);
    }
    else
    {
    /*电流设置*/
    CHASSISMotor_set(0, WHEEL_MOVE_FL);
    CHASSISMotor_set(0, WHEEL_MOVE_FR);
    CHASSISMotor_set(0, WHEEL_MOVE_BL);
    CHASSISMotor_set(0, WHEEL_MOVE_BR);

    CHASSISMotor_set(0, WHEEL_TURN_FL);
    CHASSISMotor_set(0, WHEEL_TURN_FR);
    CHASSISMotor_set(0, WHEEL_TURN_BL);
    CHASSISMotor_set(0, WHEEL_TURN_BR);
    }

}


/*-------------------- Task --------------------*/

/**
 * @brief          底盘任务
 * @param          none
 * @retval         none
 */
void Chassis_Tasks(void)
{
#if (USE_CHASSIS !=0 )
    float final_vx, final_vy, final_r;
    // 非is_aligning==5时重置对齐完成标志
    if (Chassis.region_code != 5)
    {
        Chassis.alignment_done = 0;
    }

    // 对齐中且relative_angle在5°以内，标记对齐完成
    if (Chassis.region_code == 5 && !Chassis.alignment_done && fabsf(Chassis.relative_angle) <= 5.0f)
    {
        Chassis.alignment_done = 1;
    }

    // 仅在对齐未完成时跟随大yaw
    uint8_t use_gimbal_follow = (Global.Chassis.mode == FLOW_Gimbal) ||
                                (Global.Chassis.mode == Navigation && Chassis.region_code == 5 && !Chassis.alignment_done);

    final_r = R_speed;

    if (use_gimbal_follow)
    {
        float theta;

        final_r = Chassis_Follow_Update();
        theta = Chassis.relative_angle * (PI / 180.0f);

        final_vx = X_speed * cosf(theta) - Y_speed * sinf(theta);
        final_vy = X_speed * sinf(theta) + Y_speed * cosf(theta);
    }
    else
    {
        final_vx = X_speed;
        final_vy = Y_speed;
    }

    Rudder_Angle_Calculation(final_vx, final_vy, final_r);
    Chassis_Updater();
    Chassis_Calculator(final_vx, final_vy, final_r);

    if (Referee_data.Chassis_Power_Limit > 0)
    {
        float final_limit = PowerBudget_Update();
        if (final_limit > 0.0f)
        {
            Chassis_PowerControl_Total(final_limit);
        }
    }
	// 底盘电机控制
	Chassis_Controller();
#endif
}



/*-------------------- Set --------------------*/

// @brief 设置底盘水平移动速度

void Chassis_SetX(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0) // 符号相反
    RampGenerator_SetCurrent(&Vx_ramp, 0.0f);
/*      Global.Chassis.input.x = x; */
}


// @brief 设置底盘竖直移动速度

void Chassis_SetY(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);
/*      Global.Chassis.input.y = y; */
}


// @brief 设置底盘角速度

void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}


// @brief 设置斜坡规划器加速度
 
void Chassis_SetAccel(float acc)
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

}
static void Receive_XY_Speed(uint8_t data[8])
{
    X_speed = -bytes_to_float(&data[0]);
    Y_speed = -bytes_to_float(&data[4]);
}

static float speed_yaw;
static void Receive_R_Yaw(uint8_t data[8])
{
    R_speed = bytes_to_float(&data[0]);
    speed_yaw = bytes_to_float(&data[4]);
    Global.Gimbal.input.yaw = speed_yaw; // 单位 rad
}

static void Receive_Control_Mode(uint8_t data[8])
{
    Global.Control.mode = (enum control_mode_e)(int)bytes_to_float(&data[0]);
    Global.Chassis.mode = (enum chassis_mode_e)(int)bytes_to_float(&data[4]);
}

static void Receive_IMU_Attitude(uint8_t data[8])
{
    imu.pitch   = bytes_to_float(&data[0]);
    imu.yaw_cnt = bytes_to_float(&data[4]);
}

static void Receive_IMU_Gyro(uint8_t data[8])
{
    imu.gyro[0] = bytes_to_float(&data[0]);
    imu.gyro[2] = bytes_to_float(&data[4]);
}

static void Receive_Saltation_Mode(uint8_t data[8])
{
    Gimbal.big_yaw.planning_speed = bytes_to_float(&data[0]);
    Chassis.is_aligning           = bytes_to_uint8(&data[4]);
    fromMINIPC.mode               = bytes_to_uint8(&data[5]);
    Auto_data.is_scaning          = bytes_to_uint8(&data[6]);
    Chassis.region_code           = bytes_to_uint8(&data[7]);
}

static void Receive_Trigger_Mode(uint8_t data[8])
{
    Global.Shoot.trigger_mode = (enum trigger_mode_e)(int)bytes_to_float(&data[0]);   
    Global.Auto.mode          = (enum auto_mode_e)(int)bytes_to_float(&data[4]);
}

static const CanRxEntry_t ChassisRxTable[] = {
    { CAN_ID_CHASSIS_SPEED_XY,     Receive_XY_Speed       },
    { CAN_ID_CHASSIS_SPEED_R_YAW,  Receive_R_Yaw          },
    { CAN_ID_CHASSIS_MODE,         Receive_Control_Mode   },
    { CAN_ID_CHASSIS_IMU_ATTITUDE, Receive_IMU_Attitude   },
    { CAN_ID_CHASSIS_IMU_GYRO,     Receive_IMU_Gyro       },
    { CAN_ID_SHOOT_TRIGGER_MODE,   Receive_Trigger_Mode   },
    { CAN_ID_SALTATION_MODE,       Receive_Saltation_Mode },
};

uint8_t Chassis_CAN_Dispatch(uint16_t id, uint8_t data[8])
{
    for (uint8_t i = 0; i < sizeof(ChassisRxTable)/sizeof(ChassisRxTable[0]); i++) {
        if (ChassisRxTable[i].id == id) {
            ChassisRxTable[i].handler(data);
            return 1;  // 匹配成功
        }
    }
    return 0;  // 未匹配
}
