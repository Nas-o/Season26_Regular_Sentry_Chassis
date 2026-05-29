/**
 * @file    Chassis.cpp
 * @brief   舵轮底盘控制器实现
 * @author  Nas (refactored 2026-05)
 */
#include <cmath>
#include <cstdlib>

#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "Global_status.hpp"
#include "Auto_control.hpp"
#include "remote_control.hpp"
#include "Shoot.hpp"

#include "referee_system.hpp"
#include "supercup.hpp"
#include "Stm32_time.hpp"
#include "IMU_updata.hpp"
#include "dm_imu.hpp"
#include "robot_param.hpp"

/* ============ 全局实例 ============ */
ChassisData chassis;

/* ============ 内部私有状态 (文件作用域) ============ */
static pid_t follow_pid;
static pid_t heading_pid;
static pid_t power_pid;
static struct { int interval = 250; int small_cnt = 0; int valve = 0; } random_state;

/* 静态配置 */
static constexpr uint16_t INIT_ECD[WHEEL_NUM]       = {918, 4426, 3020, 7746};
static constexpr float    DRIVE_SIGN[WHEEL_NUM]     = {1.0f, 1.0f, 1.0f, 1.0f};
static constexpr float    STEER_SIN_SIGN[WHEEL_NUM] = {-1, 1, -1, 1};
static constexpr float    STEER_COS_SIGN[WHEEL_NUM] = {1, 1, -1, -1};

/* ============ 电机ID配置表 ============ */
static constexpr std::array<DJIcan_id, WHEEL_NUM> DRIVE_ID = {
    WHEEL_MOVE_FL, WHEEL_MOVE_FR, WHEEL_MOVE_BL, WHEEL_MOVE_BR
};
static constexpr std::array<DJIcan_id, WHEEL_NUM> STEER_ID = {
    WHEEL_TURN_FL, WHEEL_TURN_FR, WHEEL_TURN_BL, WHEEL_TURN_BR
};

/* ============ 舵轮几何 ============ */
static float deta[WHEEL_NUM] = {45.0f, 45.0f, 45.0f, 45.0f};

/* ============ 功率限制参数 ============ */
// 轮向 (M3508)
#define WHEEL_K0 0.214f
#define WHEEL_K1 0.132f
#define WHEEL_K2 3.47f
#define CHASSIS_TOTAL_K3 7.6f
#define WHEEL_K3 (CHASSIS_TOTAL_K3 * 0.5f)
// 舵向 (GM6020)
#define STEER_K0 0.741f
#define STEER_K1 0.005f
#define STEER_K2 12.98f
#define STEER_K3 (CHASSIS_TOTAL_K3 * 0.5f)

#define POWER_ALLOC_STEER_RATIO 0.8f
#define POWER_LIMIT_REFEREE_MIN 40.0f
#define POWER_LIMIT_MIN 20.0f

/* ============ 慢环功率预算参数 ============ */
#define POWER_SAFETY_MARGIN     0.93f
#define CAP_MIN_VOL             13.0f
#define CAP_ALLOW_VOL           16.0f
#define CAP_FULL_VOL            24.0f
#define CAP_EXTRA_RATIO         0.5f
#define CAP_RELEASE_RATE        0.5f
#define CAP_TIGHTEN_RATE        3.0f
#define CAP_VOLT_DROP_THRESH    0.05f
#define BUFFER_TARGET           45.0f
#define BUFFER_DEADBAND         5.0f
#define BUFFER_DANGER           15.0f
#define BUFFER_DERATING_MAX     50.0f
#define BUFFER_TIGHTEN_RATE     2.0f
#define BUFFER_RELEASE_RATE     0.3f
#define POWER_LPF_ALPHA         0.02f
#define POWER_OVER_THRESHOLD    0.0f
#define POWER_DERATING_MAX      25.0f
#define POWER_TIGHTEN_RATE      1.5f
#define POWER_RELEASE_RATE      0.2f
#define BUDGET_UPDATE_PERIOD_MS 20

/* 慢环状态 */
static float s_cap_extra     = 0.0f;
static float s_buf_derating  = 0.0f;
static float s_pwr_derating  = 0.0f;
static float s_pwr_filt      = 0.0f;
static float s_cap_vol_last  = 0.0f;
static uint32_t s_budget_tick = 0;

/* ============ 工具函数 ============ */

/**
 * @brief  将角度限制到 [-180, 180] 范围
 * @param  angle  输入角度 (degree)
 * @return 归一化后的角度
 */
static float AngleLimit(float angle)
{
    uint32_t mul = (uint32_t)(fabsf(angle) / 180.0f);
    if (angle > 180.0f)
    {
        angle -= ((mul % 2 == 1) ? (mul + 1) : mul) * 180.0f;
    }
    if (angle < -180.0f)
    {
        angle += ((mul % 2 == 1) ? (mul + 1) : mul) * 180.0f;
    }
    return angle;
}

/**
 * @brief  将 x 归一化到 [-modulus/2, modulus/2] 范围
 *         用于ECD编码器差值的周期化处理
 * @param  x        输入值
 * @param  modulus   周期 (如 8192.0f)
 * @return 归一化后的值
 */
static float ModNorm(float x, float modulus)
{
    float tmp = fmodf(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return tmp - modulus / 2.0f;
}

/**
 * @brief  生成 [lo, hi] 范围内的随机浮点数
 */
static float RandomFloat(float lo, float hi)
{
    return lo + ((float)rand() / (float)RAND_MAX) * (hi - lo);
}

/* ============ 功率模型辅助 ============ */

/**
 * @brief  电流原始值 → 实际安培数 转换系数
 *         GM6020 (舵向): 满量程 3A / 16384
 *         M3508  (驱动): 满量程 20A / 16384
 */
static float Raw2Amp(uint8_t is_steer)
{
    if (is_steer)
        return 3.0f / 16384.0f;
    else
        return 20.0f / 16384.0f;
}

/**
 * @brief  估算单个电机的机械功率
 *         P = τ·ω + k1·|ω| + k2·τ? + k3/4
 * @param  lim        功率限制器 (含电机参数 k0~k3)
 * @param  cur_raw    电流原始值 (int16)
 * @param  speed_rad  电机转速 (rad/s)
 * @param  is_steer   是否为舵向电机
 * @return 估算功率 (W)
 */
static float MotorPower(const PowerLimiter *lim,
                        int16_t cur_raw,
                        float speed_rad,
                        uint8_t is_steer)
{
    float i_real = (float)cur_raw * Raw2Amp(is_steer);
    float torque = i_real * lim->k0;

    return torque * speed_rad
         + lim->k1 * fabsf(speed_rad)
         + lim->k2 * torque * torque
         + lim->k3 / 4.0f;
}

/**
 * @brief  预测4个电机的总功率 (正功率之和)
 * @param  lim       功率限制器
 * @param  cur       4个电机的电流值
 * @param  spd       4个电机的转速 (rad/s)
 * @param  is_steer  是否为舵向电机
 */
static void PredictPower(PowerLimiter *lim,
                         int16_t *cur,
                         float *spd,
                         uint8_t is_steer)
{
    float total = 0.0f;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        float p = MotorPower(lim, cur[i], spd[i], is_steer);
        if (p > 0.0f)
        {
            total += p;
        }
    }
    lim->predict_power = total;
}

/**
 * @brief  根据功率预算等比缩减电流
 *         使用二次方程求解: P = τω + k1|ω| + k2τ? + k3/4
 *         给定功率预算, 反解出最大允许力矩 → 转换为电流限幅
 * @param  lim       功率限制器
 * @param  budget    功率预算 (W)
 * @param  cur       4个电机的电流值 (in/out, 会被限幅修改)
 * @param  spd       4个电机的转速 (rad/s)
 * @param  is_steer  是否为舵向电机
 */
static void ApplyPowerLimit(PowerLimiter *lim,
                            float budget,
                            int16_t *cur,
                            float *spd,
                            uint8_t is_steer)
{
    float alloc = budget;
    float pwr_sum = 0.0f;
    float cur_sum_before = 0.0f;
    float cur_sum_after  = 0.0f;
    float cmd_pwr[WHEEL_NUM] = {};
    float ratio = Raw2Amp(is_steer);

    lim->limit_power = budget;

    /* 第一轮: 统计各电机功率 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        cmd_pwr[i] = MotorPower(lim, cur[i], spd[i], is_steer);
        cur_sum_before += fabsf((float)cur[i]);
        if (cmd_pwr[i] > 0.0f)
            pwr_sum += cmd_pwr[i];
        else
            alloc -= cmd_pwr[i];  /* 负功率(回馈)释放预算 */
    }

    /* 预测功率在预算内, 无需限制 */
    if (lim->predict_power <= budget)
    {
        lim->scaling = 1.0f;
        return;
    }
    if (alloc < 0.0f)
        alloc = 0.0f;
    if (pwr_sum <= 0.0f)
    {
        lim->scaling = 1.0f;
        return;
    }

    /* 第二轮: 按功率占比分配预算, 反解最大电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        /* 负功率(回馈)的电机不限制 */
        if (cmd_pwr[i] <= 0.0f)
        {
            cur_sum_after += fabsf((float)cur[i]);
            continue;
        }

        /* 按功率占比分配该电机的预算 */
        float weight    = cmd_pwr[i] / pwr_sum;
        float motor_bgt = weight * alloc;
        float w         = spd[i];

        /* 解二次方程: k2*τ? + ω*τ + (k1|ω| + k3/4 - budget) = 0 */
        float delta = w * w
                    - 4.0f * lim->k2 * (lim->k1 * fabsf(w) + lim->k3 / 4.0f - motor_bgt);
        float tq_lim;

        if (delta <= 0.0f)
        {
            tq_lim = -w / (2.0f * lim->k2);
        }
        else if (cur[i] >= 0)
        {
            tq_lim = (-w + sqrtf(delta)) / (2.0f * lim->k2);
        }
        else
        {
            tq_lim = (-w - sqrtf(delta)) / (2.0f * lim->k2);
        }

        /* 力矩 → 电流, 并确保不超过原始值 */
        float lim_cur = tq_lim / (lim->k0 * ratio);
        if (cur[i] >= 0 && lim_cur > cur[i])
            lim_cur = cur[i];
        else if (cur[i] < 0 && lim_cur < cur[i])
            lim_cur = cur[i];
        lim_cur = fp32_constrain(lim_cur, -CHASSIS_MAX_CURRENT, CHASSIS_MAX_CURRENT);

        cur[i] = (int16_t)lim_cur;
        cur_sum_after += fabsf(lim_cur);
    }

    /* 记录整体缩放比例 */
    lim->scaling = (cur_sum_before > 0.0f)
                 ? (cur_sum_after / cur_sum_before)
                 : 1.0f;
}

/* ================================================================
 * 底盘控制函数
 * ================================================================ */

/**
 * @brief  初始化底盘所有电机、PID参数及功率限制器
 *         - 4个 M3508 驱动电机 (速度环)
 *         - 4个 GM6020 舵向电机 (位置环+速度环)
 *         - 底盘跟随/航向/功率 PID
 */
static void Chassis_InitMotor()
{
    /* ---- 电机注册 ---- */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        DJIMotor_Init(DJI_M3508, DRIVE_ID[i]);   /* 驱动电机 */
        DJIMotor_Init(DJI_GM6020, STEER_ID[i]);   /* 舵向电机 */
    }

    /* ---- 驱动电机 PID (速度环) ---- */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        PID_Set(&chassis.wheel[i].drive.pid,
                8000.0f, 0, 0.3f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
    }

    /* ---- 舵向电机 PID (位置环 + 速度环) ---- */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        /* 位置环 */
        PID_Set(&chassis.wheel[i].steer.pos_pid,
                8.0f, 0.0f, 0.0f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
        /* 速度环 */
        PID_Set(&chassis.wheel[i].steer.spd_pid,
                6.0f, 0.0f, 2.0f, 0.0f,
                CHASSIS_MAX_CURRENT, 1500);
    }

    /* ---- 底盘跟随 PID ---- */
    PID_Set(&follow_pid,
            1.0f, 0.0f, 0.0f, 0.0f,
            120, 40);

    /* ---- 航向对齐 PID ---- */
    PID_Set(&heading_pid,
            0.5f, 0.0f, 0.0f, 0.0f,
            120, 40);

    /* ---- 功率闭环 PID ---- */
    PID_Set(&power_pid,
            0.04f, 0.0f, 0.01f, 0,
            0.09f, 0.09f);

    /* ---- 慢环状态清零 ---- */
    s_cap_extra    = 0.0f;
    s_buf_derating = 0.0f;
    s_pwr_derating = 0.0f;
    s_pwr_filt     = 0.0f;
    s_cap_vol_last = 0.0f;
    s_budget_tick  = 0;

    /* ---- 默认模式 ---- */
    Global.Chassis.mode = FLOW_Chassis;

    /* ---- 舵向初始ECD ---- */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        chassis.wheel[i].steer.now = 0;
        chassis.wheel[i].steer.set = INIT_ECD[i];
    }

    /* ---- deta角度转弧度 ---- */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        deta[i] = deta[i] * PI / 180.0f;
    }

    /* ---- 随机运动参数 ---- */
    random_state.interval = 250;
    srand(HAL_GetTick());

    /* ---- 功率限制器电机参数 ---- */
    chassis.lim_wheel = {WHEEL_K0, WHEEL_K1, WHEEL_K2, WHEEL_K3, 0, 0, 1.0f};
    chassis.lim_steer = {STEER_K0, STEER_K1, STEER_K2, STEER_K3, 0, 0, 1.0f};
}

/* -------------------- Update -------------------- */

/**
 * @brief  更新底盘传感器数据
 *         从电机编码器读取舵向角度和驱动轮速
 */
static void Chassis_Update()
{
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        /* 舵向: 读取当前编码器值 */
        chassis.wheel[i].steer.now =
            DJIMotor_GetData(STEER_ID[i]).ecd;

        /* 驱动: RPM → rad/s → 线速度 (m/s) */
        chassis.wheel[i].drive.current_velocity =
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
static void Chassis_RudderCalculation(float x, float y, float w)
{
    w *= WHEEL_TRACK;

    /* 全零速度时保持当前舵角, 不做计算 */
    if (x == 0 && y == 0 && w == 0)
    {
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            chassis.wheel[i].steer.set_angle = 0.0f;
        }
    }
    else
    {
        /*
         * 各轮舵角: angle = atan2(vy分量, vx分量)
         * vy分量 = y - COS_SIGN[i] * w * cos45
         * vx分量 = x + SIN_SIGN[i] * w * sin45
         */
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            float vy_comp = y - STEER_COS_SIGN[i] * w * 0.707107f;
            float vx_comp = x + STEER_SIN_SIGN[i] * w * 0.707107f;
            chassis.wheel[i].steer.set_angle =
                atan2f(vy_comp, vx_comp) * 180.0f / PI;
        }
    }

    /* 角度 → ECD编码器目标值 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        chassis.wheel[i].steer.set_ecd =
            INIT_ECD[i] + chassis.wheel[i].steer.set_angle * 8192.0f / 360.0f;
        chassis.wheel[i].steer.set = (int32_t)chassis.wheel[i].steer.set_ecd;
    }
}

/* -------------------- Nearby Transposition -------------------- */
/**
 * @brief  就近转位: 避免舵向转动超过90°, 通过反转驱动方向代替
 * @note   使用余弦连续调制: direction = cos(偷差角), 在±90°边界平滑过渡
 */
static void Chassis_NearbyTransposition()
{
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = chassis.wheel[i].steer;
        DriveUnit &dr = chassis.wheel[i].drive;

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
static float Chassis_FollowUpdate()
{
    chassis.relative_angle = AngleLimit(chassis.relative_angle);

    /* 死区: 相对角很小时不跟随, 避免抖动 */
    if (fabsf(chassis.relative_angle) <= 2.0f)
    {
        return 0.0f;
    }

    return -PID_Cal(&follow_pid, chassis.relative_angle, 0.0f) * RPM_TO_RAD_S;
}

/**
 * @brief  底盘航向对齐: 根据航向误差PID输出旋转速度
 * @return 底盘旋转角速度 (rad/s), 误差≤2°时返回0
 */
static float Chassis_HeadingAlign()
{
    float chassis_yaw = imu.yaw_cnt - chassis.relative_angle;
    float err = AngleLimit(chassis.heading_target - chassis_yaw);

    /* 死区: 航向误差很小时不修正 */
    if (fabsf(err) <= 2.0f)
    {
        return 0.0f;
    }

    return PID_Cal(&heading_pid, 0.0f, err) * RPM_TO_RAD_S;
}

/* -------------------- Calculate (速度分解 + PID计算) -------------------- */
/**
 * @brief  根据目标速度计算各轮电流
 * @param  vx  底盘x方向速度 (m/s)
 * @param  vy  底盘y方向速度 (m/s)
 * @param  vw  底盘旋转角速度 (rad/s)
 * @note   流程: 就近转位 → 舵向位置环 → 驱动速度分解 → 舵向速度环 → 驱动速度环
 */
static void Chassis_Calculator(float vx, float vy, float vw)
{
    /* 第1步: 就近转位 (避免舵轮转超90°) */
    Chassis_NearbyTransposition();

    /* 第2步: 舵向位置环 → 速度目标 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = chassis.wheel[i].steer;
        st.spd_set = PID_Cal(&st.pos_pid, st.now, st.set);
    }

    /* 第3步: 驱动速度分解 — 将底盘速度分解到各轮 */
    float w = vw * WHEEL_TRACK;
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        float vx_c = vx + STEER_SIN_SIGN[i] * w * sinf(deta[i]);
        float vy_c = vy + STEER_COS_SIGN[i] * w * cosf(deta[i]);
        float v_mag = sqrtf(vx_c * vx_c + vy_c * vy_c);

        /* 合速度 × 就近转位方向系数 */
        chassis.wheel[i].drive.target_velocity =
            v_mag * chassis.wheel[i].drive.direction;
    }

    /* 第4步: 舵向速度环 → 电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        SteerUnit &st = chassis.wheel[i].steer;
        st.current = (int16_t)PID_Cal(
            &st.spd_pid,
            DJIMotor_GetData(STEER_ID[i]).speed_rpm,
            st.spd_set);
    }

    /* 第5步: 驱动速度环 → 电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        DriveUnit &dr = chassis.wheel[i].drive;
        dr.current = (int16_t)PID_Cal(
            &dr.pid,
            dr.current_velocity,
            dr.target_velocity);
    }
}

/* -------------------- Power Control -------------------- */

/**
 * @brief  功率限制总控 — 将计算好的电流按功率预算等比限幅
 *         先分配舵向/驱动的功率比例, 再分别限制
 * @param  total_max  本周期功率预算总额 (W)
 */
static void Chassis_PowerTotal(float total_max)
{
    int16_t w_c[WHEEL_NUM], s_c[WHEEL_NUM];
    float   w_s[WHEEL_NUM], s_s[WHEEL_NUM];

    /* 拷贝当前电流和速度 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        w_c[i] = chassis.wheel[i].drive.current;
        s_c[i] = chassis.wheel[i].steer.current;
        w_s[i] = DJIMotor_GetData(DRIVE_ID[i]).round_speed * RPM_TO_RAD_S;
        s_s[i] = DJIMotor_GetData(STEER_ID[i]).speed_rpm * RPM_TO_RAD_S;
    }

    /* 预测两组电机的功率 */
    PredictPower(&chassis.lim_steer, s_c, s_s, 1);
    PredictPower(&chassis.lim_wheel, w_c, w_s, 0);

    /* 分配功率: 舵向最多占 POWER_ALLOC_STEER_RATIO, 剩余给驱动 */
    float steer_need  = chassis.lim_steer.predict_power;
    float steer_limit = total_max * POWER_ALLOC_STEER_RATIO;
    float wheel_limit = total_max - fminf(steer_need, steer_limit);
    if (wheel_limit < 0.0f)
    {
        wheel_limit = 0.0f;
    }

    /* 分别限制舵向和驱动的电流 */
    ApplyPowerLimit(&chassis.lim_steer, steer_limit, s_c, s_s, 1);
    ApplyPowerLimit(&chassis.lim_wheel, wheel_limit, w_c, w_s, 0);

    /* 回写限幅后的电流 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        chassis.wheel[i].drive.current = w_c[i];
        chassis.wheel[i].steer.current = s_c[i];
    }
}

/**
 * @brief  慢环功率预算更新 (每 BUDGET_UPDATE_PERIOD_MS 执行一次)
 *         综合考虑: 裁判系统功率限制、超电额外功率、Buffer缓冲、实测功率保护
 * @return 本周期的功率预算 (W)
 */
static float Chassis_BudgetUpdate()
{
    uint32_t now = HAL_GetTick();
    static float last_budget = 60.0f;

    /* 未到更新周期, 返回上次结果 */
    if ((now - s_budget_tick) < BUDGET_UPDATE_PERIOD_MS)
    {
        return last_budget;
    }
    s_budget_tick = now;

    /* 读取裁判系统功率限制 */
    float P_ref = (float)Referee_data.Chassis_Power_Limit;
    if (P_ref <= 0.0f)
    {
        last_budget = 0.0f;
        return 0.0f;
    }
    if (P_ref < POWER_LIMIT_REFEREE_MIN)
    {
        P_ref = POWER_LIMIT_REFEREE_MIN;
    }

    /* 安全裕度: 留出余量避免超功率 */
    float P_sus = P_ref * POWER_SAFETY_MARGIN;

    /* 设置超电输入功率 */
    if (P_ref > 5.0f)
        Supercap_SetPower((uint8_t)(P_ref - 5.0f));
    else
        Supercap_SetPower(0);

    /* 缓存裁判系统缓冲能量 (限255) */
    cap.cache_energy = (Referee_data.Buffer_Energy > 255u)
                     ? 255u
                     : (uint8_t)Referee_data.Buffer_Energy;

    /* ---- 超电额外功率调节 ---- */
    {
        float vol  = cap.remain_vol;
        float drop = s_cap_vol_last - vol;
        float buf_e = (float)Referee_data.Buffer_Energy;

        /* 电压过低: 立即关闭额外功率 */
        if (vol < CAP_MIN_VOL)
        {
            s_cap_extra = 0.0f;
        }
        /* 电压骤降: 收紧 */
        else if (drop > CAP_VOLT_DROP_THRESH)
        {
            s_cap_extra -= CAP_TIGHTEN_RATE;
        }
        /* 电压充足且缓冲够: 缓慢释放额外功率 */
        else if (vol > CAP_ALLOW_VOL && buf_e > BUFFER_TARGET)
        {
            s_cap_extra += CAP_RELEASE_RATE;
        }
        /* 缓冲不足: 慢收紧 */
        else if (buf_e < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            s_cap_extra -= CAP_TIGHTEN_RATE * 0.5f;
        }

        float cap_max = P_ref * CAP_EXTRA_RATIO;
        s_cap_extra = fp32_constrain(s_cap_extra, 0.0f, cap_max);
        s_cap_vol_last = vol;
    }

    /* ---- Buffer Energy 慢闭环 ---- */
    {
        float buf_e = (float)Referee_data.Buffer_Energy;

        /* 缓冲能量危险: 立即拉满降额 */
        if (buf_e < BUFFER_DANGER)
        {
            s_buf_derating = BUFFER_DERATING_MAX;
        }
        /* 缓冲偏低: 逐步收紧 */
        else if (buf_e < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            s_buf_derating += BUFFER_TIGHTEN_RATE;
        }
        /* 缓冲恢复: 逐步释放 */
        else if (buf_e > BUFFER_TARGET + BUFFER_DEADBAND)
        {
            s_buf_derating -= BUFFER_RELEASE_RATE;
        }
        s_buf_derating = fp32_constrain(s_buf_derating, 0.0f, BUFFER_DERATING_MAX);
    }

    /* ---- 实测功率保护 ---- */
    {
        float raw = cap.Chassis_power;
        if (raw > 0.0f)
        {
            s_pwr_filt += POWER_LPF_ALPHA * (raw - s_pwr_filt);
        }

        float over = s_pwr_filt - P_sus;
        if (over > POWER_OVER_THRESHOLD)
        {
            s_pwr_derating += POWER_TIGHTEN_RATE;
        }
        else if (s_pwr_filt < P_ref * 0.85f)
        {
            s_pwr_derating -= POWER_RELEASE_RATE;
        }
        s_pwr_derating = fp32_constrain(s_pwr_derating, 0.0f, POWER_DERATING_MAX);
    }

    /* ---- 汇总: 最终功率预算 ---- */
    float final_lim = P_sus + s_cap_extra - s_buf_derating - s_pwr_derating;
    if (final_lim < POWER_LIMIT_MIN)
    {
        final_lim = POWER_LIMIT_MIN;
    }

    last_budget = final_lim;
    return final_lim;
}

/* -------------------- Control -------------------- */

/**
 * @brief  底盘电流输出
 *         非锁定模式: 输出驱动+舵向电流
 *         锁定模式: 全部置零
 */
static void Chassis_Controller()
{
    if (Global.Control.mode != LOCK)
    {
        for (int i = 0; i < WHEEL_NUM; i++)
        {
            DJIMotor_Set(chassis.wheel[i].drive.current, DRIVE_ID[i]);
            DJIMotor_Set(chassis.wheel[i].steer.current, STEER_ID[i]);
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

/* =================== 兼容旧接口 =================== */

/**
 * @brief  设置底盘x方向速度 (供遥控器/自瞄调用)
 */
void Chassis_SetX(float x)
{
    chassis.vx_ = x;
}

/**
 * @brief  设置底盘y方向速度
 */
void Chassis_SetY(float y)
{
    chassis.vy_ = y;
}

/**
 * @brief  设置底盘旋转速度
 */
void Chassis_SetR(float r)
{
    Global.Chassis.input.r = r;
}

/* =================== extern "C" 接口 =================== */

extern "C" void Chassis_Init(void)
{
    Chassis_InitMotor();
}

/**
 * @brief  底盘主控制循环 (由RTOS任务周期调用)
 *         流程: 对齐判断 → 跟随/独立模式 → 舵角计算 → 速度分解 → 功率限制 → 电流输出
 */
extern "C" void Chassis_Tasks(void)
{
#if (USE_CHASSIS != 0)

    /* ---- 对齐逻辑 ---- */
    /* 离开区域5: 重置对齐完成标志 */
    if (chassis.region_code != 5)
    {
        chassis.alignment_done = 0;
    }
    /* 进入区域5 且 相对角很小: 标记对齐完成 */
    if (chassis.region_code == 5
        && !chassis.alignment_done
        && fabsf(chassis.relative_angle) <= 5.0f)
    {
        chassis.alignment_done = 1;
    }

    /* ---- 判断是否需要底盘跟随云台 ---- */
    uint8_t use_follow =
        (Global.Chassis.mode == FLOW_Gimbal) ||
        (Global.Chassis.mode == Navigation
         && chassis.region_code == 5
         && !chassis.alignment_done);

    float final_r  = chassis.vr_;
    float final_vx = chassis.vx_;
    float final_vy = chassis.vy_;

    if (use_follow)
    {
        /*
         * 跟随模式: 底盘跟随云台旋转
         * 同时将世界坐标系速度旋转到底盘坐标系
         */
        final_r = Chassis_FollowUpdate();

        float theta = chassis.relative_angle * (PI / 180.0f);
        float cos_t = cosf(theta);
        float sin_t = sinf(theta);
        final_vx = chassis.vx_ * cos_t - chassis.vy_ * sin_t;
        final_vy = chassis.vx_ * sin_t + chassis.vy_ * cos_t;
    }

    /* ---- 控制流水线 ---- */

    /* 1. 计算各轮舵向目标角度 */
    Chassis_RudderCalculation(final_vx, final_vy, final_r);

    /* 2. 更新传感器数据 */
    Chassis_Update();

    /* 3. 速度分解 + PID → 电流 */
    Chassis_Calculator(final_vx, final_vy, final_r);

    /* 4. 功率限制 (仅裁判系统有功率限制时生效) */
    if (Referee_data.Chassis_Power_Limit > 0)
    {
        float lim = Chassis_BudgetUpdate();
        if (lim > 0.0f)
        {
            Chassis_PowerTotal(lim);
        }
    }

    /* 5. 电流输出到电机 */
    Chassis_Controller();

#endif
}

extern "C" uint8_t Chassis_GetAligning(void)
{
    return chassis.is_aligning;
}
