/**
 * @file    PowerControl.cpp
 * @brief   底盘功率控制模块 — 实现
 * @author  Nas (2026-05)
 *
 * @details 包含:
 *          1. PowerLimiter 类 — 电机功率预测 + 限幅 (二次方程反解最大电流)
 *          2. PowerBudget 类 — 慢环功率预算 (超电/Buffer/实测功率 三重保护)
 *          3. extern "C" 接口 — 供 FreeRTOS C 代码调用
 */
#include <cmath>
#include "PowerControl.hpp"
#include "referee_system.hpp"
#include "supercup.hpp"
#include "motor.hpp"

/* ============ 电机配置宏 ============ */
#define CHASSIS_MAX_CURRENT MAX_CURRENT

/* ============ 电机数量 ============ */
#define MOTOR_NUM 4

/* ============ 功率模型参数 ============ */

/* 驱动电机 (M3508) */
#define WHEEL_K0            0.214f
#define WHEEL_K1            0.132f
#define WHEEL_K2            3.47f
#define CHASSIS_TOTAL_K3    7.6f
#define WHEEL_K3            (CHASSIS_TOTAL_K3 * 0.5f)

/* 舵向电机 (GM6020) */
#define STEER_K0            0.741f
#define STEER_K1            0.005f
#define STEER_K2            12.98f
#define STEER_K3            (CHASSIS_TOTAL_K3 * 0.5f)

/* 分配比例 */
#define POWER_ALLOC_STEER_RATIO 0.8f
#define POWER_LIMIT_REFEREE_MIN 40.0f
#define POWER_LIMIT_MIN         20.0f

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

/* ============ 全局实例 ============ */
PowerLimiter g_lim_wheel;
PowerLimiter g_lim_steer;
PowerBudget  g_power_budget;

/* ============================================================
 *  PowerLimiter 类实现
 * ============================================================ */

void PowerLimiter::SetParams(float _k0, float _k1, float _k2, float _k3)
{
    k0 = _k0;  k1 = _k1;  k2 = _k2;  k3 = _k3;
    limit_power   = 0;
    predict_power = 0;
    scaling       = 1.0f;
}

/**
 * @brief  电流原始值 → 安培数
 *         GM6020 (舵向): 满量程 3A / 16384
 *         M3508  (驱动): 满量程 20A / 16384
 */
float PowerLimiter::RawToAmpere(bool is_steer)
{
    return is_steer ? (3.0f / 16384.0f) : (20.0f / 16384.0f);
}

/**
 * @brief  估算单个电机的机械功率
 *         P = τ·ω + k1·|ω| + k2·τ? + k3/4
 */
float PowerLimiter::EstimateMotorPower(int16_t current_raw,
                                       float speed_rad,
                                       bool is_steer) const
{
    float ampere = (float)current_raw * RawToAmpere(is_steer);
    float torque = ampere * k0;

    return torque * speed_rad
         + k1 * fabsf(speed_rad)
         + k2 * torque * torque
         + k3 / 4.0f;
}

/**
 * @brief  预测本组 4 个电机的总功率 (只累加正功率)
 */
void PowerLimiter::Predict(int16_t current[4], float speed[4], bool is_steer)
{
    float total = 0.0f;
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        float power = EstimateMotorPower(current[i], speed[i], is_steer);
        if (power > 0.0f)
        {
            total += power;
        }
    }
    predict_power = total;
}

/**
 * @brief  根据功率预算等比缩减电流
 *         使用二次方程: k2·τ? + ω·τ + (k1|ω| + k3/4 - budget) = 0
 *         反解最大允许力矩 → 转换为电流限幅
 */
void PowerLimiter::ApplyLimit(float budget,
                              int16_t current[4],
                              float speed[4],
                              bool is_steer)
{
    float alloc = budget;
    float power_sum = 0.0f;
    float current_sum_before = 0.0f;
    float current_sum_after  = 0.0f;
    float cmd_power[MOTOR_NUM] = {};
    float amp_ratio = RawToAmpere(is_steer);

    limit_power = budget;

    /* 第一轮: 统计各电机功率 */
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        cmd_power[i] = EstimateMotorPower(current[i], speed[i], is_steer);
        current_sum_before += fabsf((float)current[i]);
        if (cmd_power[i] > 0.0f)
            power_sum += cmd_power[i];
        else
            alloc -= cmd_power[i];  /* 负功率(回馈)释放预算 */
    }

    /* 预测功率在预算内, 无需限制 */
    if (predict_power <= budget)
    {
        scaling = 1.0f;
        return;
    }
    if (alloc < 0.0f)
        alloc = 0.0f;
    if (power_sum <= 0.0f)
    {
        scaling = 1.0f;
        return;
    }

    /* 第二轮: 按功率占比分配预算, 反解最大电流 */
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        /* 负功率(回馈)的电机不限制 */
        if (cmd_power[i] <= 0.0f)
        {
            current_sum_after += fabsf((float)current[i]);
            continue;
        }

        /* 按功率占比分配该电机的预算 */
        float weight       = cmd_power[i] / power_sum;
        float motor_budget = weight * alloc;
        float w            = speed[i];

        /* 解二次方程: k2·τ? + ω·τ + (k1|ω| + k3/4 - budget) = 0 */
        float delta = w * w
                    - 4.0f * k2 * (k1 * fabsf(w) + k3 / 4.0f - motor_budget);
        float torque_limit;

        if (delta <= 0.0f)
        {
            torque_limit = -w / (2.0f * k2);
        }
        else if (current[i] >= 0)
        {
            torque_limit = (-w + sqrtf(delta)) / (2.0f * k2);
        }
        else
        {
            torque_limit = (-w - sqrtf(delta)) / (2.0f * k2);
        }

        /* 力矩 → 电流, 并确保不超过原始值 */
        float limited_current = torque_limit / (k0 * amp_ratio);
        if (current[i] >= 0 && limited_current > current[i])
            limited_current = current[i];
        else if (current[i] < 0 && limited_current < current[i])
            limited_current = current[i];
        limited_current = fp32_constrain(limited_current, -CHASSIS_MAX_CURRENT, CHASSIS_MAX_CURRENT);

        current[i] = (int16_t)limited_current;
        current_sum_after += fabsf(limited_current);
    }

    /* 记录整体缩放比例 */
    scaling = (current_sum_before > 0.0f)
            ? (current_sum_after / current_sum_before)
            : 1.0f;
}

/* ============================================================
 *  PowerBudget 类实现
 * ============================================================ */

void PowerBudget::Reset()
{
    cap_extra_power        = 0.0f;
    cap_voltage_last       = 0.0f;
    buffer_energy_derating = 0.0f;
    power_derating         = 0.0f;
    power_filtered         = 0.0f;
    last_update_tick       = 0;
    last_budget            = 60.0f;
}

/**
 * @brief  慢环功率预算更新 (每 BUDGET_UPDATE_PERIOD_MS 执行一次)
 *         综合考虑: 裁判系统功率限制、超电额外功率、Buffer缓冲、实测功率保护
 * @return 本周期的功率预算 (W)
 */
float PowerBudget::Update()
{
    uint32_t now = HAL_GetTick();

    /* 未到更新周期, 返回上次结果 */
    if ((now - last_update_tick) < BUDGET_UPDATE_PERIOD_MS)
    {
        return last_budget;
    }
    last_update_tick = now;

    /* 读取裁判系统功率限制 */
    float power_referee = (float)Referee_data.Chassis_Power_Limit;
    if (power_referee <= 0.0f)
    {
        last_budget = 0.0f;
        return 0.0f;
    }
    if (power_referee < POWER_LIMIT_REFEREE_MIN)
    {
        power_referee = POWER_LIMIT_REFEREE_MIN;
    }

    /* 安全裕度: 留出余量避免超功率 */
    float power_sustained = power_referee * POWER_SAFETY_MARGIN;

    /* 设置超电输入功率 */
    if (power_referee > 5.0f)
        Supercap_SetPower((uint8_t)(power_referee - 5.0f));
    else
        Supercap_SetPower(0);

    /* 缓存裁判系统缓冲能量 (限255) */
    cap.cache_energy = (Referee_data.Buffer_Energy > 255u)
                     ? 255u
                     : (uint8_t)Referee_data.Buffer_Energy;

    /* ---- 超电额外功率调节 ---- */
    {
        float voltage       = cap.remain_vol;
        float voltage_drop  = cap_voltage_last - voltage;
        float buffer_energy = (float)Referee_data.Buffer_Energy;

        /* 电压过低: 立即关闭额外功率 */
        if (voltage < CAP_MIN_VOL)
        {
            cap_extra_power = 0.0f;
        }
        /* 电压骤降: 收紧 */
        else if (voltage_drop > CAP_VOLT_DROP_THRESH)
        {
            cap_extra_power -= CAP_TIGHTEN_RATE;
        }
        /* 电压充足且缓冲够: 缓慢释放额外功率 */
        else if (voltage > CAP_ALLOW_VOL && buffer_energy > BUFFER_TARGET)
        {
            cap_extra_power += CAP_RELEASE_RATE;
        }
        /* 缓冲不足: 慢收紧 */
        else if (buffer_energy < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            cap_extra_power -= CAP_TIGHTEN_RATE * 0.5f;
        }

        float cap_max = power_referee * CAP_EXTRA_RATIO;
        cap_extra_power = fp32_constrain(cap_extra_power, 0.0f, cap_max);
        cap_voltage_last = voltage;
    }

    /* ---- Buffer Energy 慢闭环 ---- */
    {
        float buffer_energy = (float)Referee_data.Buffer_Energy;

        /* 缓冲能量危险: 立即拉满降额 */
        if (buffer_energy < BUFFER_DANGER)
        {
            buffer_energy_derating = BUFFER_DERATING_MAX;
        }
        /* 缓冲偏低: 逐步收紧 */
        else if (buffer_energy < BUFFER_TARGET - BUFFER_DEADBAND)
        {
            buffer_energy_derating += BUFFER_TIGHTEN_RATE;
        }
        /* 缓冲恢复: 逐步释放 */
        else if (buffer_energy > BUFFER_TARGET + BUFFER_DEADBAND)
        {
            buffer_energy_derating -= BUFFER_RELEASE_RATE;
        }
        buffer_energy_derating = fp32_constrain(buffer_energy_derating, 0.0f, BUFFER_DERATING_MAX);
    }

    /* ---- 实测功率保护 ---- */
    {
        float measured_power = cap.Chassis_power;
        if (measured_power > 0.0f)
        {
            power_filtered += POWER_LPF_ALPHA * (measured_power - power_filtered);
        }

        float over = power_filtered - power_sustained;
        if (over > POWER_OVER_THRESHOLD)
        {
            power_derating += POWER_TIGHTEN_RATE;
        }
        else if (power_filtered < power_referee * 0.85f)
        {
            power_derating -= POWER_RELEASE_RATE;
        }
        power_derating = fp32_constrain(power_derating, 0.0f, POWER_DERATING_MAX);
    }

    /* ---- 汇总: 最终功率预算 ---- */
    float final_limit = power_sustained + cap_extra_power
                      - buffer_energy_derating - power_derating;
    if (final_limit < POWER_LIMIT_MIN)
    {
        final_limit = POWER_LIMIT_MIN;
    }

    last_budget = final_limit;
    return final_limit;
}

/* ============================================================
 *  extern "C" 公开接口 (委托给类实例)
 * ============================================================ */

extern "C" void PowerControl_Init(void)
{
    g_lim_wheel.SetParams(WHEEL_K0, WHEEL_K1, WHEEL_K2, WHEEL_K3);
    g_lim_steer.SetParams(STEER_K0, STEER_K1, STEER_K2, STEER_K3);
    g_power_budget.Reset();
}

extern "C" float PowerControl_BudgetUpdate(void)
{
    return g_power_budget.Update();
}

extern "C" void PowerControl_Apply(int16_t drive_cur[4], float drive_spd[4],
                                   int16_t steer_cur[4], float steer_spd[4],
                                   float budget)
{
    /* 预测两组电机的功率 */
    g_lim_steer.Predict(steer_cur, steer_spd, true);
    g_lim_wheel.Predict(drive_cur, drive_spd, false);

    /* 分配功率: 舵向最多占 POWER_ALLOC_STEER_RATIO, 剩余给驱动 */
    float steer_need  = g_lim_steer.predict_power;
    float steer_limit = budget * POWER_ALLOC_STEER_RATIO;
    float wheel_limit = budget - fminf(steer_need, steer_limit);
    if (wheel_limit < 0.0f)
    {
        wheel_limit = 0.0f;
    }

    /* 分别限制舵向和驱动的电流 */
    g_lim_steer.ApplyLimit(steer_limit, steer_cur, steer_spd, true);
    g_lim_wheel.ApplyLimit(wheel_limit, drive_cur, drive_spd, false);
}

