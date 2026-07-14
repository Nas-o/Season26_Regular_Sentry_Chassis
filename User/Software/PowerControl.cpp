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

/* ============ 电机配置宏 (编译开关/外部宏引用) ============ */
#define CHASSIS_MAX_CURRENT MAX_CURRENT

namespace {

/* ============ 电机数量 ============ */
constexpr int kMotorNum = 4;

/* ============ 功率模型参数 ============ */

/* 驱动电机 (M3508) */
constexpr float kWheelK0          = 0.214f;
constexpr float kWheelK1          = 0.132f;
constexpr float kWheelK2          = 3.47f;
constexpr float kChassisTotalK3   = 7.6f;
constexpr float kWheelK3          = kChassisTotalK3 * 0.5f;

/* 舵向电机 (GM6020) */
constexpr float kSteerK0          = 0.741f;
constexpr float kSteerK1          = 0.005f;
constexpr float kSteerK2          = 12.98f;
constexpr float kSteerK3          = kChassisTotalK3 * 0.5f;

/* 分配比例 */
constexpr float kPowerAllocSteerRatio = 0.8f;
constexpr float kPowerLimitRefereeMin = 40.0f;
constexpr float kPowerLimitMin        = 20.0f;

/* ============ 慢环功率预算参数 ============ */
constexpr float    kPowerSafetyMargin    = 0.93f;
constexpr float    kCapEnableVol         = 16.0f;
constexpr float    kCapProtectVol        = 13.0f;
constexpr float    kCapExtraRatio        = 0.5f;
constexpr float    kBufferTarget         = 45.0f;
constexpr float    kBufferDeadband       = 5.0f;
constexpr float    kBufferDeratingMax    = 50.0f;
constexpr float    kTightenRate          = 2.0f;
constexpr float    kReleaseRate          = 0.3f;
constexpr float    kPowerLpfAlpha        = 0.02f;
constexpr float    kPowerDeratingMax     = 25.0f;
constexpr uint32_t kBudgetUpdatePeriodMs = 20;
constexpr float buffer_danger = kBufferTarget - 6.0f * kBufferDeadband;
} 

/* ============ 全局实例 ============ */
robot::power_control::PowerLimiter g_lim_wheel;
robot::power_control::PowerLimiter g_lim_steer;
robot::power_control::PowerBudget  g_power_budget;

namespace robot::power_control 
{

/* ============================================================
 *  PowerLimiter 类实现
 * ============================================================ */

    void PowerLimiter::setParams(float _k0, float _k1, float _k2, float _k3)
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
    float PowerLimiter::rawToAmpere(bool is_steer)
    {
        return is_steer ? (3.0f / 16384.0f) : (20.0f / 16384.0f);
    }

    /**
     * @brief  估算单个电机的机械功率
     *         P = τ·ω + k1·|ω| + k2·τ? + k3/4
     */
    float PowerLimiter::estimateMotorPower(int16_t current_raw, float speed_rad, bool is_steer) const
    {
        float ampere = (float)current_raw * rawToAmpere(is_steer);
        float torque = ampere * k0;

        return torque * speed_rad
            + k1 * fabsf(speed_rad)
            + k2 * torque * torque
            + k3 / 4.0f;
    }

    /**
     * @brief  预测本组 4 个电机的总功率 (只累加正功率)
     */
    void PowerLimiter::predict(int16_t current[4], float speed[4], bool is_steer)
    {
        float total = 0.0f;
        for (int i = 0; i < kMotorNum; i++)
        {
            float power = estimateMotorPower(current[i], speed[i], is_steer);
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
    void PowerLimiter::applyLimit(float budget, int16_t current[4], float speed[4], bool is_steer)
    {
        float alloc = budget;
        float power_sum = 0.0f;
        float current_sum_before = 0.0f;
        float current_sum_after  = 0.0f;
        float cmd_power[kMotorNum] = {};
        float amp_ratio = rawToAmpere(is_steer);

        limit_power = budget;

        /* 第一轮: 统计各电机功率 */
        for (int i = 0; i < kMotorNum; i++)
        {
            cmd_power[i] = estimateMotorPower(current[i], speed[i], is_steer);
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
        for (int i = 0; i < kMotorNum; i++)
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

    void PowerBudget::reset()
    {
        cap_extra_power_        = 0.0f;
        buffer_energy_derating_ = 0.0f;
        power_derating_         = 0.0f;
        power_filtered_         = 0.0f;
        last_update_tick_       = 0;
        last_budget_            = 60.0f;
    }

    /**
     * @brief  慢环功率预算更新 (每 BUDGET_UPDATE_PERIOD_MS 执行一次)
     *         综合考虑: 裁判系统功率限制、超电额外功率、Buffer缓冲、实测功率保护
     * @return 本周期的功率预算 (W)
     */
    float PowerBudget::update()
    {
        uint32_t now = HAL_GetTick();

        /* 未到更新周期, 返回上次结果 */
        if ((now - last_update_tick_) < kBudgetUpdatePeriodMs)
        {
            return last_budget_;
        }
        last_update_tick_ = now;

        /* 读取裁判系统功率限制 */
        float power_referee = (float)Referee_data.Chassis_Power_Limit;
        if (power_referee <= 0.0f)
        {
            last_budget_ = 0.0f;
            return 0.0f;
        }
        if (power_referee < kPowerLimitRefereeMin)
        {
            power_referee = kPowerLimitRefereeMin;
        }

        /* 安全裕度: 留出余量避免超功率 */
        float power_sustained = power_referee * kPowerSafetyMargin;

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
            float buffer_energy = (float)Referee_data.Buffer_Energy;

            if (voltage < kCapProtectVol)
            {
                /* 超电电压过低，立即关闭额外功率 */
                cap_extra_power_ = 0.0f;
            }
            else if (voltage > kCapEnableVol && buffer_energy > kBufferTarget)
            {
                /* 电压和缓冲都充足，缓慢释放额外功率 */
                cap_extra_power_ += kReleaseRate ;
            }
            else
            {
                /* 条件不够好时，慢慢收回额外功率，避免一直占着预算 */
                cap_extra_power_ -= kReleaseRate ;
            }

            float cap_max = power_referee * kCapExtraRatio;
            cap_extra_power_ = fp32_constrain(cap_extra_power_, 0.0f, cap_max);
        }

        /* ---- Buffer Energy 慢闭环 ---- */
        {
            float buffer_energy = (float)Referee_data.Buffer_Energy;

            /* 缓冲能量危险: 立即拉满降额 */
            if (buffer_energy < buffer_danger )
            {
                buffer_energy_derating_ = kBufferDeratingMax;
            }
            /* 缓冲偏低: 逐步收紧 */
            else if (buffer_energy < kBufferTarget - kBufferDeadband)
            {
                buffer_energy_derating_ += kTightenRate;
            }
            /* 缓冲恢复: 逐步释放 */
            else if (buffer_energy > kBufferTarget + kBufferDeadband)
            {
                buffer_energy_derating_ -= kReleaseRate;
            }
            buffer_energy_derating_ = fp32_constrain(buffer_energy_derating_, 0.0f, kBufferDeratingMax);
        }

        /* ---- 实测功率保护 ---- */
        {
            float measured_power = cap.Chassis_power;
            if (measured_power > 0.0f)
            {
                power_filtered_ += kPowerLpfAlpha * (measured_power - power_filtered_);
            }

            float over = power_filtered_ - power_sustained;
            if (over > 0.0f)
            {
                power_derating_ += kTightenRate;
            }
            else if (power_filtered_ < power_sustained - 5.0f)
            {
                power_derating_ -= kReleaseRate;
            }

            power_derating_ = fp32_constrain(power_derating_, 0.0f, kPowerDeratingMax);
        }

        /* ---- 汇总: 最终功率预算 ---- */
        float final_limit = power_sustained + cap_extra_power_
                        - buffer_energy_derating_ - power_derating_;
        if (final_limit < kPowerLimitMin)
        {
            final_limit = kPowerLimitMin;
        }

        last_budget_ = final_limit;
        return final_limit;
    }

} 

/* ============================================================
 *  extern "C" 公开接口 (委托给类实例)
 * ============================================================ */

extern "C" void PowerControl_Init(void)
{
    g_lim_wheel.setParams(kWheelK0, kWheelK1, kWheelK2, kWheelK3);
    g_lim_steer.setParams(kSteerK0, kSteerK1, kSteerK2, kSteerK3);
    g_power_budget.reset();
}

extern "C" float PowerControl_BudgetUpdate(void)
{
    return g_power_budget.update();
}

extern "C" void PowerControl_Apply(int16_t drive_cur[4], float drive_spd[4],
                                   int16_t steer_cur[4], float steer_spd[4],
                                   float budget)
{
    /* 预测两组电机的功率 */
    g_lim_steer.predict(steer_cur, steer_spd, true);
    g_lim_wheel.predict(drive_cur, drive_spd, false);

    /* 分配功率: 舵向最多占 kPowerAllocSteerRatio, 剩余给驱动 */
    float steer_need  = g_lim_steer.predict_power;
    float steer_limit = budget * kPowerAllocSteerRatio;
    float wheel_limit = budget - fminf(steer_need, steer_limit);
    if (wheel_limit < 0.0f)
    {
        wheel_limit = 0.0f;
    }

    /* 分别限制舵向和驱动的电流 */
    g_lim_steer.applyLimit(steer_limit, steer_cur, steer_spd, true);
    g_lim_wheel.applyLimit(wheel_limit, drive_cur, drive_spd, false);
}

