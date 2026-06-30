/**
 * @file    HeatControl.cpp
 * @brief   拨弹盘热量控制模块 — 实现
 * @author  Nas (1319621819@qq.com) 
 *
 * @details 包含:
 *          1. HeatModel 类 — 本地热量预测 + smoothstep 速度系数
 *          2. extern "C" 接口 — 供 FreeRTOS C 代码调用
 *
 * 数据流：
 *   裁判系统 UART (50Hz/10Hz/3Hz)
 *     -> Referee_data.{Heat_Limit, Barrel_Heat, CooLing_Value, cooling_buff}
 *       -> 本地热量预测: heat_predicted = referee_heat + bullets*10 - C_eff*dt
 *         -> margin = heat_limit - HEAT_RESERVE - heat_predicted
 *           -> speed_factor = smoothstep(margin / (C_eff * T_window))
 *             -> trigger_speed = base_speed * speed_factor
 */
#include <cmath>
#include "HeatControl.hpp"
#include "referee_system.hpp"

/* ============ 热量控制核心参数 ============ */
namespace 
{
    constexpr float    kHeatValidLimitMin   = 1.0f;      // Heat_Limit 最小有效值
    constexpr float    kHeatValidCoolingMin  = 1.0f;      // CooLing_Value 最小有效值
    constexpr float    kHeatReserve          = 15.0f;     // 预留安全热量余量 (绝对值)
    constexpr float    kHeatWindowSec        = 1.5f;      // 前瞻时间窗口 (秒)
    constexpr float    kHeatKMin             = 0.15f;     // 热量极限时保底速度比例
    constexpr float    kHeatPerBullet17mm    = 10.0f;     // 每发 17mm 弹丸产生的热量值
} 

/* ============ 全局实例 ============ */
robot::heat_control::HeatModel g_heat_model;

namespace robot::heat_control 
{

float HeatModel::clamp(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief  smoothstep: 输入 0~1, 输出 0~1, 两端斜率为 0
 *         公式: 3x? - 2x?
 */
float HeatModel::smoothStep(float x)
{
    x = clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

bool HeatModel::dataValid(float heat_limit, float cooling_value)
{
    return (heat_limit >= kHeatValidLimitMin) && (cooling_value >= kHeatValidCoolingMin);
}

void HeatModel::init()
{
    heat_predicted     = 0.0f;
    heat_ref_snapshot  = 0.0f;
    allowance_snapshot = 0;
    effective_cooling  = 0.0f;
    heat_limit         = 0.0f;
    speed_factor       = 1.0f;
    enabled            = false;
}

/**
 * @brief  同步裁判系统热量数据到本地预测模型
 *
 * 当裁判数据刷新 (Barrel_Heat 变化) 时自动校准:
 * 1. 用裁判的真实热量值替换本地预测值, 消除累积误差
 * 2. 通过 allowance 变化检测新弹丸发出, 累加到本地预测
 * 3. 融合 cooling_buff 得到有效冷却速率
 */
void HeatModel::syncFromReferee()
{
    float referee_heat_limit = (float)Referee_data.Heat_Limit;
    float barrel_heat        = (float)Referee_data.Barrel_Heat;
    float base_cooling       = (float)Referee_data.CooLing_Value;
    float cooling_buff       = (float)Referee_data.cooling_buff;
    uint16_t allowance       = Referee_data.projectile_allowance_17mm;

    /* 有效冷却 = 基础冷却 + 冷却增盋buff (能量机关/姿态等) */
    effective_cooling = base_cooling + cooling_buff;
    heat_limit = referee_heat_limit;

    /* 检查裁判数据有效性 */
    if (!dataValid(referee_heat_limit, base_cooling))
    {
        enabled = false;
        return;
    }
    enabled = true;

    /*
     * 裁判热量校准: 当裁判报告的 Barrel_Heat 与快照不同时,
     * 说明裁判系统发来了新数据帧, 用它校准本地预测
     */
    if (barrel_heat != heat_ref_snapshot)
    {
        heat_predicted     = barrel_heat;
        heat_ref_snapshot  = barrel_heat;
        allowance_snapshot = allowance;
        return;
    }

    /*
     * 两帧裁判数据之间, 通过 allowance 减少量检测新弹丸
     * allowance 每减少 1 表示打出 1 发弹丸
     */
    if (allowance < allowance_snapshot)
    {
        uint16_t bullets_fired = allowance_snapshot - allowance;
        heat_predicted += (float)bullets_fired * kHeatPerBullet17mm;
        allowance_snapshot = allowance;
    }

    /*
     * 本地冷却衰减: 每控制周期减去 C_eff * dt
     * dt ≈ 0.001s (1ms RTOS任务)
     */
    heat_predicted -= effective_cooling * 0.001f;
    if (heat_predicted < 0.0f)
    {
        heat_predicted = 0.0f;
    }
}

/**
 * @brief  计算热量速度系数
 *
 * margin = heat_limit - HEAT_RESERVE - heat_predicted
 * ratio  = margin / (C_eff * T_window)
 * factor = K_MIN + (1 - K_MIN) * smoothstep(ratio)
 */
float HeatModel::calcSpeedFactor() const
{
    if (!enabled)
    {
        /* 裁判数据无效时旁路热量控制 */
        return 1.0f;
    }

    float margin          = heat_limit - kHeatReserve - heat_predicted;
    float window_capacity = effective_cooling * kHeatWindowSec;

    /* 防止除零 */
    if (window_capacity < 0.1f)
    {
        window_capacity = 0.1f;
    }

    float ratio = clamp(margin / window_capacity, 0.0f, 1.0f);
    return kHeatKMin + (1.0f - kHeatKMin) * smoothStep(ratio);
}

void HeatModel::update()
{
    syncFromReferee();
    speed_factor = calcSpeedFactor();
}

float HeatModel::getControlledSpeed(float base_speed) const
{
    if (fabsf(base_speed) < 1.0f)
    {
        return 0.0f;
    }
    return base_speed * speed_factor;
}

} 

/* ============================================================
 *  extern "C" 公开接口 (委托给类实例)
 * ============================================================ */

extern "C" void HeatControl_Init(void)
{
    g_heat_model.init();
}

extern "C" void HeatControl_Update(void)
{
    g_heat_model.update();
}

extern "C" float HeatControl_GetFactor(void)
{
    return g_heat_model.speed_factor;
}

extern "C" float HeatControl_GetSpeed(float base_speed)
{
    return g_heat_model.getControlledSpeed(base_speed);
}

extern "C" uint8_t HeatControl_IsEnabled(void)
{
    return g_heat_model.enabled ? 1u : 0u;
}

