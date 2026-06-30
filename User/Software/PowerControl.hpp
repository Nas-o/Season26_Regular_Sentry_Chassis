/**
 * @file    PowerControl.hpp
 * @brief   底盘功率控制模块 — 公开接口
 * @author  Nas (2026-05)
 *
 * @details 将功率预测、功率限制、慢环预算三部分从 Chassis.cpp 中隔离,
 *          使用 C++ class 封装内部状态, 同时保留 extern "C" 接口供 RTOS 调用.
 */
#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#ifdef __cplusplus
extern "C" 
{
#endif

#include "stdint.h"

/**
 * @brief  初始化功率控制模块
 *         设置电机功率模型参数 (k0~k3)、慢环状态清零
 */
void PowerControl_Init(void);

/**
 * @brief  慢环功率预算更新 (建议 20ms 调用一次)
 *         综合裁判系统功率限制、超电电压、Buffer能量,
 *         输出本周期可用功率预算
 * @return 本周期功率预算 (W), <=0 表示不限制
 */
float PowerControl_BudgetUpdate(void);

/**
 * @brief  对底盘电流执行功率限幅
 *         内部按舵向/驱动分配比例分别限制
 * @param  drive_cur   驱动电机电流数组 [4] (in/out)
 * @param  drive_spd   驱动电机转速数组 [4] (rad/s)
 * @param  steer_cur   舵向电机电流数组 [4] (in/out)
 * @param  steer_spd   舵向电机转速数组 [4] (rad/s)
 * @param  budget      功率预算 (W)
 */
void PowerControl_Apply(int16_t drive_cur[4], float drive_spd[4],
                        int16_t steer_cur[4], float steer_spd[4],
                        float budget);

#ifdef __cplusplus
}
#endif

/* =================== C++ Only =================== */
#ifdef __cplusplus

#include "User_math.hpp"

namespace robot::power_control 
{

/* ==================================================================
 * PowerLimiter — 单组电机 (4个) 的功率限制器
 * ================================================================== */
class PowerLimiter 
{
public:
    /* 电机功率模型参数 */
    float k0 = 0, k1 = 0, k2 = 0, k3 = 0;

    /* 运行时状态 (供外部调试/观测) */
    float limit_power   = 0;    // 当前生效的功率上限
    float predict_power = 0;    // 预测功率
    float scaling       = 1.0f; // 缩放系数 (0~1)

    /* 设置电机参数 */
    void setParams(float _k0, float _k1, float _k2, float _k3);

    /* 预测本组 4 个电机的总功率 */
    void predict(int16_t current[4], float speed[4], bool is_steer);

    /* 根据功率预算限幅本组电流 */
    void applyLimit(float budget, int16_t current[4], float speed[4], bool is_steer);

private:
    /* 估算单个电机的机械功率 */
    float estimateMotorPower(int16_t current_raw, float speed_rad, bool is_steer) const;

    /* 电流原始值 → 安培数转换 */
    static float rawToAmpere(bool is_steer);
};

/* ==================================================================
 * PowerBudget — 慢环功率预算管理器
 * ================================================================== */
class PowerBudget 
{
public:
    void reset();
    float update();

private:
    /* 超电额外功率调节 */
    float cap_extra_power_        = 0.0f;
    float cap_voltage_last_       = 0.0f;

    /* Buffer Energy 慢闭环 */
    float buffer_energy_derating_ = 0.0f;

    /* 实测功率保护 */
    float power_derating_         = 0.0f;
    float power_filtered_         = 0.0f;

    /* 定时控制 */
    uint32_t last_update_tick_    = 0;
    float    last_budget_         = 60.0f;
};

} // namespace robot::power_control

/* 全局实例 */
extern robot::power_control::PowerLimiter g_lim_wheel;
extern robot::power_control::PowerLimiter g_lim_steer;
extern robot::power_control::PowerBudget  g_power_budget;

#endif /* __cplusplus */
#endif /* __POWER_CONTROL_H__ */

