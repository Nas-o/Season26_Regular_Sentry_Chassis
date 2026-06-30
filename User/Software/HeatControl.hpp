/**
 * @file    HeatControl.hpp
 * @brief   拨弹盘热量控制模块 — 公开接口
 * @author  Nas 
 *
 * @details 使用 C++ class 封装热量预测模型,
 *          同时保留 extern "C" 接口供 RTOS 调用.
 *
 * @note    热量控制模型 V2:
 *          使用 smoothstep 连续曲线代替状态机:
 *          margin = limit - reserve - predicted
 *          ratio  = margin / (C_eff * T_window)
 *          k_heat = K_MIN + (1 - K_MIN) * smoothstep(ratio)
 */
#ifndef __HEAT_CONTROL_H__
#define __HEAT_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/**
 * @brief  初始化热量控制模块
 *         清零预测模型状态
 */
void HeatControl_Init(void);

/**
 * @brief  每控制周期调用, 同步裁判数据并衰减本地热量预测
 * @note   Shoot_Tasks 中每 1ms 调用一次
 */
void HeatControl_Update(void);

/**
 * @brief  获取本周期热量速度系数
 * @return k_heat ∈ [HEAT_K_MIN, 1.0], 裁判数据无效时返回 1.0
 */
float HeatControl_GetFactor(void);

/**
 * @brief  直接获取热量限速后的拨弹速度
 * @param  base_speed  原始目标速度
 * @return 限速后的速度值
 */
float HeatControl_GetSpeed(float base_speed);

/**
 * @brief  查询热量控制是否启用 (裁判数据有效)
 * @return 1=启用, 0=未启用(旁路)
 */
uint8_t HeatControl_IsEnabled(void);

#ifdef __cplusplus
}
#endif

/* =================== C++ Only =================== */
#ifdef __cplusplus

namespace robot::heat_control {

/* ==================================================================
 * HeatModel — 热量预测与限速控制器
 * ================================================================== */
class HeatModel {
public:
    /* 公开状态 (供外部观测/调试) */
    float    heat_predicted      = 0.0f;  // 本地预测当前热量
    float    heat_ref_snapshot   = 0.0f;  // 上次裁判同步时的热量快照
    uint16_t allowance_snapshot  = 0;     // 上次裁判同步时的允许发弹量快照
    float    effective_cooling   = 0.0f;  // 有效冷却 = base_cooling + cooling_buff
    float    heat_limit          = 0.0f;  // 当前热量上限快照
    float    speed_factor        = 1.0f;  // 本周期热量速度系数 [K_MIN, 1.0]
    bool     enabled             = false; // 热量控制是否启用

    /* 初始化 (清零所有状态) */
    void init();

    /* 每控制周期更新: 同步裁判数据 + 冷却衰减 + 计算速度系数 */
    void update();

    /* 获取限速后的拨弹速度 */
    float getControlledSpeed(float base_speed) const;

private:
    /* 同步裁判系统热量数据到本地模型 */
    void syncFromReferee();

    /* 计算速度系数 */
    float calcSpeedFactor() const;

    /* 工具函数 */
    static float clamp(float value, float min_val, float max_val);
    static float smoothStep(float x);
    static bool  dataValid(float heat_limit, float cooling_value);
};

} // namespace robot::heat_control

/* 全局实例 */
extern robot::heat_control::HeatModel g_heat_model;

#endif /* __cplusplus */
#endif /* __HEAT_CONTROL_H__ */
