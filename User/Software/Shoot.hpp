/*
 * @Date: 2025-10-27 21:03:17
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-03 00:39:53
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Software\Shoot.h
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "CAN_receive_send.hpp"
#include "pid.hpp"

#include "motor.hpp"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,// 开始反转值
    SHOOT_SPEED_CLOSE = 0,    // 停止速度值
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,  // 正常工作值
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG, // 退弹低速值
};

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,
    TRIGGER_SPEED_AIM_HIGH = 7680,
    TRIGGER_SPEED_AIM_MID = 2400,
    TRIGGER_SPEED_AIM_LOW = 1200,
};

/*
 * ======================== 热量控制模型 V2 ========================
 * 设计思想：
 * 1. 用"本地热量预测"补偿裁判系统数据延迟（50Hz = 20ms 间隔）。
 * 2. 用"有效冷却 = 基础冷却 + 冷却增益buff"自动适配姿态切换/能量机关激活等场景。
 * 3. 用单一连续曲线 k_heat = smoothstep(margin / (C_eff * T_window))
 *    代替旧的 5 级状态机，余量多速度快、余量少速度慢，天然平滑。
 * 4. 仅需 3 个可调参数：HEAT_RESERVE、HEAT_WINDOW_SEC、HEAT_K_MIN。
 * =================================================================
 */

/* ------ 热量控制核心参数 ------ */
#define HEAT_VALID_LIMIT_MIN    1.0f      // Heat_Limit 最小有效值，低于此值视为裁判数据无效
#define HEAT_VALID_COOLING_MIN  1.0f      // CooLing_Value 最小有效值
#define HEAT_RESERVE            15.0f     // 预留安全热量余量（绝对值），防止逼近上限
#define HEAT_WINDOW_SEC         1.5f      // 前瞻时间窗口（秒），margin / (C_eff * window) 作为 smoothstep 输入
#define HEAT_K_MIN              0.15f     // 热量极限时保底速度比例（占 base_speed），保持拨盘微转不停
#define HEAT_PER_BULLET_17MM    10.0f     // 每发 17mm 弹丸产生的热量值（裁判系统固定值）
/* ------ 单发限位 ------ */
#define A_BULLET_ECD 85599
#define TRIGGER_ONE_BULLET_ECD   ((int32_t)A_BULLET_ECD)
#define TRIGGER_EXTRA_ECD        (150)
#define SINGLE_DONE_ECD_TH     1200
#define STOP_FIRE_TIMEOUT_MS     2000U
#define TRIGGER_ANGLE_SPEED_MAX  6000.0f

/* ------ 热量预测模型数据结构 ------ */
typedef struct
{
    float    heat_predicted;       // 本地预测当前热量
    float    heat_ref_snapshot;    // 上次裁判同步时的热量快照
    uint16_t allowance_snapshot;   // 上次裁判同步时的允许发弹量快照（用于检测新弹丸发出）
    float    effective_cooling;    // 有效冷却 = base_cooling + cooling_buff
    float    heat_limit;           // 当前热量上限快照
    float    k_heat;               // 本周期计算出的热量速度系数 [HEAT_K_MIN, 1.0]
    uint8_t  heat_control_enabled; // 热量控制是否启用（裁判数据有效时为 1）
} heat_model_t;

/* 内部调用 */
#define SHOOTMotor_init(type, id)    DJIMotor_Init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_Set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_Init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_Set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_GetData(id)
#define SHOOTMotor_get_data(id)      DJIMotor_GetData(id)

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

    /* 停火有限状态机 */
    typedef enum
    {
        STOP_FIRE_DISABLE = 0,
        STOP_FIRE_ACTIVE,
        STOP_FIRE_PROCESS,
    } stop_fire_state_e;

    typedef enum
    {
        TRIGGER_CTRL_SPEED = 0,
        TRIGGER_CTRL_ANGLE,
    } trigger_ctrl_mode_e;

/*内部数据类型*/
typedef struct
{
    /*-------PID-------*/
    pid_t shoot_L_speed_pid;
    pid_t shoot_R_speed_pid;
    pid_t trigger_speed_pid;
    pid_t trigger_angle_pid;

    /*-------状态量-------*/
    float shoot_speed_L_now;
    float shoot_speed_R_now;
    float trigger_speed_now;
    float trigger_current_now;

    /*-------目标量-------*/
    float trigger_speed_base;
    float trigger_speed_set;
    enum shoot_speed_e shoot_speed_set;
    float current[3];


    /*-------单发限位-------*/
    int32_t trigger_ecd_now;
    int32_t trigger_target_ecd;
    uint32_t stop_fire_tick;
    uint8_t last_trigger_mode;
    trigger_ctrl_mode_e trigger_ctrl_mode;
    stop_fire_state_e stop_fire_state;

    /*-------热量预测模型-------*/
    heat_model_t heat_model;

}Shoot_t;

/* 外部调用 */
void Shoot_Init();
void Shoot_Tasks();

#ifdef __cplusplus
}
#endif
#endif