/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       shoot.c/h
  * @brief      发射机构控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  * Version    Date            Author          Modification
  * V1.0.0   2025.10.31       Wang Zihao       1.重新构建发射机构代码结构
  * V2.0.0   2026.04.29       Copilot          2.重构热量控制为连续预测模型
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#include <stdlib.h>
#include "Shoot.hpp"
#include "Global_status.hpp"
#include "Gimbal.hpp"
#include "referee_system.hpp"

Shoot_t Shoot;

/*
 * ======================== 热量控制模块说明 V2 ========================
 * 模块目标：
 * 1. 在连续拨弹模式下，根据裁判系统热量信息自动、连续地调整拨弹速度。
 * 2. 热量余量大 -> 拨弹快；热量余量小 -> 拨弹慢；极限 -> 保底微转。
 * 3. 通过本地热量预测模型补偿裁判数据延迟（50Hz/20ms）。
 * 4. 通过融合 cooling_buff 自动适配姿态切换、能量机关激活等场景。
 *
 * 数据流：
 *   裁判系统 UART (50Hz/10Hz/3Hz)
 *     -> Referee_data.{Heat_Limit, Barrel_Heat, CooLing_Value, cooling_buff}
 *       -> 本地热量预测: heat_predicted = referee_heat + bullets*10 - C_eff*dt
 *         -> margin = heat_limit - HEAT_RESERVE - heat_predicted
 *           -> k_heat = smoothstep(margin / (C_eff * T_window))
 *             -> trigger_speed_set = base_speed * k_heat
 *
 * 核心公式：
 *   effective_cooling = CooLing_Value + cooling_buff
 *   margin = Heat_Limit - HEAT_RESERVE - heat_predicted
 *   ratio = margin / (effective_cooling * HEAT_WINDOW_SEC)
 *   k_heat = HEAT_K_MIN + (1 - HEAT_K_MIN) * smoothstep(clamp(ratio, 0, 1))
 *   speed_final = speed_base * k_heat
 *
 * 优先级：堵转反转脱困 > 热量调速 > 基础模式速度
 * =================================================================
 */

/* ======================== 工具函数 ======================== */

static float Shoot_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float Shoot_ClampFloat(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/*
 * smoothstep: 输入 0~1，输出 0~1，两端斜率为 0，过渡平滑。
 * 公式: 3x^2 - 2x^3
 */
static float Shoot_SmoothStep01(float x)
{
    x = Shoot_ClampFloat(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static float Shoot_SignNonZero(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

/* ======================== 基础速度查表 ======================== */

/*
 * 根据上层拨弹模式得到"基础速度"。
 * 只表达模式本身想要的快慢，不做任何热量限幅。
 */
static float Shoot_GetBaseSpeed(uint8_t trigger_mode)
{
    switch (trigger_mode)
    {
    case HIGH:
        return (float)TRIGGER_SPEED_HIGH;
    case MID:
        return (float)TRIGGER_SPEED_MID;
    case LOW:
        return (float)TRIGGER_SPEED_LOW;
    case DEBUG_TRIGGER:
        return (float)TRIGGER_SPEED_DEBUG;
    case ONLY_AIM_HIGH:
        return (float)TRIGGER_SPEED_AIM_HIGH;
    case ONLY_AIM_MID:
        return (float)TRIGGER_SPEED_AIM_MID;
    case ONLY_AIM_LOW:
        return (float)TRIGGER_CLOSE;
    case TRIGGER_CLOSE:
    default:
        return 0.0f;
    }
}

/* ======================== 热量预测模型 ======================== */

/*
 * 判断裁判热量数据是否有效。
 * Heat_Limit 和 CooLing_Value 都大于最小阈值才认为有效。
 */
static uint8_t Shoot_HeatDataValid(float heat_limit, float cooling_value)
{
    return (heat_limit >= HEAT_VALID_LIMIT_MIN) && (cooling_value >= HEAT_VALID_COOLING_MIN);
}

/*
 * 同步裁判系统热量快照到本地预测模型。
 * 每个控制周期都调用，当裁判数据刷新（Barrel_Heat 变化）时自动校准：
 * 1. 用裁判的真实热量值替换本地预测值，消除累积误差。
 * 2. 通过 allowance 变化检测新弹丸发出，累加到本地预测。
 * 3. 融合 cooling_buff 得到有效冷却速率。
 */
static void Shoot_SyncHeatModel(heat_model_t *model)
{
    float heat_limit    = (float)Referee_data.Heat_Limit;
    float barrel_heat   = (float)Referee_data.Barrel_Heat;
    float base_cooling  = (float)Referee_data.CooLing_Value;
    float cooling_buff  = (float)Referee_data.cooling_buff;
    uint16_t allowance  = Referee_data.projectile_allowance_17mm;

    /* 有效冷却 = 基础冷却 + 冷却增益buff（能量机关/姿态等带来的额外冷却） */
    model->effective_cooling = base_cooling + cooling_buff;
    model->heat_limit = heat_limit;

    /* 检查裁判数据有效性 */
    if (!Shoot_HeatDataValid(heat_limit, base_cooling))
    {
        model->heat_control_enabled = 0;
        return;
    }
    model->heat_control_enabled = 1;

    /*
     * 裁判热量校准：当裁判报告的 Barrel_Heat 与我们的快照不同时，
     * 说明裁判系统发来了新数据帧，用它校准本地预测。
     */
    if (barrel_heat != model->heat_ref_snapshot)
    {
        model->heat_predicted = barrel_heat;
        model->heat_ref_snapshot = barrel_heat;
        model->allowance_snapshot = allowance;
        return;
    }

    /*
     * 两帧裁判数据之间，通过 allowance 减少量检测新弹丸。
     * allowance 每减少 1 表示打出 1 发弹丸，累加 10 热量。
     */
    if (allowance < model->allowance_snapshot)
    {
        uint16_t bullets_fired = model->allowance_snapshot - allowance;
        model->heat_predicted += (float)bullets_fired * HEAT_PER_BULLET_17MM;
        model->allowance_snapshot = allowance;
    }

    /*
     * 本地冷却衰减：每控制周期减去 C_eff * dt。
     * Shoot_Tasks 在 1ms RTOS 任务中调用，dt ≈ 0.001s。
     * 冷却不会把热量降到 0 以下。
     */
    model->heat_predicted -= model->effective_cooling * 0.001f;
    if (model->heat_predicted < 0.0f)
    {
        model->heat_predicted = 0.0f;
    }
}

/*
 * 计算热量速度系数 k_heat ∈ [HEAT_K_MIN, 1.0]。
 *
 * 工作原理：
 * 1. margin = heat_limit - HEAT_RESERVE - heat_predicted
 *    代表"还能承受多少热量"。
 * 2. ratio = margin / (C_eff * T_window)
 *    代表"未来 T_window 秒内，当前余量相对于冷却能力的充裕度"。
 *    ratio >= 1 表示余量充足，ratio <= 0 表示余量耗尽。
 * 3. k_heat 通过 smoothstep 平滑映射：
 *    ratio = 0 -> k_heat = HEAT_K_MIN（保底微转）
 *    ratio = 1 -> k_heat = 1.0（全速）
 *    中间段平滑过渡，无速度跳变。
 *
 * 为什么参数变化时自动适配：
 * - 姿态切换导致 Heat_Limit 或 CooLing_Value 变化 → margin 和 C_eff 直接变化
 * - 能量机关激活导致 cooling_buff 增加 → C_eff 增大 → ratio 增大 → 自动提速
 * - 等级提升导致冷却/热量上限变化 → 裁判 robot_status 直接反映到 CooLing_Value/Heat_Limit
 * 所有这些都不需要额外的 if/else 或查表，模型自然适配。
 */
static float Shoot_CalcHeatFactor(const heat_model_t *model)
{
    float margin;
    float window_capacity;
    float ratio;

    if (!model->heat_control_enabled)
    {
        /* 裁判数据无效时旁路热量控制，返回 1.0 表示不限速 */
        return 1.0f;
    }

    margin = model->heat_limit - HEAT_RESERVE - model->heat_predicted;
    window_capacity = model->effective_cooling * HEAT_WINDOW_SEC;

    /* 防止除零 */
    if (window_capacity < 0.1f)
    {
        window_capacity = 0.1f;
    }

    ratio = margin / window_capacity;
    ratio = Shoot_ClampFloat(ratio, 0.0f, 1.0f);

    return HEAT_K_MIN + (1.0f - HEAT_K_MIN) * Shoot_SmoothStep01(ratio);
}

/*
 * 根据热量系数得到最终拨弹速度。
 * base_speed 是模式想要的速度，k_heat 是热量允许的比例。
 * 当 base_speed 接近 0 时直接返回 0，避免保底速度在停转模式下仍输出。
 */
static float Shoot_GetHeatControlledSpeed(float base_speed, float k_heat)
{
    if (Shoot_AbsFloat(base_speed) < 1.0f)
    {
        return 0.0f;
    }
    return base_speed * k_heat;
}

/*========================== 单发限位 ==========================*/

static uint8_t Shoot_IsSpeedMode(uint8_t mode)
{
    switch (mode)
    {
    case HIGH:
    case MID:
    case LOW:
    case DEBUG_TRIGGER:
    case ONLY_AIM_HIGH:
    case ONLY_AIM_MID:
    case ONLY_AIM_LOW:
        return 1;
    default:
        return 0;
    }
}

static void Shoot_StopFireFSM_Update(void)
{
    if (Shoot.stop_fire_state == STOP_FIRE_ACTIVE)
    {
        uint32_t elapsed = HAL_GetTick() - Shoot.stop_fire_tick;
        int32_t ecd_err = Shoot.trigger_target_ecd - Shoot.trigger_ecd_now;

        if (ecd_err < 0)
            ecd_err = -ecd_err;

        if (ecd_err <= SINGLE_DONE_ECD_TH ||
            elapsed >= STOP_FIRE_TIMEOUT_MS)
        {
            Shoot.stop_fire_state = STOP_FIRE_PROCESS;
        }
    }

    if (Shoot.stop_fire_state == STOP_FIRE_PROCESS)
    {
        Shoot.trigger_target_ecd = Shoot.trigger_ecd_now;
        Shoot.stop_fire_state = STOP_FIRE_DISABLE;
    }
}

/* ======================== 对外接口 ======================== */

void Shoot_Init(void)
{
    TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);
    PID_Set(&Shoot.trigger_speed_pid, 8, 0, 3, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    PID_Set(&Shoot.trigger_angle_pid, 0.20f, 0.0f, 0.0f, 0.0f, TRIGGER_ANGLE_SPEED_MAX, 0.0f);
    Shoot.trigger_speed_base = 0.0f;
    Shoot.trigger_speed_set = 0.0f;

    /* 热量预测模型初始化 */
    Shoot.heat_model.heat_predicted = 0.0f;
    Shoot.heat_model.heat_ref_snapshot = 0.0f;
    Shoot.heat_model.allowance_snapshot = 0;
    Shoot.heat_model.effective_cooling = 0.0f;
    Shoot.heat_model.heat_limit = 0.0f;
    Shoot.heat_model.k_heat = 1.0f;
    Shoot.heat_model.heat_control_enabled = 1;

    Shoot.trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
    Shoot.stop_fire_state = STOP_FIRE_DISABLE;
    Shoot.trigger_target_ecd = 0;
    Shoot.stop_fire_tick = 0;
}

void Shoot_Updater(void)
{
    uint8_t mode = Global.Shoot.trigger_mode;
    Shoot.trigger_speed_now = TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm;
    Shoot.trigger_current_now = TriggerMotor_get_data(TRIGGER_MOTOR).given_current;
    Shoot.trigger_ecd_now = (int32_t)TriggerMotor_get_data(TRIGGER_MOTOR).ecd_cnt;

    if (mode == SINGLE && Shoot.last_trigger_mode != SINGLE)
    {
        Shoot.trigger_ctrl_mode = TRIGGER_CTRL_ANGLE;
        Shoot.stop_fire_state = STOP_FIRE_ACTIVE;
        Shoot.stop_fire_tick = HAL_GetTick();

        Shoot.trigger_target_ecd = Shoot.trigger_ecd_now
                                 + TRIGGER_ONE_BULLET_ECD
                                 + TRIGGER_EXTRA_ECD;
    }
    else if (Shoot_IsSpeedMode(mode))
    {
        Shoot.trigger_ctrl_mode = TRIGGER_CTRL_SPEED;
        Shoot.stop_fire_state = STOP_FIRE_DISABLE;
    }

    Shoot_StopFireFSM_Update();

    Shoot.last_trigger_mode = mode;
    Shoot.trigger_speed_base = Shoot_GetBaseSpeed(Global.Shoot.trigger_mode);

    /* 每周期同步裁判数据并更新本地热量预测 */
    Shoot_SyncHeatModel(&Shoot.heat_model);
    Shoot.heat_model.k_heat = Shoot_CalcHeatFactor(&Shoot.heat_model);
}

void Shoot_Calculator(void)
{
    if (Shoot.trigger_ctrl_mode == TRIGGER_CTRL_ANGLE)
    {
        float speed_ref = PID_Cal(&Shoot.trigger_angle_pid,
                                  (float)Shoot.trigger_ecd_now,
                                  (float)Shoot.trigger_target_ecd);

        speed_ref = Shoot_ClampFloat(speed_ref,
                                     -TRIGGER_ANGLE_SPEED_MAX,
                                     TRIGGER_ANGLE_SPEED_MAX);

        Shoot.trigger_speed_set = speed_ref;
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid,
                                   Shoot.trigger_speed_now,
                                   speed_ref);
        return;
    }
    /* 用热量系数连续调节拨弹速度 */
    Shoot.trigger_speed_set = Shoot_GetHeatControlledSpeed(Shoot.trigger_speed_base, Shoot.heat_model.k_heat);

    /* 堵转反转脱困逻辑（优先级高于热量调速） */
    static int trigger_kill_cnt = 0;
    if (Global.Shoot.trigger_mode == TRIGGER_CLOSE &&
        Shoot_AbsFloat(Shoot.trigger_speed_set) < 1.0f)
    {
        trigger_kill_cnt = 0;
    }
    else if (Shoot_AbsFloat(Shoot.trigger_current_now) > 10000.0f)
    {
        trigger_kill_cnt = 50;
    }

    if (trigger_kill_cnt > 1)
    {
        /* 堵转反转脱困 */
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, -3000.0f);
        trigger_kill_cnt--;
    }
    else
    {
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, Shoot.trigger_speed_set);
    }
}

void Shoot_Controller(void)
{
    SHOOTMotor_set(Shoot.current[2], TRIGGER_MOTOR);
}

void Shoot_Tasks(void)
{
#if (USE_SHOOT != 0)
    Shoot_Updater();
    Shoot_Calculator();
    Shoot_Controller();
#endif
}
