/*
  ****************************(C) COPYRIGHT 2026 ADAM****************************
  * @file       shoot.c/h
  * @brief      发射机构控制器
  * @note       包括初始化，数据更新、控制量计算与直接控制量设置
  * @history
  * Version    Date            Author          Modification
  * V1.0.0   2025.10.31       Wang Zihao       1.重新构建发射机构代码结构
  * V1.1.0   2026.02.16       Gemini           2.优化单发限位逻辑，增加闭环锁止与防卡弹
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ADAM****************************
*/

#include <stdlib.h>
#include "Shoot.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "referee_system.h"

Shoot_t Shoot;

/*
 * ======================== 热量控制模块说明 ========================
 * 模块目标：
 * 1. 在连续拨弹模式下，根据裁判系统提供的热量信息自动调整拨弹速度。
 * 2. 实现“热量余量大时转得快，热量余量小时转得慢，接近上限时仍保持低速转动”的控制目标。
 * 3. 通过预留HeatReserve安全余量，尽量避免把热量推到必须停转的边界。
 * 4. 不使用拨盘孔数、齿轮比等机械模型参数参与热量决策，仅使用热量域数据进行控制。
 *
 * 输入数据：
 * 1. Referee_data.Barrel_Heat   -> 当前枪口热量
 * 2. Referee_data.Heat_Limit    -> 枪口热量上限
 * 3. Referee_data.CooLing_Value -> 每秒冷却值
 * 4. Global.Shoot.tigger_mode   -> 上层给出的拨弹模式
 *
 * 输出数据：
 * 1. Shoot.trigger_speed_base -> 模式本身想要的基础速度
 * 2. Shoot.trigger_speed_set  -> 经过热量状态机修正后的最终速度参考值
 *
 * 状态说明：
 * 1. HEAT_BYPASS
 *    未接收到有效裁判热量数据时，旁路热量控制，直接输出基础速度。
 *    这是当前项目约定的预期行为，不属于保护模式。
 * 2. HEAT_BOOST
 *    热量余量充足，允许按基础速度运行。
 * 3. HEAT_TRACK
 *    热量进入受控利用区，通过平滑插值从BOOST速度过渡到KEEPALIVE速度。
 * 4. HEAT_KEEPALIVE
 *    热量偏紧张时维持低速保活，既尽量保持拨弹连续性，又限制热量继续快速上涨。
 * 5. HEAT_GUARD
 *    热量已经逼近安全余量时，进一步降低到极低保底速度，但不主动停转。
 *
 * 工作顺序：
 * 1. 先由 Shoot_GetBaseSpeed() 生成基础速度。
 * 2. 再由热量状态机决定当前处于哪个热量区间。
 * 3. 最后由 Shoot_GetHeatControlledSpeed() 输出最终的速度参考。
 *
 * 优先级关系：
 * 1. 单发控制优先级最高，不受热量状态机覆盖。
 * 2. 堵转反转脱困优先级高于常规热量调速。
 * 3. 热量状态机只作用于连续拨弹模式。
 * 4. 热量控制只负责“降速”，主动停转仍只由TRIGGER_CLOSE、SINGLE和故障逻辑触发。
 * =================================================================
 */

/* 返回浮点数绝对值，统一热量控制里的速度、电流比较口径。 */
static float Shoot_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/* 将输入夹紧到[min_value, max_value]，避免保活速度和插值系数越界。 */
static float Shoot_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/*
 * 平滑阶跃函数，输入0~1，输出0~1。
 * 与线性插值相比，它在两端斜率为0，能降低TRACK区间切换时的速度顿挫。
 */
static float Shoot_SmoothStep01(float value)
{
    value = Shoot_ClampFloat(value, 0.0f, 1.0f);
    return value * value * (3.0f - 2.0f * value);
}

// 0 速场景统一返回正方向，调用方需要先保证零速不会直接参与调速计算。
static float Shoot_SignNonZero(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

/*
 * 根据上层拨弹模式得到“基础速度”。
 * 这一步只表达模式本身想要的快慢，不做任何热量限幅，热量状态机在后级再二次修正。
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
    case SINGLE:
    default:
        return 0.0f;
    }
}

/*
 * 判断本周期裁判热量数据是否有效。
 * 这里使用调用方传入的单次快照，避免在同一控制周期里重复读取全局数据造成前后不一致。
 */
static uint8_t Shoot_HeatControlAvailable(float heat_limit, float cooling_value)
{
    if (heat_limit < HEAT_VALID_LIMIT_MIN)
    {
        return 0;
    }

    if (cooling_value < HEAT_VALID_COOLING_MIN)
    {
        return 0;
    }

    return 1;
}

/*
 * 当状态机刚从BYPASS接入热量控制时，按当前热量余量直接归位到目标状态。
 * 这能避免“默认先进入BOOST再切换”的瞬态风险。
 */
static trigger_heat_state_e Shoot_GetHeatStateFromMargin(float safe_margin, float cooling_value)
{
    if (safe_margin >= (HEAT_BOOST_ENTER_FACTOR * cooling_value))
    {
        return HEAT_BOOST;
    }

    if (safe_margin >= (HEAT_TRACK_ENTER_FACTOR * cooling_value))
    {
        return HEAT_TRACK;
    }

    if (safe_margin >= (HEAT_KEEP_ENTER_FACTOR * cooling_value))
    {
        return HEAT_KEEPALIVE;
    }

    return HEAT_GUARD;
}

/*
 * 更新热量状态机。
 * 核心思想：
 * 1. 先判断热量数据是否有效；无效时进入BYPASS，旁路热量控制。
 * 2. 数据有效后，先减去HeatReserve安全余量，再根据safe_margin与cooling_value的倍数关系切换状态。
 * 3. 各状态采用不同的进入/退出阈值，形成回差，避免在临界点来回抖动。
 */
static void Shoot_UpdateHeatState(float safe_margin, float cooling_value, uint8_t heat_control_available)
{
    if (!heat_control_available)
    {
        Shoot.heat_control_enabled = 0;
        Shoot.heat_state = HEAT_BYPASS;
        return;
    }

    Shoot.heat_control_enabled = 1;

    switch (Shoot.heat_state)
    {
    case HEAT_BYPASS:
        Shoot.heat_state = Shoot_GetHeatStateFromMargin(safe_margin, cooling_value);
        break;

    case HEAT_BOOST:
        if (safe_margin < (HEAT_BOOST_EXIT_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_TRACK;
        }
        break;

    case HEAT_TRACK:
        if (safe_margin > (HEAT_BOOST_ENTER_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_BOOST;
        }
        else if (safe_margin < (HEAT_KEEP_EXIT_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_KEEPALIVE;
        }
        break;

    case HEAT_KEEPALIVE:
        if (safe_margin > (HEAT_TRACK_RETURN_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_TRACK;
        }
        else if (safe_margin < (HEAT_GUARD_ENTER_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_GUARD;
        }
        break;

    case HEAT_GUARD:
        if (safe_margin > (HEAT_GUARD_EXIT_FACTOR * cooling_value))
        {
            Shoot.heat_state = HEAT_KEEPALIVE;
        }
        break;

    default:
        Shoot.heat_state = HEAT_BYPASS;
        Shoot.heat_control_enabled = 0;
        break;
    }
}

/*
 * 计算GUARD状态下的极低保底速度。
 * 工作原理：
 * 1. 在已经逼近热量安全余量时，仍然给拨弹盘一个很小的非零转速，避免主观卡死电机。
 * 2. 速度同样受上下限约束，保证不同基础模式下都能统一地落到“低风险但不断转”的区间。
 */
static float Shoot_GetGuardSpeed(float base_speed)
{
    float guard_speed = Shoot_ClampFloat(Shoot_AbsFloat(base_speed) * HEAT_GUARD_SPEED_RATIO,
                                         HEAT_GUARD_SPEED_MIN,
                                         HEAT_GUARD_SPEED_MAX);
    return Shoot_SignNonZero(base_speed) * guard_speed;
}

/*
 * 计算KEEPALIVE状态下的低速保活速度。
 * 工作原理：
 * 1. 先计算KEEPALIVE目标速度和GUARD目标速度。
 * 2. 在safe_margin较低时，输出逐渐向GUARD速度靠拢；在safe_margin较高时，再平滑过渡到KEEPALIVE目标速度。
 * 3. 这样既能保证热量紧张时继续转动，又不会在KEEPALIVE与GUARD切换时产生速度跳变。
 */
static float Shoot_GetKeepaliveSpeed(float base_speed, float safe_margin, float cooling_value)
{
    float base_speed_abs = Shoot_AbsFloat(base_speed);
    float keep_speed = Shoot_ClampFloat(base_speed_abs * HEAT_KEEP_SPEED_RATIO,
                                        HEAT_KEEP_SPEED_MIN,
                                        HEAT_KEEP_SPEED_MAX);
    float guard_speed = Shoot_AbsFloat(Shoot_GetGuardSpeed(base_speed));
    float ramp_start = HEAT_GUARD_EXIT_FACTOR * cooling_value;
    float ramp_end = HEAT_TRACK_RETURN_FACTOR * cooling_value;
    float x;
    float k_min;
    float k_max;
    float k;

    if (base_speed_abs < 1.0f)
    {
        return 0.0f;
    }

    if (ramp_end <= ramp_start)
    {
        return Shoot_SignNonZero(base_speed) * keep_speed;
    }

    if (safe_margin <= ramp_start)
    {
        return Shoot_SignNonZero(base_speed) * guard_speed;
    }

    if (safe_margin >= ramp_end)
    {
        return Shoot_SignNonZero(base_speed) * keep_speed;
    }

    x = (safe_margin - ramp_start) / (ramp_end - ramp_start);
    k_min = Shoot_ClampFloat(guard_speed / base_speed_abs, 0.0f, 1.0f);
    k_max = Shoot_ClampFloat(keep_speed / base_speed_abs, 0.0f, 1.0f);
    k = k_min + (k_max - k_min) * Shoot_SmoothStep01(x);
    return Shoot_SignNonZero(base_speed) * base_speed_abs * k;
}

/*
 * 计算TRACK状态下的目标速度。
 * 工作原理：
 * 1. TRACK区间不是硬切换，而是从KEEPALIVE速度平滑过渡到BOOST速度。
 * 2. ramp_start对应“靠近KEEPALIVE端”的热量余量，ramp_end对应“靠近BOOST端”的热量余量。
 * 3. k_min使用当前KEEPALIVE速度与基础速度的比值，保证TRACK下界和KEEPALIVE严格对齐，消除状态切换跳变。
 * 4. 过渡曲线采用smoothstep，使中间段柔和、两端衔接平稳。
 */
static float Shoot_GetTrackSpeed(float base_speed, float safe_margin, float cooling_value)
{
    float base_speed_abs = Shoot_AbsFloat(base_speed);
    float keep_speed;
    float keep_speed_abs;
    float ramp_start;
    float ramp_end;
    float x;
    float k_min;
    float k;

    if (base_speed_abs < 1.0f)
    {
        return 0.0f;
    }

    keep_speed = Shoot_GetKeepaliveSpeed(base_speed, HEAT_TRACK_RETURN_FACTOR * cooling_value, cooling_value);
    keep_speed_abs = Shoot_AbsFloat(keep_speed);
    ramp_start = HEAT_TRACK_RETURN_FACTOR * cooling_value;
    ramp_end = HEAT_BOOST_EXIT_FACTOR * cooling_value;

    if (ramp_end <= ramp_start)
    {
        return keep_speed;
    }

    if (safe_margin <= ramp_start)
    {
        return keep_speed;
    }

    if (safe_margin >= ramp_end)
    {
        return base_speed;
    }

    x = (safe_margin - ramp_start) / (ramp_end - ramp_start);
    k_min = Shoot_ClampFloat(keep_speed_abs / base_speed_abs, 0.0f, 1.0f);
    k = k_min + (1.0f - k_min) * Shoot_SmoothStep01(x);
    return Shoot_SignNonZero(base_speed) * base_speed_abs * k;
}

/*
 * 根据本周期的热量快照输出最终限热速度。
 * 工作原理：
 * 1. 先快照读取Heat_Limit / Barrel_Heat / CooLing_Value。
 * 2. 再减去预留的HeatReserve，得到真正参与状态机判断的safe_margin。
 * 3. 用同一组快照完成“有效性判断 + 状态更新”，避免竞态。
 * 4. 数据无效时返回base_speed，表示旁路热量控制。
 * 5. 数据有效时，按当前热量状态分别输出BOOST/TRACK/KEEPALIVE/GUARD对应速度。
 */
static float Shoot_GetHeatControlledSpeed(float base_speed)
{
    float heat_limit = (float)Referee_data.Heat_Limit;
    float barrel_heat = (float)Referee_data.Barrel_Heat;
    float cooling_value = (float)Referee_data.CooLing_Value;
    float safe_margin = heat_limit - HEAT_RESERVE_MARGIN - barrel_heat;
    uint8_t heat_control_available = Shoot_HeatControlAvailable(heat_limit, cooling_value);

    Shoot_UpdateHeatState(safe_margin, cooling_value, heat_control_available);

    if (!Shoot.heat_control_enabled)
    {
        // HEAT_BYPASS 输出全基础速度，无热保护。
        return base_speed;
    }

    switch (Shoot.heat_state)
    {
    case HEAT_BOOST:
        return base_speed;

    case HEAT_TRACK:
        return Shoot_GetTrackSpeed(base_speed, safe_margin, cooling_value);

    case HEAT_KEEPALIVE:
        return Shoot_GetKeepaliveSpeed(base_speed, safe_margin, cooling_value);

    case HEAT_GUARD:
        return Shoot_GetGuardSpeed(base_speed);

    case HEAT_BYPASS:
    default:
        // HEAT_BYPASS 输出全基础速度，无热保护。
        return base_speed;
    }
}

void Shoot_Init()
{
    TriggerMotor_init(DJI_M2006, TRIGGER_MOTOR);
    PID_Set(&Shoot.trigger_speed_pid, 8, 0, 3, 0.0f, SHOOTMOTOR_MAX_CURRENT, 0);
    Shoot.single_shot_state = SINGLE_IDLE;
    Shoot.last_trigger_mode = TRIGGER_CLOSE;
    Shoot.trigger_speed_base = 0.0f;
    Shoot.trigger_speed_set = 0.0f;
    Shoot.heat_control_enabled = 0;
    Shoot.heat_state = HEAT_BYPASS;
}

void Shoot_Updater()
{
    Shoot.trigger_speed_now = TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm;
    Shoot.trigger_current_now = TriggerMotor_get_data(TRIGGER_MOTOR).given_current;
    Shoot.trigger_ecd_now = (int32_t)TriggerMotor_get_data(TRIGGER_MOTOR).ecd_cnt;

    if (Global.Shoot.tigger_mode == SINGLE && Shoot.last_trigger_mode != SINGLE)
    {
        Shoot.single_shot_start_ecd = Shoot.trigger_ecd_now;
        Shoot.single_shot_state = SINGLE_FIRING;
    }
    else if (Global.Shoot.tigger_mode != SINGLE && Shoot.single_shot_state == SINGLE_DONE)
    {
        Shoot.single_shot_state = SINGLE_IDLE;
    }

    Shoot.last_trigger_mode = Global.Shoot.tigger_mode;
    Shoot.trigger_speed_base = Shoot_GetBaseSpeed(Global.Shoot.tigger_mode);
}

void Shoot_Calculator()
{
    // 1. 优先级最高：执行单发任务
    if (Shoot.single_shot_state == SINGLE_FIRING)
    {
        // 计算自触发时刻起的相对位移
        int32_t moved_ecd = Shoot.trigger_ecd_now - Shoot.single_shot_start_ecd;
        int32_t ecd_err = (int32_t)SINGLE_BULLET_ECD - moved_ecd;

        // 到位判定
        if (ecd_err <= SINGLE_SHOT_THRESHOLD)
        {
            Shoot.single_shot_state = SINGLE_DONE;
            Shoot.trigger_speed_set = 0.0f;
            Shoot.current[2] = 0;
            return;
        }

        // 速度规划
        float speed_set = (ecd_err <= SINGLE_SHOT_SLOWDOWN_ECD) ? 
                          (float)SINGLE_SHOT_SLOW_SPEED : (float)TRIGGER_SPEED_HIGH;
        Shoot.trigger_speed_set = speed_set;

        // PID 计算
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, speed_set);

        /**
         * 【关键优化：强力启动补偿】
         * 如果在单发启动初期（转动还没超过总行程的 10%），且发现速度没起来，
         * 直接给一个足以克服摩擦力的最小保底电流（例如 1000）。
         */
        if (moved_ecd < (SINGLE_BULLET_ECD / 10)) 
        {
            if (abs((int32_t)Shoot.trigger_speed_now) < 100) 
            {
                // 给一个方向正确的额外推力
                Shoot.current[2] += (speed_set > 0) ? 1000.0f : -1000.0f;
            }
        }

        // 防卡弹逻辑
        static uint16_t block_timer = 0;
        if (abs((int32_t)Shoot.trigger_speed_now) < 100 && abs((int32_t)Shoot.trigger_current_now) > 8000)
            block_timer++;
        else
            block_timer = 0;

        if (block_timer > 300) 
        {
            Shoot.single_shot_state = SINGLE_DONE;
            Shoot.trigger_speed_set = 0.0f;
            Shoot.current[2] = 0;
            block_timer = 0;
        }
        return; 
    }

    // 2. 单发停机锁止
    if (Global.Shoot.tigger_mode == SINGLE)
    {
        Shoot.trigger_speed_set = 0.0f;
        Shoot.current[2] = 0;
        return;
    }

    // 3. 常规模式逻辑 (HIGH/MID/LOW/AIM 等)
    Shoot.trigger_speed_set = Shoot_GetHeatControlledSpeed(Shoot.trigger_speed_base);

    static int trigger_kill_cnt = 0;
    if (Global.Shoot.tigger_mode == TRIGGER_CLOSE &&
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
        // 堵转反转脱困
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, -3000.0f);
        trigger_kill_cnt--;
    }
    else
    {
        Shoot.current[2] = PID_Cal(&Shoot.trigger_speed_pid, Shoot.trigger_speed_now, Shoot.trigger_speed_set);
    }
}

void Shoot_Controller()
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
