#include "DeadTime.h"

/*
 * 当前工程 TIM1 关键参数：
 * ARR = 6999，中心对齐 PWM，硬件 DeadTime = 50。
 * 因此默认补偿量先按 50 tick 处理，实际效果再通过 compensation_gain 微调。
 */
#define DEADTIME_DEFAULT_TICKS          (50.0f)
#define DEADTIME_DEFAULT_MAX_COMPARE    (6999.0f)
#define DEADTIME_DEFAULT_MIN_COMPARE    (1.0f)
#define DEADTIME_DEFAULT_GAIN           (0.8f)
#define DEADTIME_DEFAULT_CURRENT_TH_A   (0.05f)


DeadTimeComp_Handle_t DTC;




/* fabsf 的轻量替代，避免为了一个绝对值额外依赖 math 库 */
static float DeadTimeComp_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 简单限幅函数，用于保护补偿系数、CCR 范围和过零比例 */
static float DeadTimeComp_ClampF(float x, float min_value, float max_value)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}

/* float 转 uint16_t，带四舍五入和范围保护 */
static uint16_t DeadTimeComp_F32ToU16(float x)
{
    if (x <= 0.0f)
    {
        return 0U;
    }

    if (x >= 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)(x + 0.5f);
}

/* 填入一组保守默认值，方便外部快速启用 */
void DeadTimeComp_DefaultConfig(DeadTimeComp_Handle_t *dtc)
{
    if (dtc == 0)
    {
        return;
    }

    dtc->enable = 1U;
    dtc->deadtime_ticks = DEADTIME_DEFAULT_TICKS;
    dtc->compensation_gain = DEADTIME_DEFAULT_GAIN;
    dtc->current_threshold_a = DEADTIME_DEFAULT_CURRENT_TH_A;
    dtc->min_compare = DEADTIME_DEFAULT_MIN_COMPARE;
    dtc->max_compare = DEADTIME_DEFAULT_MAX_COMPARE;
}

/* 初始化接口：外部只需要关心硬件死区 tick 和 CCR 最大值 */
void DeadTimeComp_Init(DeadTimeComp_Handle_t *dtc,
                       float deadtime_ticks,
                       float max_compare)
{
    DeadTimeComp_DefaultConfig(dtc);

    if (dtc == 0)
    {
        return;
    }

    DeadTimeComp_SetDeadTimeTicks(dtc, deadtime_ticks);
    DeadTimeComp_SetCompareLimit(dtc, DEADTIME_DEFAULT_MIN_COMPARE, max_compare);
}

/* 使能控制：用于实验时快速对比“开补偿”和“关补偿”的差别 */
void DeadTimeComp_SetEnable(DeadTimeComp_Handle_t *dtc, uint8_t enable)
{
    if (dtc == 0)
    {
        return;
    }

    dtc->enable = (enable != 0U) ? 1U : 0U;
}

/* 补偿系数：0 表示不补偿，1 表示按硬件死区全量补偿 */
void DeadTimeComp_SetGain(DeadTimeComp_Handle_t *dtc, float gain)
{
    if (dtc == 0)
    {
        return;
    }

    dtc->compensation_gain = DeadTimeComp_ClampF(gain, 0.0f, 2.0f);
}

/* 设置硬件死区对应的 timer tick，通常与 CubeMX 中 TIM1 DeadTime 保持一致 */
void DeadTimeComp_SetDeadTimeTicks(DeadTimeComp_Handle_t *dtc, float deadtime_ticks)
{
    if (dtc == 0)
    {
        return;
    }

    if (deadtime_ticks < 0.0f)
    {
        deadtime_ticks = 0.0f;
    }

    dtc->deadtime_ticks = deadtime_ticks;
}

/* 设置电流过零区域宽度，避免电流符号在零附近跳变导致 CCR 来回抖 */
void DeadTimeComp_SetCurrentThreshold(DeadTimeComp_Handle_t *dtc, float threshold_a)
{
    if (dtc == 0)
    {
        return;
    }

    if (threshold_a < 0.0f)
    {
        threshold_a = -threshold_a;
    }

    dtc->current_threshold_a = threshold_a;
}

/* 设置补偿后的 CCR 合法范围，当前工程一般使用 1~6999 */
void DeadTimeComp_SetCompareLimit(DeadTimeComp_Handle_t *dtc,
                                  float min_compare,
                                  float max_compare)
{
    if (dtc == 0)
    {
        return;
    }

    if (min_compare < 0.0f)
    {
        min_compare = 0.0f;
    }

    if (max_compare < min_compare)
    {
        max_compare = min_compare;
    }

    dtc->min_compare = min_compare;
    dtc->max_compare = max_compare;
}

/*
 * 根据相电流方向计算 CCR 补偿量。
 *
 * 约定：
 * phase_current_a > 0 时，增加该相高桥臂有效导通时间，即 CCR 加补偿量。
 * phase_current_a < 0 时，减少该相高桥臂有效导通时间，即 CCR 减补偿量。
 *
 * 过零处理：
 * 当 |Iphase| 小于 current_threshold_a 时，补偿量按比例缩小。
 * 例如阈值 0.05A，电流 0.025A 时，只补偿 50%，这样过零更平滑。
 */
float DeadTimeComp_CalcOffset(const DeadTimeComp_Handle_t *dtc, float phase_current_a)
{
    float sign_gain;
    float abs_current;

    if ((dtc == 0) || (dtc->enable == 0U))
    {
        return 0.0f;
    }

    abs_current = DeadTimeComp_AbsF(phase_current_a);

    if (abs_current <= 0.0f)
    {
        return 0.0f;
    }

    if (dtc->current_threshold_a > 0.0f)
    {
        sign_gain = DeadTimeComp_ClampF(abs_current / dtc->current_threshold_a, 0.0f, 1.0f);
    }
    else
    {
        sign_gain = 1.0f;
    }

    if (phase_current_a < 0.0f)
    {
        sign_gain = -sign_gain;
    }

    return dtc->deadtime_ticks * dtc->compensation_gain * sign_gain;
}

/* 单相 CCR 补偿：原始 CCR + 由电流方向决定的 offset，然后再限幅 */
float DeadTimeComp_ApplyOneF32(const DeadTimeComp_Handle_t *dtc,
                               float compare,
                               float phase_current_a)
{
    /*
     * 未初始化或关闭补偿时，必须原样返回。
     * 这样即使在 Encoder_Align_Zero() 早于 DeadTimeComp_Init() 调用时，
     * 也不会因为 min_compare/max_compare 还是0而把PWM比较值夹成0。
     */
    if ((dtc == 0) || (dtc->enable == 0U))
    {
        return compare;
    }

    compare += DeadTimeComp_CalcOffset(dtc, phase_current_a);
    return DeadTimeComp_ClampF(compare, dtc->min_compare, dtc->max_compare);
}

/*
 * 三相 float CCR 补偿接口。
 *
 * 输入：
 * iu_a/iv_a/iw_a：三相电流，单位 A。
 * cmp_u/cmp_v/cmp_w：指向三相 CCR 比较值的指针，调用前是原始 SVPWM 结果。
 *
 * 输出：
 * cmp_u/cmp_v/cmp_w 指向的值会被原地修改为补偿后的 CCR。
 */
void DeadTimeComp_ApplyCompareF32(const DeadTimeComp_Handle_t *dtc,
                                  float iu_a,
                                  float iv_a,
                                  float iw_a,
                                  float *cmp_u,
                                  float *cmp_v,
                                  float *cmp_w)
{
    if ((dtc == 0) || (cmp_u == 0) || (cmp_v == 0) || (cmp_w == 0))
    {
        return;
    }

    *cmp_u = DeadTimeComp_ApplyOneF32(dtc, *cmp_u, iu_a);
    *cmp_v = DeadTimeComp_ApplyOneF32(dtc, *cmp_v, iv_a);
    *cmp_w = DeadTimeComp_ApplyOneF32(dtc, *cmp_w, iw_a);
}

/*
 * 三相 uint16_t CCR 补偿接口。
 *
 * 作用和 DeadTimeComp_ApplyCompareF32 一样，只是输入输出比较值为 uint16_t。
 * 当前工程 Foc->Tcm1/2/3 是 float，所以优先使用 ApplyCompareF32。
 */
void DeadTimeComp_ApplyCompareU16(const DeadTimeComp_Handle_t *dtc,
                                  float iu_a,
                                  float iv_a,
                                  float iw_a,
                                  uint16_t *cmp_u,
                                  uint16_t *cmp_v,
                                  uint16_t *cmp_w)
{
    float cmp_u_f;
    float cmp_v_f;
    float cmp_w_f;

    if ((dtc == 0) || (cmp_u == 0) || (cmp_v == 0) || (cmp_w == 0))
    {
        return;
    }

    cmp_u_f = (float)(*cmp_u);
    cmp_v_f = (float)(*cmp_v);
    cmp_w_f = (float)(*cmp_w);

    DeadTimeComp_ApplyCompareF32(dtc, iu_a, iv_a, iw_a,
                                 &cmp_u_f, &cmp_v_f, &cmp_w_f);

    *cmp_u = DeadTimeComp_F32ToU16(cmp_u_f);
    *cmp_v = DeadTimeComp_F32ToU16(cmp_v_f);
    *cmp_w = DeadTimeComp_F32ToU16(cmp_w_f);
}
