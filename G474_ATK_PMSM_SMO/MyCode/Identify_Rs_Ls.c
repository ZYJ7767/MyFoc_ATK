#include "Identify_Rs_Ls.h"
#include <string.h>
#include <math.h>

/*
 * PMSM离线Rs/Ls辨识模块
 *
 * 使用方式：
 * 1) 10kHz ADC中断中调用 Id_Rs_Ls_Update_10kHz()
 * 2) out.takeover=1时，外部用 VF_OpenLoop() 输出 out.ud_cmd_v/out.uq_cmd_v/out.theta_rad
 * 3) 辨识成功后读取 Identify_Rs_Ohm 和 Identify_Ls_H
 *
 * 注意：
 * 该方法要求电机基本静止。辨识期间不要再跑电流环，否则电流环会改变Ud/Uq，
 * 导致“电压注入 -> 电流响应”的物理关系被破坏。
 */

#ifndef ID_RS_LS_PI
#define ID_RS_LS_PI       3.14159265358979323846f
#endif

/* 最终辨识结果，供外部直接读取 */
float Identify_Rs_Ohm = 0.0f;
float Identify_Ls_H   = 0.0f;

/* 简单限幅函数，用于保护注入电压 */
static float Id_Rs_Ls_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* fabsf的轻量替代，避免到处写fabsf */
static float Id_Rs_Ls_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/* 把角度限制到0~2pi，防止用户传入过大角度 */
static float Id_Rs_Ls_normalize_theta(float theta)
{
    const float two_pi = 2.0f * ID_RS_LS_PI;

    while (theta >= two_pi) theta -= two_pi;
    while (theta < 0.0f) theta += two_pi;

    return theta;
}

/* 固定角度下只需要普通sinf/cosf即可 */
static void Id_Rs_Ls_sin_cos(float theta, float *s, float *c)
{
    *s = sinf(theta);
    *c = cosf(theta);
}

/* 根据三相电流计算固定坐标下的Id/Iq。
 * Clarke变换和工程里的Foc_Function.c保持一致：
 * Ialpha = Iu
 * Ibeta  = 1/sqrt(3)*Iu + 2/sqrt(3)*Iv
 *
 * 然后用固定theta做Park变换。因为辨识时theta不动，
 * Id就是沿注入电压方向的电流，Iq用于观察是否出现明显转矩分量。
 */
static void Id_Rs_Ls_calc_current(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Input_t *in)
{
    float sin_t;
    float cos_t;
    float ialpha;
    float ibeta;

    ialpha = in->iu_a;
    ibeta = 0.57735026919f * in->iu_a + 1.15470053838f * in->iv_a;

    Id_Rs_Ls_sin_cos(h->cfg.theta_rad, &sin_t, &cos_t);

    h->id_now = ialpha * cos_t + ibeta * sin_t;
    h->iq_now = -ialpha * sin_t + ibeta * cos_t;

    h->sample.ialpha_a = ialpha;
    h->sample.ibeta_a = ibeta;
    h->sample.id_a = h->id_now;
    h->sample.iq_a = h->iq_now;
}

/* 切换状态时清零本状态计时，同时清除Ls脉冲起点标志 */
static void Id_Rs_Ls_set_state(ID_Rs_Ls_Handle_t *h, ID_Rs_Ls_State_t state)
{
    h->state = state;
    h->ticks_in_state = 0U;
    h->ls_pulse_active = 0U;
}

/* 异常中止：立即撤销输出，外部下一拍会看到takeover=0或0V输出 */
static void Id_Rs_Ls_abort(ID_Rs_Ls_Handle_t *h)
{
    h->running = 0U;
    h->finished = 1U;
    h->aborted = 1U;
    h->ud_cmd_now = 0.0f;
    h->uq_cmd_now = 0.0f;
    h->state = ID_RS_LS_STATE_ABORT;
}

/* 基础安全保护：
 * run_flag掉线、转速过高、Id/Iq超过限流都会中止。
 * 这里同时监视Iq，是为了防止固定角度没有对齐好时产生较大转矩电流。
 */
static uint8_t Id_Rs_Ls_is_safe(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Input_t *in)
{
    if (in->run_flag == 0U) return 0U;
    if (Id_Rs_Ls_absf(in->speed_rpm) > h->cfg.speed_safe_rpm) return 0U;
    if (Id_Rs_Ls_absf(h->id_now) > h->cfg.current_abs_limit_a) return 0U;
    if (Id_Rs_Ls_absf(h->iq_now) > h->cfg.current_abs_limit_a) return 0U;
    return 1U;
}

/* 刷新调试采样结构体，方便Watch/JScope观察辨识过程 */
static void Id_Rs_Ls_update_sample(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Input_t *in)
{
    h->sample.t_s = (float)h->total_ticks * ID_RS_LS_TS_S;
    h->sample.ud_cmd_v = h->ud_cmd_now;
    h->sample.uq_cmd_v = h->uq_cmd_now;
    h->sample.iu_a = in->iu_a;
    h->sample.iv_a = in->iv_a;
    h->sample.iw_a = in->iw_a;
    h->sample.rs_ohm = h->result.rs_ohm;
    h->sample.ls_h = h->result.ls_h;
    h->sample.state = (uint8_t)h->state;
    h->sample.running = h->running;
    h->sample.finished = h->finished;
    h->sample.aborted = h->aborted;
    h->sample.sample_idx++;
}

/* 计算Rs：
 * 电机静止且电流进入稳态后，电感项 L*di/dt 约等于0，
 * d轴电压方程近似为 Ud = Rs * Id。
 *
 * 正向：Rs_pos = +Ud / +Id
 * 反向：Rs_neg = -Ud / -Id
 * 最后取有效结果平均，正反向平均能抵消一部分ADC零漂和死区误差。
 */
static uint8_t Id_Rs_Ls_finish_rs(ID_Rs_Ls_Handle_t *h)
{
    float i_pos;
    float i_neg;
    float rs_sum = 0.0f;
    uint8_t rs_cnt = 0U;

    if (h->rs_pos_cnt > 0U)
    {
        i_pos = h->rs_pos_i_sum / (float)h->rs_pos_cnt;
        if (Id_Rs_Ls_absf(i_pos) >= h->cfg.min_current_for_rs_a)
        {
            h->result.rs_pos_ohm = h->cfg.rs_test_voltage_v / i_pos;
            if (h->result.rs_pos_ohm > 0.0f)
            {
                rs_sum += h->result.rs_pos_ohm;
                rs_cnt++;
            }
        }
    }

    if (h->rs_neg_cnt > 0U)
    {
        i_neg = h->rs_neg_i_sum / (float)h->rs_neg_cnt;
        if (Id_Rs_Ls_absf(i_neg) >= h->cfg.min_current_for_rs_a)
        {
            h->result.rs_neg_ohm = -h->cfg.rs_test_voltage_v / i_neg;
            if (h->result.rs_neg_ohm > 0.0f)
            {
                rs_sum += h->result.rs_neg_ohm;
                rs_cnt++;
            }
        }
    }

    if (rs_cnt == 0U) return 0U;

    h->result.rs_ohm = rs_sum / (float)rs_cnt;
    Identify_Rs_Ohm = h->result.rs_ohm;
    return 1U;
}

/* 计算单次Ls脉冲：
 * d轴电压方程为：Ud = Rs*i + L*di/dt
 * 移项并积分：L = ∫(Ud - Rs*i)dt / Δi
 *
 * 这里用积分法而不是单点di/dt，是为了降低电流采样噪声影响。
 */
static uint8_t Id_Rs_Ls_finish_ls_pulse(ID_Rs_Ls_Handle_t *h, float *ls_out)
{
    float delta_i;
    float ls;

    delta_i = h->ls_end_i - h->ls_start_i;
    if (Id_Rs_Ls_absf(delta_i) < h->cfg.min_delta_i_for_ls_a) return 0U;

    ls = h->ls_flux_int / delta_i;
    if (ls <= 0.0f) return 0U;

    *ls_out = ls;
    return 1U;
}

/* 汇总正、反向Ls结果，并写入最终全局变量 */
static void Id_Rs_Ls_finish_all(ID_Rs_Ls_Handle_t *h)
{
    float ls_sum = 0.0f;
    uint8_t ls_cnt = 0U;

    if (h->result.ls_pos_h > 0.0f)
    {
        ls_sum += h->result.ls_pos_h;
        ls_cnt++;
    }

    if (h->result.ls_neg_h > 0.0f)
    {
        ls_sum += h->result.ls_neg_h;
        ls_cnt++;
    }

    if (ls_cnt > 0U)
    {
        h->result.ls_h = ls_sum / (float)ls_cnt;
        Identify_Ls_H = h->result.ls_h;
        h->result.valid = 1U;
        h->finished = 1U;
        h->running = 0U;
        h->ud_cmd_now = 0.0f;
        h->uq_cmd_now = 0.0f;
        h->state = ID_RS_LS_STATE_DONE;
    }
    else
    {
        Id_Rs_Ls_abort(h);
    }
}

/* 默认参数偏保守，适合第一次试车。
 * 如果Rs阶段电流太小，可小幅提高rs_test_voltage_v；
 * 如果Ls阶段Δi太小，可小幅提高ls_test_voltage_v或ls_pulse_ticks。
 */
void Id_Rs_Ls_DefaultConfig(ID_Rs_Ls_Config_t *cfg)
{
    if (cfg == 0) return;
    memset(cfg, 0, sizeof(*cfg));

    cfg->theta_rad = 0.0f;
    cfg->rs_test_voltage_v = 1.0f;
    cfg->ls_test_voltage_v = 2.0f;
    cfg->voltage_abs_limit_v = 3.5f;
    cfg->current_abs_limit_a = 3.5f;
    cfg->speed_safe_rpm = 500.0f;
    cfg->min_current_for_rs_a = 0.20f;
    cfg->min_delta_i_for_ls_a = 0.25f;
    cfg->ls_current_stop_a = 2.0f;

    cfg->prepare_ticks = 2000U;
    cfg->rs_settle_ticks = 3000U;
    cfg->rs_sample_ticks = 3000U;
    cfg->ls_zero_ticks = 500U;
    cfg->ls_pulse_ticks = 30U;
}

/* 初始化句柄，复制配置，并把注入角度规范化 */
void Id_Rs_Ls_Init(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Config_t *cfg)
{
    if ((h == 0) || (cfg == 0)) return;

    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;
    h->cfg.theta_rad = Id_Rs_Ls_normalize_theta(h->cfg.theta_rad);
    h->state = ID_RS_LS_STATE_IDLE;
}

/* 启动一次完整辨识流程 */
void Id_Rs_Ls_Start(ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return;

    memset(&h->sample, 0, sizeof(h->sample));
    memset(&h->result, 0, sizeof(h->result));

    h->running = 1U;
    h->finished = 0U;
    h->aborted = 0U;
    h->ticks_in_state = 0U;
    h->total_ticks = 0U;

    h->ud_cmd_now = 0.0f;
    h->uq_cmd_now = 0.0f;
    h->rs_pos_i_sum = 0.0f;
    h->rs_neg_i_sum = 0.0f;
    h->rs_pos_cnt = 0U;
    h->rs_neg_cnt = 0U;
    h->ls_start_i = 0.0f;
    h->ls_end_i = 0.0f;
    h->ls_prev_i = 0.0f;
    h->ls_flux_int = 0.0f;
    h->ls_pulse_active = 0U;

    Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_PREPARE);
}

/* 手动停止，撤销输出 */
void Id_Rs_Ls_Stop(ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return;

    h->running = 0U;
    h->finished = 1U;
    h->ud_cmd_now = 0.0f;
    h->uq_cmd_now = 0.0f;
    h->state = ID_RS_LS_STATE_DONE;
}

/* 10kHz主更新函数：
 * 该函数只计算“下一拍希望输出的电压命令”，不直接操作PWM。
 * 外部看到out.takeover=1后，再调用VF_OpenLoop输出电压。
 */
void Id_Rs_Ls_Update_10kHz(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Input_t *in, ID_Rs_Ls_Output_t *out)
{
    if ((h == 0) || (in == 0) || (out == 0)) return;

    out->takeover = 0U;
    out->ud_cmd_v = 0.0f;
    out->uq_cmd_v = 0.0f;
    out->theta_rad = h->cfg.theta_rad;

    if (h->running == 0U) return;

    h->total_ticks++;
    h->ticks_in_state++;

    Id_Rs_Ls_calc_current(h, in);

    if (Id_Rs_Ls_is_safe(h, in) == 0U)
    {
        Id_Rs_Ls_abort(h);
        Id_Rs_Ls_update_sample(h, in);
        return;
    }

    switch (h->state)
    {
        case ID_RS_LS_STATE_PREPARE:
            /* 先输出0V，让电流和转子状态安静下来 */
            h->ud_cmd_now = 0.0f;
            h->uq_cmd_now = 0.0f;
            if (h->ticks_in_state >= h->cfg.prepare_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_RS_POS_SETTLE);
            }
            break;

        case ID_RS_LS_STATE_RS_POS_SETTLE:
            /* 正向直流注入，先等待电感暂态结束，电流进入稳态 */
            h->ud_cmd_now = h->cfg.rs_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            if (h->ticks_in_state >= h->cfg.rs_settle_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_RS_POS_SAMPLE);
            }
            break;

        case ID_RS_LS_STATE_RS_POS_SAMPLE:
            /* 正向稳态电流取平均，用于计算Rs_pos */
            h->ud_cmd_now = h->cfg.rs_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            h->rs_pos_i_sum += h->id_now;
            h->rs_pos_cnt++;
            if (h->ticks_in_state >= h->cfg.rs_sample_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_RS_NEG_SETTLE);
            }
            break;

        case ID_RS_LS_STATE_RS_NEG_SETTLE:
            /* 反向直流注入，同样先等待稳态 */
            h->ud_cmd_now = -h->cfg.rs_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            if (h->ticks_in_state >= h->cfg.rs_settle_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_RS_NEG_SAMPLE);
            }
            break;

        case ID_RS_LS_STATE_RS_NEG_SAMPLE:
            /* 反向稳态电流取平均，用于计算Rs_neg */
            h->ud_cmd_now = -h->cfg.rs_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            h->rs_neg_i_sum += h->id_now;
            h->rs_neg_cnt++;
            if (h->ticks_in_state >= h->cfg.rs_sample_ticks)
            {
                if (Id_Rs_Ls_finish_rs(h) != 0U)
                {
                    Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_LS_ZERO_1);
                }
                else
                {
                    Id_Rs_Ls_abort(h);
                }
            }
            break;

        case ID_RS_LS_STATE_LS_ZERO_1:
            /* 做Ls正向脉冲前先回到0V，尽量让初始电流接近0 */
            h->ud_cmd_now = 0.0f;
            h->uq_cmd_now = 0.0f;
            if (h->ticks_in_state >= h->cfg.ls_zero_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_LS_POS_PULSE);
            }
            break;

        case ID_RS_LS_STATE_LS_POS_PULSE:
            /* 正向短脉冲：记录起点，然后积分(U-Ri)dt并记录电流变化 */
            h->ud_cmd_now = h->cfg.ls_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            if (h->ls_pulse_active == 0U)
            {
                h->ls_start_i = h->id_now;
                h->ls_end_i = h->id_now;
                h->ls_prev_i = h->id_now;
                h->ls_flux_int = 0.0f;
                h->ls_pulse_active = 1U;
            }
            else
            {
                /* 用上一拍电流参与积分，更贴近“上一拍电压导致这一拍电流变化”的离散过程 */
                h->ls_flux_int += (h->ud_cmd_now - h->result.rs_ohm * h->ls_prev_i) * ID_RS_LS_TS_S;
                h->ls_end_i = h->id_now;
                h->ls_prev_i = h->id_now;
            }
            if ((h->ticks_in_state >= h->cfg.ls_pulse_ticks) || (h->id_now >= h->cfg.ls_current_stop_a))
            {
                (void)Id_Rs_Ls_finish_ls_pulse(h, &h->result.ls_pos_h);
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_LS_ZERO_2);
            }
            break;

        case ID_RS_LS_STATE_LS_ZERO_2:
            /* 做Ls反向脉冲前再次回零，避免正向脉冲残留电流影响 */
            h->ud_cmd_now = 0.0f;
            h->uq_cmd_now = 0.0f;
            if (h->ticks_in_state >= h->cfg.ls_zero_ticks)
            {
                Id_Rs_Ls_set_state(h, ID_RS_LS_STATE_LS_NEG_PULSE);
            }
            break;

        case ID_RS_LS_STATE_LS_NEG_PULSE:
            /* 反向短脉冲，计算方法和正向一致 */
            h->ud_cmd_now = -h->cfg.ls_test_voltage_v;
            h->uq_cmd_now = 0.0f;
            if (h->ls_pulse_active == 0U)
            {
                h->ls_start_i = h->id_now;
                h->ls_end_i = h->id_now;
                h->ls_prev_i = h->id_now;
                h->ls_flux_int = 0.0f;
                h->ls_pulse_active = 1U;
            }
            else
            {
                h->ls_flux_int += (h->ud_cmd_now - h->result.rs_ohm * h->ls_prev_i) * ID_RS_LS_TS_S;
                h->ls_end_i = h->id_now;
                h->ls_prev_i = h->id_now;
            }
            if ((h->ticks_in_state >= h->cfg.ls_pulse_ticks) || (h->id_now <= -h->cfg.ls_current_stop_a))
            {
                (void)Id_Rs_Ls_finish_ls_pulse(h, &h->result.ls_neg_h);
                Id_Rs_Ls_finish_all(h);
            }
            break;

        default:
            /* 正常不会进入这里；进入则保护中止 */
            Id_Rs_Ls_abort(h);
            break;
    }

    /* 最后再做一次电压硬限幅，防止配置误填 */
    h->ud_cmd_now = Id_Rs_Ls_clampf(h->ud_cmd_now, -h->cfg.voltage_abs_limit_v, h->cfg.voltage_abs_limit_v);
    h->uq_cmd_now = Id_Rs_Ls_clampf(h->uq_cmd_now, -h->cfg.voltage_abs_limit_v, h->cfg.voltage_abs_limit_v);

    /* 告诉外部：本拍由辨识模块接管电压命令 */
    out->takeover = 1U;
    out->ud_cmd_v = h->ud_cmd_now;
    out->uq_cmd_v = h->uq_cmd_now;
    out->theta_rad = h->cfg.theta_rad;

    Id_Rs_Ls_update_sample(h, in);
}

uint8_t Id_Rs_Ls_IsRunning(const ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->running;
}

uint8_t Id_Rs_Ls_IsFinished(const ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->finished;
}

uint8_t Id_Rs_Ls_IsAborted(const ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->aborted;
}

uint8_t Id_Rs_Ls_ResultValid(const ID_Rs_Ls_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->result.valid;
}
