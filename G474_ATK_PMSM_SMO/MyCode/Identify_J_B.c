#include "Identify_J_B.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float Id_J_B_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float Id_J_B_rpm_to_rad_s(float rpm)
{
    return rpm * (2.0f * (float)M_PI / 60.0f);
}

void Id_J_B_DefaultConfig(ID_J_B_Config_t *cfg)
{
    if (cfg == 0) return;
    memset(cfg, 0, sizeof(*cfg));

    /* 按ReadMe安全优先 */
    cfg->iq_test_a = 0.6f;
    cfg->iq_abs_limit_a = 1.0f;

    cfg->prepare_ms = 300;
    cfg->pulse_ms = 220;
    cfg->coast_ms = 350;
    cfg->cycles = 20;

    /* 额定4000rpm，安全上限*/
    cfg->speed_safe_rpm = 3500.0f;
    cfg->alpha_lpf = 0.20f;
}

void Id_J_B_Init(ID_J_B_Handle_t *h, const ID_J_B_Config_t *cfg)
{
    if ((h == 0) || (cfg == 0)) return;
    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;
    h->state = ID_J_B_STATE_IDLE;
}

void Id_J_B_Start(ID_J_B_Handle_t *h)
{
    if (h == 0) return;

    memset(&h->sample, 0, sizeof(h->sample));

    h->state = ID_J_B_STATE_PREPARE;
    h->running = 1U;
    h->finished = 0U;
    h->aborted = 0U;

    h->ms_in_state = 0U;
    h->cycle_cnt = 0U;
    h->total_ms = 0U;

    h->omega_prev = 0.0f;
    h->alpha_filt = 0.0f;
    h->omega_inited = 0U;

    h->iq_cmd_now = 0.0f;
}

void Id_J_B_Stop(ID_J_B_Handle_t *h)
{
    if (h == 0) return;
    h->running = 0U;
    h->finished = 1U;
    h->iq_cmd_now = 0.0f;
    h->state = ID_J_B_STATE_DONE;
}

static void Id_J_B_abort(ID_J_B_Handle_t *h)
{
    h->running = 0U;
    h->finished = 1U;
    h->aborted = 1U;
    h->iq_cmd_now = 0.0f;
    h->state = ID_J_B_STATE_ABORT;
}

static void Id_J_B_update_sample(ID_J_B_Handle_t *h, const ID_J_B_Input_t *in, float omega, float alpha)
{
    h->sample.t_s          = (float)h->total_ms * 0.001f;
    h->sample.iq_cmd_a     = h->iq_cmd_now;
    h->sample.iq_meas_a    = in->iq_meas_a;
    h->sample.speed_rpm    = in->speed_rpm;
    h->sample.omega_rad_s  = omega;
    h->sample.alpha_rad_s2 = alpha;
    h->sample.state        = (uint8_t)h->state;
    h->sample.running      = h->running;
    h->sample.aborted      = h->aborted;
    h->sample.sample_idx++;
}

void Id_J_B_Update_1kHz(ID_J_B_Handle_t *h, const ID_J_B_Input_t *in, ID_J_B_Output_t *out)
{
    float omega, alpha_raw;

    if ((h == 0) || (in == 0) || (out == 0)) return;

    out->takeover = 0U;
    out->iq_cmd_a = 0.0f;

    if (!h->running) return;

    h->total_ms++;
    h->ms_in_state++;

    /* 安全保护 */
    if (in->run_flag == 0U)
    {
        Id_J_B_abort(h);
        return;
    }
    if (fabsf(in->speed_rpm) > h->cfg.speed_safe_rpm)
    {
        Id_J_B_abort(h);
        return;
    }

    /* 机械角速度/角加速度 */
    omega = Id_J_B_rpm_to_rad_s(in->speed_rpm);
    if (h->omega_inited == 0U)
    {
        h->omega_prev = omega;
        h->alpha_filt = 0.0f;
        h->omega_inited = 1U;
    }

    alpha_raw = (omega - h->omega_prev) / ID_J_B_TS_S;
    h->omega_prev = omega;

    h->alpha_filt = h->cfg.alpha_lpf * alpha_raw + (1.0f - h->cfg.alpha_lpf) * h->alpha_filt;

    /* 状态机 */
    switch (h->state)
    {
        case ID_J_B_STATE_PREPARE:
            h->iq_cmd_now = 0.0f;
            if ((h->ms_in_state >= h->cfg.prepare_ms) && (fabsf(in->speed_rpm) < 30.0f))
            {
                h->state = ID_J_B_STATE_PULSE_POS;
                h->ms_in_state = 0U;
            }
            break;

        case ID_J_B_STATE_PULSE_POS:
            h->iq_cmd_now = +h->cfg.iq_test_a;
            if (h->ms_in_state >= h->cfg.pulse_ms)
            {
                h->state = ID_J_B_STATE_COAST_1;
                h->ms_in_state = 0U;
            }
            break;

        case ID_J_B_STATE_COAST_1:
            h->iq_cmd_now = 0.0f;
            if (h->ms_in_state >= h->cfg.coast_ms)
            {
                h->state = ID_J_B_STATE_PULSE_NEG;
                h->ms_in_state = 0U;
            }
            break;

        case ID_J_B_STATE_PULSE_NEG:
            h->iq_cmd_now = -h->cfg.iq_test_a;
            if (h->ms_in_state >= h->cfg.pulse_ms)
            {
                h->state = ID_J_B_STATE_COAST_2;
                h->ms_in_state = 0U;
            }
            break;

        case ID_J_B_STATE_COAST_2:
            h->iq_cmd_now = 0.0f;
            if (h->ms_in_state >= h->cfg.coast_ms)
            {
                h->cycle_cnt++;
                if (h->cycle_cnt >= h->cfg.cycles)
                {
                    h->running = 0U;
                    h->finished = 1U;
                    h->state = ID_J_B_STATE_DONE;
                }
                else
                {
                    h->state = ID_J_B_STATE_PULSE_POS;
                    h->ms_in_state = 0U;
                }
            }
            break;

        default:
            Id_J_B_abort(h);
            break;
    }

    /* 电流命令硬限幅 */
    h->iq_cmd_now = Id_J_B_clampf(h->iq_cmd_now, -h->cfg.iq_abs_limit_a, h->cfg.iq_abs_limit_a);

    /* 输出 */
    out->takeover = 1U;
    out->iq_cmd_a = h->iq_cmd_now;

    /* 记录 */
    Id_J_B_update_sample(h, in, omega, h->alpha_filt);

}

uint8_t Id_J_B_IsRunning(const ID_J_B_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->running;
}

uint8_t Id_J_B_IsFinished(const ID_J_B_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->finished;
}

uint8_t Id_J_B_IsAborted(const ID_J_B_Handle_t *h)
{
    if (h == 0) return 0U;
    return h->aborted;
}

