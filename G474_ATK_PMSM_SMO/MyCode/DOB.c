#include "DOB.h"
#include <string.h>


#define DOB_EPS                 (1.0e-9f)
#define QDOB_DEFAULT_WQ_RAD_S   (80.0f)
#define FULL_DOB_DEFAULT_WO     (100.0f)
#define DOB_DEFAULT_TL_LIMIT_NM (0.50f)


static float DOB_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}


static float DOB_ClampF(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}


static float DOB_RpmToRadS(float rpm)
{
    return rpm * DOB_RPM_TO_RAD_S;
}


static void QDOB_ProtectConfig(QDOB_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    if (cfg->h <= 0.0f)
    {
        cfg->h = DOB_TS_S;
    }

    if (DOB_AbsF(cfg->Kt) < DOB_EPS)
    {
        cfg->Kt = DOB_KT_NM_A;
    }

    if (cfg->J <= DOB_EPS)
    {
        cfg->J = DOB_J_KG_M2;
    }

    if (cfg->B < 0.0f)
    {
        cfg->B = DOB_B_NM_S_RAD;
    }

    if (cfg->wq <= 0.0f)
    {
        cfg->wq = QDOB_DEFAULT_WQ_RAD_S;
    }
}


static void FullDOB_ProtectConfig(FullLuenbergerDOB_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    if (cfg->h <= 0.0f)
    {
        cfg->h = DOB_TS_S;
    }

    if (DOB_AbsF(cfg->Kt) < DOB_EPS)
    {
        cfg->Kt = DOB_KT_NM_A;
    }

    if (cfg->J <= DOB_EPS)
    {
        cfg->J = DOB_J_KG_M2;
    }

    if (cfg->B < 0.0f)
    {
        cfg->B = DOB_B_NM_S_RAD;
    }

    if (cfg->wo <= 0.0f)
    {
        cfg->wo = FULL_DOB_DEFAULT_WO;
    }
}


/***************************************** Q-DOB / 降阶 Luenberger DOB *****************************************/

void QDOB_DefaultConfig(QDOB_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    cfg->h = DOB_TS_S;
    cfg->Kt = DOB_KT_NM_A;
    cfg->J = DOB_J_KG_M2;
    cfg->B = DOB_B_NM_S_RAD;

    /*
     * Q滤波器带宽：
     * 80rad/s 约等于12.7Hz，先偏保守，避免编码器速度噪声把 TL_hat 打得太毛。
     */
    cfg->wq = QDOB_DEFAULT_WQ_RAD_S;

    /*
     * 电机额定转矩约0.19N*m，这里给0.5N*m作为观测输出保护。
     * 如果只想看原始观测量，可把该值设为0关闭限幅。
     */
    cfg->tl_abs_limit_nm = DOB_DEFAULT_TL_LIMIT_NM;
}


void QDOB_Init(QDOB_Handle_t *h, const QDOB_Config_t *cfg)
{
    QDOB_Config_t default_cfg;

    if (h == 0)
    {
        return;
    }

    memset(h, 0, sizeof(*h));

    if (cfg == 0)
    {
        QDOB_DefaultConfig(&default_cfg);
        h->cfg = default_cfg;
    }
    else
    {
        h->cfg = *cfg;
    }

    QDOB_ProtectConfig(&h->cfg);
    h->inited = 0U;
}


void QDOB_Reset(QDOB_Handle_t *h, float speed_rpm)
{
    float omega;

    if (h == 0)
    {
        return;
    }

    QDOB_ProtectConfig(&h->cfg);

    omega = DOB_RpmToRadS(speed_rpm);

    h->omega_rad_s = omega;
    h->te_nm = 0.0f;

    /*
     * 令 TL_hat = z - J*wq*omega = 0。
     * 这样运行中复位不会因为当前速度非零产生一个假的负载转矩尖峰。
     */
    h->z = h->cfg.J * h->cfg.wq * omega;
    h->tl_hat_nm = 0.0f;
    h->inited = 1U;
}


float QDOB_Update_1kHz(QDOB_Handle_t *h, float iq_meas_a, float speed_rpm)
{
    float omega;
    float input;
    float denom;
    float tl_hat;

    if (h == 0)
    {
        return 0.0f;
    }

    QDOB_ProtectConfig(&h->cfg);

    omega = DOB_RpmToRadS(speed_rpm);

    h->omega_rad_s = omega;
    h->te_nm = h->cfg.Kt * iq_meas_a;

    if (h->inited == 0U)
    {
        QDOB_Reset(h, speed_rpm);
        h->te_nm = h->cfg.Kt * iq_meas_a;
    }

    /*
     * 降阶观测器：
     *   z_dot  = -wq*z + (J*wq*wq - B*wq)*omega + Kt*wq*Iq
     *   TL_hat = z - J*wq*omega
     *
     * 这里用后向欧拉积分：
     *   z(k) = [z(k-1) + h*input(k)] / [1 + h*wq]
     *
     * 后向欧拉比普通欧拉更稳，适合嵌入式实时计算。
     */
    input = (h->cfg.J * h->cfg.wq * h->cfg.wq - h->cfg.B * h->cfg.wq) * omega
          + h->cfg.Kt * h->cfg.wq * iq_meas_a;

    denom = 1.0f + h->cfg.h * h->cfg.wq;
    h->z = (h->z + h->cfg.h * input) / denom;

    tl_hat = h->z - h->cfg.J * h->cfg.wq * omega;

    if (h->cfg.tl_abs_limit_nm > 0.0f)
    {
        tl_hat = DOB_ClampF(tl_hat, -h->cfg.tl_abs_limit_nm, h->cfg.tl_abs_limit_nm);

        /*
         * 限幅后同步内部状态，避免外部看到的 TL_hat 已限幅，
         * 但内部 z 继续积累到很远，恢复时产生慢回弹。
         */
        h->z = tl_hat + h->cfg.J * h->cfg.wq * omega;
    }

    h->tl_hat_nm = tl_hat;
    return h->tl_hat_nm;
}


/***************************************** 全阶 Luenberger DOB *****************************************/

void FullLuenbergerDOB_DefaultConfig(FullLuenbergerDOB_Config_t *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    cfg->h = DOB_TS_S;
    cfg->Kt = DOB_KT_NM_A;
    cfg->J = DOB_J_KG_M2;
    cfg->B = DOB_B_NM_S_RAD;

    /*
     * 全阶观测器重复极点带宽。
     * 100rad/s 约等于15.9Hz，先偏保守；速度反馈干净后再逐步加大。
     */
    cfg->wo = FULL_DOB_DEFAULT_WO;
    cfg->tl_abs_limit_nm = DOB_DEFAULT_TL_LIMIT_NM;
}


void FullLuenbergerDOB_Init(FullLuenbergerDOB_Handle_t *h, const FullLuenbergerDOB_Config_t *cfg)
{
    FullLuenbergerDOB_Config_t default_cfg;

    if (h == 0)
    {
        return;
    }

    memset(h, 0, sizeof(*h));

    if (cfg == 0)
    {
        FullLuenbergerDOB_DefaultConfig(&default_cfg);
        h->cfg = default_cfg;
    }
    else
    {
        h->cfg = *cfg;
    }

    FullDOB_ProtectConfig(&h->cfg);
    FullLuenbergerDOB_CalcGain(h);
    h->inited = 0U;
}


void FullLuenbergerDOB_Reset(FullLuenbergerDOB_Handle_t *h, float speed_rpm)
{
    float omega;

    if (h == 0)
    {
        return;
    }

    FullDOB_ProtectConfig(&h->cfg);
    FullLuenbergerDOB_CalcGain(h);

    omega = DOB_RpmToRadS(speed_rpm);

    h->omega_meas_rad_s = omega;
    h->omega_hat_rad_s = omega;
    h->tl_hat_nm = 0.0f;
    h->te_nm = 0.0f;
    h->err_omega_rad_s = 0.0f;
    h->inited = 1U;
}


void FullLuenbergerDOB_CalcGain(FullLuenbergerDOB_Handle_t *h)
{
    float j_inv;

    if (h == 0)
    {
        return;
    }

    FullDOB_ProtectConfig(&h->cfg);

    j_inv = 1.0f / h->cfg.J;

    /*
     * 机械模型：
     *   omega_dot = -B/J*omega + Kt/J*Iq - 1/J*TL
     *
     * 误差方程特征多项式：
     *   s^2 + (B/J + l1)*s - l2/J
     *
     * 配置为重复极点：
     *   (s + wo)^2 = s^2 + 2*wo*s + wo^2
     *
     * 得：
     *   l1 = 2*wo - B/J
     *   l2 = -J*wo^2
     */
    h->l1 = 2.0f * h->cfg.wo - h->cfg.B * j_inv;
    h->l2 = -h->cfg.J * h->cfg.wo * h->cfg.wo;
}


float FullLuenbergerDOB_Update_1kHz(FullLuenbergerDOB_Handle_t *h, float iq_meas_a, float speed_rpm)
{
    float omega;
    float omega_dot;
    float tl_dot;
    float tl_hat;

    if (h == 0)
    {
        return 0.0f;
    }

    FullDOB_ProtectConfig(&h->cfg);
    FullLuenbergerDOB_CalcGain(h);

    omega = DOB_RpmToRadS(speed_rpm);

    h->omega_meas_rad_s = omega;
    h->te_nm = h->cfg.Kt * iq_meas_a;

    if (h->inited == 0U)
    {
        FullLuenbergerDOB_Reset(h, speed_rpm);
        h->te_nm = h->cfg.Kt * iq_meas_a;
    }

    h->err_omega_rad_s = h->omega_meas_rad_s - h->omega_hat_rad_s;

    /*
     * 全阶 Luenberger 观测器：
     *   omega_hat_dot = (-B*omega_hat + Kt*Iq - TL_hat) / J + l1*err
     *   TL_hat_dot    = l2*err
     *
     * err = omega_meas - omega_hat。
     * 当实际负载增大时，真实速度会低于模型预测速度，err为负；
     * 本符号约定下 l2为负，所以 TL_hat 会向正方向增加。
     */
    omega_dot = (-h->cfg.B * h->omega_hat_rad_s + h->cfg.Kt * iq_meas_a - h->tl_hat_nm) / h->cfg.J
              + h->l1 * h->err_omega_rad_s;
    tl_dot = h->l2 * h->err_omega_rad_s;

    h->omega_hat_rad_s += h->cfg.h * omega_dot;
    tl_hat = h->tl_hat_nm + h->cfg.h * tl_dot;

    if (h->cfg.tl_abs_limit_nm > 0.0f)
    {
        tl_hat = DOB_ClampF(tl_hat, -h->cfg.tl_abs_limit_nm, h->cfg.tl_abs_limit_nm);
    }

    h->tl_hat_nm = tl_hat;
    return h->tl_hat_nm;
}
