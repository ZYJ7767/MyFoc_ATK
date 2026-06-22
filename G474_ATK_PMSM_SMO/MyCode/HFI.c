#include "HFI.h"
#include "arm_math.h"

#define HFI_THETA_INT_K     162.9746617f
#define HFI_PLL_ERR_LIMIT   0.5235987756f
#define HFI_EPS             0.000001f
#define HFI_UD_LIMIT        10.0f
#define HFI_UQ_LIMIT        13.8f

HFI_TypeDef HFI = {  0,          //enable
                     0.0001f,    //Ts
                     0.8f,       //Vh
                    -1,          //inject_dir，第一次Update后输出+Vh
                     0.0f,       //Vdh
                     0.0f,       //Vqh
                     0.0f,       //i_alpha_last
                     0.0f,       //i_beta_last
                     0.0f,       //i_alpha_h
                     0.0f,       //i_beta_h
                     0.0f,       //Idh
                     0.0f,       //Iqh
                     0.08f,      //LPF_K
                     266.5f,     //Kp
                     35531.0f,   //Ki
                     0.0f,       //Up
                     0.0f,       //Ui
                     0.0f,       //Err
                     0.0f,       //Est_we
                     0,          //Est_RPM
                     0.0f,       //Est_theta
                     0,          //Est_theta_int
                     0.0f,       //raw_theta
                    -1,          //demod_dir
                     0,          //polarity_ok
                     0,          //polarity
                     0,          //polarity_cnt
                     200,        //polarity_samples，100us下约20ms
                     0.0f,       //idh_pos_sum
                     0.0f,       //idh_neg_sum
                     0.0f,       //polarity_diff
                     0.001f      //polarity_threshold
                  };

/***** 限幅函数 *****/
static float HFI_Limit(float x, float max, float min)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/**************** HFI状态复位 ****************/
void HFI_Reset(HFI_TypeDef *hfi, float theta)
{
    if (hfi == 0)
    {
        return;
    }

    hfi->inject_dir = -1;
    hfi->Vdh = 0.0f;
    hfi->Vqh = 0.0f;

    hfi->i_alpha_last = 0.0f;
    hfi->i_beta_last  = 0.0f;
    hfi->i_alpha_h = 0.0f;
    hfi->i_beta_h  = 0.0f;
    hfi->Idh = 0.0f;
    hfi->Iqh = 0.0f;

    hfi->Up = 0.0f;
    hfi->Ui = 0.0f;
    hfi->Err = 0.0f;
    hfi->Est_we = 0.0f;
    hfi->Est_RPM = 0;
    hfi->Est_theta = Normalize_theta(theta);
    hfi->Est_theta_int = (uint16_t)(hfi->Est_theta * HFI_THETA_INT_K);
    hfi->raw_theta = hfi->Est_theta;

    hfi->demod_dir = -1;
    hfi->polarity_ok = 0;
    hfi->polarity = 0;
    hfi->polarity_cnt = 0;
    hfi->idh_pos_sum = 0.0f;
    hfi->idh_neg_sum = 0.0f;
    hfi->polarity_diff = 0.0f;
}

/**************** 高频方波注入 + PLL角度估算 ****************/
float HFI_Update(HFI_TypeDef *hfi, float i_alpha, float i_beta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    float ih_alpha = 0.0f;
    float ih_beta  = 0.0f;
    float ih_mag   = 0.0f;

    if (hfi == 0) return 0.0f;

    if (hfi->enable == 0U)
    {
        hfi->Vdh = 0.0f;
        hfi->Vqh = 0.0f;
        hfi->i_alpha_last = i_alpha;
        hfi->i_beta_last  = i_beta;
        return hfi->Est_theta;
    }

    /***** 1. 高频电流解调：电流差分乘以上一拍注入极性 *****/
    hfi->demod_dir = hfi->inject_dir;
    ih_alpha = (float)hfi->inject_dir * (i_alpha - hfi->i_alpha_last);
    ih_beta  = (float)hfi->inject_dir * (i_beta  - hfi->i_beta_last);
    hfi->i_alpha_last = i_alpha;
    hfi->i_beta_last  = i_beta;

    hfi->i_alpha_h += hfi->LPF_K * (ih_alpha - hfi->i_alpha_h);
    hfi->i_beta_h  += hfi->LPF_K * (ih_beta  - hfi->i_beta_h);
    hfi->raw_theta = Normalize_theta(atan2f(hfi->i_beta_h, hfi->i_alpha_h));

    /***** 2. 投影到估算dq轴，目标是让高频q轴电流为0 *****/
    arm_sin_cos_f32(hfi->Est_theta * RAD_TO_DEG, &SinValue, &CosValue);
    hfi->Idh =  hfi->i_alpha_h * CosValue + hfi->i_beta_h * SinValue;
    hfi->Iqh = -hfi->i_alpha_h * SinValue + hfi->i_beta_h * CosValue;

    ih_mag = sqrtf(hfi->i_alpha_h * hfi->i_alpha_h + hfi->i_beta_h * hfi->i_beta_h);
    hfi->Err = hfi->Iqh / (ih_mag + HFI_EPS);
    hfi->Err = HFI_Limit(hfi->Err, HFI_PLL_ERR_LIMIT, -HFI_PLL_ERR_LIMIT);

    /***** 3. 简单PLL锁定角度 *****/
    hfi->Up = hfi->Kp * hfi->Err;
    hfi->Ui += hfi->Ki * hfi->Err * hfi->Ts;
    hfi->Ui = HFI_Limit(hfi->Ui, 2500.0f, -2500.0f);

    hfi->Est_we = hfi->Up + hfi->Ui;
    hfi->Est_we = HFI_Limit(hfi->Est_we, 2500.0f, -2500.0f);
    hfi->Est_RPM = (int16_t)(hfi->Est_we * (60.0f / (2.0f * pi * Pn)));

    hfi->Est_theta += hfi->Est_we * hfi->Ts;
    hfi->Est_theta = Normalize_theta(hfi->Est_theta);
    hfi->Est_theta_int = (uint16_t)(hfi->Est_theta * HFI_THETA_INT_K);

    /***** 4. 更新下一拍注入电压：d轴方波注入，q轴不注入 *****/
    hfi->inject_dir = (hfi->inject_dir > 0) ? -1 : 1;
    hfi->Vdh = (float)hfi->inject_dir * hfi->Vh;
    hfi->Vqh = 0.0f;

    return hfi->Est_theta;
}

/**************** HFI极性判断 ****************/
uint8_t HFI_PolarityDetect(HFI_TypeDef *hfi)
{
    if (hfi == 0) return 0;
    if (hfi->enable == 0U) return 0;
    if (hfi->polarity_ok != 0U) return 1;

    /***** 正负注入分别累计高频d轴电流响应，用磁饱和差异判断N/S极 *****/
    if (hfi->demod_dir > 0)
    {
        hfi->idh_pos_sum += fabsf(hfi->Idh);
    }
    else
    {
        hfi->idh_neg_sum += fabsf(hfi->Idh);
    }

    hfi->polarity_cnt++;
    if (hfi->polarity_cnt < hfi->polarity_samples)
    {
        return 0;
    }

    hfi->polarity_diff = hfi->idh_pos_sum - hfi->idh_neg_sum;

    /*
     * 若负向注入响应明显更强，说明当前估算轴方向反了，角度加pi。
     * 不同电机和采样极性可能相反，若实测启动方向固定反，就把这里的判断符号反过来。
     */
    if (hfi->polarity_diff < -hfi->polarity_threshold)
    {
        hfi->polarity = 1;
        hfi->Est_theta = Normalize_theta(hfi->Est_theta + pi);
        hfi->Est_theta_int = (uint16_t)(hfi->Est_theta * HFI_THETA_INT_K);
    }
    else
    {
        hfi->polarity = 0;
    }

    hfi->polarity_ok = 1;
    return 1;
}

/**************** HFI电流环示例 ****************/
float HFI_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, HFI_TypeDef *hfi,
                  float IU, float IV, float IW, float Iq_ref)
{
    float theta_hfi = 0.0f;

    if ((Foc == 0) || (pi_ctrl == 0) || (hfi == 0)) return 0.0f;

    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;

    Clarke(Foc);
    theta_hfi = HFI_Update(hfi, Foc->Ialpha, Foc->Ibeta);

    pi_ctrl->Id_ref = 0.0f;
    pi_ctrl->Iq_ref = Iq_ref;

    Park(Foc, theta_hfi);
    CurrentPI(Foc, pi_ctrl);

    /***** 在电流PI输出后叠加高频d轴方波电压 *****/
    Foc->Ud += hfi->Vdh;
    Foc->Uq += hfi->Vqh;
    Foc->Ud = HFI_Limit(Foc->Ud, HFI_UD_LIMIT, -HFI_UD_LIMIT);
    Foc->Uq = HFI_Limit(Foc->Uq, HFI_UQ_LIMIT, -HFI_UQ_LIMIT);

    Invpark(Foc, theta_hfi);
    Svpwm(Foc);

    return theta_hfi;
}
