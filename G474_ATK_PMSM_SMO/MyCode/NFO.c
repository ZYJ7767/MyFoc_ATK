#include "NFO.h"
#include "arm_math.h"

#define NFO_EPS             0.000001f
#define NFO_PLL_ERR_LIMIT   0.5235987756f
#define NFO_THETA_INT_K     162.9746617f

NonlinearFluxObserver NFO = {   0.375f,        //Rs
                                0.000305f,     //Ls
                                0.00512f,      //phi_f
                                40000000.0f,   //gamma
                                0.0001f,       //Ts
                                0.00512f,      //x_alpha
                                0.0f,          //x_beta
                                0.00512f,      //eta_alpha
                                0.0f,          //eta_beta
                                0.0f,          //flux_err
                                0.00512f,      //flux_mag
                                0.0f,          //raw_theta
                                888.6f,        //Kp
                                39.48f,        //Ki
                                0.0f,          //Up
                                0.0f,          //Ui
                                0.0f,          //Err
                                2500.0f,       //Ui_Max
                                2500.0f,       //We_Max
                                0.0f,          //Est_we
                                0,             //Est_RPM
                                0.0f,          //Est_theta
                                0              //Est_theta_int
                           };

/***** 限幅函数 *****/
static float NFO_Limit(float x, float max, float min)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/**************** NFO状态复位 ****************/
void NFO_Reset(NonlinearFluxObserver *nfo, float theta, float i_alpha, float i_beta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;

    if (nfo == 0)
    {
        return;
    }

    theta = Normalize_theta(theta);
    arm_sin_cos_f32(theta * RAD_TO_DEG, &SinValue, &CosValue);

    nfo->x_alpha  = nfo->Ls * i_alpha + nfo->phi_f * CosValue;
    nfo->x_beta   = nfo->Ls * i_beta  + nfo->phi_f * SinValue;
    nfo->eta_alpha = nfo->phi_f * CosValue;
    nfo->eta_beta  = nfo->phi_f * SinValue;
    nfo->flux_err  = 0.0f;
    nfo->flux_mag  = nfo->phi_f;
    nfo->raw_theta = theta;

    nfo->Up = 0.0f;
    nfo->Ui = 0.0f;
    nfo->Err = 0.0f;
    nfo->Est_we = 0.0f;
    nfo->Est_RPM = 0;
    nfo->Est_theta = theta;
    nfo->Est_theta_int = (uint16_t)(theta * NFO_THETA_INT_K);
}

/**************** NFO+PLL 离散非线性磁链观测器 ****************/
float NFO_PLL_Update(NonlinearFluxObserver *nfo, float u_alpha, float u_beta, float i_alpha, float i_beta)
{
    float y_alpha = 0.0f;
    float y_beta  = 0.0f;
    float eta_sq  = 0.0f;   //当前估计磁链幅值的平方
    float phi_sq  = 0.0f;   //目标永磁磁链幅值的平方
    float k_nfo   = 0.0f;   //中间变量用以储存gamma/2 * (phi_f^2 - |eta|^2)
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    
    /***** 保护 *****/
    if (nfo == 0) return 0.0f;
    if (nfo->Ts <= 0.0f)    nfo->Ts = 0.0001f;
    if (nfo->phi_f <= NFO_EPS) nfo->phi_f = NFO_EPS;

    /***** 电压模型输入 y=u-Rs*i *****/
    y_alpha = u_alpha - nfo->Rs * i_alpha;
    y_beta  = u_beta  - nfo->Rs * i_beta;

    /***** 转子磁链 eta=x-Ls*i *****/
    nfo->eta_alpha = nfo->x_alpha - nfo->Ls * i_alpha;
    nfo->eta_beta  = nfo->x_beta  - nfo->Ls * i_beta;

    eta_sq = nfo->eta_alpha * nfo->eta_alpha + nfo->eta_beta * nfo->eta_beta;
    phi_sq = nfo->phi_f * nfo->phi_f;
    nfo->flux_err = phi_sq - eta_sq;

    /***** 欧拉离散：x(k)=x(k-1)+Ts*[y+gamma/2*eta*(phi_f^2-|eta|^2)] *****/
    k_nfo = 0.5f * nfo->gamma * nfo->flux_err;
    nfo->x_alpha += nfo->Ts * (y_alpha + k_nfo * nfo->eta_alpha);
    nfo->x_beta  += nfo->Ts * (y_beta  + k_nfo * nfo->eta_beta);

    /***** 用更新后的定子磁链重新计算转子磁链 *****/
    nfo->eta_alpha = nfo->x_alpha - nfo->Ls * i_alpha;
    nfo->eta_beta  = nfo->x_beta  - nfo->Ls * i_beta;
    eta_sq = nfo->eta_alpha * nfo->eta_alpha + nfo->eta_beta * nfo->eta_beta;
    nfo->flux_mag = sqrtf(eta_sq);

    /***** 直接角度可用于调试，控制角度使用PLL输出 *****/
    nfo->raw_theta = Normalize_theta(atan2f(nfo->eta_beta, nfo->eta_alpha));

    /***** PLL锁定角度：Err=sin(theta-theta_hat) *****/
    arm_sin_cos_f32(nfo->Est_theta * RAD_TO_DEG, &SinValue, &CosValue);
    nfo->Err = nfo->eta_beta * CosValue - nfo->eta_alpha * SinValue;
    nfo->Err = nfo->Err / (nfo->flux_mag + NFO_EPS);
    nfo->Err = NFO_Limit(nfo->Err, NFO_PLL_ERR_LIMIT, -NFO_PLL_ERR_LIMIT);

    nfo->Up = nfo->Err * nfo->Kp;
    nfo->Ui += nfo->Err * nfo->Ki;
    nfo->Ui = NFO_Limit(nfo->Ui, nfo->Ui_Max, -nfo->Ui_Max);

    nfo->Est_we = nfo->Up + nfo->Ui;
    nfo->Est_we = NFO_Limit(nfo->Est_we, nfo->We_Max, -nfo->We_Max);
    nfo->Est_RPM = (int16_t)(nfo->Est_we * (60.0f / (2.0f * pi * Pn)));

    nfo->Est_theta += nfo->Est_we * nfo->Ts;
    nfo->Est_theta = Normalize_theta(nfo->Est_theta);
    nfo->Est_theta_int = (uint16_t)(nfo->Est_theta * NFO_THETA_INT_K);

    return nfo->Est_theta;
}
