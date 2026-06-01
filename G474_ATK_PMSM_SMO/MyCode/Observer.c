#include "Observer.h"
#include "Filter.h"
#include "arm_math.h"

StepMotor Mo = { 0.375,     //Rs
                 0.000305,   //Ls
                 0.00512f,     //phi_f
                 0.000305f,     //Ld
                 0.000305f      //Lq
                };

SlidingModeObserver SMO = {     0.8843f,    //A
                                0.3086f,    //B
                                12,         //K
                                0.0001,     //Ts
                                0,          //est_Theta
                                0,          //prev_theta
                                0,          //est_Speed
                                0,          //est_ialpha
                                0,          //est_ibeta
                                0,          //est_ialpha_dt
                                0,          //est_ibeta_dt
                                0,          //phase_delay
                                0,          //E_alpha
                                0           //E_beta
                           };               // 1.55   k=0.75-6.25,Ts=0.0001s 

PLL_Handle PLL = {  888.6,      //Kp = 2ζωn
                    39.48,      //Ki = ωn*ωn  100hz
                    0,          //Up
                    0,          //Ui
                    0.0001,     //Ts
                    0,          //Err
                    0,          //Est_we
                    0,          //Est_RPM
                    0,          //Est_theta
                    0,          //Est_theta_int
                    0           //Pre_Est_Theta
                 };

LPF1_t LPF_Ealpha = {0};
LPF1_t LPF_Ebeta  = {0};

/*****符号函数 *****/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

float sat(float x)
{
    if (x > 0.5f)  return 1.0f;
    if (x < -0.5f) return -1.0f;
    return x / 0.5f;   // 边界层内线性
}

float sigmoid(float x)
{
    return tanhf(3.0f * x);
}
/**************** SMO+PLL 离散滑膜观测器 袁雷 ****************/
float SMO_PLL_Update(SlidingModeObserver *smo, PLL_Handle *PLL, float u_alpha, float u_beta, float i_alpha, float i_beta) 
{
    // 更新alpha轴
    smo->E_alpha = smo->K * sign(smo->est_ialpha - i_alpha);
    smo->E_alpha = LPF1_Update(&LPF_Ealpha, smo->E_alpha, 0.7); 
    smo->est_ialpha = smo->A * smo->est_ialpha + smo->B * (u_alpha - smo->E_alpha);

    // 更新beta轴
    smo->E_beta  = smo->K * sign(smo->est_ibeta - i_beta);
    smo->E_beta  = LPF1_Update(&LPF_Ebeta, smo->E_beta, 0.7); 
    smo->est_ibeta = smo->A * smo->est_ibeta + smo->B * (u_beta - smo->E_beta);
    
    // PLL锁定角度
    PLL_calculate(PLL, smo->E_alpha, smo->E_beta );
    
    return PLL->Est_theta;
}

/****************  PLL锁相环计算函数  ****************/ 
void PLL_calculate(PLL_Handle *PLL ,float Ealpha ,float Ebeta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    float Em_Mag   = 0.0f;  // [新增] 用于存储反电动势幅值
    
    arm_sin_cos_f32(PLL->Est_theta * RAD_TO_DEG, &SinValue, &CosValue);
    
    PLL->Err = -Ealpha *CosValue - Ebeta *SinValue;
    Em_Mag = sqrtf(Ealpha * Ealpha + Ebeta * Ebeta);
    PLL->Err = PLL->Err / (Em_Mag + 0.001f);
    
    PLL->Err = (PLL->Err > 0.5236f)  ?  (0.5236f) : (PLL->Err);                   //当Δθ小于pi/6时，认为sin（Δθ）= Δθ
    PLL->Err = (PLL->Err < -0.5236f) ? (-0.5236f) : (PLL->Err);

    PLL->Up  = PLL->Err * PLL->Kp;
    PLL->Ui += PLL->Err * PLL->Ki;
    
//    if (PLL->Ui > 2500.0f)  PLL->Ui = 2500.0f;
//    if (PLL->Ui < -2500.0f) PLL->Ui = -2500.0f;
    
    PLL->Est_we  = PLL->Up + PLL->Ui;
    
//    if (PLL->Est_we > 2500.0f)  PLL->Est_we = 2500.0f;
//    if (PLL->Est_we < -2500.0f) PLL->Est_we = -2500.0f;
    

    PLL->Est_RPM = PLL->Est_we * 2.387f;                                //1.194f   8对极   2.387f   4duiji
    
    PLL->Est_theta += PLL->Est_we * PLL->Ts;
    
    PLL->Est_theta = Normalize_theta(PLL->Est_theta);
}







