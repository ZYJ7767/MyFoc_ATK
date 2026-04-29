#include "Observer.h"


StepMotor Mo = { 0.295,     //Rs
                 0.00033,   //Ls
                 0.00f,     //phi_f
                 0.00f,     //Ld
                 0.00f      //Lq
                };

SlidingModeObserver SMO = {     0.9145f,    //A
                                0.2899f,    //B
                                2.55,       //K
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
                                0          //E_beta
                           };              // 1.55   k=0.75-6.25,Ts=0.0001s 

PLL_Handle PLL = {  350.0,      //Kp = 2ζωn
                    6.25,       //Ki = ωn*ωn
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


/*****符号函数 *****/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}
/**************** SMO+PLL 离散滑膜观测器 袁雷 ****************/
float SMO_PLL_Update(SlidingModeObserver *smo, PLL_Handle *PLL, float u_alpha, float u_beta, float i_alpha, float i_beta) 
{
    // 更新alpha轴
    smo->E_alpha = smo->K * sign(smo->est_ialpha - i_alpha);
    smo->E_alpha = LowPassFilter(smo->E_alpha ,0.7);    //0.15
    smo->est_ialpha = smo->A * smo->est_ialpha + smo->B * (u_alpha - smo->E_alpha);

    // 更新beta轴
    smo->E_beta  = smo->K * sign(smo->est_ibeta - i_beta);
    smo->E_beta  = LowPassFilter(smo->E_beta  ,0.7);    //0.15
    smo->est_ibeta = smo->A * smo->est_ibeta + smo->B * (u_beta - smo->E_beta);
    
    // PLL锁定角度
    PLL_calculate(PLL, smo->E_alpha, smo->E_beta );
    
    return PLL->Est_theta;
}

/****************  PLL锁相环计算函数  ****************/ 
void PLL_calculate(PLL_Handle *PLL ,float Ealpha ,float Ebeta)
{
    float sin_value = 0;
    float cos_value = 0;
    
    sin_value = sinf(PLL->Est_theta);
    cos_value = cosf(PLL->Est_theta);
    
    PLL->Err = -Ealpha *cos_value - Ebeta *sin_value;
    
    
    PLL->Err = (PLL->Err > 0.5236f)  ?  (0.5236f) : (PLL->Err);                   //当Δθ小于pi/6时，认为sin（Δθ）= Δθ
    PLL->Err = (PLL->Err < -0.5236f) ? (-0.5236f) : (PLL->Err);

    PLL->Up  = PLL->Err * PLL->Kp;
    PLL->Ui += PLL->Err * PLL->Ki;
    
    PLL->Est_we  = PLL->Up + PLL->Ui;
    
    PLL->Est_RPM = (uint16_t)PLL->Est_we * 2.387f;                                //4对极
    MyFoc.speed = PLL->Est_RPM;                                                   //及时将观测速度传给foc结构体
    
    PLL->Est_theta += PLL->Est_we * PLL->Ts;
    
    PLL->Est_theta = Normalize_theta(PLL->Est_theta);
}







