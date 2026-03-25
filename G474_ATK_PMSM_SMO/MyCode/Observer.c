#include "Observer.h"


StepMotor Mo            = { 0.295,0.00033,0.00f,0.00f,0.00f              };              //正点原子电机电阻Rs=0.295Ω，电感Ls = 0.00033H
SlidingModeObserver SMO = { 0.9145f,0.2899f,2.55,0.0001,0,0,0,0,0,0,0,0  };              // 1.55   k=0.75-6.25,Ts=0.0001s 
PLL_Handle PLL          = { 350.0,6.25,0,0,0.0001,0,0,0,0,0              };             // 9.898,49     Kp=2ζωn=2*0.707*7     Ki = ωn*ωn=7^2


/**************** SMO+反正切 离散滑膜观测器 袁雷 ****************/
/*****符号函数 *****/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

/*****更新滑膜观测器 *****/
float SMO_Update(SlidingModeObserver *smo,float u_alpha, float u_beta, float i_alpha, float i_beta) 
{
    // 更新alpha轴
    smo->E_alpha = smo->K * sign(smo->est_ialpha - i_alpha);
    smo->E_alpha = LowPassFilter(smo->E_alpha ,0.8);
    smo->est_ialpha = smo->A * smo->est_ialpha + smo->B * (u_alpha - smo->E_alpha);

    // 更新beta轴
    smo->E_beta  = smo->K * sign(smo->est_ibeta - i_beta);
    smo->E_beta  = LowPassFilter(smo->E_beta  ,0.8);
    smo->est_ibeta = smo->A * smo->est_ibeta + smo->B * (u_beta - smo->E_beta);
    
    // 直接反正切
    float e_theta = atan2f((-smo->E_alpha),smo->E_beta) ;
    if(e_theta < 0) e_theta += 2 * 3.141592;
    
    // PLL锁定角度
    
    return e_theta;
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







