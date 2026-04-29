#include "Foc_Function.h"
#include "math.h"
#include "arm_math.h"
#include "stdint.h"
#include "tim.h"

extern float Enc_Speed;


FOC_TypeDef         MyFoc = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PI_CURRENT_TypeDef  C_PI  = {0,0,0,0,0,0,0,0};
PI_SPEED_TypeDef    S_PI  = {0,0,0,0,0};
PI_POSITION_TypeDef P_PI  = {0,0,0};



float My_limit(float *limit, float limit_max, float limit_min)
{
    if(*limit > limit_max){*limit = limit_max;}
    if(*limit < limit_min){*limit = limit_min;}
    return *limit;
}



float Normalize_theta(float theta)
{
    const float TwoPi = 2.0f * pi;
    if (theta >= TwoPi)
    {
        theta -= TwoPi;
    }
    else if (theta < 0.0f)
    {
        theta += TwoPi;
    }
    return theta;
}



void Clarke(FOC_TypeDef *Foc)
{
    Foc->Ialpha = Foc->Iu; // Ia
    Foc->Ibeta  = _1_sqrt3 * Foc->Iu + _2_sqrt3 * Foc->Iv;
}



void Park(FOC_TypeDef *Foc , float theta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    arm_sin_cos_f32(theta * (180.0f / pi), &SinValue, &CosValue);
    
    Foc->Id =  Foc->Ialpha * CosValue + Foc->Ibeta * SinValue;
    Foc->Iq = -Foc->Ialpha * SinValue + Foc->Ibeta * CosValue;
}



void Invpark(FOC_TypeDef *Foc , float theta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    arm_sin_cos_f32(theta * (180.0f / pi), &SinValue, &CosValue);

    Foc->Ualpha = Foc->Ud * CosValue - Foc->Uq * SinValue;
    Foc->Ubeta  = Foc->Ud * SinValue + Foc->Uq * CosValue;
}



void Svpwm(FOC_TypeDef *Foc)
{

    
    float u1 = Foc->Ubeta;
    float u2 = _sqrt3_2  * Foc->Ualpha - Foc->Ubeta * _1_2;
    float u3 = -_sqrt3_2 * Foc->Ualpha - Foc->Ubeta * _1_2;
    float TmnSun;
  
    
    uint8_t A =u1>0?1:0;
    uint8_t B =u2>0?1:0;
    uint8_t C =u3>0?1:0;
    uint8_t N =4*C+2*B+A;


    float X =  u1 * (_sqrt3*TS/Udc);
    float Y = -u3 * (_sqrt3*TS/Udc);
    float Z = -u2 * (_sqrt3*TS/Udc);
    
    float Tm = 0;
    float Tn = 0;
    
    switch (N){
        case 1:
                Tm = Z;
                Tn = Y;
                break ;
        case 2:
                Tm = Y;
                Tn = -X;     
                break ;            
        case 3:
                Tm = -Z;
                Tn = X;      
                break ;    
        case 4:
                Tm = -X;
                Tn = Z;             
                break ;    
        case 5:
                Tm = X;
                Tn = -Y;     
                break ;    
        case 6:
                Tm = -Y;
                Tn = -Z;      
                break ;    
        default:
            Foc->Tcm1 = 0.5f * (float)ARR;
            Foc->Tcm2 = 0.5f * (float)ARR;
            Foc->Tcm3 = 0.5f * (float)ARR;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Foc->Tcm1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Foc->Tcm2);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Foc->Tcm3);
            return;
    }
    
    TmnSun = Tm+Tn;
    if((Tm+Tn)>TS)
    {
        Tm = (Tm*TS)/(TmnSun);
        Tn = (Tn*TS)/(TmnSun);
    }   
    

    float Ta=(TS-Tm-Tn)/2;
    float Tb= Ta + Tm/1;
    float Tc= Tb + Tn/1;
    
    switch (N)
    {
        case 1:
                Foc->Tcm1=Tb;
                Foc->Tcm2=Ta;
                Foc->Tcm3=Tc;
                break ;
        case 2:
                Foc->Tcm1=Ta;
                Foc->Tcm2=Tc;
                Foc->Tcm3=Tb;
                break ;
        case 3:
                Foc->Tcm1=Ta;
                Foc->Tcm2=Tb;
                Foc->Tcm3=Tc;
                break ;
        case 4:
                Foc->Tcm1=Tc;
                Foc->Tcm2=Tb;
                Foc->Tcm3=Ta;
                break ;
        case 5:
                Foc->Tcm1=Tc;
                Foc->Tcm2=Ta;
                Foc->Tcm3=Tb;
                break ;
        case 6:
                Foc->Tcm1=Tb;
                Foc->Tcm2=Tc;
                Foc->Tcm3=Ta; 
                break ;
    }
    
    if(Foc->Tcm1 >= 6999) Foc->Tcm1=6999;
    if(Foc->Tcm2 >= 6999) Foc->Tcm2=6999;
    if(Foc->Tcm3 >= 6999) Foc->Tcm3=6999;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Foc->Tcm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Foc->Tcm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Foc->Tcm3);
}


void VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta)
{
    Foc->Ud = Ud;
    Foc->Uq = Uq;
    Invpark(Foc,theta);
    Svpwm(Foc);
}


void IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    pi_ctrl->Iq_ref = Iq_ref;
    
    theta = Normalize_theta(theta);
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,pi_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}



void CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    pi_ctrl->Iq_ref = Iq_ref;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,pi_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}



void SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    pi_ctrl->Iq_ref = Iq_ref;
    pi_ctrl->Id_ref = 0;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,pi_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


void SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;

    Clarke(Foc);
    Park(Foc,theta);
    
    float Iq_ref;
    
    S_PI->speed_ref = Speed_ref;
    SpeedPI(Foc, S_PI, &Iq_ref);
    
    C_PI->Iq_ref = Iq_ref;
    C_PI->Id_ref = 0;
    
    CurrentPI(Foc, C_PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


void CurrentPI(FOC_TypeDef *Foc , PI_CURRENT_TypeDef *pi_ctrl)
{

    pi_ctrl->err_Id = pi_ctrl->Id_ref - Foc->Id;
    pi_ctrl->err_Iq = pi_ctrl->Iq_ref - Foc->Iq;

    pi_ctrl->Id_KI_sum += pi_ctrl->err_Id;
    pi_ctrl->Iq_KI_sum += pi_ctrl->err_Iq;
    
    if (pi_ctrl->Id_KI_sum > 30)  pi_ctrl->Id_KI_sum =  30;
    if (pi_ctrl->Id_KI_sum < -30) pi_ctrl->Id_KI_sum = -30;
    if (pi_ctrl->Iq_KI_sum > 60)  pi_ctrl->Iq_KI_sum =  60;
    if (pi_ctrl->Iq_KI_sum < -60) pi_ctrl->Iq_KI_sum = -60;
    

    Foc->Ud =(pi_ctrl->Kp * pi_ctrl->err_Id) + (pi_ctrl->Ki * pi_ctrl->Id_KI_sum);
    Foc->Uq =(pi_ctrl->Kp * pi_ctrl->err_Iq) + (pi_ctrl->Ki * pi_ctrl->Iq_KI_sum);
    
        float L = 0.000305f; 
        float Psi = 0.00512f;
        
        float Vd_ff = -Enc_Speed * 0.418879f* L * Foc->Iq;
        float Vq_ff = Enc_Speed * 0.418879f* L * Foc->Id + Enc_Speed * 0.418879f * Psi;
        
        Foc->Ud += Vd_ff;
        Foc->Uq += Vq_ff;

    if (Foc->Ud > 5)   Foc->Ud =  5;
    if (Foc->Ud < -5)  Foc->Ud = -5;
    if (Foc->Uq > 13.8)   Foc->Uq =  13.8;
    if (Foc->Uq < -13.8)  Foc->Uq = -13.8;
}


void SpeedPI(FOC_TypeDef *Foc, PI_SPEED_TypeDef *pi_ctrl, float *Iqref)
{

    pi_ctrl->err_speed = pi_ctrl->speed_ref - Foc->speed;
    

    pi_ctrl->speed_KI_sum += pi_ctrl->err_speed;
    
    if (pi_ctrl->speed_KI_sum >  800000) pi_ctrl->speed_KI_sum =  800000;
    if (pi_ctrl->speed_KI_sum < -800000) pi_ctrl->speed_KI_sum = -800000;
    

    (*Iqref) =(pi_ctrl->Kp * pi_ctrl->err_speed) + (pi_ctrl->Ki * pi_ctrl->speed_KI_sum);


    if (*Iqref >  3.0f)  *Iqref =  3.0f;
//    if (*Iqref < 0)  *Iqref = 0;
    if (*Iqref < -3.0f)  *Iqref = -3.0f;
}


void PositionPI(FOC_TypeDef *Foc, PI_POSITION_TypeDef *pi_ctrl, int16_t *Speedref)
{
#define MECH_PPR  4000.0f

    float err = (float)pi_ctrl->position_ref - (float)Foc->position;


    if (err < 0.0f && err > -150.0f)
    {
        err = 0.0f; 
    }
    else if (err <= -150.0f)
    {
        err += MECH_PPR;
    }

    if (fabsf(err) < 5.0f)
    {
        *Speedref = 0;
        pi_ctrl->err_position = 0;
        pi_ctrl->Last_Err = err;
        return;
    }
    

    float P_Term = pi_ctrl->Kp * err;
    float D_Term = pi_ctrl->Kd * (err - pi_ctrl->Last_Err);
    
    float target_speed = P_Term + D_Term;


    pi_ctrl->Last_Err = err;


    if (target_speed >  300.0f) target_speed =  300.0f;
    if (target_speed <  0.0f)   target_speed =  0.0f;
    *Speedref = (int16_t)target_speed;
}
















