#ifndef __OBSERVER_H
#define __OBSERVER_H

#include "Foc_Function.h"
#include "stdint.h"
#include "math.h"
#include "main.h"

/******* 电机参数结构体 *******/
typedef struct
{
    float Rs ;
    float Ls ;
    float phi_f;
    float Ld ;
    float Lq ;
    
}StepMotor;

/******* SMO 结构体 *******/
typedef struct
{
    float A;                // A=exp(-R/LsTs)
    float B;                // B=(1-A)1/R 袁雷
    float K;                // 滑膜增益
    float Ts;               // 采样周期
    float est_Theta;        // 转子位置
    float prev_theta;       // 上一次的转子位置
    float est_Speed;        // 转子速度
    float est_ialpha;       // 估计的 i_alpha 电流
    float est_ibeta;        // 估计的 i_beta 电流
    float est_ialpha_dt;
    float est_ibeta_dt;
    float phase_delay;      // 相位延迟值
    float E_alpha;          // alpha轴反电动势
    float E_beta;           // beta轴反电动势
    
    
} SlidingModeObserver;

/******* PLL结构体 *******/
typedef struct
{
    float Kp ;              // 2ζωn  当ζ=0.707时，自然频率ωn就为系统带宽ωb
    float Ki ;              // ωn*ωn
    float Up ;
    float Ui ;
    float Ts ;
    float Err ;
    float Est_we;           // 估计电角速度
    uint16_t Est_RPM;       // 估计 转速
    float Est_theta;        // 估计电角度
    uint16_t Est_theta_int; // 1024格式的电角度
    float Pre_Est_Theta;    // 上一次估计电角度
    
}PLL_Handle;

/**** 结构体声明 ****/
extern StepMotor Mo;
extern SlidingModeObserver SMO;
extern PLL_Handle PLL;

/**** 函数声明 ****/
float sign(float x);                                                                                                            //符号函数
void  PLL_calculate(PLL_Handle *PLL ,float Ealpha ,float Ebeta);                                                                //PLL函数
float SMO_Update(SlidingModeObserver *smo, float u_alpha, float u_beta, float i_alpha, float i_beta);                           //SMO atan
float SMO_PLL_Update(SlidingModeObserver *smo, PLL_Handle *PLL, float u_alpha, float u_beta, float i_alpha, float i_beta);      //SMO PLL




#endif

