#ifndef __NFO_H
#define __NFO_H

#include "Foc_Function.h"
#include "stdint.h"
#include "math.h"
#include "main.h"

/******* 非线性磁链观测器结构体 *******/
typedef struct
{
    float Rs;               // 相电阻
    float Ls;               // 相电感Ld=Lq=Ls
    float phi_f;            // 永磁体磁链
    float gamma;            // 非线性观测器增益
    float Ts;               // 采样周期

    float x_alpha;          // alpha轴定子磁链估计值
    float x_beta;           // beta轴定子磁链估计值
    float eta_alpha;        // alpha轴转子磁链估计值 x_alpha-Ls*i_alpha
    float eta_beta;         // beta轴转子磁链估计值 x_beta-Ls*i_beta
    float flux_err;         // 磁链幅值误差 phi_f^2-|eta|^2
    float flux_mag;         // 转子磁链幅值
    float raw_theta;        // atan2直接计算的电角度

    float Kp;               // PLL比例增益
    float Ki;               // PLL积分增益，按采样周期整定
    float Up;               // PLL比例项
    float Ui;               // PLL积分项
    float Err;              // PLL相位误差
    float Ui_Max;           // PLL积分限幅
    float We_Max;           // PLL电角速度限幅
    float Est_we;           // 估计电角速度 rad/s
    int16_t Est_RPM;        // 估计机械转速 RPM
    float Est_theta;        // PLL锁定后的电角度
    uint16_t Est_theta_int; // 1024格式的电角度

} NonlinearFluxObserver;

/**** 结构体声明 ****/
extern NonlinearFluxObserver NFO;

/**** 函数声明 ****/
void  NFO_Reset(NonlinearFluxObserver *nfo, float theta, float i_alpha, float i_beta);                                      //NFO复位
float NFO_PLL_Update(NonlinearFluxObserver *nfo, float u_alpha, float u_beta, float i_alpha, float i_beta);                  //NFO PLL

#endif
