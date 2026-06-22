#ifndef __EKF_H
#define __EKF_H

#include "Foc_Function.h"
#include "stdint.h"
#include "math.h"
#include "main.h"

/******* EKF状态变量下标 *******
 * x[0] = i_alpha：alpha轴电流估计值
 * x[1] = i_beta ：beta轴电流估计值
 * x[2] = we     ：电角速度估计值，单位rad/s
 * x[3] = theta  ：电角度估计值，单位rad
 */
typedef enum
{
    EKF_X_IALPHA = 0,
    EKF_X_IBETA,
    EKF_X_WE,
    EKF_X_THETA,
    EKF_X_NUM
} EKF_StateIndex_t;

/******* 扩展卡尔曼位置观测器结构体 *******/
typedef struct
{
    float Rs;               // 相电阻
    float Ls;               // 相电感，当前工程Ld=Lq=Ls
    float phi_f;            // 永磁体磁链
    float Ts;               // 采样周期

    float x[4];             // 状态量数组：[i_alpha, i_beta, we, theta]
    float P[16];            // 状态协方差矩阵，4x4按行存放
    float Q[4];             // 过程噪声矩阵对角线：[Qi_alpha, Qi_beta, Qwe, Qtheta]
    float R[2];             // 电流测量噪声矩阵对角线：[Ri_alpha, Ri_beta]
    float F[16];            // 状态转移雅可比矩阵，4x4按行存放，调试用
    float K[8];             // 卡尔曼增益矩阵，4x2按行存放，调试用

    float err_alpha;        // alpha轴电流观测残差 i_alpha_meas - i_alpha_est
    float err_beta;         // beta轴电流观测残差 i_beta_meas - i_beta_est
    float S_det;            // 残差协方差行列式，调试数值稳定性用

    float We_Max;           // 电角速度限幅
    float Est_ialpha;       // 输出：估计alpha轴电流
    float Est_ibeta;        // 输出：估计beta轴电流
    float Est_we;           // 输出：估计电角速度 rad/s
    int16_t Est_RPM;        // 输出：估计机械转速 RPM
    float Est_theta;        // 输出：估计电角度 rad
    uint16_t Est_theta_int; // 输出：1024格式电角度

} EKF_PositionObserver;

/**** 结构体声明 ****/
extern EKF_PositionObserver EKF;

/**** 函数声明 ****/
void  EKF_SetMotor(EKF_PositionObserver *ekf, float Rs, float Ls, float phi_f, float Ts);                 // 设置电机参数和采样周期
void  EKF_SetNoise(EKF_PositionObserver *ekf, float q_i, float q_we, float q_theta, float r_i);            // 设置Q/R噪声参数
void  EKF_Reset(EKF_PositionObserver *ekf, float theta, float i_alpha, float i_beta, float we);            // EKF状态复位
float EKF_Update(EKF_PositionObserver *ekf, float u_alpha, float u_beta, float i_alpha, float i_beta);     // EKF位置观测更新
float EKF_Update_light(EKF_PositionObserver *ekf, float u_alpha, float u_beta, float i_alpha, float i_beta);// EKF轻量版位置观测更新

#endif
