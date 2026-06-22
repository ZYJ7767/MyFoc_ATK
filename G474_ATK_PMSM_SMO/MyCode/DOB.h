#ifndef __DOB_H
#define __DOB_H

#include <stdint.h>

/*
 * DOB（扰动转矩观测器）
 *
 * 适用对象：
 *   J * omega_dot + B * omega = Kt * Iq - TL
 *
 * 变量单位：
 *   Iq        : A
 *   speed_rpm : rpm，机械转速，直接使用编码器速度
 *   omega     : rad/s，机械角速度
 *   TL_hat    : N*m，估计负载/扰动转矩
 *
 * 默认电机参数来自 MyCode/ReadMe.c 的 J/B 辨识结果。
 */

#define DOB_TS_S                 (0.001f)        // 速度环/观测器默认周期：1ms
#define DOB_KT_NM_A              (0.030000f)     // 转矩常数：N*m/A
#define DOB_J_KG_M2              (6.858516e-06f) // 转动惯量：kg*m^2
#define DOB_B_NM_S_RAD           (3.071631e-05f) // 粘滞摩擦系数：N*m*s/rad
#define DOB_RPM_TO_RAD_S         (0.1047197551f) // 2*pi/60


/***************************************** Q-DOB / 降阶 Luenberger DOB *****************************************/
/*
 * 说明：
 * 1. 本模块实现一阶 Q-DOB，同时也是降阶 Luenberger 负载转矩观测器的等价实现。
 * 2. 不直接对速度做微分，避免编码器速度噪声被 J*s 放大。
 * 3. 连续形式等价于：
 *
 *      TL_hat = Q(s) * [Kt*Iq - B*omega - J*s*omega]
 *      Q(s)   = wq / (s + wq)
 *
 * 4. 这里用内部状态 z 避开 omega_dot：
 *
 *      z_dot  = -wq*z + (J*wq*wq - B*wq)*omega + Kt*wq*Iq
 *      TL_hat = z - J*wq*omega
 *
 * 5. wq 越大，观测越快，但速度噪声越明显；建议先从 50~100rad/s 试起。
 */
typedef struct
{
    float h;                // 采样周期，单位：s，默认1ms
    float Kt;               // 转矩常数，单位：N*m/A
    float J;                // 转动惯量，单位：kg*m^2
    float B;                // 粘滞摩擦系数，单位：N*m*s/rad
    float wq;               // Q滤波器带宽，单位：rad/s
    float tl_abs_limit_nm;  // TL_hat绝对限幅，单位：N*m；<=0表示不启用
} QDOB_Config_t;

typedef struct
{
    QDOB_Config_t cfg;

    float omega_rad_s;      // 当前机械角速度，单位：rad/s
    float te_nm;            // 当前电磁转矩，Te = Kt*Iq，单位：N*m
    float z;                // 降阶观测器内部状态
    float tl_hat_nm;        // 输出：估计负载/扰动转矩，单位：N*m

    uint8_t inited;         // 1=已用当前速度对状态初始化
} QDOB_Handle_t;


/***************************************** 全阶 Luenberger DOB *****************************************/
/*
 * 说明：
 * 1. 全阶观测器状态为 [omega_hat, TL_hat]。
 * 2. omega_hat 不是替代编码器速度，而是模型预测速度；用 omega_meas - omega_hat 修正 TL_hat。
 * 3. 连续形式：
 *
 *      omega_hat_dot = -B/J*omega_hat + Kt/J*Iq - TL_hat/J
 *                      + l1*(omega_meas - omega_hat)
 *
 *      TL_hat_dot    = l2*(omega_meas - omega_hat)
 *
 * 4. 注意本工程机械方程里 TL 是负载阻力项：
 *
 *      J*omega_dot + B*omega = Kt*Iq - TL
 *
 *    因此极点配置时 l2 为负值，这是符号约定决定的，不是写反。
 */
typedef struct
{
    float h;                // 采样周期，单位：s，默认1ms
    float Kt;               // 转矩常数，单位：N*m/A
    float J;                // 转动惯量，单位：kg*m^2
    float B;                // 粘滞摩擦系数，单位：N*m*s/rad
    float wo;               // 全阶观测器重复极点带宽，单位：rad/s
    float tl_abs_limit_nm;  // TL_hat绝对限幅，单位：N*m；<=0表示不启用
} FullLuenbergerDOB_Config_t;

typedef struct
{
    FullLuenbergerDOB_Config_t cfg;

    float omega_meas_rad_s; // 当前测量机械角速度，单位：rad/s
    float omega_hat_rad_s;  // 观测器估计/预测机械角速度，单位：rad/s
    float tl_hat_nm;        // 输出：估计负载/扰动转矩，单位：N*m
    float te_nm;            // 当前电磁转矩，Te = Kt*Iq，单位：N*m
    float err_omega_rad_s;  // 速度观测误差：omega_meas - omega_hat

    float l1;               // 全阶观测器增益1
    float l2;               // 全阶观测器增益2，本符号约定下一般为负

    uint8_t inited;         // 1=已用当前速度对状态初始化
} FullLuenbergerDOB_Handle_t;


/************* Q-DOB / 降阶 Luenberger DOB 对外接口 *************/
void  QDOB_DefaultConfig(QDOB_Config_t *cfg);
void  QDOB_Init(QDOB_Handle_t *h, const QDOB_Config_t *cfg);
void  QDOB_Reset(QDOB_Handle_t *h, float speed_rpm);
float QDOB_Update_1kHz(QDOB_Handle_t *h, float iq_meas_a, float speed_rpm);


/************* 全阶 Luenberger DOB 对外接口 *************/
void  FullLuenbergerDOB_DefaultConfig(FullLuenbergerDOB_Config_t *cfg);
void  FullLuenbergerDOB_Init(FullLuenbergerDOB_Handle_t *h, const FullLuenbergerDOB_Config_t *cfg);
void  FullLuenbergerDOB_Reset(FullLuenbergerDOB_Handle_t *h, float speed_rpm);
void  FullLuenbergerDOB_CalcGain(FullLuenbergerDOB_Handle_t *h);
float FullLuenbergerDOB_Update_1kHz(FullLuenbergerDOB_Handle_t *h, float iq_meas_a, float speed_rpm);

#endif
