#ifndef __HFI_H
#define __HFI_H

#include "Foc_Function.h"
#include "stdint.h"
#include "math.h"
#include "main.h"

/******* 高频方波注入结构体 *******/
typedef struct
{
    uint8_t enable;         // 使能标志，1开启，0关闭
    float Ts;               // 控制周期，单位：s
    float Vh;               // 高频注入电压幅值，单位：V
    int8_t inject_dir;      // 当前注入方向，+1或-1
    float Vdh;              // d轴注入电压，单位：V
    float Vqh;              // q轴注入电压，固定为0

    float i_alpha_last;     // 上一拍alpha轴电流
    float i_beta_last;      // 上一拍beta轴电流
    float i_alpha_h;        // 解调后的高频alpha轴电流
    float i_beta_h;         // 解调后的高频beta轴电流
    float Idh;              // 高频电流d轴分量
    float Iqh;              // 高频电流q轴分量
    float LPF_K;            // 高频电流低通系数

    float Kp;               // PLL比例增益
    float Ki;               // PLL积分增益
    float Up;               // PLL比例项
    float Ui;               // PLL积分项
    float Err;              // PLL误差
    float Est_we;           // 估计电角速度 rad/s
    int16_t Est_RPM;        // 估计机械转速 RPM
    float Est_theta;        // HFI估计电角度
    uint16_t Est_theta_int; // 1024格式的电角度
    float raw_theta;        // atan2直接算出的角度，仅用于调试

    int8_t demod_dir;       // 本拍解调用到的注入方向
    uint8_t polarity_ok;    // 极性判断完成标志
    uint8_t polarity;       // 极性结果，0不翻转，1角度加pi
    uint16_t polarity_cnt;  // 极性判断计数
    uint16_t polarity_samples;// 极性判断采样次数
    float idh_pos_sum;      // 正向注入时的Idh累计
    float idh_neg_sum;      // 反向注入时的Idh累计
    float polarity_diff;    // 正反向响应差值
    float polarity_threshold;// 极性判断阈值

} HFI_TypeDef;

/**** 结构体声明 ****/
extern HFI_TypeDef HFI;

/**** 函数声明 ****/
void  HFI_Reset(HFI_TypeDef *hfi, float theta);                                                       //HFI复位
float HFI_Update(HFI_TypeDef *hfi, float i_alpha, float i_beta);                                      //HFI角度估算和方波更新
uint8_t HFI_PolarityDetect(HFI_TypeDef *hfi);                                                        //HFI极性判断
float HFI_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, HFI_TypeDef *hfi,
                  float IU, float IV, float IW, float Iq_ref);                                       //HFI电流环示例

#endif
