#ifndef __ENCODE_H
#define __ENCODE_H

#include "stdint.h"
#include "tim.h"
#include "Foc_Function.h"
#include "main.h"


//硬件参数配置
#define POLE_PAIRS      4                   // 极对数 (根据你的图片: 磁极数8 -> 极对数4)
#define ENCODER_LINES   1000                // 编码器物理线数
#define ENCODER_PPR     (ENCODER_LINES * 4) // STM32 4倍频后的每圈计数值 (4000)

//方向配置
#define ENCODER_DIR     1           

typedef struct
{
    uint16_t Raw_Value;                     // 编码器原始计数值 (0-3999)
    uint16_t Offset;                        // 0电角度对应的编码器偏置值
    float    Mech_Angle;                    // 机械角度 (0 - 2pi)
    float    Elec_Angle;                    // 电气角度 (0 - 2pi) 用于FOC Park变换
    uint16_t Last_Raw_Value;                // 上一次计数值
    int16_t  Speed_RPM;                     // 最终输出的转速 (RPM)
    float    Speed_Flt;                     // 速度滤波中间变量
    
} Encoder_TypeDef;


extern Encoder_TypeDef MyEnc;


void Encoder_Init(void);                    // 初始化函数
void Encoder_Align_Zero(void);              // 找零校准函数
void Encoder_Update_Angle(void);            // 测角度函数
void Encoder_Calculate_Speed(void);         // 测速函数

#endif





