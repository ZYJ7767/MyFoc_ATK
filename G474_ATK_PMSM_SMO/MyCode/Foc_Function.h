#ifndef __FOC_H
#define __FOC_H

#include "stdint.h"
#include "main.h"



//宏定义
#define  pi             3.1415926535f
#define _2sqrt3_3       1.1547005383f
#define _sqrt3_3        0.5773502691f
#define _sqrt3_2        0.8660254037844f
#define _sqrt3          1.7320508075688f
#define _1_sqrt3        0.57735026919f 
#define _2_sqrt3        1.15470053838f 
#define _1_2            0.5f
#define _2_3            0.6666666666666f


#define RAD_TO_DEG      57.2957795131f

#define ARR             7000
#define TS              7000
#define Udc             24
#define Pn              4        //极对数

#define ESO_SUBSTEPS    5


//FOC控制电机结构体
typedef struct
{
    float Iu;
    float Iv;
    float Iw;
    float Ialpha;
    float Ibeta;
    float Id;
    float Iq;
    float Ud;
    float Uq;
    float Ualpha;
    float Ubeta;
    float Tcm1;
    float Tcm2;
    float Tcm3;
    float Ealpha;
    float Ebeta;
    float Ialpha_prev;
    float Ibeta_prev;
    float PIalpha;
    float PIbeta;
    int16_t  speed;
    uint16_t position;  
    
}FOC_TypeDef;

//电流环PI控制器结构体
typedef struct
{
    float Id_ref;
    float Iq_ref;
    float err_Id;
    float err_Iq;
    float Ki;
    float Kp;
    float Id_KI_sum;
    float Iq_KI_sum;
    
}PI_CURRENT_TypeDef;

//速度环PI控制器结构体
typedef struct
{
    int16_t speed_ref;
    float err_speed;         //RPM
    float Ki;
    float Kp;
    float speed_KI_sum;

}PI_SPEED_TypeDef;

//位置环P控制器
typedef struct
{
    float position_ref;
    uint16_t err_position;
    float Kp;
    float Kd;
    float Last_Err;
}PI_POSITION_TypeDef;

//速度环LADRC控制器(结构体为二阶LADRC，速度环这里用一阶)
typedef struct
{
    float speed_ref;     // 速度给定，单位：RPM
    float speed_fdb;     // 速度反馈，单位：RPM
    float err_speed;     // 速度误差，单位：RPM

    float h;             // 控制周期，单位：s，1ms = 0.001f
    float b0;            // 名义对象增益，单位：RPM/s^2/A
    float wc;            // 控制器带宽，单位：rad/s
    float w0;            // 观测器带宽，单位：rad/s
    float kp;            // 线性状态误差反馈参数，kp = wc * wc
    float kd;            // 线性状态误差反馈参数，kd = 2 * wc

    float l1;            // ESO参数，l1 = 3 * w0
    float l2;            // ESO参数，l2 = 3 * w0 * w0
    float l3;            // ESO参数，l3 = w0 * w0 * w0

    float z1;            // ESO估计的速度
    float z2;            // ESO估计的速度变化率
    float z3;            // ESO估计的总扰动
    
    float dz1_last;      // ESO导数缓存（梯形积分用）
    float dz2_last;
    float dz3_last;

    float e;             // ESO观测误差
    float u0;            // 未补偿控制量
    float u;             // 最终输出
    float last_u;        // 上一拍输出，用于离散ESO更新

    float Iq_Max;        // Iq上限，单位：A
    float Iq_Min;        // Iq下限，单位：A
    float Iq_Rate;       // Iq变化率限制，单位：A/s，0表示关闭
} LADRC_SPEED_TypeDef;

// 速度环 SMC 滑模控制器
typedef struct
{
    float speed_ref;     // 速度给定，单位 RPM
    float speed_fdb;     // 速度反馈，单位 RPM
    float err_speed;     // 速度误差 speed_ref - speed_fdb
    float err_sum;       // 误差积分，用于构造滑模面
    float s;             // 滑模面 s = err + c * err_sum

    float h;             // 控制周期，单位 s
    float c;             // 滑模面系数，越大稳态误差收敛越快
    float Ksw;           // 滑模增益，决定最大输出能力
    float phi;           // 边界层厚度，越大越平滑，越小越硬

    float out;           // 当前输出 Iqref
    float last_out;      // 上一次输出
    float out_max;       // 输出上限，单位 A
    float out_min;       // 输出下限，单位 A
    float out_rate;      // 输出变化率限制，单位 A/s，0表示关闭
    float sum_max;       // 积分限幅
} SMC_TypeDef;


// 弱磁控制器结构体
typedef struct
{
    float Vdc;         // 实时直流母线电压，单位V
    float Vs_max;      // 电压极限边界 (带一定裕量)
    float Vs_ref;      // 当前合成电压矢量幅值
    float err_V;       // 电压误差
    
    // PI 参数
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float fw_KI_sum;   // 积分项 (单向、防饱和)
    
    float id_fw;       // 输出的弱磁电流指令 (负值)
    float id_fw_min;   // 深度弱磁硬限幅 (如 -Is_max 或 -ψf/Ld)
    float Is_max;      // 系统最大允许相电流 (A)
    
} PI_FW_TypeDef;


// MIT关节控制器结构体
typedef struct
{
    float p_des;          // 期望机械位置，单位rad
    float v_des;          // 期望机械速度，单位rad/s
    float kp;             // 位置刚度，单位Nm/rad
    float kd;             // 速度阻尼，单位Nm/(rad/s)
    float tau_ff;         // 前馈转矩，单位Nm

    float kt;             // 转矩常数，单位Nm/A，tau = kt * Iq
    float h;              // 控制周期，单位s
    float iq_max;         // Iq输出上限，单位A
    float iq_min;         // Iq输出下限，单位A
    float iq_rate;        // Iq变化率限制，单位A/s，<=0表示关闭

    float p_fdb;          // 当前机械位置反馈，单位rad
    float v_fdb;          // 当前机械速度反馈，单位rad/s
    float err_p;          // 位置误差
    float err_v;          // 速度误差
    float tau_cmd;        // MIT计算出的目标转矩，单位Nm
    float iq_cmd;         // MIT计算出的目标Iq，单位A
    float last_iq_cmd;    // 上一拍Iq输出，单位A
} MIT_TypeDef;



extern FOC_TypeDef         MyFoc;

extern PI_CURRENT_TypeDef  C_PI;
extern PI_SPEED_TypeDef    S_PI;
extern PI_POSITION_TypeDef P_PI;
extern LADRC_SPEED_TypeDef S_LADRC;
extern SMC_TypeDef         S_SMC;
extern MIT_TypeDef         MIT_Ctrl;

extern PI_FW_TypeDef       FW_PI;
extern const float CogComp_Table[256];//齿槽转矩补偿表


/************ 功能函数声明 ************/
//FOC控制函数接口
float My_limit(float *limit, float limit_max, float limit_min);
float Normalize_theta(float theta);
void  Clarke(FOC_TypeDef *Foc);
void  Park(FOC_TypeDef *Foc , float theta);
void  Invpark(FOC_TypeDef *Foc , float theta);
void  Svpwm(FOC_TypeDef *Foc);

//FOC集成函数接口
void  VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta);
void  IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float Id_ref, float theta);
void  CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta);

//PI控制器函数接口
void  CurrentPI (FOC_TypeDef *Foc , PI_CURRENT_TypeDef *pi_ctrl);
void  SpeedPI   (FOC_TypeDef *Foc , PI_SPEED_TypeDef   *pi_ctrl , float *Iqref);
void  PositionPI(int32_t actual_pos, int32_t target_pos, PI_POSITION_TypeDef *pi_ctrl, int16_t *Speedref);

//LADRC控制器函数接口
void  SpeedLADRC_CalcGain(LADRC_SPEED_TypeDef *ctrl);
void  SpeedLADRC_Init(LADRC_SPEED_TypeDef *ctrl);
void  SpeedLADRC_Reset(LADRC_SPEED_TypeDef *ctrl, float speed_now);
void  SpeedLADRC(FOC_TypeDef *Foc, LADRC_SPEED_TypeDef *ctrl, float speed_ref, float *Iqref);

//SMC控制器函数接口
void  SMC_Init(SMC_TypeDef *ctrl);
void  SMC_Reset(SMC_TypeDef *ctrl, float iq_now);
void  SpeedSMC(FOC_TypeDef *Foc, SMC_TypeDef *ctrl, float speed_ref, float *Iqref);

//MTPA控制函数接口
void  MTPA_Control(float *Target_id, float flux, float Ld, float Lq, float iq);

//弱磁控制函数接口
void  FieldWeakening_Control(FOC_TypeDef *Foc, PI_FW_TypeDef *FW_PI, float Iq_ref_in, float *Id_ref_out, float *Iq_ref_out);

//MIT控制模式函数接口
void  MIT_Init(MIT_TypeDef *ctrl);
void  MIT_Reset(MIT_TypeDef *ctrl, float iq_now);
void  MIT_SetCommand(MIT_TypeDef *ctrl, float p_des, float v_des, float kp, float kd, float tau_ff);
void  MIT_CalcIq(MIT_TypeDef *ctrl, float p_mech_rad, float v_mech_rad_s, float *Iqref);
void  MIT_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, MIT_TypeDef *mit_ctrl,
                  float IU, float IV, float IW,
                  float p_mech_rad, float v_mech_rad_s, float theta);

                  








#endif


