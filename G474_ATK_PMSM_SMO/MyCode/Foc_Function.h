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


#define TS              7000     //ARR
#define Udc             24
#define Pn              4        //极对数

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


extern FOC_TypeDef         MyFoc;
extern PI_CURRENT_TypeDef  C_PI;
extern PI_SPEED_TypeDef    S_PI;
extern PI_POSITION_TypeDef P_PI;

/****功能函数声明****/
//FOC控制函数接口
float My_limit(float *limit, float limit_max, float limit_min);
float Normalize_theta(float theta);
void  Clarke(FOC_TypeDef *Foc);
void  Park(FOC_TypeDef *Foc , float theta);
void  Invpark(FOC_TypeDef *Foc , float theta);
void  Svpwm(FOC_TypeDef *Foc);

//FOC集成函数接口
void  VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta);
void  IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta);
void  CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta);

//控制器函数接口
void  CurrentPI (FOC_TypeDef *Foc , PI_CURRENT_TypeDef *pi_ctrl);
void  SpeedPI   (FOC_TypeDef *Foc , PI_SPEED_TypeDef   *pi_ctrl , float *Iqref);
void  PositionPI(FOC_TypeDef *Foc, PI_POSITION_TypeDef *pi_ctrl, int16_t *Speedref);



#endif
