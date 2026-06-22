#include "Foc_Function.h"
#include "math.h"
#include "arm_math.h"
#include "stdint.h"
#include "tim.h"
#include "DeadTime.h"

extern float Enc_Speed;

FOC_TypeDef         MyFoc   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PI_CURRENT_TypeDef  C_PI    = {0,0,0,0,0,0,0,0};
PI_SPEED_TypeDef    S_PI    = {0,0,0,0,0};
PI_POSITION_TypeDef P_PI    = {0,0,0};
LADRC_SPEED_TypeDef S_LADRC = {0};
SMC_TypeDef         S_SMC   = {0};
MIT_TypeDef         MIT_Ctrl = {0};
PI_FW_TypeDef       FW_PI   = {0};




/***************************************** FOC 算法  *************************************************/
float My_limit(float *limit, float limit_max, float limit_min)
{
    if(*limit > limit_max){*limit = limit_max;}
    if(*limit < limit_min){*limit = limit_min;}
    return *limit;
}



float Normalize_theta(float theta)
{
    const float TwoPi = 2.0f * pi;
    while (theta >= TwoPi)
    {
        theta -= TwoPi;
    }

    while (theta < 0.0f)
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
    
        DeadTimeComp_ApplyCompareF32(&DTC, Foc->Iu, Foc->Iv, Foc->Iw, &Foc->Tcm1, &Foc->Tcm2, &Foc->Tcm3);
    
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Foc->Tcm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Foc->Tcm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Foc->Tcm3);
}


/***************************************** 控制模式接口  *************************************************/
void VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta)
{
    Foc->Ud = Ud;
    Foc->Uq = Uq;
    Invpark(Foc,theta);
    Svpwm(Foc);
}


void IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, float IU, float IV, float IW, float Iq_ref, float Id_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    pi_ctrl->Iq_ref = Iq_ref;
    pi_ctrl->Id_ref = Id_ref;
    
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


/***************************************** PI控制器  *************************************************/
void CurrentPI(FOC_TypeDef *Foc , PI_CURRENT_TypeDef *pi_ctrl)
{

    pi_ctrl->err_Id = pi_ctrl->Id_ref - Foc->Id;
    pi_ctrl->err_Iq = pi_ctrl->Iq_ref - Foc->Iq;

    pi_ctrl->Id_KI_sum += pi_ctrl->err_Id;
    pi_ctrl->Iq_KI_sum += pi_ctrl->err_Iq;
    
    if (pi_ctrl->Id_KI_sum > 60)  pi_ctrl->Id_KI_sum =  60;
    if (pi_ctrl->Id_KI_sum < -60) pi_ctrl->Id_KI_sum = -60;
    if (pi_ctrl->Iq_KI_sum > 60)  pi_ctrl->Iq_KI_sum =  60;
    if (pi_ctrl->Iq_KI_sum < -60) pi_ctrl->Iq_KI_sum = -60;

    Foc->Ud =(pi_ctrl->Kp * pi_ctrl->err_Id) + (pi_ctrl->Ki * pi_ctrl->Id_KI_sum);
    Foc->Uq =(pi_ctrl->Kp * pi_ctrl->err_Iq) + (pi_ctrl->Ki * pi_ctrl->Iq_KI_sum);
    
        //反电动势前馈解耦
        float L = 0.000305f; 
        float Psi = 0.00512f;
        float Vd_ff = -Enc_Speed * 0.418879f * L * Foc->Iq;
        float Vq_ff = Enc_Speed * 0.418879f * L * Foc->Id + Enc_Speed * 0.418879f * Psi;
        Foc->Ud += Vd_ff;
        Foc->Uq += Vq_ff;

    if (Foc->Ud > 10)   Foc->Ud =  10;
    if (Foc->Ud < -10)  Foc->Ud = -10;
    if (Foc->Uq > 13.8f)   Foc->Uq =  13.8f;
    if (Foc->Uq < -13.8f)  Foc->Uq = -13.8f;
}


void SpeedPI(FOC_TypeDef *Foc, PI_SPEED_TypeDef *pi_ctrl, float *Iqref)
{

    pi_ctrl->err_speed = pi_ctrl->speed_ref - Foc->speed;

    pi_ctrl->speed_KI_sum += pi_ctrl->err_speed;
    
    if (pi_ctrl->speed_KI_sum > 10000) pi_ctrl->speed_KI_sum =  10000;
    if (pi_ctrl->speed_KI_sum < -10000) pi_ctrl->speed_KI_sum = -10000;

    (*Iqref) =(pi_ctrl->Kp * pi_ctrl->err_speed) + (pi_ctrl->Ki * pi_ctrl->speed_KI_sum);

    if (*Iqref >  4.0f)  *Iqref =  4.0f;
    if (*Iqref < -4.0f)  *Iqref = -4.0f;
}


void PositionPI(int32_t actual_pos, int32_t target_pos, PI_POSITION_TypeDef *pi_ctrl, int16_t *Speedref)
{
    float err = (float)(target_pos - actual_pos);

    if (fabsf(err) < 3.0f)                                      //消除死区抖动
    {
        *Speedref = 0;
        pi_ctrl->Last_Err = err;
        return;
    }

    float P_Term = pi_ctrl->Kp * err;
    float D_Term = pi_ctrl->Kd * (err - pi_ctrl->Last_Err);
    float target_speed = P_Term + D_Term;

    pi_ctrl->Last_Err = err;

    if (target_speed >  500.0f) target_speed =  500.0f ;        // 对称限幅
    if (target_speed < -500.0f) target_speed = -500.0f;

    *Speedref = (int16_t)target_speed;
}


/***************************************** 一阶LADRC控制器（二阶LESO版） *****************************************/
/*
 * 说明：
 * 1. 适用于速度环     2. 被控对象按一阶模型处理：\dot{w} = f + b0 * u
 * 3. 采用二阶LESO：z1 -> 速度估计，z2 -> 总扰动估计
 * 4. 输出为 Iq_ref    5. 控制周期 1ms
 */

/******* 参数计算 *******/
void SpeedLADRC_CalcGain(LADRC_SPEED_TypeDef *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    /* 采样周期保护 */
    if (ctrl->h <= 0.0f)
    {
        ctrl->h = 0.001f;
    }

    /* 控制带宽保护 */
    if (ctrl->wc <= 0.0f)
    {
        ctrl->wc = 20.0f;
    }

    /* 观测器带宽保护 */
    if (ctrl->w0 <= 0.0f)
    {
        ctrl->w0 = 60.0f;
    }

    /*
     * 一阶对象的标准LADRC参数：
     * u0 = kp * e
     * kp = wc
     *
     * 想更“硬”一点，可以再调大 wc
     */
    ctrl->kp = ctrl->wc;

    /*
     * 二阶LESO：
     * z1_dot = z2 + b0*u + l1*(y - z1)
     * z2_dot = l2*(y - z1)
     *
     * 标准配置：
     * l1 = 2*w0
     * l2 = w0^2
     */
    ctrl->l1 = 2.0f * ctrl->w0;
    ctrl->l2 = ctrl->w0 * ctrl->w0;

    /*
     * kd在二阶对象里常见
     * 对一阶速度对象，通常不需要显式微分项。
     * 为了兼容你当前架构，保留但设为0。
     */
    ctrl->kd = 0.0f;
}

/******* LADRC初始化 *******/
void SpeedLADRC_Init(LADRC_SPEED_TypeDef *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->speed_ref = 0.0f;
    ctrl->speed_fdb = 0.0f;
    ctrl->err_speed = 0.0f;

    /*
     * 控制周期
     * 你原来速度环是1ms
     */
    ctrl->h = 0.001f;

    /*
     * b0 是名义对象增益b0=Kt/J
     * 这里先给一个保守值，后面再根据电机参数整定
     */
    ctrl->b0 = 41770.0f;

    /*
     * wc：控制器带宽
     * w0：观测器带宽
     * 一般 w0 > wc，常见 2~5 倍
     */
    ctrl->wc = 94.0f;        //15Hz带宽
    ctrl->w0 = 377.0f;

    /* LESO状态初始化 */
    ctrl->z1 = 0.0f;
    ctrl->z2 = 0.0f;

    /* 中间变量初始化 */
    ctrl->e = 0.0f;
    ctrl->u0 = 0.0f;
    ctrl->u = 0.0f;
    ctrl->last_u = 0.0f;

    ctrl->dz1_last = 0.0f;
    ctrl->dz2_last = 0.0f;

    /* 输出限幅 */
    ctrl->Iq_Max = 3.0f;
    ctrl->Iq_Min = -3.0f;

    /*
     * Iq变化率限制
     * 单位：A/s
     * 例如 800A/s，则1ms步长为0.8A
     */
    ctrl->Iq_Rate = 800.0f;

    /* 计算参数 */
    SpeedLADRC_CalcGain(ctrl);
}

/******* LADRC复位 *******/
void SpeedLADRC_Reset(LADRC_SPEED_TypeDef *ctrl, float speed_now)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->speed_ref = speed_now;
    ctrl->speed_fdb = speed_now;
    ctrl->err_speed = 0.0f;

    /*
     * 复位时把 z1 直接拉到当前速度
     * 这样可以避免启动瞬间误差过大
     */
    ctrl->z1 = speed_now;
    ctrl->z2 = 0.0f;

    ctrl->e = 0.0f;
    ctrl->u0 = 0.0f;
    ctrl->u = 0.0f;
    ctrl->last_u = 0.0f;

    ctrl->dz1_last = 0.0f;
    ctrl->dz2_last = 0.0f;
}

/******* 二阶LADRC更新主函数 *******/
void SpeedLADRC(FOC_TypeDef *Foc, LADRC_SPEED_TypeDef *ctrl, float speed_ref, float *Iqref)
{
    if (Foc == NULL || ctrl == NULL || Iqref == NULL)
    {
        return;
    }

    float speed_now = (float)Foc->speed;  // 当前速度反馈，单位RPM
    float iq_cmd = 0.0f;
    float iq_step = 0.0f;
    float iq_delta = 0.0f;

    /* 先更新参数 */
    SpeedLADRC_CalcGain(ctrl);

    /* b0保护，避免除零 */
    if (ctrl->b0 > 0.0f && ctrl->b0 < 1.0f)
    {
        ctrl->b0 = 1.0f;
    }
    if (ctrl->b0 < 0.0f && ctrl->b0 > -1.0f)
    {
        ctrl->b0 = -1.0f;
    }
    if (ctrl->b0 == 0.0f)
    {
        ctrl->b0 = 1.0f;
    }

    /* 保存给定与反馈 */
    ctrl->speed_ref = speed_ref;
    ctrl->speed_fdb = speed_now;

    /* 观测误差 */
    ctrl->e = speed_now - ctrl->z1;

    /*=========================================================
     * （1）二阶LESO（离散化）
     *
     * 连续形式：
     *   z1_dot = z2 + b0*u + l1*(y - z1)
     *   z2_dot = l2*(y - z1)
     *
     * 这里用欧拉离散
     * 更稳可以换成Tustin
     *=========================================================*/
    {
        float dz1;
        float dz2;
        float e_old = ctrl->e;

        dz1 = ctrl->z2 + ctrl->b0 * ctrl->last_u + ctrl->l1 * e_old;
        dz2 = ctrl->l2 * e_old;

        /*
         * 离散积分
         * z(k) = z(k-1) + h * dz
         */
        ctrl->z1 += ctrl->h * dz1;
        ctrl->z2 += ctrl->h * dz2;

        /* 保存上一拍导数，后面可改成Tustin */
        ctrl->dz1_last = dz1;
        ctrl->dz2_last = dz2;
    }

    /* 总扰动限幅 */
    {
        float z2_lim = ctrl->b0 * ctrl->Iq_Max * 1.5f;
        if (ctrl->z2 > z2_lim) ctrl->z2 = z2_lim;
        if (ctrl->z2 < -z2_lim) ctrl->z2 = -z2_lim;
    }

    
     /* （2）线性状态误差反馈（LSEF）*/
    ctrl->err_speed = ctrl->speed_ref - ctrl->z1;
    ctrl->u0 = ctrl->kp * ctrl->err_speed;
    /* LADRC 最终输出 */
    iq_cmd = (ctrl->u0 - ctrl->z2) / ctrl->b0;

    /* Iq变化率限制 */
    if (ctrl->Iq_Rate > 0.0f)
    {
        iq_step = ctrl->Iq_Rate * ctrl->h;
        iq_delta = iq_cmd - ctrl->last_u;

        if (iq_delta > iq_step)
        {
            iq_cmd = ctrl->last_u + iq_step;
        }
        else if (iq_delta < -iq_step)
        {
            iq_cmd = ctrl->last_u - iq_step;
        }
    }

    /* Iq幅值限幅 */
    if (iq_cmd > ctrl->Iq_Max) iq_cmd = ctrl->Iq_Max;
    if (iq_cmd < ctrl->Iq_Min) iq_cmd = ctrl->Iq_Min;

    /* 保存状态 */
    ctrl->u = iq_cmd;
    ctrl->last_u = iq_cmd;
    *Iqref = iq_cmd;
}


/***************************************** SMC 滑模速度控制 *****************************************/

/* 边界层饱和函数，代替 sign，减小抖振 */
static float SMC_Sat(float x)
{
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}


/******* SMC 初始化 *******/
void SMC_Init(SMC_TypeDef *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->speed_ref = 0.0f;
    ctrl->speed_fdb = 0.0f;
    ctrl->err_speed = 0.0f;
    ctrl->err_sum   = 0.0f;
    ctrl->s         = 0.0f;

    ctrl->h         = 0.001f;     // 速度环周期 1ms
    ctrl->c         = 20.0f;      // 滑模面系数
    ctrl->Ksw       = 4.0f;       // 初始最大输出约 1A
    ctrl->phi       = 1000.0f;     // 速度边界层，单位约 RPM

    ctrl->out       = 0.0f;
    ctrl->last_out  = 0.0f;
    ctrl->out_max   = 4.0f;
    ctrl->out_min   = -4.0f;
    ctrl->out_rate  = 800.0f;     // 1ms 最大变化 0.8A
    ctrl->sum_max   = 5000.0f;
}


/******* SMC 复位 *******/
void SMC_Reset(SMC_TypeDef *ctrl, float iq_now)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->err_speed = 0.0f;
    ctrl->err_sum   = 0.0f;
    ctrl->s         = 0.0f;
    ctrl->out       = iq_now;
    ctrl->last_out  = iq_now;
}


/******* 速度环 SMC：输入速度给定，输出 Iqref *******/
void SpeedSMC(FOC_TypeDef *Foc, SMC_TypeDef *ctrl, float speed_ref, float *Iqref)
{
    if (Foc == NULL || ctrl == NULL || Iqref == NULL)
    {
        return;
    }

    float iq_cmd = 0.0f;
    float iq_step = 0.0f;
    float iq_delta = 0.0f;
    float err_sum_bak = ctrl->err_sum;

    if (ctrl->h <= 0.0f)
    {
        ctrl->h = 0.001f;
    }

    if (ctrl->phi <= 0.0f)
    {
        ctrl->phi = 1.0f;
    }

    ctrl->speed_ref = speed_ref;
    ctrl->speed_fdb = (float)Foc->speed;
    ctrl->err_speed = ctrl->speed_ref - ctrl->speed_fdb;

    /* 误差积分只用于构造滑模面，不是 PI 输出 */
    ctrl->err_sum += ctrl->err_speed * ctrl->h;

    if (ctrl->err_sum >  ctrl->sum_max) ctrl->err_sum =  ctrl->sum_max;
    if (ctrl->err_sum < -ctrl->sum_max) ctrl->err_sum = -ctrl->sum_max;

    /* 滑模面 */
    ctrl->s = ctrl->err_speed + ctrl->c * ctrl->err_sum;

    /* 纯 SMC 输出 */
    iq_cmd = ctrl->Ksw * SMC_Sat(ctrl->s / ctrl->phi);

    /* Iq 变化率限制 */
    if (ctrl->out_rate > 0.0f)
    {
        iq_step = ctrl->out_rate * ctrl->h;
        iq_delta = iq_cmd - ctrl->last_out;

        if (iq_delta > iq_step)
        {
            iq_cmd = ctrl->last_out + iq_step;
        }
        else if (iq_delta < -iq_step)
        {
            iq_cmd = ctrl->last_out - iq_step;
        }
    }

    /* 输出限幅 + 简单抗积分饱和 */
    if (iq_cmd > ctrl->out_max)
    {
        iq_cmd = ctrl->out_max;
        if (ctrl->err_speed > 0.0f) ctrl->err_sum = err_sum_bak;
    }

    if (iq_cmd < ctrl->out_min)
    {
        iq_cmd = ctrl->out_min;
        if (ctrl->err_speed < 0.0f) ctrl->err_sum = err_sum_bak;
    }

    ctrl->out = iq_cmd;
    ctrl->last_out = iq_cmd;
    *Iqref = iq_cmd;
}



/***************************************** MTPA控制控制器  *************************************************/
/* 
 * 经典最大转矩电流id计算公式
 */
void MTPA_Control(float *Target_id, float flux, float Ld, float Lq, float iq)
{
    if (Target_id == NULL)
    {
        return;
    }

    *Target_id = 0.0f;

    float delta_L = Lq - Ld;
    float abs_Ld  = fabsf(Ld);
    float abs_dL  = fabsf(delta_L);

    /*
     * 自动判断：
     * 1) 表贴式电机 / 近似无凸极：delta_L 很小，直接 id = 0
     * 2) 凸极不明显：|Lq-Ld| / Ld < 5% 时，MTPA收益很小，也直接 id = 0
     * 3) 仅在凸极明显且 Lq > Ld 时，使用 MTPA
     */
    if (abs_dL < 1.0e-6f || delta_L <= 0.0f || abs_Ld < 1.0e-8f)  return;

    if ((abs_dL / abs_Ld) < 0.05f)  return;
    
    /* 标准 MTPA 公式 */
    float temp = sqrtf(flux * flux + 4.0f * delta_L * delta_L * iq * iq);
    *Target_id = (flux - temp) / (2.0f * delta_L);

    /* MTPA 输出通常应为负值，正值直接钳位为 0 */
    if (*Target_id > 0.0f)  *Target_id = 0.0f;
}


/***************************************** 弱磁控制控制器  *************************************************/
/* 电压反馈法
 * 输入参数: Foc(获取电压反馈)、FW_PI(弱磁状态)、Iq_ref_in(速度环原始Iq输出或外部给定的Iq) 
 * 输出参数: *Id_ref_out (输出最终Id指令，未弱磁时为0，弱磁时为负值)
 *           *Iq_ref_out (经极限电流圆限幅后的最终Iq指令)
 */
void FieldWeakening_Control(FOC_TypeDef *Foc, PI_FW_TypeDef *FW_PI, float Iq_ref_in, float *Id_ref_out, float *Iq_ref_out)
{
    // 1. 获取当前电压矢量的幅值 Vs_ref
    FW_PI->Vs_ref = sqrtf(Foc->Ud * Foc->Ud + Foc->Uq * Foc->Uq);
    
    // 2. 计算最大可用电压 Vs_max (SVPWM下理论最大为 Vdc/√3，此处留5%裕量)
    FW_PI->Vs_max = FW_PI->Vdc * _1_sqrt3 * 0.95f; 
    
    // 3. 计算电压误差：不能超过 Vs_max (饱和时产生负数误差)
    FW_PI->err_V = FW_PI->Vs_max - FW_PI->Vs_ref;
    
    // 4. PI 积分项计算及防饱和 (单向积分：只允许往负半轴积分)
    FW_PI->fw_KI_sum += FW_PI->err_V;
    
    if (FW_PI->fw_KI_sum < -100) {
        FW_PI->fw_KI_sum = -100;
    }
    if (FW_PI->fw_KI_sum > 0.0f) {
        FW_PI->fw_KI_sum = 0.0f;
    }
    
    // 5. 比例与积分叠加，并进行最终输出限幅
    FW_PI->id_fw = (FW_PI->Kp * FW_PI->err_V) + (FW_PI->Ki * FW_PI->fw_KI_sum);
    
    if (FW_PI->id_fw < FW_PI->id_fw_min) {
        FW_PI->id_fw = FW_PI->id_fw_min;
    }
    if (FW_PI->id_fw > 0.0f) {
        FW_PI->id_fw = 0.0f;
    }
    
    // ---> 输出1: 赋值目标 Id (未弱磁时自然保持为 0.0f)
    *Id_ref_out = FW_PI->id_fw;
    
    // 6. 极限圆约束判断，保障 id2 + iq2 <= Is_max2
    float iq_max_sq = (FW_PI->Is_max * FW_PI->Is_max) - ((*Id_ref_out) * (*Id_ref_out));
    float max_iq_allowed = 0.0f;
    
    if (iq_max_sq > 0.0f) {
        max_iq_allowed = sqrtf(iq_max_sq);
    }
    
    // ---> 输出2: 对传入的 Iq_ref_in 根据极限圆进行限幅约束
    if (Iq_ref_in > max_iq_allowed) {
        *Iq_ref_out = max_iq_allowed;
    } else if (Iq_ref_in < -max_iq_allowed) {
        *Iq_ref_out = -max_iq_allowed;
    } else {
        *Iq_ref_out = Iq_ref_in;
    }
}



/***************************************** 齿槽转矩补偿表格  *************************************************/
const float CogComp_Table[256] = {
     0.0202862f,  0.0222595f,  0.0236803f,  0.0241593f,  0.0246147f,  0.0250940f,  0.0252304f,  0.0255764f, 
     0.0260360f,  0.0265499f,  0.0275224f,  0.0283181f,  0.0286940f,  0.0283089f,  0.0271168f,  0.0257899f, 
     0.0242769f,  0.0217518f,  0.0196530f,  0.0174636f,  0.0156197f,  0.0143805f,  0.0125061f,  0.0111705f, 
     0.0105892f,  0.0094767f,  0.0081228f,  0.0071029f,  0.0062524f,  0.0056597f,  0.0043593f,  0.0030321f, 
     0.0010806f, -0.0000978f, -0.0017560f, -0.0033368f, -0.0046649f, -0.0058348f, -0.0063927f, -0.0061565f, 
    -0.0055641f, -0.0047163f, -0.0038049f, -0.0024583f, -0.0007378f,  0.0003663f,  0.0018050f,  0.0026860f, 
     0.0040793f,  0.0057628f,  0.0072802f,  0.0094530f,  0.0124769f,  0.0152528f,  0.0183899f,  0.0214753f, 
     0.0243233f,  0.0269893f,  0.0290432f,  0.0300220f,  0.0312731f,  0.0320490f,  0.0325122f,  0.0331464f, 
     0.0337625f,  0.0342147f,  0.0350301f,  0.0355392f,  0.0359024f,  0.0361509f,  0.0361463f,  0.0358697f, 
     0.0349991f,  0.0338764f,  0.0323622f,  0.0306619f,  0.0285813f,  0.0258497f,  0.0228011f,  0.0202250f, 
     0.0176173f,  0.0151545f,  0.0129640f,  0.0110421f,  0.0094714f,  0.0081641f,  0.0066139f,  0.0050284f, 
     0.0035370f,  0.0020025f,  0.0003854f, -0.0010558f, -0.0019519f, -0.0030199f, -0.0036065f, -0.0040399f, 
    -0.0042404f, -0.0044646f, -0.0048532f, -0.0055695f, -0.0060784f, -0.0065390f, -0.0075717f, -0.0082755f, 
    -0.0085408f, -0.0083955f, -0.0079102f, -0.0073395f, -0.0069370f, -0.0061344f, -0.0058408f, -0.0053086f, 
    -0.0051200f, -0.0053082f, -0.0051584f, -0.0048147f, -0.0046128f, -0.0039146f, -0.0032800f, -0.0023429f, 
    -0.0008735f,  0.0005196f,  0.0015918f,  0.0029089f,  0.0044806f,  0.0055937f,  0.0067893f,  0.0076624f, 
     0.0083469f,  0.0087130f,  0.0090637f,  0.0087265f,  0.0085015f,  0.0082392f,  0.0079489f,  0.0074575f, 
     0.0075110f,  0.0076331f,  0.0073860f,  0.0070998f,  0.0062193f,  0.0046033f,  0.0028191f,  0.0006000f, 
    -0.0023378f, -0.0053102f, -0.0082178f, -0.0106710f, -0.0128616f, -0.0145595f, -0.0163312f, -0.0176878f, 
    -0.0184074f, -0.0196103f, -0.0214330f, -0.0231017f, -0.0247378f, -0.0264232f, -0.0286281f, -0.0309525f, 
    -0.0326894f, -0.0342411f, -0.0358809f, -0.0373021f, -0.0382709f, -0.0383125f, -0.0382194f, -0.0375849f, 
    -0.0364533f, -0.0350260f, -0.0338080f, -0.0328166f, -0.0315910f, -0.0306403f, -0.0300269f, -0.0293006f, 
    -0.0285425f, -0.0273470f, -0.0256505f, -0.0234267f, -0.0203931f, -0.0172248f, -0.0141437f, -0.0105554f, 
    -0.0075088f, -0.0047032f, -0.0029666f, -0.0019684f, -0.0009075f, -0.0003854f,  0.0002278f,  0.0005625f, 
     0.0012785f,  0.0025657f,  0.0037672f,  0.0043805f,  0.0050919f,  0.0052553f,  0.0054204f,  0.0050962f, 
     0.0039282f,  0.0030639f,  0.0016043f,  0.0001592f, -0.0018736f, -0.0041511f, -0.0068209f, -0.0088992f, 
    -0.0114851f, -0.0131702f, -0.0145062f, -0.0159939f, -0.0164062f, -0.0163961f, -0.0167683f, -0.0172981f, 
    -0.0182880f, -0.0192704f, -0.0198955f, -0.0206531f, -0.0215762f, -0.0222857f, -0.0223720f, -0.0220620f, 
    -0.0216688f, -0.0209446f, -0.0209554f, -0.0207084f, -0.0204260f, -0.0198673f, -0.0200825f, -0.0199213f, 
    -0.0190409f, -0.0178008f, -0.0159870f, -0.0143567f, -0.0128366f, -0.0107655f, -0.0089818f, -0.0080444f, 
    -0.0069955f, -0.0060070f, -0.0048375f, -0.0035402f, -0.0018682f, -0.0002140f,  0.0016402f,  0.0035257f, 
     0.0054493f,  0.0069396f,  0.0083766f,  0.0100858f,  0.0121716f,  0.0142827f,  0.0162745f,  0.0181640f
};



/***************************************** MIT控制模式  *************************************************/

static float MIT_Clamp(float value, float min, float max)
{
    if (value > max) value = max;
    if (value < min) value = min;
    return value;
}


/******* MIT控制器初始化 *******/
void MIT_Init(MIT_TypeDef *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->p_des = 0.0f;
    ctrl->v_des = 0.0f;
    ctrl->kp = 0.0f;
    ctrl->kd = 0.0f;
    ctrl->tau_ff = 0.0f;

    /*
     * kt需要按实际电机填写。
     * 默认1.0表示 1Nm -> 1A，只作为安全占位，正式使用前建议改成实测Kt。
     */
    ctrl->kt = 0.03f;

    /* 默认按1kHz关节控制周期配置 */
    ctrl->h = 0.001f;

    /* 输出限幅，和当前速度环电流幅值保持接近 */
    ctrl->iq_max = 3.0f;
    ctrl->iq_min = -3.0f;
    ctrl->iq_rate = 800.0f;

    ctrl->p_fdb = 0.0f;
    ctrl->v_fdb = 0.0f;
    ctrl->err_p = 0.0f;
    ctrl->err_v = 0.0f;
    ctrl->tau_cmd = 0.0f;
    ctrl->iq_cmd = 0.0f;
    ctrl->last_iq_cmd = 0.0f;
}


/******* MIT控制器复位 *******/
void MIT_Reset(MIT_TypeDef *ctrl, float iq_now)
{
    if (ctrl == NULL)
    {
        return;
    }

    if (ctrl->h <= 0.0f)
    {
        ctrl->h = 0.001f;
    }

    if (ctrl->iq_max <= ctrl->iq_min)
    {
        ctrl->iq_max = 3.0f;
        ctrl->iq_min = -3.0f;
    }

    ctrl->err_p = 0.0f;
    ctrl->err_v = 0.0f;
    ctrl->tau_cmd = 0.0f;
    ctrl->iq_cmd = MIT_Clamp(iq_now, ctrl->iq_min, ctrl->iq_max);
    ctrl->last_iq_cmd = ctrl->iq_cmd;
}


/******* MIT指令写入 *******/
void MIT_SetCommand(MIT_TypeDef *ctrl, float p_des, float v_des, float kp, float kd, float tau_ff)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->p_des = p_des;
    ctrl->v_des = v_des;
    ctrl->kp = kp;
    ctrl->kd = kd;
    ctrl->tau_ff = tau_ff;

    /* 刚度和阻尼不允许为负，避免等效成负弹簧导致发散 */
    if (ctrl->kp < 0.0f) ctrl->kp = 0.0f;
    if (ctrl->kd < 0.0f) ctrl->kd = 0.0f;
}


/******* MIT计算Iq指令 *******/
void MIT_CalcIq(MIT_TypeDef *ctrl, float p_mech_rad, float v_mech_rad_s, float *Iqref)
{
    if (ctrl == NULL || Iqref == NULL)
    {
        return;
    }

    float iq_cmd = 0.0f;
    float iq_step = 0.0f;
    float iq_delta = 0.0f;

    if (ctrl->h <= 0.0f)
    {
        ctrl->h = 0.001f;
    }

    if (ctrl->iq_max <= ctrl->iq_min)
    {
        ctrl->iq_max = 3.0f;
        ctrl->iq_min = -3.0f;
    }

    ctrl->p_fdb = p_mech_rad;
    ctrl->v_fdb = v_mech_rad_s;

    /*
     * MIT模式核心公式：
     * tau = kp * (p_des - p) + kd * (v_des - v) + tau_ff
     */
    ctrl->err_p = ctrl->p_des - ctrl->p_fdb;
    ctrl->err_v = ctrl->v_des - ctrl->v_fdb;
    ctrl->tau_cmd = ctrl->kp * ctrl->err_p + ctrl->kd * ctrl->err_v + ctrl->tau_ff;

    /* 转矩换算成Iq：Iq = tau / Kt */
    if (fabsf(ctrl->kt) > 1.0e-6f)
    {
        iq_cmd = ctrl->tau_cmd / ctrl->kt;
    }
    else
    {
        iq_cmd = 0.0f;
    }

    /* Iq变化率限制，抑制上位机指令跳变 */
    if (ctrl->iq_rate > 0.0f)
    {
        iq_step = ctrl->iq_rate * ctrl->h;
        iq_delta = iq_cmd - ctrl->last_iq_cmd;

        if (iq_delta > iq_step)
        {
            iq_cmd = ctrl->last_iq_cmd + iq_step;
        }
        else if (iq_delta < -iq_step)
        {
            iq_cmd = ctrl->last_iq_cmd - iq_step;
        }
    }

    iq_cmd = MIT_Clamp(iq_cmd, ctrl->iq_min, ctrl->iq_max);

    ctrl->iq_cmd = iq_cmd;
    ctrl->last_iq_cmd = iq_cmd;
    *Iqref = iq_cmd;
}


/******* MIT控制模式：MIT外环 + FOC电流环 *******/
void MIT_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *pi_ctrl, MIT_TypeDef *mit_ctrl,
                 float IU, float IV, float IW,
                 float p_mech_rad, float v_mech_rad_s, float theta)
{
    if (Foc == NULL || pi_ctrl == NULL || mit_ctrl == NULL)
    {
        return;
    }

    float iq_ref = 0.0f;

    MIT_CalcIq(mit_ctrl, p_mech_rad, v_mech_rad_s, &iq_ref);

    /* MIT输出作为q轴电流指令，d轴默认给0 */
    IF_OpenLoop(Foc, pi_ctrl, IU, IV, IW, iq_ref, 0.0f, theta);
}
