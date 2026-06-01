#include "Foc_Function.h"
#include "math.h"
#include "arm_math.h"
#include "stdint.h"
#include "tim.h"

extern float Enc_Speed;


FOC_TypeDef         MyFoc   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PI_CURRENT_TypeDef  C_PI    = {0,0,0,0,0,0,0,0};
PI_SPEED_TypeDef    S_PI    = {0,0,0,0,0};
PI_POSITION_TypeDef P_PI    = {0,0,0};
LADRC_SPEED_TypeDef S_LADRC = {0};
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
    
    if (pi_ctrl->speed_KI_sum > 10000) pi_ctrl->speed_KI_sum =  0000;
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
 * 1. 适用于速度环
 * 2. 被控对象按一阶模型处理：
 *      \dot{w} = f + b0 * u
 * 3. 采用二阶LESO：
 *      z1 -> 速度估计
 *      z2 -> 总扰动估计
 * 4. 输出为 Iq_ref
 * 5. 控制周期建议 1ms
 */

/*========================
 * 1. 参数计算
 *========================*/
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
     * 如果你想更“硬”一点，也可以后面再调大 wc
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
     * 这里kd在二阶对象里常见；
     * 对一阶速度对象，通常不需要显式微分项。
     * 为了兼容你当前架构，保留但设为0。
     */
    ctrl->kd = 0.0f;
}

/*========================
 * 2. 初始化
 *========================*/
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
     * 这里先给一个保守值，后面你再根据电机参数整定
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

/*========================
 * 3. 复位
 *========================*/
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

/*========================
 * 4. 二阶LADRC主函数
 *========================*/
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

    /*
     * 观测误差：
     * e = y - z1
     * y = 实际速度
     * z1 = 速度估计值
     */
    ctrl->e = speed_now - ctrl->z1;

    /*=========================================================
     * 二阶LESO（离散化）
     *
     * 连续形式：
     *   z1_dot = z2 + b0*u + l1*(y - z1)
     *   z2_dot = l2*(y - z1)
     *
     * 这里用显式欧拉离散
     * 如果你后面想更稳，也可以换成Tustin
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

        /* 保存上一拍导数，便于后面改成Tustin */
        ctrl->dz1_last = dz1;
        ctrl->dz2_last = dz2;
    }

    /*
     * 总扰动限幅
     * 防止异常情况下 z2 发散
     */
    {
        float z2_lim = ctrl->b0 * ctrl->Iq_Max * 1.5f;
        if (ctrl->z2 > z2_lim) ctrl->z2 = z2_lim;
        if (ctrl->z2 < -z2_lim) ctrl->z2 = -z2_lim;
    }

    /*
     * 线性状态误差反馈（LSEF）
     *
     * 一阶对象常用：
     *   u0 = kp * (r - z1)
     *
     * 其中：
     *   r  = speed_ref
     *   z1 = 估计速度
     */
    ctrl->err_speed = ctrl->speed_ref - ctrl->z1;
    ctrl->u0 = ctrl->kp * ctrl->err_speed;

    /*
     * LADRC 最终输出：
     * u = (u0 - z2) / b0
     *
     * 这里 u 就是 Iq_ref
     */
    iq_cmd = (ctrl->u0 - ctrl->z2) / ctrl->b0;

    /*
     * Iq变化率限制
     * 目的是减少电流冲击和速度环突变
     */
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









