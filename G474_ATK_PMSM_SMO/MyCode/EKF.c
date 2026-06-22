#include "EKF.h"
#include "arm_math.h"

#define EKF_EPS             0.000001f
#define EKF_THETA_INT_K     162.9746617f

#define EKF_MAT4(m, r, c)   ((m)[(r) * 4 + (c)])
#define EKF_MAT42(m, r, c)  ((m)[(r) * 2 + (c)])

EKF_PositionObserver EKF = {    0.375f,        //Rs
                                0.000305f,     //Ls
                                0.00512f,      //phi_f
                                0.0001f,       //Ts
                                {0.0f, 0.0f, 0.0f, 0.0f},
                                {               //P，初始协方差，4x4按行存放
                                    0.05f, 0.0f, 0.0f,   0.0f,
                                    0.0f,  0.05f,0.0f,   0.0f,
                                    0.0f,  0.0f, 500.0f, 0.0f,
                                    0.0f,  0.0f, 0.0f,   0.5f
                                },
                                {0.00002f, 0.00002f, 50.0f, 0.0001f},   //Q
                                {0.01f, 0.01f},                         //R
                                {0.0f},                                  //F
                                {0.0f},                                  //K
                                0.0f,                                    //err_alpha
                                0.0f,                                    //err_beta
                                0.0f,                                    //S_det
                                2500.0f,                                 //We_Max
                                0.0f,                                    //Est_ialpha
                                0.0f,                                    //Est_ibeta
                                0.0f,                                    //Est_we
                                0,                                       //Est_RPM
                                0.0f,                                    //Est_theta
                                0                                        //Est_theta_int
                           };

/***** 限幅函数 *****/
static float EKF_Limit(float x, float max, float min)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/***** 协方差矩阵对称化，减小浮点舍入误差带来的P矩阵不对称 *****/
static void EKF_SymmetrizeP(float P[16])
{
    uint8_t i = 0U;
    uint8_t j = 0U;

    for (i = 0U; i < 4U; i++)
    {
        for (j = (uint8_t)(i + 1U); j < 4U; j++)
        {
            float p_avg = 0.5f * (EKF_MAT4(P, i, j) + EKF_MAT4(P, j, i));
            EKF_MAT4(P, i, j) = p_avg;
            EKF_MAT4(P, j, i) = p_avg;
        }

        if (EKF_MAT4(P, i, i) < EKF_EPS)
        {
            EKF_MAT4(P, i, i) = EKF_EPS;
        }
    }
}

/**************** EKF电机参数设置 ****************/
void EKF_SetMotor(EKF_PositionObserver *ekf, float Rs, float Ls, float phi_f, float Ts)
{
    if (ekf == 0)
    {
        return;
    }

    if (Rs > EKF_EPS)    ekf->Rs = Rs;
    if (Ls > EKF_EPS)    ekf->Ls = Ls;
    if (phi_f > EKF_EPS) ekf->phi_f = phi_f;
    if (Ts > EKF_EPS)    ekf->Ts = Ts;
}

/**************** EKF噪声参数设置 ****************/
void EKF_SetNoise(EKF_PositionObserver *ekf, float q_i, float q_we, float q_theta, float r_i)
{
    if (ekf == 0)
    {
        return;
    }

    if (q_i < 0.0f)     q_i = 0.0f;
    if (q_we < 0.0f)    q_we = 0.0f;
    if (q_theta < 0.0f) q_theta = 0.0f;
    if (r_i < EKF_EPS)  r_i = EKF_EPS;

    ekf->Q[0] = q_i;
    ekf->Q[1] = q_i;
    ekf->Q[2] = q_we;
    ekf->Q[3] = q_theta;

    ekf->R[0] = r_i;
    ekf->R[1] = r_i;
}

/**************** EKF状态复位 ****************/
void EKF_Reset(EKF_PositionObserver *ekf, float theta, float i_alpha, float i_beta, float we)
{
    uint8_t i = 0U;

    if (ekf == 0)
    {
        return;
    }

    if (ekf->Ts <= EKF_EPS)    ekf->Ts = 0.0001f;
    if (ekf->Ls <= EKF_EPS)    ekf->Ls = EKF_EPS;
    if (ekf->phi_f <= EKF_EPS) ekf->phi_f = EKF_EPS;

    theta = Normalize_theta(theta);
    we = EKF_Limit(we, ekf->We_Max, -ekf->We_Max);

    /***** 1. 状态量复位：电流取当前采样，角度和速度取外部给定初值 *****/
    ekf->x[EKF_X_IALPHA] = i_alpha;
    ekf->x[EKF_X_IBETA]  = i_beta;
    ekf->x[EKF_X_WE]     = we;
    ekf->x[EKF_X_THETA]  = theta;

    /***** 2. 清空P/F/K矩阵，重新给P矩阵对角线初值 *****/
    for (i = 0U; i < 16U; i++)
    {
        ekf->P[i] = 0.0f;
        ekf->F[i] = 0.0f;
    }

    for (i = 0U; i < 8U; i++)
    {
        ekf->K[i] = 0.0f;
    }

    EKF_MAT4(ekf->P, EKF_X_IALPHA, EKF_X_IALPHA) = 0.05f;
    EKF_MAT4(ekf->P, EKF_X_IBETA,  EKF_X_IBETA)  = 0.05f;
    EKF_MAT4(ekf->P, EKF_X_WE,     EKF_X_WE)     = 500.0f;
    EKF_MAT4(ekf->P, EKF_X_THETA,  EKF_X_THETA)  = 0.5f;

    /***** 3. 输出量同步清零/同步到初值 *****/
    ekf->err_alpha = 0.0f;
    ekf->err_beta  = 0.0f;
    ekf->S_det     = 0.0f;

    ekf->Est_ialpha = i_alpha;
    ekf->Est_ibeta  = i_beta;
    ekf->Est_we     = we;
    ekf->Est_RPM    = (int16_t)(we * (60.0f / (2.0f * pi * Pn)));
    ekf->Est_theta  = theta;
    ekf->Est_theta_int = (uint16_t)(theta * EKF_THETA_INT_K);
}

/**************** EKF位置观测器更新 ****************/
float EKF_Update(EKF_PositionObserver *ekf, float u_alpha, float u_beta, float i_alpha, float i_beta)
{
    uint8_t i = 0U;
    uint8_t j = 0U;
    uint8_t k = 0U;

    float SinValue = 0.0f;
    float CosValue = 0.0f;
    float inv_Ls   = 0.0f;
    float a_ii     = 0.0f;
    float b_emf    = 0.0f;

    float x_pre[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float FP[16]   = {0.0f};
    float P_pre[16]= {0.0f};
    float P_new[16]= {0.0f};

    float S00 = 0.0f;
    float S01 = 0.0f;
    float S10 = 0.0f;
    float S11 = 0.0f;
    float det = 0.0f;
    float invS00 = 0.0f;
    float invS01 = 0.0f;
    float invS10 = 0.0f;
    float invS11 = 0.0f;

    if (ekf == 0)
    {
        return 0.0f;
    }

    /***** 0. 参数保护，避免除零和速度异常发散 *****/
    if (ekf->Ts <= EKF_EPS)    ekf->Ts = 0.0001f;
    if (ekf->Ls <= EKF_EPS)    ekf->Ls = EKF_EPS;
    if (ekf->phi_f <= EKF_EPS) ekf->phi_f = EKF_EPS;
    if (ekf->We_Max <= 0.0f)   ekf->We_Max = 2500.0f;

    ekf->x[EKF_X_THETA] = Normalize_theta(ekf->x[EKF_X_THETA]);
    ekf->x[EKF_X_WE] = EKF_Limit(ekf->x[EKF_X_WE], ekf->We_Max, -ekf->We_Max);

    inv_Ls = 1.0f / ekf->Ls;
    arm_sin_cos_f32(ekf->x[EKF_X_THETA] * RAD_TO_DEG, &SinValue, &CosValue);

    /***** 1. 状态预测
     * PMSM静止坐标系模型：
     * di_alpha/dt = (u_alpha - Rs*i_alpha + we*phi_f*sin(theta)) / Ls
     * di_beta/dt  = (u_beta  - Rs*i_beta  - we*phi_f*cos(theta)) / Ls
     * dwe/dt      = 0
     * dtheta/dt   = we
     */
    x_pre[EKF_X_IALPHA] = ekf->x[EKF_X_IALPHA] + ekf->Ts * ((u_alpha - ekf->Rs * ekf->x[EKF_X_IALPHA] + ekf->x[EKF_X_WE] * ekf->phi_f * SinValue) * inv_Ls);
    x_pre[EKF_X_IBETA]  = ekf->x[EKF_X_IBETA]  + ekf->Ts * ((u_beta  - ekf->Rs * ekf->x[EKF_X_IBETA]  - ekf->x[EKF_X_WE] * ekf->phi_f * CosValue) * inv_Ls);
    x_pre[EKF_X_WE]     = ekf->x[EKF_X_WE];
    x_pre[EKF_X_THETA]  = Normalize_theta(ekf->x[EKF_X_THETA] + ekf->Ts * ekf->x[EKF_X_WE]);

    /***** 2. 计算状态转移雅可比矩阵F
     * F是把非线性模型在当前工作点线性化后的4x4矩阵。
     * 这里用数组按行存放，避免引入通用矩阵库。
     */
    for (i = 0U; i < 16U; i++)
    {
        ekf->F[i] = 0.0f;
    }

    a_ii  = 1.0f - ekf->Ts * ekf->Rs * inv_Ls;
    b_emf = ekf->Ts * ekf->phi_f * inv_Ls;

    EKF_MAT4(ekf->F, EKF_X_IALPHA, EKF_X_IALPHA) = a_ii;
    EKF_MAT4(ekf->F, EKF_X_IALPHA, EKF_X_WE)     = b_emf * SinValue;
    EKF_MAT4(ekf->F, EKF_X_IALPHA, EKF_X_THETA)  = b_emf * ekf->x[EKF_X_WE] * CosValue;

    EKF_MAT4(ekf->F, EKF_X_IBETA,  EKF_X_IBETA)  = a_ii;
    EKF_MAT4(ekf->F, EKF_X_IBETA,  EKF_X_WE)     = -b_emf * CosValue;
    EKF_MAT4(ekf->F, EKF_X_IBETA,  EKF_X_THETA)  = b_emf * ekf->x[EKF_X_WE] * SinValue;

    EKF_MAT4(ekf->F, EKF_X_WE,     EKF_X_WE)     = 1.0f;
    EKF_MAT4(ekf->F, EKF_X_THETA,  EKF_X_WE)     = ekf->Ts;
    EKF_MAT4(ekf->F, EKF_X_THETA,  EKF_X_THETA)  = 1.0f;

    /***** 3. 协方差预测：P_pre = F * P * F^T + Q *****/
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            FP[i * 4U + j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                FP[i * 4U + j] += EKF_MAT4(ekf->F, i, k) * EKF_MAT4(ekf->P, k, j);
            }
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            P_pre[i * 4U + j] = 0.0f;
            for (k = 0U; k < 4U; k++)
            {
                P_pre[i * 4U + j] += FP[i * 4U + k] * EKF_MAT4(ekf->F, j, k);
            }
        }
    }

    EKF_MAT4(P_pre, EKF_X_IALPHA, EKF_X_IALPHA) += ekf->Q[0];
    EKF_MAT4(P_pre, EKF_X_IBETA,  EKF_X_IBETA)  += ekf->Q[1];
    EKF_MAT4(P_pre, EKF_X_WE,     EKF_X_WE)     += ekf->Q[2];
    EKF_MAT4(P_pre, EKF_X_THETA,  EKF_X_THETA)  += ekf->Q[3];

    /***** 4. 观测残差：测量量只有i_alpha和i_beta *****/
    ekf->err_alpha = i_alpha - x_pre[EKF_X_IALPHA];
    ekf->err_beta  = i_beta  - x_pre[EKF_X_IBETA];

    /***** 5. 计算残差协方差S及其2x2逆矩阵
     * H = [1 0 0 0; 0 1 0 0]
     * 所以 S = H * P_pre * H^T + R，就是P_pre左上角2x2加电流测量噪声。
     */
    S00 = EKF_MAT4(P_pre, 0U, 0U) + ekf->R[0];
    S01 = EKF_MAT4(P_pre, 0U, 1U);
    S10 = EKF_MAT4(P_pre, 1U, 0U);
    S11 = EKF_MAT4(P_pre, 1U, 1U) + ekf->R[1];

    det = S00 * S11 - S01 * S10;
    if (det < EKF_EPS)
    {
        det = EKF_EPS;
    }
    ekf->S_det = det;

    invS00 =  S11 / det;
    invS01 = -S01 / det;
    invS10 = -S10 / det;
    invS11 =  S00 / det;

    /***** 6. 卡尔曼增益：K = P_pre * H^T * inv(S) *****/
    for (i = 0U; i < 4U; i++)
    {
        EKF_MAT42(ekf->K, i, 0U) = EKF_MAT4(P_pre, i, 0U) * invS00 + EKF_MAT4(P_pre, i, 1U) * invS10;
        EKF_MAT42(ekf->K, i, 1U) = EKF_MAT4(P_pre, i, 0U) * invS01 + EKF_MAT4(P_pre, i, 1U) * invS11;
    }

    /***** 7. 状态修正：x = x_pre + K * 残差 *****/
    for (i = 0U; i < 4U; i++)
    {
        ekf->x[i] = x_pre[i] + EKF_MAT42(ekf->K, i, 0U) * ekf->err_alpha + EKF_MAT42(ekf->K, i, 1U) * ekf->err_beta;
    }

    ekf->x[EKF_X_WE]    = EKF_Limit(ekf->x[EKF_X_WE], ekf->We_Max, -ekf->We_Max);
    ekf->x[EKF_X_THETA] = Normalize_theta(ekf->x[EKF_X_THETA]);

    /***** 8. 协方差修正：P = (I - K*H) * P_pre
     * 由于H只取前两个电流状态，展开后只需要减去K乘P_pre前两行。
     */
    for (i = 0U; i < 4U; i++)
    {
        for (j = 0U; j < 4U; j++)
        {
            P_new[i * 4U + j] = P_pre[i * 4U + j]
                              - EKF_MAT42(ekf->K, i, 0U) * P_pre[j]
                              - EKF_MAT42(ekf->K, i, 1U) * P_pre[4U + j];
        }
    }

    EKF_SymmetrizeP(P_new);

    for (i = 0U; i < 16U; i++)
    {
        ekf->P[i] = P_new[i];
    }

    /***** 9. 输出量更新，接口风格和NFO/SMO保持一致 *****/
    ekf->Est_ialpha = ekf->x[EKF_X_IALPHA];
    ekf->Est_ibeta  = ekf->x[EKF_X_IBETA];
    ekf->Est_we     = ekf->x[EKF_X_WE];
    ekf->Est_RPM    = (int16_t)(ekf->Est_we * (60.0f / (2.0f * pi * Pn)));
    ekf->Est_theta  = ekf->x[EKF_X_THETA];
    ekf->Est_theta_int = (uint16_t)(ekf->Est_theta * EKF_THETA_INT_K);

    return ekf->Est_theta;
}

/**************** EKF轻量版位置观测器更新 ****************/
float EKF_Update_light(EKF_PositionObserver *ekf, float u_alpha, float u_beta, float i_alpha, float i_beta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    float inv_Ls   = 0.0f;
    float f00      = 0.0f;
    float f02      = 0.0f;
    float f03      = 0.0f;
    float f12      = 0.0f;
    float f13      = 0.0f;

    float x0_pre = 0.0f;
    float x1_pre = 0.0f;
    float x2_pre = 0.0f;
    float x3_pre = 0.0f;

    float p00 = 0.0f;
    float p01 = 0.0f;
    float p02 = 0.0f;
    float p03 = 0.0f;
    float p11 = 0.0f;
    float p12 = 0.0f;
    float p13 = 0.0f;
    float p22 = 0.0f;
    float p23 = 0.0f;
    float p33 = 0.0f;

    float pp00 = 0.0f;
    float pp01 = 0.0f;
    float pp02 = 0.0f;
    float pp03 = 0.0f;
    float pp11 = 0.0f;
    float pp12 = 0.0f;
    float pp13 = 0.0f;
    float pp22 = 0.0f;
    float pp23 = 0.0f;
    float pp33 = 0.0f;

    float S00 = 0.0f;
    float S01 = 0.0f;
    float S11 = 0.0f;
    float det = 0.0f;
    float invS00 = 0.0f;
    float invS01 = 0.0f;
    float invS10 = 0.0f;
    float invS11 = 0.0f;

    float k00 = 0.0f;
    float k01 = 0.0f;
    float k10 = 0.0f;
    float k11 = 0.0f;
    float k20 = 0.0f;
    float k21 = 0.0f;
    float k30 = 0.0f;
    float k31 = 0.0f;

    float n00 = 0.0f;
    float n01 = 0.0f;
    float n02 = 0.0f;
    float n03 = 0.0f;
    float n10 = 0.0f;
    float n11 = 0.0f;
    float n12 = 0.0f;
    float n13 = 0.0f;
    float n20 = 0.0f;
    float n21 = 0.0f;
    float n22 = 0.0f;
    float n23 = 0.0f;
    float n30 = 0.0f;
    float n31 = 0.0f;
    float n32 = 0.0f;
    float n33 = 0.0f;

    if (ekf == 0)
    {
        return 0.0f;
    }

    /***** 0. 参数保护，避免除零和速度异常发散 *****/
    if (ekf->Ts <= EKF_EPS)    ekf->Ts = 0.0001f;
    if (ekf->Ls <= EKF_EPS)    ekf->Ls = EKF_EPS;
    if (ekf->phi_f <= EKF_EPS) ekf->phi_f = EKF_EPS;
    if (ekf->We_Max <= 0.0f)   ekf->We_Max = 2500.0f;

    ekf->x[EKF_X_THETA] = Normalize_theta(ekf->x[EKF_X_THETA]);
    ekf->x[EKF_X_WE] = EKF_Limit(ekf->x[EKF_X_WE], ekf->We_Max, -ekf->We_Max);

    inv_Ls = 1.0f / ekf->Ls;
    arm_sin_cos_f32(ekf->x[EKF_X_THETA] * RAD_TO_DEG, &SinValue, &CosValue);

    /***** 1. 状态预测，模型与完整版EKF_Update保持一致 *****/
    x0_pre = ekf->x[EKF_X_IALPHA] + ekf->Ts * ((u_alpha - ekf->Rs * ekf->x[EKF_X_IALPHA] + ekf->x[EKF_X_WE] * ekf->phi_f * SinValue) * inv_Ls);
    x1_pre = ekf->x[EKF_X_IBETA]  + ekf->Ts * ((u_beta  - ekf->Rs * ekf->x[EKF_X_IBETA]  - ekf->x[EKF_X_WE] * ekf->phi_f * CosValue) * inv_Ls);
    x2_pre = ekf->x[EKF_X_WE];
    x3_pre = Normalize_theta(ekf->x[EKF_X_THETA] + ekf->Ts * ekf->x[EKF_X_WE]);

    /***** 2. 只计算F矩阵的非零项
     * F = [f00  0   f02  f03
     *       0  f00  f12  f13
     *       0   0    1    0
     *       0   0    Ts   1]
     */
    f00 = 1.0f - ekf->Ts * ekf->Rs * inv_Ls;
    f02 = ekf->Ts * ekf->phi_f * inv_Ls * SinValue;
    f03 = ekf->Ts * ekf->phi_f * inv_Ls * ekf->x[EKF_X_WE] * CosValue;
    f12 = -ekf->Ts * ekf->phi_f * inv_Ls * CosValue;
    f13 = ekf->Ts * ekf->phi_f * inv_Ls * ekf->x[EKF_X_WE] * SinValue;

    /***** 3. 保存F矩阵，方便调试观察，未使用的元素直接写0 *****/
    ekf->F[0]  = f00;   ekf->F[1]  = 0.0f;  ekf->F[2]  = f02;     ekf->F[3]  = f03;
    ekf->F[4]  = 0.0f;  ekf->F[5]  = f00;   ekf->F[6]  = f12;     ekf->F[7]  = f13;
    ekf->F[8]  = 0.0f;  ekf->F[9]  = 0.0f;  ekf->F[10] = 1.0f;    ekf->F[11] = 0.0f;
    ekf->F[12] = 0.0f;  ekf->F[13] = 0.0f;  ekf->F[14] = ekf->Ts; ekf->F[15] = 1.0f;

    /***** 4. 读取P矩阵上三角，P每次都会对称化，所以只需要10个有效量 *****/
    p00 = ekf->P[0];
    p01 = ekf->P[1];
    p02 = ekf->P[2];
    p03 = ekf->P[3];
    p11 = ekf->P[5];
    p12 = ekf->P[6];
    p13 = ekf->P[7];
    p22 = ekf->P[10];
    p23 = ekf->P[11];
    p33 = ekf->P[15];

    /***** 5. 协方差预测：P_pre = F * P * F^T + Q
     * 这里按F矩阵非零项手写展开，省掉通用4x4矩阵乘法里的大量0乘法。
     */
    pp00 = f00 * f00 * p00
         + f02 * f02 * p22
         + f03 * f03 * p33
         + 2.0f * f00 * f02 * p02
         + 2.0f * f00 * f03 * p03
         + 2.0f * f02 * f03 * p23
         + ekf->Q[0];

    pp01 = f00 * (f00 * p01 + f12 * p02 + f13 * p03)
         + f02 * (f00 * p12 + f12 * p22 + f13 * p23)
         + f03 * (f00 * p13 + f12 * p23 + f13 * p33);

    pp02 = f00 * p02 + f02 * p22 + f03 * p23;
    pp03 = ekf->Ts * pp02 + f00 * p03 + f02 * p23 + f03 * p33;

    pp11 = f00 * f00 * p11
         + f12 * f12 * p22
         + f13 * f13 * p33
         + 2.0f * f00 * f12 * p12
         + 2.0f * f00 * f13 * p13
         + 2.0f * f12 * f13 * p23
         + ekf->Q[1];

    pp12 = f00 * p12 + f12 * p22 + f13 * p23;
    pp13 = ekf->Ts * pp12 + f00 * p13 + f12 * p23 + f13 * p33;

    pp22 = p22 + ekf->Q[2];
    pp23 = p23 + ekf->Ts * p22;
    pp33 = p33 + 2.0f * ekf->Ts * p23 + ekf->Ts * ekf->Ts * p22 + ekf->Q[3];

    /***** 6. 观测残差：测量量只有i_alpha和i_beta *****/
    ekf->err_alpha = i_alpha - x0_pre;
    ekf->err_beta  = i_beta  - x1_pre;

    /***** 7. 残差协方差S及2x2逆矩阵 *****/
    S00 = pp00 + ekf->R[0];
    S01 = pp01;
    S11 = pp11 + ekf->R[1];

    det = S00 * S11 - S01 * S01;
    if (det < EKF_EPS)
    {
        det = EKF_EPS;
    }
    ekf->S_det = det;

    invS00 =  S11 / det;
    invS01 = -S01 / det;
    invS10 = -S01 / det;
    invS11 =  S00 / det;

    /***** 8. 卡尔曼增益：K = P_pre * H^T * inv(S)
     * H只取前两个电流状态，所以只用P_pre的第0、1列。
     */
    k00 = pp00 * invS00 + pp01 * invS10;
    k01 = pp00 * invS01 + pp01 * invS11;
    k10 = pp01 * invS00 + pp11 * invS10;
    k11 = pp01 * invS01 + pp11 * invS11;
    k20 = pp02 * invS00 + pp12 * invS10;
    k21 = pp02 * invS01 + pp12 * invS11;
    k30 = pp03 * invS00 + pp13 * invS10;
    k31 = pp03 * invS01 + pp13 * invS11;

    ekf->K[0] = k00; ekf->K[1] = k01;
    ekf->K[2] = k10; ekf->K[3] = k11;
    ekf->K[4] = k20; ekf->K[5] = k21;
    ekf->K[6] = k30; ekf->K[7] = k31;

    /***** 9. 状态修正：x = x_pre + K * 残差 *****/
    ekf->x[EKF_X_IALPHA] = x0_pre + k00 * ekf->err_alpha + k01 * ekf->err_beta;
    ekf->x[EKF_X_IBETA]  = x1_pre + k10 * ekf->err_alpha + k11 * ekf->err_beta;
    ekf->x[EKF_X_WE]     = x2_pre + k20 * ekf->err_alpha + k21 * ekf->err_beta;
    ekf->x[EKF_X_THETA]  = x3_pre + k30 * ekf->err_alpha + k31 * ekf->err_beta;

    ekf->x[EKF_X_WE]    = EKF_Limit(ekf->x[EKF_X_WE], ekf->We_Max, -ekf->We_Max);
    ekf->x[EKF_X_THETA] = Normalize_theta(ekf->x[EKF_X_THETA]);

    /***** 10. 协方差修正：P = (I - K*H) * P_pre
     * H只取前两行，直接展开后再做一次对称化。
     */
    n00 = pp00 - k00 * pp00 - k01 * pp01;
    n01 = pp01 - k00 * pp01 - k01 * pp11;
    n02 = pp02 - k00 * pp02 - k01 * pp12;
    n03 = pp03 - k00 * pp03 - k01 * pp13;

    n10 = pp01 - k10 * pp00 - k11 * pp01;
    n11 = pp11 - k10 * pp01 - k11 * pp11;
    n12 = pp12 - k10 * pp02 - k11 * pp12;
    n13 = pp13 - k10 * pp03 - k11 * pp13;

    n20 = pp02 - k20 * pp00 - k21 * pp01;
    n21 = pp12 - k20 * pp01 - k21 * pp11;
    n22 = pp22 - k20 * pp02 - k21 * pp12;
    n23 = pp23 - k20 * pp03 - k21 * pp13;

    n30 = pp03 - k30 * pp00 - k31 * pp01;
    n31 = pp13 - k30 * pp01 - k31 * pp11;
    n32 = pp23 - k30 * pp02 - k31 * pp12;
    n33 = pp33 - k30 * pp03 - k31 * pp13;

    ekf->P[0]  = (n00 > EKF_EPS) ? n00 : EKF_EPS;
    ekf->P[1]  = 0.5f * (n01 + n10);
    ekf->P[2]  = 0.5f * (n02 + n20);
    ekf->P[3]  = 0.5f * (n03 + n30);

    ekf->P[4]  = ekf->P[1];
    ekf->P[5]  = (n11 > EKF_EPS) ? n11 : EKF_EPS;
    ekf->P[6]  = 0.5f * (n12 + n21);
    ekf->P[7]  = 0.5f * (n13 + n31);

    ekf->P[8]  = ekf->P[2];
    ekf->P[9]  = ekf->P[6];
    ekf->P[10] = (n22 > EKF_EPS) ? n22 : EKF_EPS;
    ekf->P[11] = 0.5f * (n23 + n32);

    ekf->P[12] = ekf->P[3];
    ekf->P[13] = ekf->P[7];
    ekf->P[14] = ekf->P[11];
    ekf->P[15] = (n33 > EKF_EPS) ? n33 : EKF_EPS;

    /***** 11. 输出量更新，接口风格和完整版EKF保持一致 *****/
    ekf->Est_ialpha = ekf->x[EKF_X_IALPHA];
    ekf->Est_ibeta  = ekf->x[EKF_X_IBETA];
    ekf->Est_we     = ekf->x[EKF_X_WE];
    ekf->Est_RPM    = (int16_t)(ekf->Est_we * (60.0f / (2.0f * pi * Pn)));
    ekf->Est_theta  = ekf->x[EKF_X_THETA];
    ekf->Est_theta_int = (uint16_t)(ekf->Est_theta * EKF_THETA_INT_K);

    return ekf->Est_theta;
}
