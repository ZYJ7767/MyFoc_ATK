#ifndef __DEADTIME_H
#define __DEADTIME_H

#include <stdint.h>

/*
 * PMSM 三相逆变器死区补偿模块
 *
 * 使用方式：
 * 1) 定义一个 DeadTimeComp_Handle_t 句柄，例如：DeadTimeComp_Handle_t DTC;
 * 2) PWM 初始化后调用 DeadTimeComp_Init(&DTC, 50.0f, 6999.0f);
 * 3) 在 SVPWM 算出三相 CCR 比较值后、写入 TIM1 CCR1/2/3 前调用补偿函数。
 *
 * 注意：
 * 该模块只根据“三相电流方向”修正“三相 PWM 比较值”，不直接操作定时器寄存器。
 * 这样做是为了保持接口简单，也方便你在原有 FOC/SVPWM 框架中插入或关闭。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 死区补偿句柄：外部定义一个全局变量即可，内部不使用动态内存 */
typedef struct
{
    uint8_t enable;              /* 1=使能补偿，0=关闭补偿 */
    float deadtime_ticks;        /* 硬件死区对应的定时器 tick 数，例如 TIM1 DeadTime=50 */
    float compensation_gain;     /* 补偿系数，1.0 表示按 deadtime_ticks 全量补偿 */
    float current_threshold_a;   /* 电流过零软切换阈值，单位 A，用于减小过零抖动 */
    float min_compare;           /* CCR 下限，防止补偿后过小 */
    float max_compare;           /* CCR 上限，防止补偿后超过 ARR */
} DeadTimeComp_Handle_t;

extern DeadTimeComp_Handle_t DTC;



/* 填入默认参数：适配当前工程 TIM1 ARR=6999、DeadTime=50 的配置 */
void DeadTimeComp_DefaultConfig(DeadTimeComp_Handle_t *dtc);

/* 初始化常用参数：deadtime_ticks 为硬件死区 tick，max_compare 一般填 ARR 或 6999 */
void DeadTimeComp_Init(DeadTimeComp_Handle_t *dtc,float deadtime_ticks,float max_compare);

/* 运行中打开或关闭补偿，方便对比补偿前后的电流波形 */
void DeadTimeComp_SetEnable(DeadTimeComp_Handle_t *dtc, uint8_t enable);

/* 设置补偿强度，内部限制在 0.0~2.0，建议先从 0.5 开始调试 */
void DeadTimeComp_SetGain(DeadTimeComp_Handle_t *dtc, float gain);

/* 设置死区 tick 数，应与 TIM1 的 DeadTime 配置一致 */
void DeadTimeComp_SetDeadTimeTicks(DeadTimeComp_Handle_t *dtc, float deadtime_ticks);

/* 设置电流过零软切换阈值，阈值越大，过零附近补偿变化越柔和 */
void DeadTimeComp_SetCurrentThreshold(DeadTimeComp_Handle_t *dtc, float threshold_a);

/* 设置 CCR 限幅范围，避免补偿后产生非法比较值 */
void DeadTimeComp_SetCompareLimit(DeadTimeComp_Handle_t *dtc, float min_compare, float max_compare);

/* 根据单相电流计算该相 CCR 需要增加或减少的 tick 数 */
float DeadTimeComp_CalcOffset(const DeadTimeComp_Handle_t *dtc, float phase_current_a);

/* 对单相 CCR 进行补偿并限幅，输入输出都是 float，适合当前 FOC 的 Tcm 变量 */
float DeadTimeComp_ApplyOneF32(const DeadTimeComp_Handle_t *dtc, float compare, float phase_current_a);

/* 对 U/V/W 三相 float CCR 同时补偿，推荐在当前工程 Svpwm() 中使用这个接口 */
void DeadTimeComp_ApplyCompareF32(const DeadTimeComp_Handle_t *dtc,
                                  float iu_a,
                                  float iv_a,
                                  float iw_a,
                                  float *cmp_u,
                                  float *cmp_v,
                                  float *cmp_w);

/* 对 U/V/W 三相 uint16_t CCR 同时补偿，适合后续你改成整型 CCR 时使用 */
void DeadTimeComp_ApplyCompareU16(const DeadTimeComp_Handle_t *dtc,
                                  float iu_a,
                                  float iv_a,
                                  float iw_a,
                                  uint16_t *cmp_u,
                                  uint16_t *cmp_v,
                                  uint16_t *cmp_w);

#ifdef __cplusplus
}
#endif

#endif
