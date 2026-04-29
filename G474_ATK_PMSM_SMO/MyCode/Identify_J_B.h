#ifndef __IDENTIFY_J_B_H
#define __IDENTIFY_J_B_H

#include <stdint.h>

/* 采样缓存大小：1kHz下8000点=8秒 */
#define ID_J_B_TS_S         (0.001f)   /* 1ms */

/* 状态机 */
typedef enum
{
    ID_J_B_STATE_IDLE = 0,
    ID_J_B_STATE_PREPARE,
    ID_J_B_STATE_PULSE_POS,
    ID_J_B_STATE_COAST_1,
    ID_J_B_STATE_PULSE_NEG,
    ID_J_B_STATE_COAST_2,
    ID_J_B_STATE_DONE,
    ID_J_B_STATE_ABORT
} ID_J_B_State_t;

/* 输入：每1ms喂给模块 */
typedef struct
{
    float iq_meas_a;    /* 实际Iq(A)，建议MyFoc.Iq */
    float speed_rpm;    /* 机械转速(rpm)，建议MyFoc.speed */
    uint8_t run_flag;   /* 运行标志，建议Run_Flag */
} ID_J_B_Input_t;

/* 输出：给外部控制层 */
typedef struct
{
    float iq_cmd_a;     /* 辨识模块输出的Iq命令 */
    uint8_t takeover;   /* 1=接管Iq命令，0=不接管 */
} ID_J_B_Output_t;

/* 配置参数 */
typedef struct
{
    float iq_test_a;          /* 脉冲Iq幅值，建议1.0~1.5A */
    float iq_abs_limit_a;     /* 绝对限流，建议<=2.0A */

    uint16_t prepare_ms;      /* 预备时间 */
    uint16_t pulse_ms;        /* 脉冲持续时间 */
    uint16_t coast_ms;        /* 滑行时间 */
    uint8_t cycles;           /* 正负脉冲循环次数 */

    float speed_safe_rpm;     /* 安全转速上限 */
    float alpha_lpf;          /* 角加速度低通系数(0~1) */
} ID_J_B_Config_t;

/* 采集日志（JScope可直接看并导CSV） */
typedef struct
{
    float t_s;
    float iq_cmd_a;
    float iq_meas_a;
    float speed_rpm;
    float omega_rad_s;
    float alpha_rad_s2;
    uint8_t state;
    uint8_t running;
    uint8_t aborted;
    uint8_t reserved;
    uint32_t sample_idx;   /* 每1ms+1，方便JScope对齐时间轴 */
} ID_J_B_Sample_t;

/* 句柄 */
typedef struct
{
    ID_J_B_Config_t cfg;
    ID_J_B_Sample_t sample;

    ID_J_B_State_t state;
    uint8_t running;
    uint8_t finished;
    uint8_t aborted;

    uint16_t ms_in_state;
    uint8_t cycle_cnt;
    uint32_t total_ms;

    float omega_prev;
    float alpha_filt;
    uint8_t omega_inited;

    float iq_cmd_now;
} ID_J_B_Handle_t;

/* ========= 对外接口（全部统一Id_J_B_xxx） ========= */
void Id_J_B_DefaultConfig(ID_J_B_Config_t *cfg);
void Id_J_B_Init(ID_J_B_Handle_t *h, const ID_J_B_Config_t *cfg);
void Id_J_B_Start(ID_J_B_Handle_t *h);
void Id_J_B_Stop(ID_J_B_Handle_t *h);
void Id_J_B_Update_1kHz(ID_J_B_Handle_t *h, const ID_J_B_Input_t *in, ID_J_B_Output_t *out);

uint8_t Id_J_B_IsRunning(const ID_J_B_Handle_t *h);
uint8_t Id_J_B_IsFinished(const ID_J_B_Handle_t *h);
uint8_t Id_J_B_IsAborted(const ID_J_B_Handle_t *h);

#endif
