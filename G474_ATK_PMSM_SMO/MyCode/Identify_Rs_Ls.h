#ifndef __IDENTIFY_RS_LS_H
#define __IDENTIFY_RS_LS_H

#include <stdint.h>

/* 该辨识模块默认放在ADC注入中断中调用，控制频率为10kHz，即100us一次 */
#define ID_RS_LS_TS_S        (0.0001f)

/* 辨识状态机：
 * Rs：正向直流注入 -> 采样，反向直流注入 -> 采样
 * Ls：回零 -> 正向短脉冲，回零 -> 反向短脉冲
 */
typedef enum
{
    ID_RS_LS_STATE_IDLE = 0,         /* 空闲 */
    ID_RS_LS_STATE_PREPARE,          /* 准备阶段，输出0V，等待电流/转速稳定 */
    ID_RS_LS_STATE_RS_POS_SETTLE,    /* Rs正向注入等待电流进入稳态 */
    ID_RS_LS_STATE_RS_POS_SAMPLE,    /* Rs正向稳态电流采样 */
    ID_RS_LS_STATE_RS_NEG_SETTLE,    /* Rs反向注入等待电流进入稳态 */
    ID_RS_LS_STATE_RS_NEG_SAMPLE,    /* Rs反向稳态电流采样 */
    ID_RS_LS_STATE_LS_ZERO_1,        /* Ls正向脉冲前先回零 */
    ID_RS_LS_STATE_LS_POS_PULSE,     /* Ls正向短电压脉冲 */
    ID_RS_LS_STATE_LS_ZERO_2,        /* Ls反向脉冲前先回零 */
    ID_RS_LS_STATE_LS_NEG_PULSE,     /* Ls反向短电压脉冲 */
    ID_RS_LS_STATE_DONE,             /* 辨识完成 */
    ID_RS_LS_STATE_ABORT             /* 安全保护触发，中止 */
} ID_Rs_Ls_State_t;

/* 外部每100us喂给辨识模块的实时量 */
typedef struct
{
    float iu_a;            /* U相电流，单位A */
    float iv_a;            /* V相电流，单位A */
    float iw_a;            /* W相电流，单位A，仅记录备用，计算Id时暂未用到 */
    float speed_rpm;       /* 机械转速，单位rpm，用于安全保护 */
    uint8_t run_flag;      /* 系统运行标志，0时立即中止 */
} ID_Rs_Ls_Input_t;

/* 辨识模块给FOC控制层的输出。
 * takeover=1时，外部应暂停电流环，直接用VF_OpenLoop输出这里的Ud/Uq/theta。
 */
typedef struct
{
    float ud_cmd_v;        /* d轴注入电压，单位V */
    float uq_cmd_v;        /* q轴注入电压，单位V，辨识时固定为0 */
    float theta_rad;       /* 固定电角度，单位rad */
    uint8_t takeover;      /* 1=辨识模块接管PWM输出，0=外部正常控制 */
} ID_Rs_Ls_Output_t;

/* 辨识配置参数。
 * tick单位都是10kHz控制周期：1 tick = 100us。
 */
typedef struct
{
    float theta_rad;               /* 注入电压的固定电角度，默认0rad */
    float rs_test_voltage_v;       /* Rs直流测试电压，建议先从0.3~0.8V试 */
    float ls_test_voltage_v;       /* Ls短脉冲测试电压，建议先从0.5~1.2V试 */
    float voltage_abs_limit_v;     /* 注入电压绝对限幅 */
    float current_abs_limit_a;     /* 总安全限流，Id/Iq任一超过即中止 */
    float speed_safe_rpm;          /* 安全转速阈值，辨识要求电机基本静止 */
    float min_current_for_rs_a;    /* Rs计算所需最小稳态电流，太小会除法放大噪声 */
    float min_delta_i_for_ls_a;    /* Ls计算所需最小电流变化量 */
    float ls_current_stop_a;       /* Ls脉冲过程中达到该电流即提前结束 */

    uint16_t prepare_ticks;        /* 准备时间 */
    uint16_t rs_settle_ticks;      /* Rs注入后等待稳态时间 */
    uint16_t rs_sample_ticks;      /* Rs稳态电流平均采样时间 */
    uint16_t ls_zero_ticks;        /* Ls脉冲之间回零等待时间 */
    uint16_t ls_pulse_ticks;       /* Ls最大脉冲宽度 */
} ID_Rs_Ls_Config_t;

/* 单点调试/观察用采样量，方便JScope或Watch窗口查看状态 */
typedef struct
{
    float t_s;             /* 辨识开始后的时间，单位s */
    float ud_cmd_v;        /* 当前输出Ud */
    float uq_cmd_v;        /* 当前输出Uq */
    float iu_a;            /* U相电流 */
    float iv_a;            /* V相电流 */
    float iw_a;            /* W相电流 */
    float ialpha_a;        /* Clarke后的Ialpha */
    float ibeta_a;         /* Clarke后的Ibeta */
    float id_a;            /* 固定角度下的Id */
    float iq_a;            /* 固定角度下的Iq */
    float rs_ohm;          /* 当前已得到的Rs结果 */
    float ls_h;            /* 当前已得到的Ls结果 */
    uint8_t state;         /* 当前状态机状态 */
    uint8_t running;       /* 是否运行中 */
    uint8_t finished;      /* 是否结束 */
    uint8_t aborted;       /* 是否异常中止 */
    uint32_t sample_idx;   /* 采样计数 */
} ID_Rs_Ls_Sample_t;

/* 辨识结果。
 * rs_pos/rs_neg、ls_pos/ls_neg用于检查正反向一致性；
 * rs_ohm/ls_h是最终平均结果。
 */
typedef struct
{
    float rs_ohm;          /* 最终相电阻估计值，单位Ohm */
    float ls_h;            /* 最终相电感估计值，单位H */
    float rs_pos_ohm;      /* 正向直流注入得到的Rs */
    float rs_neg_ohm;      /* 反向直流注入得到的Rs */
    float ls_pos_h;        /* 正向短脉冲得到的Ls */
    float ls_neg_h;        /* 反向短脉冲得到的Ls */
    uint8_t valid;         /* 1=结果有效 */
} ID_Rs_Ls_Result_t;

/* 辨识句柄：外部定义一个全局变量即可，内部保存状态机和累加量 */
typedef struct
{
    ID_Rs_Ls_Config_t cfg;
    ID_Rs_Ls_Sample_t sample;
    ID_Rs_Ls_Result_t result;
    ID_Rs_Ls_State_t state;

    uint8_t running;           /* 运行标志 */
    uint8_t finished;          /* 完成标志 */
    uint8_t aborted;           /* 中止标志 */

    uint16_t ticks_in_state;   /* 当前状态已运行tick数 */
    uint32_t total_ticks;      /* 总运行tick数 */

    float id_now;              /* 当前Id */
    float iq_now;              /* 当前Iq */
    float ud_cmd_now;          /* 当前Ud命令 */
    float uq_cmd_now;          /* 当前Uq命令 */

    float rs_pos_i_sum;        /* Rs正向采样电流累加 */
    float rs_neg_i_sum;        /* Rs反向采样电流累加 */
    uint16_t rs_pos_cnt;       /* Rs正向采样点数 */
    uint16_t rs_neg_cnt;       /* Rs反向采样点数 */

    float ls_start_i;          /* Ls脉冲开始时的Id */
    float ls_end_i;            /* Ls脉冲结束时的Id */
    float ls_prev_i;           /* Ls积分用的上一拍Id */
    float ls_flux_int;         /* 积分量：∫(U - R*i)dt */
    uint8_t ls_pulse_active;   /* Ls脉冲是否已经记录起点 */
} ID_Rs_Ls_Handle_t;

/* 用户要求的两个最终结果变量，辨识成功后自动更新 */
extern float Identify_Rs_Ohm;
extern float Identify_Ls_H;

/* 对外接口 */
void Id_Rs_Ls_DefaultConfig(ID_Rs_Ls_Config_t *cfg);
void Id_Rs_Ls_Init(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Config_t *cfg);
void Id_Rs_Ls_Start(ID_Rs_Ls_Handle_t *h);
void Id_Rs_Ls_Stop(ID_Rs_Ls_Handle_t *h);
void Id_Rs_Ls_Update_10kHz(ID_Rs_Ls_Handle_t *h, const ID_Rs_Ls_Input_t *in, ID_Rs_Ls_Output_t *out);

uint8_t Id_Rs_Ls_IsRunning(const ID_Rs_Ls_Handle_t *h);
uint8_t Id_Rs_Ls_IsFinished(const ID_Rs_Ls_Handle_t *h);
uint8_t Id_Rs_Ls_IsAborted(const ID_Rs_Ls_Handle_t *h);
uint8_t Id_Rs_Ls_ResultValid(const ID_Rs_Ls_Handle_t *h);

#endif
