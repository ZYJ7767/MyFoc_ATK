#ifndef DWT_TIMER_H
#define DWT_TIMER_H

#include <stdint.h>



typedef struct
{
    uint32_t start_cycle;
    uint32_t cycle;   // 本次耗时(周期数)
    float    us;      // 本次耗时(微秒)
} DWT_Time_t;




void DWT_Timer_Init(void);          // 固定按140MHz
void DWT_Timer_Start(DWT_Time_t *t);
void DWT_Timer_Stop(DWT_Time_t *t); // 更新 cycle 和 us



#endif
