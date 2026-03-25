#include "main.h"
#include "Dwt_timer.h"


#define DWT_CPU_HZ 140000000UL


void DWT_Timer_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


void DWT_Timer_Start(DWT_Time_t *t)
{
    if (t == 0) return;
    t->start_cycle = DWT->CYCCNT;
}


void DWT_Timer_Stop(DWT_Time_t *t)
{
    uint32_t end_cycle;

    if (t == 0) return;

    end_cycle = DWT->CYCCNT;
    t->cycle = end_cycle - t->start_cycle; // 32位回绕自动成立
    t->us = ((float)t->cycle * 1000000.0f) / (float)DWT_CPU_HZ;
}
