#include "Filter.h"

//重置滤波器状态
void LPF1_Reset(LPF1_t *filter, float initial_output)
{
    if (filter == 0)
    {
        return;
    }

    filter->prev_output = initial_output;
    filter->initialized = 1U;
}

// 一阶低通滤波 
float LPF1_Update(LPF1_t *filter, float input, float a)
{
    float output;

    //空指针保护
    if (filter == 0)
    {
        return input;
    }
    //限制滤波系数范围
    if (a < 0.0f)
    {
        a = 0.0f;
    }
    else if (a > 1.0f)
    {
        a = 1.0f;
    }
    //首次初始化逻辑
    if (filter->initialized == 0U)
    {
        filter->prev_output = input;
        filter->initialized = 1U;
        return input;
    }
    //滤波计算
    output = a * input + (1.0f - a) * filter->prev_output;
    filter->prev_output = output;

    return output;
}

