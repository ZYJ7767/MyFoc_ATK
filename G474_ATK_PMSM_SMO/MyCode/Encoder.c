#include "Encoder.h"


Encoder_TypeDef MyEnc = {0};

/* 编码器变量初始化 */
void Encoder_Init(void)
{
    MyEnc.Raw_Value  = 0;
    MyEnc.Offset     = 0;
    MyEnc.Mech_Angle = 0.0f;
    MyEnc.Elec_Angle = 0.0f;
    MyEnc.Speed_RPM  = 0;
    
    __HAL_TIM_SET_COUNTER(&htim3, 0);                           // 确保CNT从0开始，防止上电随机值
    
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);             // 开启定时器编码器模式
}


/* 获取电气角度 */
void Encoder_Update_Angle(void)
{
   
    MyEnc.Raw_Value = __HAL_TIM_GET_COUNTER(&htim3);            // 读取CNT (0 ~ 3999)

    int32_t delta = (int32_t)MyEnc.Raw_Value - (int32_t)MyEnc.Offset;

    delta *= ENCODER_DIR;
    
    while (delta >= ENCODER_PPR) delta -= ENCODER_PPR;          // 处理周期回绕 (归一化)
    while (delta < 0)            delta += ENCODER_PPR;
    
    MyEnc.Mech_Angle = (float)delta * 0.0015707963f;            // 计算机械角度 (0 ~ 2pi)   2 * pi / 4000 = 0.0015707963f
    MyEnc.Elec_Angle = MyEnc.Mech_Angle * (float)POLE_PAIRS;    // 计算电气角度 = 机械角度 * 极对数
    MyEnc.Elec_Angle = Normalize_theta(MyEnc.Elec_Angle);       // 归一化电气角度到 0~2pi
}


/* 开机0位校准 (强拖定位法) 阻塞式函数，必须确保 ADC 偏置校准完成后再调用 */
void Encoder_Align_Zero(void)
{
    float align_voltage = 0.5f;                                 // 校准电压 0.5V 
    for(int i=0; i<500; i++)                                    // 持续0.5秒钟
    {
        VF_OpenLoop(&MyFoc,align_voltage ,0 , 0);               // 强制设置开环电压，固定在0度
        HAL_Delay(1);
    }
    MyEnc.Offset = __HAL_TIM_GET_COUNTER(&htim3);               // 已物理对齐到电角度0度，记录CNT作为偏置
    VF_OpenLoop(&MyFoc, 0, 0, 0.0f);                            // 停止输出电压
}


/* 计算速度 (M法：固定时间测脉冲数) 在 1ms 中断中调用*/
void Encoder_Calculate_Speed(void)
{
    uint16_t cur_cnt = __HAL_TIM_GET_COUNTER(&htim3);                   // 1. 读取当前计数值
    int32_t diff = (int32_t)cur_cnt - (int32_t)MyEnc.Last_Raw_Value;    // 2. 计算差值 (转了多少个脉冲)强转 int32_t 防止溢出
    if (diff < -2000)       diff += ENCODER_PPR;                        // 3. 处理过零点 
    else if (diff > 2000)   diff -= ENCODER_PPR;                        //    反向跨越0点
    MyEnc.Last_Raw_Value = cur_cnt;                                     // 4. 更新旧值
    float cur_rpm   = (float)diff * 15.0f;                              // 5. 计算瞬时RPM = (diff / 4000) * (60 / 0.001) = diff * 15
    MyEnc.Speed_Flt = MyEnc.Speed_Flt * 0.6f + cur_rpm * 0.4f;        // 6. 速度强低通滤波 
    MyEnc.Speed_RPM = (int16_t)MyEnc.Speed_Flt;                         // 7. 输出整数 RPM
}









