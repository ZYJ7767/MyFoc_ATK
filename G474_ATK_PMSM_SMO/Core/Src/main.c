/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Led_Key.h"
#include "Encoder.h"
#include "Foc_Function.h"
#include "Observer.h"
#include "math.h"
#include "arm_math.h"
#include "Dwt_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float     ref          = 2.0;
float     step         = 0.05f;

uint16_t  ADC_Vbus     = 0;
uint16_t  ADC_IsensU   = 0;
uint16_t  ADC_IsensV   = 0;
uint16_t  ADC_IsensW   = 0;

uint32_t  U_Offset     = 0;             //ADC偏执
uint32_t  V_Offset     = 0;
uint32_t  W_Offset     = 0;

float     Vbus         = 0;
float     IsensU_raw   = 0;
float     IsensV_raw   = 0;
float     IsensW_raw   = 0;
float     IsensU       = 0;
float     IsensV       = 0;
float     IsensW       = 0;

uint8_t   Offset_Flag  = 0;             //偏执采集结束标志位
uint8_t   Run_Flag     = 0;             //点击启动标志为
uint8_t   Run_Switch   = 0;             //切换标志位
uint8_t   ThetaUp_Flag = 0;
uint8_t   Z_flag       = 0;

uint8_t   Close_Flag   = 0;             //闭环标志位
uint8_t   Hat_Flag     = 0;

uint8_t   key          = 0;             //按键值
float     my_theta     = 0;             //自增角

uint16_t  EncValueRaw  = 0;             //��������ǰֵ
uint16_t  EncValue     = 0;             //������ֵ
float     Enc_eTheta   = 0;             //��ʵ��Ƕ� ����������
float     Enc_Speed    = 0;             //��ʵ�ٶ�   ����������
uint16_t  Enc_Pos      = 0;             //������λ��(4000)
uint16_t  Enc_cnt      = 0;             //��������Ȧ

float     Smo_e_Theta     = 0;          //SMO_Atan估计电角度
float     Smo_Pll_e_Theta = 0;          //SMO_PLL估计电角度
float     Bef_e_Theta     = 0;          
float     Smo_Speed       = 0;          //SMO_RPM
float     Smo_err         = 0;          //SMO切换时差值

float     kp           = 2.07f;          // Kp = wc*L    = 2*pi*1000*0.00033     = 2.070
float     ki           = 0.185f;         // Ki = wc*R*Ts = 2*pi*1000*0.295*0.0001= 0.185
float     skp          = 0.035f;
float     ski          = 0.0000015f;
float     pkp          = 0.120f;
float     pkd          = 0.000f;

float     Iqref        = 0.0f;
int16_t   Speedref     = 0;
uint16_t  Posref       = 0;

int16_t   Speedref_ramp = 0;            // 斜坡加减速速度给定
int16_t   Speed_step_up = 2;            // 斜坡加减速梯度
int16_t   Speed_step_dn = 3;  

float     Ud;                           //Jlink调试变量
float     Uq;
float     Ualpha;
float     Ubeta;
float     Id;
float     Iq;
float     Ialpha;
float     Ibeta;

DWT_Time_t t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Timer_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  
  /******* LED全部关闭先 *******/
  LED0(1);
  LED1(1);
  
  /******* 开启SHUTDOWN *******/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  
  /******** TIM1&6&7 PWM *********/
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  HAL_TIM_Base_Start( &htim1);                                      //TIM1 PWM
  HAL_TIM_PWM_Start ( &htim1, TIM_CHANNEL_4);
  
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2);  
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3); 
  
  /******** ADC ********/
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);                     //清理标志位
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);                      //End of Conversion
  
  HAL_ADCEx_Calibration_Start(&hadc1 , ADC_SINGLE_ENDED);           //ADC校准
  HAL_ADCEx_InjectedStart_IT(&hadc1);                               //注入组开启
  
  /******** 编码器初始化 ********/
  Encoder_Init();                                                   //编码器初始化
  while(Offset_Flag == 0)                                           //等ADC偏执采集完在开始对0
  {
      HAL_Delay(1); 
  }
  Encoder_Align_Zero();                                             //编码器0位对齐
  
  /******** PID赋值ֵ ********/
  C_PI.Kp = kp;
  C_PI.Ki = ki;
  S_PI.Kp = skp;
  S_PI.Ki = ski;
  P_PI.Kp = pkp;
  P_PI.Kd = pkd;

  
//  Run_Flag = 1;                                                     //允许启动
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /***** 母线电压ADC*****/
      HAL_ADC_Start(&hadc1);
      ADC_Vbus = HAL_ADC_GetValue(&hadc1);
      Vbus = (float)ADC_Vbus * 0.0201416f;
      LED0_TOGGLE();

    /****** KEY_LED 功能******/
      key = key_scan(0);
      key_function( key , &Iqref , &Speedref);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*********************************** ADC中断***********************************/
/*ADC中断  FOC控制 10kHz 100us*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)                  //10kHz中断 100us
{
    static uint8_t  offset_cnt  = 0;                                              //偏执次数计数
    static uint32_t Spdloop_cnt = 0;                                              //速度环计数
//    static uint8_t  Posloop_cnt = 0;                                              //位置环计数
    

    UNUSED(hadc);
    if(hadc == &hadc1)
    {
        if(Offset_Flag == 0)
        {
            offset_cnt++;
            ADC_IsensU = hadc1.Instance->JDR1;
            ADC_IsensV = hadc1.Instance->JDR2;
            ADC_IsensW = hadc1.Instance->JDR3;
            U_Offset += ADC_IsensU;
            V_Offset += ADC_IsensV;
            W_Offset += ADC_IsensW;
            if(offset_cnt >= 5)
            {
                U_Offset = U_Offset/5;
                V_Offset = V_Offset/5;
                W_Offset = W_Offset/5;
                Offset_Flag  = 1;                                               //ƫ����� ��־λ��1
            }
        }
        else
        {
            ADC_IsensU = hadc1.Instance->JDR1;
            ADC_IsensV = hadc1.Instance->JDR2;
            ADC_IsensW = hadc1.Instance->JDR3;
            IsensU =  (float)((int)ADC_IsensU - (int)U_Offset)* 0.006713867f;
            IsensV =  (float)((int)ADC_IsensV - (int)V_Offset)* 0.006713867f;
            IsensW =  (float)((int)ADC_IsensW - (int)W_Offset)* 0.006713867f;
        }
     }

     if(Run_Flag)
     {
        Encoder_Update_Angle();
        MyFoc.position = MyEnc.Raw_Value;
        Enc_eTheta     = MyEnc.Elec_Angle;

//        Posloop_cnt++;
//        if (Posloop_cnt >= 20)
//        {
//            Posloop_cnt = 0;
//            P_PI.position_ref = Posref;
//            PositionPI(&MyFoc, &P_PI, &Speedref);
//        }
        
        Spdloop_cnt++;
        if (Spdloop_cnt >= 10)
        {
            Spdloop_cnt = 0;
            
            if (Speedref_ramp < Speedref)
            {
                Speedref_ramp += Speed_step_up;
                if (Speedref_ramp > Speedref) Speedref_ramp = Speedref;
            } 
            else if (Speedref_ramp > Speedref) 
            {
                Speedref_ramp -= Speed_step_dn;
                if (Speedref_ramp < Speedref) Speedref_ramp = Speedref;
            }

            S_PI.speed_ref = Speedref_ramp;
            SpeedPI(&MyFoc, &S_PI, &Iqref);
            
            
            if (Speedref == 0)
            {
                S_PI.speed_KI_sum = 0.0f;
                C_PI.Iq_KI_sum    = 0.0f;
                Iqref             = 0.0f;
            }
        }
       
        IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, Enc_eTheta  ); 
     } //Run_Flag
     
//     Smo_Pll_e_Theta =  SMO_PLL_Update(&SMO, &PLL, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta);

     

    //jlinkt调试变量
    Ud      = MyFoc.Ud;
    Uq      = MyFoc.Uq;
    Ualpha  = MyFoc.Ualpha;
    Ubeta   = MyFoc.Ubeta;
    Id      = MyFoc.Id;
    Iq      = MyFoc.Iq;
    Ialpha  = MyFoc.Ialpha;
    Ibeta   = MyFoc.Ibeta; 

}//ADC IT END


/*** 外部中断 ***/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // PE4 
    if (GPIO_Pin == GPIO_PIN_4)
    {
//        if(Z_flag == 0)
//        {
//            __HAL_TIM_SET_COUNTER(&htim3, 0);
//            Z_flag = 1;
//            HAL_NVIC_DisableIRQ(EXTI4_IRQn);
//        }
    }
}




/*** 定时器中断 ***/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
{
    if (htim->Instance == TIM7)                // TIM7  1kHz 1ms
    {
        Encoder_Calculate_Speed();
        Enc_Speed   = MyEnc.Speed_RPM;
        MyFoc.speed = MyEnc.Speed_RPM;
        
        DWT_Timer_Start(&t);
        static uint8_t can_tx_cnt = 0;
        can_tx_cnt++;
        if ( can_tx_cnt>= 100U)
        {
            uint8_t tx_data[8] = {
                0x54, 0x53, 0x4D, 0x31,                     // ASCII: 'T' 'S' 'M' '1'
                can_tx_cnt,                                 // 发送计数
                g_fdcan1_last_rx_len,                       // 最近接收长度
                (uint8_t)g_fdcan1_rx_count,                 // 接收计数(低8位)
                (uint8_t)g_fdcan1_error_count               // 错误计数(低8位)
            };

            (void)FDCAN1_SendStd(0x123, tx_data, 8);       // 发标准ID=0x123
            can_tx_cnt = 0;
        }
        DWT_Timer_Stop(&t);
    }
    
//    if (htim->Instance == TIM6)
//    if (htim->Instance == TIM3)
}//TIM IT END






/*** 低通滤波（暂时有内存保护的问题）***/ 
float LowPassFilter(float input , float a)                                       //a滤波系数
{
    static float prev_output = 0;
    static uint8_t init_flag = 0;
    
     if(!init_flag) 
     {
        prev_output = input;
        init_flag = 1;
        return prev_output;
     }
    
    float output = a * input + (1.0f - a) * prev_output;
    prev_output = output;
    return output;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
