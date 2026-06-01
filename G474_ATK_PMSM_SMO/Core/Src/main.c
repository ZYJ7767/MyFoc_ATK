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
#include "Identify_J_B.h"
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

uint32_t  U_Offset     = 0; 
uint32_t  V_Offset     = 0;
uint32_t  W_Offset     = 0;

float     Vbus         = 0;
float     IsensU_raw   = 0;
float     IsensV_raw   = 0;
float     IsensW_raw   = 0;
float     IsensU       = 0;
float     IsensV       = 0;
float     IsensW       = 0;

uint8_t   Offset_Flag  = 0;           
uint8_t   Run_Flag     = 0;            
uint8_t   Run_Switch   = 0;            
uint8_t   ThetaUp_Flag = 0;
uint8_t   Z_flag       = 0;

uint8_t   Close_Flag   = 0;         
uint8_t   Hat_Flag     = 0;

uint8_t   key          = 0;           
float     my_theta     = 0;            

uint16_t  EncValueRaw  = 0;             
uint16_t  EncValue     = 0;          
float     Enc_eTheta   = 0;           
float     Enc_Speed    = 0;            
uint16_t  Enc_Pos      = 0;             
uint16_t  Enc_cnt      = 0;            

float     Smo_e_Theta     = 0;          
float     Smo_Pll_e_Theta = 0;         
float     Bef_e_Theta     = 0;          
float     Smo_Speed       = 0;         
float     Smo_err         = 0;         

float     kp           = 1.92f;         // Kp  = wc*L    = 2*pi*1000*0.000305     = 1.92     带宽1khz
float     ki           = 0.236f;        // Ki  = wc*R*Ts = 2*pi*1000*0.375*0.0001 = 0.236
float     skp          = 0.00591f;      // skp = (2*pi/60) * (2*wc*J - B) / Kt               带宽20hz(以rpm为速度)
float     ski          = 0.000378f;     // ski = [(2*pi/60) * (wc*wc*J) / Kt] * Ts
float     pkp          = 0.15f;
float     pkd          = 0.000f;

float     Iqref        = 0.0f;
float     Idref        = 0.0f;
int16_t   Speedref     = 0;
int32_t   Posref       = 0;

int32_t  Total_Position = 0;            // 多圈绝对位置累加器
uint16_t Last_Raw_Value = 0;            // 上一次的单圈机械值

int16_t   Speedref_ramp = 0;            
int16_t   Speed_step_up = 1;            
int16_t   Speed_step_dn = 10;  

float     Ud;                           //Jlink
float     Uq;
float     Ualpha;
float     Ubeta;
float     Id;
float     Iq;
float     Ialpha;
float     Ibeta;

//J B identify
ID_J_B_Handle_t g_idjb;
ID_J_B_Output_t g_idjb_out;
ID_J_B_Config_t g_idjb_cfg;
ID_J_B_Input_t  idjb_in;

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
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  
  /******* LED 关闭 *******/
  LED0(1);
  LED1(1);
  
  /******* SHUTDOWN *******/
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  
  /******** TIM1&6&7 PWM *********/
//  HAL_TIM_Base_Start_IT(&htim6);
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
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);                     
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);                      //End of Conversion
  
  HAL_ADCEx_Calibration_Start(&hadc1 , ADC_SINGLE_ENDED);           //ADC
  HAL_ADCEx_InjectedStart_IT(&hadc1);                               
  
  HAL_ADC_Start(&hadc1);
  /******** 编码器0位校准 ********/
  Encoder_Init();                                                   
  while(Offset_Flag == 0)                                           
  {
      HAL_Delay(1); 
  }
  Encoder_Align_Zero();                                             
  
  /******** LADRC初始化 ********/
  SpeedLADRC_Init(&S_LADRC);
  SpeedLADRC_Reset(&S_LADRC, 0.0f);
  
  /******** PID赋值 ********/
  C_PI.Kp = kp;
  C_PI.Ki = ki;
  S_PI.Kp = skp;
  S_PI.Ki = ski;
  P_PI.Kp = pkp;
  P_PI.Kd = pkd;
  
  /******** FW结构体赋值 ********/
  FW_PI.Vdc         = Udc;         // 目前宏定义是24，如果有实时采样可以在控制大循环中更新
  FW_PI.Is_max      = 4.0f;        // 电流极限圆半径：依据速度环的最大电流限幅设置
  FW_PI.id_fw_min   = -4.0f;       // 弱磁深度底线直接用-Is_max防止烧毁
  FW_PI.Kp          = 0.3f;              
  FW_PI.Ki          = 0.04f;             
  FW_PI.fw_KI_sum   = 0.0f;
  
  /******** J辨识初始化 ********/
  Id_J_B_DefaultConfig(&g_idjb_cfg);
  Id_J_B_Init(&g_idjb, &g_idjb_cfg);
  
  Run_Flag = 1;                                                     
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /***** 母线ADC*****/
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
        ADC_Vbus = HAL_ADC_GetValue(&hadc1);
        Vbus = (float)ADC_Vbus * 0.0201416f;
    }
    /****** KEY_LED ******/
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
/*ADC中断 FOC 10kHz 100us*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)                  //10kHz  100us
{
    static uint8_t  offset_cnt  = 0;                                              
    static uint32_t Spdloop_cnt = 0;                                              
    static uint8_t  Posloop_cnt = 0;                                              
    
    DWT_Timer_Start(&t);
    
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
            if(offset_cnt >= 10)
            {
                U_Offset = U_Offset/10;
                V_Offset = V_Offset/10;
                W_Offset = W_Offset/10;
                Offset_Flag  = 1;                                               
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
/************************************* Encoder Theta ******************************************/
        Encoder_Update_Angle();
        MyFoc.position = MyEnc.Raw_Value;
        Enc_eTheta     = MyEnc.Elec_Angle;
         
/************************************* Position Loop ******************************************/
//        // 软件扩展 位置环 多圈位置计算
//        int16_t delta = MyEnc.Raw_Value - Last_Raw_Value;
//        if (delta >  2000) delta -= 4000; // 发生反转溢出
//        if (delta < -2000) delta += 4000; // 发生正转溢出
//        Total_Position += delta;          // 累加到多圈总位置
//        Last_Raw_Value = MyEnc.Raw_Value; // 更新历史值

//        // 位置环执行 (每20个周期执行一次，即2ms，也就是500Hz跑一次位置环)
//        Posloop_cnt++;
//        if (Posloop_cnt >= 20)
//        {
//            Posloop_cnt = 0;
//            PositionPI(Total_Position, (int32_t)Posref, &P_PI, &Speedref);
//        }
//        
/************************************** Speed Loop *******************************************/
        Spdloop_cnt++;
        if (Spdloop_cnt >= 10)
        {
            Spdloop_cnt = 0;
            int16_t step;
            if ((Speedref_ramp >= 0 && Speedref >= Speedref_ramp) || 
                (Speedref_ramp <= 0 && Speedref <= Speedref_ramp)) {
                step = Speed_step_up; // 加速状态
            } else {
                step = Speed_step_dn; // 减速(刹车)状态
            }
            
            if (Speedref_ramp < Speedref)
            {
                Speedref_ramp += step;
                if (Speedref_ramp > Speedref) Speedref_ramp = Speedref;
            } 
            else if (Speedref_ramp > Speedref) 
            {
                Speedref_ramp -= step;
                if (Speedref_ramp < Speedref) Speedref_ramp = Speedref;
            }

            /******** 速度环LADRC控制器 ********/
            SpeedLADRC(&MyFoc, &S_LADRC, Speedref_ramp, &Iqref);
            
            /********* 速度环PI控制器 *********/
//            S_PI.speed_ref = Speedref_ramp;
//            SpeedPI(&MyFoc, &S_PI, &Iqref);
        }
        
/************************************** FW Update ******************************************/
        
//        FW_PI.Vdc = 24;
//        FieldWeakening_Control(&MyFoc, &FW_PI, Iqref, &Idref, &Iqref);
        
/************************************* Current Loop ******************************************/
        float iq_cmd = Iqref;
        float id_cmd = Idref;
        
//        if (g_idjb_out.takeover) iq_cmd = g_idjb_out.iq_cmd_a;  /* 辨识时接管 */

        IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, iq_cmd, id_cmd , Enc_eTheta);
     } //Run_Flag
     
     Smo_Pll_e_Theta =  SMO_PLL_Update(&SMO, &PLL, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta);
     DWT_Timer_Stop(&t);

    //jlinkt
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
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM7)                // TIM7  1kHz 1ms
    {
        /******************* Encoder SpeedCal *********************/
        Encoder_Calculate_Speed();
        Enc_Speed   = MyEnc.Speed_RPM;
        MyFoc.speed = MyEnc.Speed_RPM;

        /********************* J_B Identify ***********************/
//        idjb_in.iq_meas_a = MyFoc.Iq;
//        idjb_in.speed_rpm = MyFoc.speed;
//        idjb_in.run_flag  = Run_Flag;

//        Id_J_B_Update_1kHz(&g_idjb, &idjb_in, &g_idjb_out);

        /*********************** CAN COM ***********************/
//        static uint8_t can_tx_cnt = 0;
//        can_tx_cnt++;
//        if ( can_tx_cnt>= 100U)
//        {
//            uint8_t tx_data[8] = {
//                0x54, 0x53, 0x4D, 0x31,                     // ASCII: 'T' 'S' 'M' '1'
//                can_tx_cnt,                                 // 
//                g_fdcan1_last_rx_len,                       // 接收长度
//                (uint8_t)g_fdcan1_rx_count,                 // 
//                (uint8_t)g_fdcan1_error_count               // 
//            };

//            (void)FDCAN1_SendStd(0x123, tx_data, 8);        // ID=0x123
//            can_tx_cnt = 0;
//        }

    }
  /* USER CODE END Callback 1 */
}

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
