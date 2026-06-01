/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_Pin GPIO_PIN_4
#define Z_GPIO_Port GPIOE
#define Z_EXTI_IRQn EXTI4_IRQn
#define ShutDowm_Pin GPIO_PIN_13
#define ShutDowm_GPIO_Port GPIOC
#define I_U_Pin GPIO_PIN_1
#define I_U_GPIO_Port GPIOC
#define I_V_Pin GPIO_PIN_2
#define I_V_GPIO_Port GPIOC
#define I_W_Pin GPIO_PIN_3
#define I_W_GPIO_Port GPIOC
#define Vbus_Pin GPIO_PIN_3
#define Vbus_GPIO_Port GPIOA
#define KEY0_Pin GPIO_PIN_12
#define KEY0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOE
#define PWM_UL_Pin GPIO_PIN_13
#define PWM_UL_GPIO_Port GPIOB
#define PWM_VL_Pin GPIO_PIN_14
#define PWM_VL_GPIO_Port GPIOB
#define PWM_WL_Pin GPIO_PIN_15
#define PWM_WL_GPIO_Port GPIOB
#define PWM_U_Pin GPIO_PIN_8
#define PWM_U_GPIO_Port GPIOA
#define PWM_V_Pin GPIO_PIN_9
#define PWM_V_GPIO_Port GPIOA
#define PWM_W_Pin GPIO_PIN_10
#define PWM_W_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
float LowPassFilter(float input , float a);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
