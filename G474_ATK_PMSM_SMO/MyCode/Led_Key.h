#ifndef __LED_KEY_H
#define __LED_KEY_H

#include "main.h"

/************************************▲▲LED部分▲▲*****************************************/
/* LED端口定义 */
#define LED0(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED    1为关闭0为开启*/

#define LED1(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = GREEN  1为关闭0为开启*/

/* LED取反定义 */
#define LED0_TOGGLE()    do{ HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin); }while(0)            /* LED0 = !LED0 */
#define LED1_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); }while(0)            /* LED1 = !LED1 */



/************************************▲▲KEY部分▲▲*****************************************/

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin)                                  /* 读取KEY0引脚 */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)                                  /* 读取KEY1引脚 */
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)                                  /* 读取KEY2引脚 */


#define KEY0_PRES    1                                                                          /* KEY0按下 */
#define KEY1_PRES    2                                                                          /* KEY1按下 */
#define KEY2_PRES    3                                                                          /* KEY2按下 */




/* 外部接口函数*/
void    led_init(void);                                                                         /* 初始化 */
uint8_t key_scan(uint8_t mode);                                                                 /* 按键扫描函数 */
void    key_function(uint8_t key , float *Iqref , int16_t *Speedref);                             /* 按键控制函数 */

#endif

