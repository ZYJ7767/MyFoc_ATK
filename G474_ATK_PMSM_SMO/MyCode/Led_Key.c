#include "Led_Key.h"
#include "Identify_J_B.h"

extern ID_J_B_Handle_t g_idjb;

extern uint8_t   Run_Flag;

/******* KEY 扫描功能函数 *******/
/* @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): KEY0 > KEY1 > KEY2!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              KEY2_PRES, 3, KEY2按下
*/
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* 支持连按 */

    if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
//        HAL_Delay(10);           /* 去抖动 */
        key_up = 0;

        if (KEY0 == 0)  keyval = KEY0_PRES;

        if (KEY1 == 0)  keyval = KEY1_PRES;

        if (KEY2 == 0)  keyval = KEY2_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1)         /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}


/******* KEY 控制功能函数 *******/
void key_function(uint8_t key , float *Iqref , int16_t *Speedref)
{
      if (key)
      {
        switch (key)
        {
            /********* Key0 *********/
            case KEY0_PRES: 
                LED0_TOGGLE();
            
//                Id_J_B_Start(&g_idjb);
            
//                *Iqref += 0.01f;
//                if(*Iqref >= 2) *Iqref = 2;
                *Speedref += 500;
                if(*Speedref >= 4500) *Speedref =4500 ;
                break;
            
            /********* Key1 *********/
            case KEY1_PRES:
                LED1_TOGGLE();
//                *Iqref -= 0.01f;
//                if(*Iqref <= -2) *Iqref = -2;
                *Speedref -= 500;
                if(*Speedref <= -4500) *Speedref = -4500;
                break;
            
            /********* Key2 *********/
            case KEY2_PRES:
                LED0_TOGGLE();
                LED1_TOGGLE();
//                *Iqref = 0;
                *Speedref = 0;
                break;
            default : break;
        }
      }
}












