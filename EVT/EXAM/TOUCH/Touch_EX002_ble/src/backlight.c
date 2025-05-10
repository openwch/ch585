/********************************** (C) COPYRIGHT *******************************
 * File Name          : LED.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2022/01/18
 * Description        :
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "backlight.h"

TKY_Backlight_S BacklightArray[ TKY_MAX_QUEUE_NUM ];
void TKY_BacklightOff (void);

void TKY_KeyBacklightOut (uint8_t key, FunctionalState s);

volatile uint8_t led_scanflag = 0;

/*
 * 背光关闭
 * */
void TKY_BacklightOff (void)
{
    LED_COM1_OFF();
    LED_COM2_OFF();
    LED_SEG1_OFF();
    LED_SEG2_OFF();
    LED_SEG3_OFF();
    LED_SEG4_OFF();
}

/*
 * 背光全开
 * */
void TKY_BacklightOn (void)
{
    LED_SEG1_ON();
    LED_SEG2_ON();
    LED_SEG3_ON();
    LED_SEG4_ON();

    LED_COM1_ON();
    LED_COM2_ON();
}

/*
 *  背光相关外设和参数初始化
 * */
void TKY_BacklightInit (void)
{
    for (uint8_t i = 0; i < TKY_MAX_QUEUE_NUM; i++)
    {
        BacklightArray[ i ].TKY_BacklightOnTime = 0;
        BacklightArray[ i ].BacklightStates = 0;
    }

    //--------Backlight---------
    {

        GPIOB_ModeCfg (LED_SEG1_PIN | LED_SEG2_PIN | LED_SEG3_PIN | LED_SEG4_PIN |
                           LED_COM1_PIN | LED_COM2_PIN,
                       GPIO_ModeOut_PP_5mA);
    }

    TKY_BacklightTaskStart();
}

void TKY_KeyBacklightOut (uint8_t key, FunctionalState s)
{
    // PRINT("%d,%d",key,s);
    if (DISABLE == s)
    {
        BacklightArray[ key ].BacklightStates = 0;
    }
    else
    {
        BacklightArray[ key ].BacklightStates = 1;
    }
}

uint8_t getBacklightState(uint8_t key)
{
    return BacklightArray[ key ].BacklightStates;
}

/*
 * 设置按键背光状态
 * */
void setBacklightState (uint8_t Idx, uint8_t state)
{
    switch (Idx)
    {
        case 0 :
            if (state)
            {
                LED_SEG1_ON();
            }
            else
            {
                LED_SEG1_OFF();
            }
            break;
        case 1 :
            if (state)
            {
                LED_SEG2_ON();
            }

            else
            {
                LED_SEG2_OFF();
            }

            break;
        case 2 :
            if (state)
            {
                LED_SEG3_ON();
            }

            else
            {
                LED_SEG3_OFF();
            }

            break;
        case 3 :
            if (state)
            {
                LED_SEG4_ON();
            }

            else
            {
                LED_SEG4_OFF();
            }

            break;

        default :
            break;
    }
}

void TKY_BacklightTaskStart ()
{
//    TKY_BacklightOn();
    led_scanflag = 1;
}

void TKY_BacklightTaskStop ()
{
    TKY_BacklightOff();
    led_scanflag = 0;
}

void TKY_BacklightProcess (void)
{
    uint8_t i;
    static uint8_t g_BacklightIdleState = 1;
    static uint8_t backLightScanState = 0;

    if (backLightScanState)
    {
        for (i = 4 * (backLightScanState - 1); i < 4 * backLightScanState; i++)
        {
            setBacklightState (i - 4 * (backLightScanState - 1), BacklightArray[ i ].BacklightStates);
        }
    }
    switch (backLightScanState)
    {
        case 0 :
            LED_COM1_OFF();
            LED_COM2_OFF();
            backLightScanState = g_BacklightIdleState;
            break;
        case 1 :
            LED_COM1_ON();
            LED_COM2_OFF();
            backLightScanState = 0;
            g_BacklightIdleState = 2;
            break;

        case 2 :
            LED_COM1_OFF();
            LED_COM2_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 1;
            break;

        default :
            break;
    }
}

void TKY_BeepInit (void)
{
}

void TKY_Beepout (FunctionalState s)
{
    if (s == DISABLE)
    {
    }
    else
    {
    }
}

/**
 * @fn      IRQHandler
 *
 * @brief   中断函数
 *
 * @return  none
 */
// __attribute__ ((interrupt ("WCH-Interrupt-fast"))) void TMR1_IRQHandler (void)          // TMR0 定时中断
// {
//     static uint16_t counter = 0;
//     if (TMR1_GetITFlag (TMR0_3_IT_CYC_END))
//     {
//         TMR1_ClearITFlag (TMR0_3_IT_CYC_END);                                           // 清除中断标志
//         counter++;
//         if (counter > 10)                                                               // 定时1ms
//         {
//             counter = 0;
//             TKY_BacklightProcess();                                                     // poll背光状态
//         }
//     }
// }

/******************************** endfile @ led ******************************/
