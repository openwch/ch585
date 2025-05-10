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
#include "LED.h"

#define TKY_BACKLIGHT_NUM   20
TKY_Backlight_S BacklightArray[ TKY_BACKLIGHT_NUM ] = {0};

volatile uint8_t led_scanflag = 0;

/*
 * 背光关闭
 * */
void TKY_BacklightOff (void)
{
    LED_SEG1_OFF();
    LED_SEG2_OFF();
    LED_SEG3_OFF();
    LED_SEG4_OFF();

    LED_COM1_OFF();
    LED_COM2_OFF();
    LED_COM3_OFF();
    LED_COM4_OFF();
    LED_COM5_OFF();
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
    LED_COM3_ON();
    LED_COM4_ON();
    LED_COM5_ON();
}

/*
 * 启动背光扫描任务
 * */
void TKY_BacklightTaskStart ()
{
    TKY_BacklightOn();
    led_scanflag = 1;
}

/*
 * 停止背光扫描任务
 * */
void TKY_BacklightTaskStop ()
{
    TKY_BacklightOff();
    led_scanflag = 0;
}

/*
 *  背光相关外设和参数初始化
 * */
void TKY_BacklightInit (void)
{
    for (uint8_t i = 0; i < TKY_BACKLIGHT_NUM; i++)
    {
        BacklightArray[ i ].TKY_BacklightOnTime = 0;
        BacklightArray[ i ].BacklightStates = 0;
    }

    //--------Backlight---------
    {

        GPIOB_ModeCfg (LED_SEG1_PIN | LED_SEG2_PIN | LED_SEG3_PIN | LED_SEG4_PIN |
                           LED_COM1_PIN | LED_COM2_PIN | LED_COM3_PIN | LED_COM4_PIN | LED_COM5_PIN,
                       GPIO_ModeOut_PP_5mA);
    }

    TKY_BacklightTaskStart();
}

/*
 * 设置按键背光状态
 * */
void TKY_KeyBacklightOut (uint8_t key, FunctionalState s)
{
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
 * 背光扫描过程中，seg段的设置
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

/*
 * 背光扫描的处理过程
 * */
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
            LED_COM3_OFF();
            LED_COM4_OFF();
            LED_COM5_OFF();
            backLightScanState = g_BacklightIdleState;
            break;
        case 1 :
            LED_COM1_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 2;
            break;

        case 2 :
            LED_COM2_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 3;
            break;

        case 3 :
            LED_COM3_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 4;
            break;
        case 4 :
            LED_COM4_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 5;
            break;
        case 5 :
            LED_COM5_ON();
            backLightScanState = 0;
            g_BacklightIdleState = 1;
            break;
        default :
            break;
    }
}

/******************************** endfile @ led ******************************/
