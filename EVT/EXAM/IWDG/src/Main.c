/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : IWDG例程：独立看门狗，由内部专用低速时钟驱动，演示周期喂狗与停止喂狗触发系统复位。
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* 预分频IWDG_PR_32，重装载值0x3FF, 超时约1s(具体值与内部LSI时钟频率有关，以上板实测为准) */
#define IWDG_RLR          1023

/* 喂狗判断阈值：递减到该值前重装载, 留有复位余量 */
#define IWDG_FEED_TH      200

/* 喂狗次数：喂够后停止喂狗, 演示看门狗复位 */
#define IWDG_FEED_TIMES   5

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
}

/*********************************************************************
 * @fn      PrintResetCause
 *
 * @brief   打印最近一次复位原因
 *
 * @return  none
 */
void PrintResetCause(void)
{
    switch(SYS_GetLastResetSta())
    {
        case RST_STATUS_SW:
            PRINT("  reset cause: software reset SR\n");
            break;
        case RST_STATUS_RPOR:
            PRINT("  reset cause: power-on reset RPOR\n");
            break;
        case RST_STATUS_WTR:
            PRINT("  reset cause: watchdog timeout reset WTR\n");
            break;
        case RST_STATUS_MR:
            PRINT("  reset cause: external pin reset MR\n");
            break;
        case RST_STATUS_GRWSM:
            PRINT("  reset cause: power-down wake-up reset GRWSM\n");
            break;
        default:
            PRINT("  reset cause: code 0x%x\n", SYS_GetLastResetSta());
            break;
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   none.
 *
 * @return  none.
 */
int main(void)
{
    uint8_t feeds = 0;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Time:%s; Start @ChipID=%02X\n", __TIMESTAMP__, R8_CHIP_ID);
    PrintResetCause();

    if(IWDG_Enable(IWDG_PR_32, IWDG_RLR))
    {
        PRINT("IWDG enable failed!\n");
        while(1)
            ;
    }
    IWDG_Feed(); // 装载计数值，从重装载值开始递减

    PRINT("\nIWDG started (PR=32 div, RLR=%d), feeding %d times ...\n", (int)IWDG_RLR, (int)IWDG_FEED_TIMES);

    while(feeds < IWDG_FEED_TIMES)
    {
        if(IWDG_Count_Get() < IWDG_FEED_TH)
        {
            IWDG_Feed(); // 喂狗：重装载计数值
            feeds++;
            PRINT("  feed %d ok\n", feeds);
        }
    }

    PRINT("\nStop feeding, IWDG will reset system ...\n");
    while(1)
        ;
    return 0;
}
