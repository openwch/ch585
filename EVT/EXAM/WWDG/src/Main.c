/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : WWDG例程：看门狗
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* 喂狗周期：需小于看门狗溢出时间，计数周期131072/62.4MHz≈2.1ms，8位计数满约0.54s */
#define WDOG_FEED_MS      100
/* 喂狗次数：喂够后停止喂狗，演示看门狗复位 */
#define WDOG_FEED_TIMES   5

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

    /* 1. 使能看门狗超时复位 */
    WWDG_SetCounter(0);    // 重新装载计数值，从0开始一个完整计数周期
    WWDG_ResetCfg(ENABLE); // 计数溢出后复位系统

    PRINT("\nSystem watchdog(WWDG) enabled, feeding %d times ...\n", WDOG_FEED_TIMES);

    /* 2. 喂狗阶段：每100ms重新装载一次计数值，喂满设定次数 */
    while(feeds < WDOG_FEED_TIMES)
    {
        DelayMs(WDOG_FEED_MS);
        WWDG_SetCounter(0); // 喂狗：重新装载计数值
        feeds++;
        PRINT("  feed %d ok\n", feeds);
    }

    /* 3. 停止喂狗，等待看门狗超时复位（8位计数满约0.54s） */
    PRINT("\nStop feeding, watchdog will reset system ...\n");
    while(1);
    return 0;
}
