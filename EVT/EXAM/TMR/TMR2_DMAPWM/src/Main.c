/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : TMR2 DMA PWM例程，PB11重映射输出，DMA循环渐变占空比
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

__attribute__((aligned(4))) uint32_t PwmBuf[100];

volatile uint8_t dmaEndFlag = 0;

#define  g_10us  (FREQ_SYS/100000)

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
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    uint8_t i;
    uint32_t wait_cnt = 0;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("TMR2 DMA PWM on PB11, triangle duty\n");

    GPIOB_ModeCfg(GPIO_Pin_11, GPIO_ModeOut_PP_5mA);
    GPIOPinRemap(ENABLE, RB_PIN_TMR2);

    TMR2_PWMCycleCfg(g_10us * 250); 
    for(i = 0; i < 50; i++)
    {
        PwmBuf[i] = (g_10us / 2 * 10) * i;
    }
    for(i = 50; i < 100; i++)
    {
        PwmBuf[i] = (g_10us / 2 * 10) * (100 - i);
    } 
    TMR2_PWMInit(Low_Level, PWM_Times_16);
    TMR2_DMACfg(ENABLE, (uint32_t)&PwmBuf[0], (uint32_t)&PwmBuf[100], Mode_LOOP);
    TMR2_PWMEnable();
    TMR2_Enable();
    /* 开启DMA完成中断 */
    TMR2_ClearITFlag(TMR0_3_IT_DMA_END);
    TMR2_ITCfg(ENABLE, TMR0_3_IT_DMA_END);
    PFIC_EnableIRQ(TMR2_IRQn);

    while(dmaEndFlag == 0)
    {
        if(++wait_cnt >= 0x1000000) // 超时上限，避免DMA异常时无限等待
        {
            PRINT("TMR2 DMA wait timeout!\n");
            break;
        }
    }
    PRINT("DMA end\n");

    while(1);
    return 0;
}

/*********************************************************************
 * @fn      TMR2_IRQHandler
 *
 * @brief   TMR2中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR2_IRQHandler(void)
{
    if(TMR2_GetITFlag(TMR0_3_IT_DMA_END))
    {
        TMR2_ITCfg(DISABLE, TMR0_3_IT_DMA_END); // DMA 配置为循环模式(Mode_LOOP)，这里关闭 DMA 完成中断使能，否则每轮循环都会上报中断。
        TMR2_ClearITFlag(TMR0_3_IT_DMA_END);
        dmaEndFlag = 1;
        /* DMA 结束 */
        /* 用户可自行添加需要的处理 */
    }
}
