/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : TMR1捕捉例程，PA10边沿捕捉+DMA存储，PA10为LSE晶振引脚需关闭LSE
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

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

__attribute__((aligned(4))) uint32_t CapBuf[102]; // 注意前两个数据为无效数据

volatile uint8_t capFlag = 0;

#define  g_10us  (FREQ_SYS/100000)

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
    PRINT("TMR1 capture on PA10, DMA single mode\n");
    PWR_UnitModCfg(DISABLE, UNIT_SYS_LSE); // 注意此引脚是LSE晶振引脚，要保证关闭才能使用其他功能
    GPIOA_ResetBits(GPIO_Pin_10);          // 配置捕捉脚 PA10
    GPIOA_ModeCfg(GPIO_Pin_10, GPIO_ModeIN_PU);

    TMR1_CapInit(Edge_To_Edge);
    TMR1_CAPTimeoutCfg(0x1FFFFFF); // 设置捕捉超时时间
    /* 起止地址传完整32位地址，由驱动内部按0x1FFFF掩码，不可提前截断为16位 */
    TMR1_DMACfg(ENABLE, (uint32_t)&CapBuf[0], (uint32_t)&CapBuf[102], Mode_Single);
    TMR1_ITCfg(ENABLE, TMR0_3_IT_DMA_END); // 开启DMA完成中断
    PFIC_EnableIRQ(TMR1_IRQn);

    while(capFlag == 0)
    {
        if(++wait_cnt >= 0x1000000) // 超时上限，避免捕捉异常时无限等待
        {
            PRINT("TMR1 capture wait timeout!\n");
            break;
        }
    }
    capFlag = 0;
    PRINT("capture data:\n");
    for(i = 2; i < 102; i++)   // 注意前两个数据为无效数据
    {
        PRINT("%08lu ", (uint32_t)(CapBuf[i] & 0x1ffffff)); // 26bit, 最高位表示 高电平还是低电平
    }
    PRINT("\n");

    while(1);
    return 0;
}

/*********************************************************************
 * @fn      TMR1_IRQHandler
 *
 * @brief   TMR1中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR1_IRQHandler(void) // TMR1 捕捉DMA完成中断
{
    if(TMR1_GetITFlag(TMR0_3_IT_DMA_END))
    {
        TMR1_ITCfg(DISABLE, TMR0_3_IT_DMA_END); // 使用单次DMA功能+中断，注意完成后关闭此中断使能，否则会一直上报中断。
        TMR1_ClearITFlag(TMR0_3_IT_DMA_END);    // 清除中断标志
        capFlag = 1;
    }
}
