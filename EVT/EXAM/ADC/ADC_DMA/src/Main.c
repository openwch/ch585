/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC单通道DMA采样例程，PA4(AIN0)自动连续采样40点
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

uint16_t adcBuff[40];

volatile uint8_t DMA_end = 0;

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("ADC single channel PA4, DMA 40 points\n");

    uint8_t i;
    uint32_t wait_cnt = 0;

    /* DMA单通道采样：选择adc通道0做采样，对应 PA4引脚 */
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_4);

    ADC_ChannelCfg(0);
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    ADC_ExcutSingleConver(); // 时间足够时建议再次转换并丢弃首次ADC数据
    ADC_AutoConverCycle(192); // 采样周期为 (256-192)*16个系统时钟
    ADC_DMACfg(ENABLE, (uint32_t)&adcBuff[0], (uint32_t)&adcBuff[40], ADC_Mode_Single);
    PFIC_EnableIRQ(ADC_IRQn);
    ADC_StartAutoDMA();
    while(!DMA_end)
    {
        if(++wait_cnt >= 0x1000000) // 超时上限，避免ADC异常时无限等待
        {
            PRINT("ADC DMA wait timeout!\n");
            break;
        }
    }
    DMA_end = 0;
    ADC_DMACfg(DISABLE, 0, 0, 0);

    for(i = 0; i < 40; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    while(1);
    return 0;
}

/*********************************************************************
 * @fn      ADC_IRQHandler
 *
 * @brief   ADC中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void ADC_IRQHandler(void) // adc中断服务程序
{
    if(ADC_GetDMAStatus())
    {
        ADC_StopAutoDMA();
        R32_ADC_DMA_BEG = ((uint32_t)adcBuff) & 0x1ffff;
        ADC_ClearDMAFlag();
        DMA_end = 1;
    }
}
