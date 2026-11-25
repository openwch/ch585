/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC单通道中断方式采样例程，PA5(AIN1)采20点
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

uint16_t adcBuff[20];
volatile uint8_t adcCnt;   /* 已采集点数：中断自增、主循环等待 */

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
    PRINT("ADC single channel PA5, interrupt mode\n");

    /* 单通道采样：中断方式，选择adc通道1做采样，对应 PA5引脚，不带数据校准功能 */
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_5);

    ADC_ChannelCfg(1);
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);
    ADC_ExcutSingleConver(); // 时间足够时建议丢弃首次ADC数据
    adcCnt = 0;
    ADC_ClearITFlag();
    PFIC_EnableIRQ(ADC_IRQn);
    ADC_StartUp();
    while(adcCnt < 20)
    {
        if(++wait_cnt >= 0x1000000) // 超时上限，避免ADC异常时无限等待
        {
            PRINT("ADC wait timeout!\n");
            break;
        }
    }
    PFIC_DisableIRQ(ADC_IRQn);
    for(i = 0; i < 20; i++)
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
    if(ADC_GetITStatus())
    {
        ADC_ClearITFlag();
        if(adcCnt < 20)
        {
            adcBuff[adcCnt] = ADC_ReadConverValue();
            ADC_StartUp(); // 作用清除中断标志并开启新一轮采样
            adcCnt++; // 仅在未采满 20 点时自增，计满后不再触发新一轮转换
        }
    }
}
