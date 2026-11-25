/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC单通道采样例程，PA4(AIN0)带粗调校准
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
    signed short roughCalibValue = 0; // ADC粗调偏差值
    uint16_t adcBuff[20];

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("ADC single channel PA4 with rough calib\n");

    /* 单通道采样：选择adc通道0做采样，对应 PA4引脚，带数据校准功能 */
    GPIOA_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_4);

    // ADC init之前需要先配置通道
    ADC_ChannelCfg(0);
    // 采样率最高8M
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);

    roughCalibValue = ADC_DataCalib_Rough(); // 用于计算ADC内部偏差，记录到全局变量 RoughCalib_Value中
    PRINT("roughCalibValue =%d \n", roughCalibValue);

    ADC_ExcutSingleConver(); // 时间足够时建议丢弃首次ADC数据

    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver() + roughCalibValue; // 连续采样20次
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]); // 注意：由于ADC内部偏差的存在，当采样电压在所选增益范围极限附近的时候，可能会出现数据溢出的现象
    }

    while(1);
    return 0;
}
