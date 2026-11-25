/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC TouchKey采样例程，选择adc通道2，对应PA12
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

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("ADC TouchKey sampling on PA12\n");

    /* TouchKey采样：选择adc通道 2 做采样，对应 PA12 */
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_12);  /* 模拟通道需关闭数字输入 */

    ADC_ChannelCfg(2);
    TouchKey_ChSampInit();

    for(i = 0; i < 20; i++)
    {
        // charg=0x10: 充电时间16*Tadc; disch=0: 放电时间0，连续采样20次
        adcBuff[i] = TouchKey_ExcutSingleConver(0x10, 0);
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d \n", adcBuff[i]);
    }

    while(1);
    return 0;
}
