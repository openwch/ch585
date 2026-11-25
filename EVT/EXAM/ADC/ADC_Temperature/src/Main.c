/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC温度检测例程，内部温度传感器采样并换算温度
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
    uint16_t adcBuff[20];

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("ADC temperature sampling\n");

    /* 温度采样并输出 */
    ADC_InterTSSampInit();
    ADC_ExcutSingleConver(); // 时间足够时建议再次转换并丢弃首次ADC数据
    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver(); // 连续采样20次
    }
    uint32_t c25Calib = 0;                                   /* 25度校准值，ROM 固定值 */
    c25Calib = (*((PUINT32)ROM_CFG_TMP_25C));
    for(i = 0; i < 20; i++)
    {
        PRINT("%d %d %u \n", adc_to_temperature_celsius(adcBuff[i]), adcBuff[i], c25Calib);
    }

    while(1);
    return 0;
}
