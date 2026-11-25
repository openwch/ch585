/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : PWM4-PWM10 8位数据宽度输出例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

#define PWMX_CYCLE_NUM     64                      /* 周期 = 64 个 cycle */
#define PWMX_DUTY_25PCT    (PWMX_CYCLE_NUM / 4)    /* 25% 占空比比较值 */
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
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("PWMX 8bit: PWM4-10 output 25%% duty\n");

    /* 配置GPIO */
    GPIOA_ModeCfg(GPIO_Pin_12, GPIO_ModeOut_PP_5mA); // PA12 - PWM4
    GPIOA_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA); // PA13 - PWM5
    GPIOB_ModeCfg(GPIO_Pin_0, GPIO_ModeOut_PP_5mA);  // PB0 - PWM6
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeOut_PP_5mA);  // PB4 - PWM7
    GPIOB_ModeCfg(GPIO_Pin_6, GPIO_ModeOut_PP_5mA);  // PB6 - PWM8
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);  // PB7 - PWM9
    GPIOB_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA); // PB14 - PWM10
//  GPIOB_ModeCfg(GPIO_Pin_23, GPIO_ModeOut_PP_5mA); // PB23 - PWM11 此脚复用为外部复位脚，需要关闭此功能才能开启PWM功能

    PWMX_CLKCfg(4);                                   // cycle = 4/Fsys
    PWMX_CycleCfg(PWMX_Cycle_64);                     // 周期 = 64*cycle
    PWMX_ACTOUT(CH_PWM4, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM5, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM6, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM7, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM8, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM9, PWMX_DUTY_25PCT, Low_Level, ENABLE);  // 25% 占空比
    PWMX_ACTOUT(CH_PWM10, PWMX_DUTY_25PCT, Low_Level, ENABLE); // 25% 占空比

    while(1);
    return 0;
}
