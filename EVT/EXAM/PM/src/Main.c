/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 系统睡眠模式并唤醒演示：GPIOA_5作为唤醒源，共4种睡眠等级
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 注意：切换到HSE时钟源，所需等待稳定时间和选择的外置晶体参数有关，选择一款新的晶体最好阅读厂家提供的晶体及其
 负载电容参数值。通过配置R8_XT32M_TUNE寄存器，可以配置不同的负载电容和偏置电流，调整晶体稳定时间。
 */

#include "CH58x_common.h"

#define CLK_PER_US                  (1.0 / ((1.0 / CAB_LSIFQ) * 1000 * 1000))

#define US_PER_CLK                  (1.0 / CLK_PER_US)

#define RTC_TO_US(clk)              ((uint32_t)((clk) * US_PER_CLK + 0.5))

#define US_TO_RTC(us)               ((uint32_t)((us) * CLK_PER_US + 0.5))

void PM_LowPower_Sleep(void);
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
int main()
{
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    PWR_DCDCCfg(ENABLE);
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02x\n", R8_CHIP_ID);
    DelayMs(200);

#if 1
    /* 配置唤醒源为 GPIO - PA5 */
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_5, GPIO_ITMode_FallEdge); // 下降沿唤醒
    PFIC_EnableIRQ(GPIO_A_IRQn);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, Long_Delay);
#endif

#if 1
    PRINT("IDLE mode sleep \n");
    DelayMs(1);
    LowPower_Idle();
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("Halt mode sleep \n");
    DelayMs(2);
    LowPower_Halt();
    HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
    DelayMs(2);
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("sleep mode sleep \n");
    DelayMs(2);
    PM_LowPower_Sleep();
    PRINT("wake.. \n");
    DelayMs(500);
#endif

#if 1
    PRINT("shut down mode sleep \n");
    DelayMs(2);
    LowPower_Shutdown(0); //全部断电，唤醒后复位
    /*
     此模式唤醒后会执行复位，所以下面代码不会运行，
     注意要确保系统睡下去再唤醒才是唤醒复位，否则有可能变成IDLE等级唤醒
     */
    HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
    PRINT("wake.. \n");
    DelayMs(500);
#endif

    while(1)
        ;
}

/*********************************************************************
 * @fn      LowPowerGapProcess
 *
 * @brief   外部时钟不稳定期间执行，可用于执行对时钟要求不高的处理，且flash未准备好，需要在RAM中运行
 *
 * @return  none
 */
__HIGH_CODE
void LowPowerGapProcess()
{
    //执行对时钟要求不高的处理
}

/*********************************************************************
 * @fn      PM_LowPower_Sleep
 *
 * @brief   调用Sleep睡眠驱动，此函数需要在RAM中运行
 *
 * @return  none
 */
__HIGH_CODE
void PM_LowPower_Sleep(void)
{
    uint32_t t;
    uint8_t wake_ctrl;
    unsigned long irq_status;

    //切换内部时钟
    sys_safe_access_enable();
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
    R16_CLK_SYS_CFG &= ~RB_OSC32M_SEL;
    sys_safe_access_disable();
    LowPower_Sleep(RB_PWR_RAM96K | RB_PWR_RAM32K ); //只保留96+32K SRAM 供电
    // 此时外部时钟不稳定，且flash未准备好，只能运行RAM中代码
    SYS_DisableAllIrq(&irq_status);
    wake_ctrl = R8_SLP_WAKE_CTRL;
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = RB_WAKE_EV_MODE | RB_SLP_RTC_WAKE; // RTC唤醒
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;  // 触发模式
    sys_safe_access_disable();
    t = RTC_GetCycle32k() + US_TO_RTC(1600);
    if(t > RTC_MAX_COUNT)
    {
        t -= RTC_MAX_COUNT;
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();
    LowPowerGapProcess();
    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; //flash关闭

    PFIC->SCTLR &= ~(1 << 2); // sleep
    __WFE();
    __nop();
    __nop();
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = wake_ctrl;
    sys_safe_access_disable();
    HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
    //切换外部时钟
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    SYS_RecoverIrq(irq_status);

}

/*********************************************************************
 * @fn      GPIOA_IRQHandler
 *
 * @brief   GPIOA中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void)
{
    GPIOA_ClearITFlagBit(GPIO_Pin_5);
}
