/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : ϵͳ˯��ģʽ��������ʾ��GPIOA_5��Ϊ����Դ����4��˯�ߵȼ�
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 ע�⣺�л���HSEʱ��Դ������ȴ��ȶ�ʱ���ѡ������þ�������йأ�ѡ��һ���µľ�������Ķ������ṩ�ľ��弰��
 ���ص��ݲ���ֵ��ͨ������R8_XT32M_TUNE�Ĵ������������ò�ͬ�ĸ��ص��ݺ�ƫ�õ��������������ȶ�ʱ�䡣
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
 * @brief   ���Գ�ʼ��
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
 * @brief   ������
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

    /* ���ô��ڵ��� */
    DebugInit();
    PRINT("Start @ChipID=%02x\n", R8_CHIP_ID);
    DelayMs(200);

#if 1
    /* ���û���ԴΪ GPIO - PA5 */
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeIN_PU);
    GPIOA_ITModeCfg(GPIO_Pin_5, GPIO_ITMode_FallEdge); // �½��ػ���
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
    HSECFG_Current(HSE_RCur_100); // ��Ϊ�����(�͹��ĺ�����������HSEƫ�õ���)
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
    LowPower_Shutdown(0); //ȫ���ϵ磬���Ѻ�λ
    /*
     ��ģʽ���Ѻ��ִ�и�λ������������벻�����У�
     ע��Ҫȷ��ϵͳ˯��ȥ�ٻ��Ѳ��ǻ��Ѹ�λ�������п��ܱ��IDLE�ȼ�����
     */
    HSECFG_Current(HSE_RCur_100); // ��Ϊ�����(�͹��ĺ�����������HSEƫ�õ���)
    PRINT("wake.. \n");
    DelayMs(500);
#endif

    while(1)
        ;
}

/*********************************************************************
 * @fn      LowPowerGapProcess
 *
 * @brief   �ⲿʱ�Ӳ��ȶ��ڼ�ִ�У�������ִ�ж�ʱ��Ҫ�󲻸ߵĴ�����flashδ׼���ã���Ҫ��RAM������
 *
 * @return  none
 */
__HIGH_CODE
void LowPowerGapProcess()
{
    //ִ�ж�ʱ��Ҫ�󲻸ߵĴ���
}

/*********************************************************************
 * @fn      PM_LowPower_Sleep
 *
 * @brief   ����Sleep˯���������˺�����Ҫ��RAM������
 *
 * @return  none
 */
__HIGH_CODE
void PM_LowPower_Sleep(void)
{
    uint32_t t;
    uint8_t wake_ctrl;
    unsigned long irq_status;

    //�л��ڲ�ʱ��
    sys_safe_access_enable();
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;
    R16_CLK_SYS_CFG &= ~RB_OSC32M_SEL;
    sys_safe_access_disable();
    LowPower_Sleep(RB_PWR_RAM96K | RB_PWR_RAM32K ); //ֻ����96+32K SRAM ����
    // ��ʱ�ⲿʱ�Ӳ��ȶ�����flashδ׼���ã�ֻ������RAM�д���
    SYS_DisableAllIrq(&irq_status);
    wake_ctrl = R8_SLP_WAKE_CTRL;
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = RB_WAKE_EV_MODE | RB_SLP_RTC_WAKE; // RTC����
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;  // ����ģʽ
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
    R8_FLASH_CTRL = 0x04; //flash�ر�

    PFIC->SCTLR &= ~(1 << 2); // sleep
    __WFE();
    __nop();
    __nop();
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
    sys_safe_access_enable();
    R8_SLP_WAKE_CTRL = wake_ctrl;
    sys_safe_access_disable();
    HSECFG_Current(HSE_RCur_100); // ��Ϊ�����(�͹��ĺ�����������HSEƫ�õ���)
    //�л��ⲿʱ��
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);
    SYS_RecoverIrq(irq_status);

}

/*********************************************************************
 * @fn      GPIOA_IRQHandler
 *
 * @brief   GPIOA�жϺ���
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void)
{
    GPIOA_ClearITFlagBit(GPIO_Pin_5);
}
