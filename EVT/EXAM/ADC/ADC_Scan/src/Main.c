/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : ADC扫描模式例程，14通道自动扫描+DMA(注意：扫描模式下打印脚为PB7)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* 注意当使用ADC扫描功能时，例程默认打印脚由PA14更改为PB7 */
#define ADC_SCAN_CH_NUM            (14)

#define ADC_SCAN1_CH_IDX_1 (0)
#define ADC_SCAN1_CH_IDX_2 (4)
#define ADC_SCAN1_CH_IDX_3 (8)
#define ADC_SCAN1_CH_IDX_4 (12)
#define ADC_SCAN1_CH_IDX_5 (16)
#define ADC_SCAN1_CH_IDX_6 (20)
#define ADC_SCAN1_CH_IDX_7 (24)
#define ADC_SCAN1_CH_IDX_8 (28)
#define ADC_SCAN2_CH_IDX_9 (0)
#define ADC_SCAN2_CH_IDX_10 (4)
#define ADC_SCAN2_CH_IDX_11 (8)
#define ADC_SCAN2_CH_IDX_12 (12)
#define ADC_SCAN2_CH_IDX_13 (16)

const uint32_t ADC_PIN[14] = {
    0x00000010,                                           // PA4,AIN0
    0x00000020,                                           // PA5,AIN1
    0x00001000,                                           // PA12,AIN2
    0x00002000,                                           // PA13,AIN3
    0x00004000,                                           // PA14,AIN4
    0x00008000,                                           // PA15,AIN5
    0x00000008,                                           // PA3,AIN6
    0x00000004,                                           // PA2,AIN7
    0x00000002,                                           // PA1,AIN8
    0x00000001,                                           // PA0,AIN9
    0x00000040,                                           // PA6,AIN10
    0x00000080,                                           // PA7,AIN11
    0x00000100,                                           // PA8,AIN12
    0x00000200                                            // PA9,AIN13
};

const uint8_t adc_ch_seq[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

uint16_t adcBuff[40];

volatile uint8_t DMA_end = 0;

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化(扫描模式使用PB7打印)
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOB_SetBits(GPIO_Pin_7);
    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
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
    uint32_t temp = 0;
    uint32_t wait_cnt = 0;
    
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("ADC scan mode: 14 channels + DMA\n");

    GPIOA_ModeCfg(ADC_PIN[adc_ch_seq[0]] |
                      ADC_PIN[adc_ch_seq[1]] |
                      ADC_PIN[adc_ch_seq[2]] |
                      ADC_PIN[adc_ch_seq[3]] |
                      ADC_PIN[adc_ch_seq[4]] |
                      ADC_PIN[adc_ch_seq[5]] |
                      ADC_PIN[adc_ch_seq[6]] |
                      ADC_PIN[adc_ch_seq[7]] |
                      ADC_PIN[adc_ch_seq[8]] |
                      ADC_PIN[adc_ch_seq[9]] |
                      ADC_PIN[adc_ch_seq[10]] |
                      ADC_PIN[adc_ch_seq[11]] |
                      ADC_PIN[adc_ch_seq[12]] |
                      ADC_PIN[adc_ch_seq[13]],
                  GPIO_ModeIN_Floating);

    GPIOADigitalCfg(DISABLE, ADC_PIN[adc_ch_seq[0]] |
            ADC_PIN[adc_ch_seq[1]] |
            ADC_PIN[adc_ch_seq[2]] |
            ADC_PIN[adc_ch_seq[3]] |
            ADC_PIN[adc_ch_seq[4]] |
            ADC_PIN[adc_ch_seq[5]] |
            ADC_PIN[adc_ch_seq[6]] |
            ADC_PIN[adc_ch_seq[7]] |
            ADC_PIN[adc_ch_seq[8]] |
            ADC_PIN[adc_ch_seq[9]] |
            ADC_PIN[adc_ch_seq[10]] |
            ADC_PIN[adc_ch_seq[11]] |
            ADC_PIN[adc_ch_seq[12]] |
            ADC_PIN[adc_ch_seq[13]]);

    ADC_ExtSingleChSampInit(SampleFreq_4_or_2, ADC_PGA_0);     // 4M采样频率，0dB增益
    R8_ADC_CONVERT |= RB_ADC_SAMPLE_TIME;                      // 7个采样周期
    ADC_ChannelCfg(adc_ch_seq[0]);

    temp = (adc_ch_seq[1] << ADC_SCAN1_CH_IDX_1) |
           (adc_ch_seq[2] << ADC_SCAN1_CH_IDX_2) |
           (adc_ch_seq[3] << ADC_SCAN1_CH_IDX_3) |
           (adc_ch_seq[4] << ADC_SCAN1_CH_IDX_4) |
           (adc_ch_seq[5] << ADC_SCAN1_CH_IDX_5) |
           (adc_ch_seq[6] << ADC_SCAN1_CH_IDX_6) |
           (adc_ch_seq[7] << ADC_SCAN1_CH_IDX_7) |
           (adc_ch_seq[8] << ADC_SCAN1_CH_IDX_8);
    R32_ADC_SCAN_CFG1 = temp;
    temp = (adc_ch_seq[9] << ADC_SCAN2_CH_IDX_9) |
           (adc_ch_seq[10] << ADC_SCAN2_CH_IDX_10) |
           (adc_ch_seq[11] << ADC_SCAN2_CH_IDX_11) |
           (adc_ch_seq[12] << ADC_SCAN2_CH_IDX_12) |
           (adc_ch_seq[13] << ADC_SCAN2_CH_IDX_13);
    temp |= RB_ADC_SCAN_SEL | RB_ADC_IE_SCAN_END | ((ADC_SCAN_CH_NUM - 1) << 24); // RB_ADC_SCAN_NUM = ADC_SCAN_CH_NUM - 1
    R32_ADC_SCAN_CFG2 = temp;

    R32_ADC_DMA_BEG = (uint32_t)&adcBuff[0] & 0x1FFFF;
    R32_ADC_DMA_END = (uint32_t)&adcBuff[40] & 0x1FFFF;
    R8_ADC_CTRL_DMA |= RB_ADC_DMA_ENABLE | RB_SCAN_AUTO_TYPE;
    PFIC_EnableIRQ(ADC_IRQn);
    R8_ADC_CONVERT |= RB_ADC_START;  

    while(!DMA_end)
    {
        if(++wait_cnt >= 0x1000000) // 超时上限，避免ADC异常时无限等待
        {
            PRINT("ADC scan DMA wait timeout!\n");
            break;
        }
    }
    DMA_end = 0;

    for(i = 0; i < ADC_SCAN_CH_NUM; i++)
    {
        PRINT("%d:%d ", i, adcBuff[i]);
        PRINT("\n");
        adcBuff[i] = 0;
    }
    PRINT("\n");

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
    if(R8_ADC_INT_FLAG & RB_ADC_IF_EOC)
    {
        ADC_ClearITFlag();
    }

    if(R32_ADC_SCAN_CFG2 & RB_ADC_IF_SCAN_END)
    {
        R32_ADC_SCAN_CFG2 |= RB_ADC_IF_SCAN_END;
        R32_ADC_DMA_BEG = (uint32_t)adcBuff & 0x1ffff;
        DMA_end = 1;
    }
}
