/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/02/27
 * Description        : 触摸按键例程
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CH58x_common.h"
#include "wchtouch_ct.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#define TOUCH_SHIELD_EN   1       //屏蔽使能
#define TOUCH_CX_CH_IDX   13      //外接电容对应的ADC通道
#define TOUCH_CH_NUM1     0       //触摸使用通道1
#define TOUCH_CH_NUM2     1       //触摸使用通道2
#define ADC_CH_NUM        2       //ADC单通道采样通道

extern void TKY_Enable();
extern void TKY_Disable();

const uint32_t Touch_Pin[ 14 ] = {
    GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_3,
    GPIO_Pin_2, GPIO_Pin_1, GPIO_Pin_0, GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9 };

static void Touch_RegInit( uint16_t tkyChAll, uint32_t touch_pin_a )
{
    ADC_SignalPGATypeDef ga = ADC_PGA_0;
    R8_ADC_CFG      = RB_ADC_POWER_ON | RB_ADC_BUF_EN | ( (ga&0xF) << 4 ) | ( SampleFreq_8_or_4 << 6 );
    if( ga & ADC_PGA_2_ )
    {
        R8_ADC_CONVERT |= RB_ADC_PGA_GAIN2;
    }
    else
    {
        R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    }
    R8_ADC_CONVERT &= ~( RB_ADC_SAMPLE_TIME );
    R8_ADC_CONVERT |= RB_ADC_SAMPLE_TIME;
    R8_TKEY_CFG     = 0;
    // 配置外接电容Cx引脚IO
    R32_PIN_IN_DIS |= touch_pin_a;//关闭数字输入功能，降低功耗
    GPIOA_ModeCfg(touch_pin_a, GPIO_ModeIN_Floating);
    ADC_ChannelCfg( TOUCH_CX_CH_IDX ); // 配置接入外部参考电容Cx的通道
#if TOUCH_SHIELD_EN
    R8_TKEY_CFG  |= RB_TKEY_DRV_EN;
    R32_TKEY_SEL |= ( tkyChAll << 18 );
#endif
}

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   串口打印初始化
 *
 * @return  none
 *
 * @note	Uart1的默认映射引脚可能与触摸通道冲突，
 * 			在触摸应用中应选用其他UART或将引脚重映射
 */
void DebugInit(void)
{
#ifdef  DEBUG
#if DEBUG == Debug_UART0
	GPIOB_SetBits(GPIO_Pin_7);
	GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
	UART0_DefInit();
#elif DEBUG == Debug_UART1
	GPIOPinRemap(ENABLE, RB_PIN_UART1);
	GPIOB_SetBits(GPIO_Pin_13);
	GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);
	UART1_DefInit();
#elif DEBUG == Debug_UART2
	GPIOPinRemap(ENABLE, RB_PIN_UART2);
	GPIOB_SetBits(GPIO_Pin_23);
	GPIOB_ModeCfg(GPIO_Pin_23, GPIO_ModeOut_PP_5mA);
	UART2_DefInit();
#elif DEBUG == Debug_UART3
	GPIOPinRemap(ENABLE, RB_PIN_UART3);
	GPIOB_SetBits(GPIO_Pin_21);
	GPIOB_ModeCfg(GPIO_Pin_21, GPIO_ModeOut_PP_5mA);
	UART3_DefInit();
#endif
#endif
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
    uint8_t sta = 0;
    uint8_t i;
    uint16_t sampleData[ 128 ];
    uint16_t adcBuff[40];
    uint16_t tkyCh = 0;
    uint32_t touch_pin = 0;
    signed short RoughCalib_Value = 0; // ADC粗调偏差值

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);
	DebugInit();
	PRINT("Touch Example\n");

	//库版本校验
    if ( memcmp( TOUCH_VER_LIB, TOUCH_VER_FILE, strlen( TOUCH_VER_FILE ) ) )
    {
        PRINT( "head file error...\n" );
        while ( 1 );
    }

    //TouchKey单通道采样，默认使用通道0，PA4
    PRINT("\n1.TouchKey Single channel sampling...\n");
    tkyCh = 1 << TOUCH_CH_NUM1;
    touch_pin = Touch_Pin[TOUCH_CH_NUM1] | Touch_Pin[TOUCH_CX_CH_IDX];
    Touch_RegInit(tkyCh, touch_pin);
    TKY_Enable();
    sta = TKY_GetCurChannelData( tkyCh, 32, 1, sampleData, 20 );
    if(sta == 0){
        for(int i=0;i<20;i++){
            PRINT("%d ", sampleData[i]);
        }
        PRINT("\n");
    }

    //ADC单通道采样，默认使用通道2，PA12
    PRINT("\n2.ADC Single channel sampling...\n");
    TKY_Disable();
    GPIOA_ModeCfg(Touch_Pin[ADC_CH_NUM], GPIO_ModeIN_Floating);
    // 采样率最高8M
    ADC_ExtSingleChSampInit(SampleFreq_8_or_4, ADC_PGA_0);

    RoughCalib_Value = ADC_DataCalib_Rough(); // 用于计算ADC内部偏差，记录到全局变量 RoughCalib_Value中

    ADC_ChannelCfg(ADC_CH_NUM);
    ADC_ExcutSingleConver();//时间足够时建议再次转换并丢弃首次ADC数据

    for(i = 0; i < 20; i++)
    {
        adcBuff[i] = ADC_ExcutSingleConver() + RoughCalib_Value; // 连续采样20次
    }
    for(i = 0; i < 20; i++)
    {
        PRINT("%d ", adcBuff[i]); // 注意：由于ADC内部偏差的存在，当采样电压在所选增益范围极限附近的时候，可能会出现数据溢出的现象
    }
    PRINT("\n");

    //TouchKey多通道采样，默认使用通道0——PA4，通道1——PA5
    PRINT("\n3.TouchKey Multi-channel sampling...\n");
    tkyCh = (1 << TOUCH_CH_NUM1) | (1 << TOUCH_CH_NUM2);
    touch_pin = Touch_Pin[TOUCH_CH_NUM1] | Touch_Pin[TOUCH_CH_NUM2] | Touch_Pin[TOUCH_CX_CH_IDX];
    Touch_RegInit(tkyCh, touch_pin);
    TKY_Enable();
    sta = TKY_GetCurChannelData( tkyCh, 32, 1, sampleData, 20 );
    if(sta == 0){
        for(int i=0;i<20;i++){
            PRINT("%d ", sampleData[i]);
        }
        PRINT("\n");
    }

	while(1);
}

/******************************** endfile @ main ******************************/
