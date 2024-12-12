/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *(1),Example routine to emulate a simulate USB-CDC Device, USE USART2(PA6/PA7),
 *Please note: This code uses the default serial port 1 for debugging,
 *if you need to modify the debugging serial port, please do not use USART2
 *(2),Merged HID devices, using endpoint 4 data downlink to fetch reverse upload;
 *
 *
 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/
#include "UART.h"
#include "CH58x_common.h"

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   µ÷ÊÔ³õÊ¼»¯
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
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    SetSysClock( CLK_SOURCE_HSE_PLL_62_4MHz );
    DebugInit();
    PRINT( "Simulate CDC-HID Device running on USBHS Controller\r\n" );

    /* Tim2 init */
    TMR2_TimerInit(FREQ_SYS / 10000);
    TMR2_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ(TMR2_IRQn);

    /* Usart2 init */
    UART2_Init( 1, DEF_UARTx_BAUDRATE, DEF_UARTx_STOPBIT, DEF_UARTx_PARITY  );

    USBHS_Device_Init(ENABLE);
    PFIC_EnableIRQ( USB2_DEVICE_IRQn );

    while(1)
    {
        UART2_DataRx_Deal( );
        UART2_DataTx_Deal( );
    }
}
