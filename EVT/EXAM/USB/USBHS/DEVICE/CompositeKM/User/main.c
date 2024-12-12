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

#include <ch585_usbhs_device.h>
#include "usbd_composite_km.h"

/*
 * @Note
 * Composite Keyboard and Mouse Example:
 * This example uses PB0-PB3 and PA8-PA11 to simulate keyboard key pressing and mouse
 * movement respectively, active low.
 * At the same time, it also uses USART2 to receive the specified data sent from
 * the host to simulate the pressing and releasing of the following specific keyboard
 * keys. Data is sent in hexadecimal format and 1 byte at a time.
 * 'W' -> 0x1A
 * 'A' -> 0x04
 * 'S' -> 0x16
 * 'D' -> 0x07
 *
 *  If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
 */

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
    PRINT( "Compatibility HID Running On USBHS Controller\n" );

    /* Initialize USART2 for receiving the specified keyboard data */
    USART2_Init( 115200 );
    PRINT( "USART2 Init OK!\r\n" );

    PWR_PeriphWakeUpCfg(ENABLE,RB_SLP_GPIO_WAKE,Short_Delay);
    PWR_PeriphWakeUpCfg(ENABLE,RB_SLP_USB2_WAKE,Short_Delay);
    /* Initialize GPIO for keyboard scan */
    KB_Scan_Init( );
    KB_Sleep_Wakeup_Cfg( );
    PRINT( "KB Scan Init OK!\r\n" );

    /* Initialize GPIO for mouse scan */
    MS_Scan_Init( );
    MS_Sleep_Wakeup_Cfg( );
    PRINT( "MS Scan Init OK!\r\n" );

    /* Initialize timer for Keyboard and mouse scan timing */
    TMR3_TimerInit(FREQ_SYS / 10000);
    TMR3_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ(TMR3_IRQn);
    PRINT( "TIM3 Init OK!\r\n" );

    /* Initialize USBHS interface to communicate with the host  */
    USBHS_Device_Init(ENABLE);
    PFIC_EnableIRQ( USB2_DEVICE_IRQn );

    while (1)
    {
        if( USBHS_DevEnumStatus )
        {
            /* Handle keyboard scan data */
            KB_Scan_Handle( );

            /* Handle keyboard lighting */
            KB_LED_Handle( );

            /* Handle mouse scan data */
            MS_Scan_Handle( );

            /* Handle USART2 receiving data */
            USART2_Receive_Handle( );
        }
    }
}
