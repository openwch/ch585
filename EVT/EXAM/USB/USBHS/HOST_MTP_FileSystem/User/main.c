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
 * @Note
 * This example demonstrates the enumeration process of a USB host to a device that
 * supports MTP and PTP protocols, and reads its files.
 *
 * If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/

#include "usb_host_config.h"

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
    DebugInit( );

    PRINT( "USBHS Host Init\r\n" );
    USBHS_Host_Init( ENABLE );
    memset( &RootHubDev[ DEF_USBHS_PORT_INDEX ].bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
    memset( &HostCtl[ DEF_USBHS_PORT_INDEX * DEF_ONE_USB_SUP_DEV_TOTAL ].InterfaceNum, 0, DEF_ONE_USB_SUP_DEV_TOTAL * sizeof( HOST_CTL ) );

    while( 1 )
    {
        USBH_MainDeal( );
    }
}
