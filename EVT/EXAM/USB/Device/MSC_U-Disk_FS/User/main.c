/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2026/11/1
 * Description        : Main program body of the full-speed (USBFS) U-disk demo.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/* @Note
 * UDisk Example (USB full-speed device):
 * This program provides examples of UDisk. Three storage media are supported and
 * selected by STORAGE_MEDIUM in SW_UDISK.h:
 *     MEDIUM_INTERNAL_FLASH : on-chip Flash
 *     MEDIUM_SPI_FLASH      : external SPI-Flash on SPI0 (PA12~PA15)
 *
 * The USB full-speed controller is used, EP2 is the bulk IN endpoint and EP3 is
 * the bulk OUT endpoint, both with a 64-byte max packet size.
 */

#include "usb_device.h"
#include "CH58x_common.h"
#include "Internal_Flash.h"
#include "SPI_FLASH.h"
#include "SW_UDISK.h"

/******** user defined endpoint RAM ****************************************/
__attribute__((aligned(4))) uint8_t EP0_Databuf[64 + 64 + 64]; //EP0 control(64) + reserved(64 + 64)

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   debug uart initialization, UART1 on PB12(RXD)/PB13(TXD)
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOB_SetBits(GPIO_Pin_13);
    GPIOPinRemap(ENABLE, RB_PIN_UART1);
    GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
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
    HSECFG_Capacitance(HSECap_20p);
    SetSysClock(SYSCLK_FREQ);

    DebugInit();	// SPI与默认串口打印引脚冲突，所以此例程改用UART1输出打印
    PRINT("start\n");

#if (STORAGE_MEDIUM == MEDIUM_SPI_FLASH)
    PRINT( "USBFS UDisk Demo\r\nStorage Medium: SPI FLASH \r\n" );
    /* SPI flash init */
    FLASH_Port_Init( );
    /* FLASH ID check */
    FLASH_IC_Check( );

#elif (STORAGE_MEDIUM == MEDIUM_INTERNAL_FLASH)
    PRINT( "USBFS UDisk Demo\r\nStorage Medium: INTERNAL FLASH \r\n" );
    Flash_Sector_Count = IFLASH_UDISK_SIZE  / DEF_UDISK_SECTOR_SIZE;
    Flash_Sector_Size = DEF_UDISK_SECTOR_SIZE;
#endif

    /* Enable Udisk */
    Udisk_Capability = Flash_Sector_Count;
    Udisk_Status |= DEF_UDISK_EN_FLAG;

    /* Endpoint buffer assignment, must be done before USB_DeviceInit() */
    pEP0_RAM_Addr = EP0_Databuf;

    /* USB device initialization */
    USB_DeviceInit();
    /* U-disk end-point configuration: EP2 bulk IN, EP3 bulk OUT */
    USB_Device_Endp_Init();
    PFIC_EnableIRQ( USB_IRQn );

    while (1);
}
