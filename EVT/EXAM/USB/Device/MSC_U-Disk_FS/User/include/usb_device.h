/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_device.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file of usb_device.c
 *                      USB full-speed (USBFS) device layer for the U-disk demo.
 *                      The low level driver is CH58x_usbdev.c from the SDK
 *                      (USB_DeviceInit / DevEPx_IN_Deal / pEPx_RAM_Addr).
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __USB_DEVICE_H__
#define __USB_DEVICE_H__

/*******************************************************************************/
/* Header File */
#include "CH58x_common.h"
#include "string.h"
#include "usb_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************/
/* Macro Definition */

/* General */
#define pUSB_SetupReqPak              ((PUSB_SETUP_REQ)pEP0_DataBuf)

#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
/* Endpoint Status */
#define DEF_UEP_BUSY                  0x01
#define DEF_UEP_FREE                  0x00
/* Endpoint Number */
#define DEF_UEP_NUM                   8
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07

#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */

/******************************************************************************/
/* Variable Declaration */

/* SetUp Request Values */
extern const uint8_t *pUSB_Descr;

/* Setup Request */
extern volatile uint8_t  USB_SetupReqCode;
extern volatile uint8_t  USB_SetupReqType;
extern volatile uint16_t USB_SetupReqValue;
extern volatile uint16_t USB_SetupReqIndex;
extern volatile uint16_t USB_SetupReqLen;

/* USB Device Status */
extern volatile uint8_t  USB_DevConfig;
extern volatile uint8_t  USB_DevAddr;
extern volatile uint8_t  USB_DevSleepStatus;
extern volatile uint8_t  USB_DevEnumStatus;

/* Endpoint tx busy flag */
extern volatile uint8_t  USB_Endp_Busy[ ];

/* Endpoint Buffer (bulk IN / bulk OUT of the U-disk) */
extern __attribute__((aligned(4))) uint8_t UDisk_In_Buf[ ];
extern __attribute__((aligned(4))) uint8_t UDisk_Out_Buf[ ];

/********************************************************************************/
/* Function Declaration */

/**
 * @brief   initialize the U-disk end-points: EP2 as bulk IN, EP3 as bulk OUT
 */
extern void USB_Device_Endp_Init ( void );
/**
 * @brief   upload one packet of data through the bulk IN end-point
 *
 * @param   endp - end-point number (DEF_UEP2)
 * @param   pbuf - pointer to the data to be uploaded
 * @param   len - data length, no more than 64 bytes
 * @param   mod - DEF_UEP_DMA_LOAD or DEF_UEP_CPY_LOAD
 *
 * @return  0 when the upload is started, 1 when the end-point is busy
 */
extern uint8_t USB_Endp_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod );
/**
 * @brief   handle the USB transfer interrupt: control transfers and U-disk bulk transfers
 */
extern void USB_DevTransProcess( void );
/**
 * @brief   USB interrupt service routine
 */
extern void USB_IRQHandler( void );

#ifdef __cplusplus
}
#endif

#endif
