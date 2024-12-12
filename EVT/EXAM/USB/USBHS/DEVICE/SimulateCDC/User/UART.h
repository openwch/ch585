/********************************** (C) COPYRIGHT *******************************
* File Name          : UART.H
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : UART communication-related headers
*******************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "string.h"
#include "CH58x_common.h"
#include "string.h"
#include <ch585_usbhs_device.h>

/******************************************************************************/
/* Related macro definitions */
/* Serial buffer related definitions */
#define DEF_UARTx_RX_BUF_LEN       ( 4 * 512 )                                  /* Serial x receive buffer size */
#define DEF_UARTx_TX_BUF_LEN       ( 2 * 512 )                                  /* Serial x transmit buffer size */
#define DEF_USB_FS_PACK_LEN        64                                         /* USB full speed mode packet size for serial x data */
#define DEF_USB_HS_PACK_LEN        DEF_USBD_HS_PACK_SIZE                        /* USB high speed mode packet size for serial x data */
#define DEF_UARTx_TX_BUF_NUM_MAX   ( DEF_UARTx_TX_BUF_LEN / DEF_USB_HS_PACK_LEN ) /* Serial x transmit buffer size */

/* Serial port receive timeout related macro definition */
#define DEF_UARTx_BAUDRATE         115200                                       /* Default baud rate for serial port */
#define DEF_UARTx_STOPBIT          0                                            /* Default stop bit for serial port */
#define DEF_UARTx_PARITY           0                                            /* Default parity bit for serial port */
#define DEF_UARTx_DATABIT          8                                            /* Default data bit for serial port */
#define DEF_UARTx_RX_TIMEOUT       30                                           /* Serial port receive timeout, in 100uS */
#define DEF_UARTx_USB_UP_TIMEOUT   60000                                        /* Serial port receive upload timeout, in 100uS */

/* Serial port transceiver DMA channel related macro definition */
#define DEF_UART2_TX_DMA_CH        DMA1_Channel7                                /* Serial 2 transmit channel DMA channel */
#define DEF_UART2_RX_DMA_CH        DMA1_Channel6                                /* Serial 1 transmit channel DMA channel */

#define UART_REV_BUFFLEN           DEF_USB_HS_PACK_LEN*2

/************************************************************/
/* Serial port X related structure definition */
typedef struct __attribute__((packed)) _CDC_CTL
{
    volatile uint16_t USB_Output_Ptr;
    volatile uint16_t Uart_Output_Ptr;
    volatile uint16_t Uart_Input_Ptr;      //Circular buffer write pointer
    volatile uint16_t Uart_RecLen;         //The number of bytes remaining to be fetched in the current buffer
    volatile uint16_t USB_RecLen;          //Data received by USB endpoint
    volatile uint8_t  UploadPoint_Busy;    //Upload whether the endpoint is busy
    volatile uint8_t  DownloadPoint_Busy;  //Download whether the endpoint is busy
    volatile uint16_t Uart_Timeout_Count;  //Timeout processing calculation time
    volatile uint8_t  Com_Cfg[ 8 ];        /* Serial x parameter configuration (default baud rate is 115200, 1 stop bit, no parity, 8 data bits) */
}CDC_CTL, *PCDC_CTL;

/***********************************************************************************************************************/
/* Constant, variable extents */
/* The following are serial port transmit and receive related variables and buffers */
extern volatile CDC_CTL CDC;                                                    /* Serial x control related structure */
extern __attribute__ ((aligned(4))) uint8_t UART2_Tx_Buf[ UART_REV_BUFFLEN ]; /* Serial x transmit buffer */
extern __attribute__ ((aligned(4))) uint8_t UART2_Rx_Buf[ UART_REV_BUFFLEN ]; /* Serial x transmit buffer */

/***********************************************************************************************************************/
/* Function extensibility */
extern void TIM2_Init( void );
extern void UART2_CfgInit( uint32_t baudrate, uint8_t stopbits, uint8_t parity ); /* UART1 initialization */
extern void UART2_ParaInit( uint8_t mode );                                       /* Serial port parameter initialization */
extern void UART2_DMAInit( uint8_t type, uint8_t *pbuf, uint32_t len );           /* Serial port 1-related DMA initialization */
extern void UART2_Init( uint8_t mode, uint32_t baudrate, uint8_t stopbits, uint8_t parity ); /* Serial port 1 initialization */
extern void UART2_DataTx_Deal( void );                                            /* Serial port 1 data sending processing  */
extern void UART2_DataRx_Deal( void );                                            /* Serial port 1 data reception processing */
extern void UART2_USB_Init( void );                                               /* USB serial port initialization*/

#ifdef __cplusplus
}
#endif

#endif

/***********************************************************************************************************************/


