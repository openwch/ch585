/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_compatibility_hid.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file of usbd_compatibility_hid.c
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef USER_USBD_COMPATIBILITY_HID_H_
#define USER_USBD_COMPATIBILITY_HID_H_

#include "usb_desc.h"
#define DEF_UART2_BUF_SIZE            8192
#define DEF_UART2_TOUT_TIME           30             // NOTE: the timeout time should be set according to the actual baud rate.

#define SET_REPORT_DEAL_OVER          0x00
#define SET_REPORT_WAIT_DEAL          0x01

#define USBHS_UART_REV_LEN         512               //uart receive buffer size
#define UART_REV_BUFFLEN           USBHS_UART_REV_LEN*2

extern __attribute__ ((aligned(4))) uint8_t UART2_Rx_Buf[UART_REV_BUFFLEN];  // UART2 Rx Buffer
extern uint8_t  HID_Report_Buffer[DEF_USBD_HS_PACK_SIZE];               // HID Report Buffer
extern volatile uint8_t HID_Set_Report_Flag;
extern volatile uint16_t Data_Pack_Max_Len ;                            // UART data packet length in hid packet
extern volatile uint16_t Head_Pack_Len;                                 // UART head packet( valid data length ) length in hid packet
extern volatile uint16_t Uart_Input_Ptr;      //Circular buffer write pointer
extern volatile uint16_t Uart_RecLen;         //The number of bytes remaining to be fetched in the current buffer
extern volatile uint16_t USB_RecLen;          //Data received by USB endpoint
extern volatile uint8_t  UploadPoint_Busy;    //Upload whether the endpoint is busy
extern volatile uint16_t Uart_Timeout_Count;  //Timeout processing calculation time

extern void UART2_Tx_Service( void );
extern void UART2_Rx_Service( void );
extern void USART2_Init( uint32_t baudrate );
extern void UART2_DMA_Init( void );
extern void TIM2_Init( void );
extern void HID_Set_Report_Deal( void );

#endif /* USER_USBD_COMPATIBILITY_HID_H_ */
