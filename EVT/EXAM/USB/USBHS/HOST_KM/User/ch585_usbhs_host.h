/********************************** (C) COPYRIGHT *******************************
* File Name          : ch585_usbhs_host.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : header file of ch585_usbhs_device.c
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __CH585_USBHS_HOST_H__
#define __CH585_USBHS_HOST_H__

/*******************************************************************************/
/* Header File */
#include "CH58x_common.h"
#include "string.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Variable Declaration */
extern __attribute__((aligned(4))) uint8_t  RxBuffer[ ];
extern __attribute__((aligned(4))) uint8_t  TxBuffer[ ];

/* USB Setup Request */
#define pUSBHS_SetupRequest        ( (PUSB_SETUP_REQ)TxBuffer )

/* USB Buffer Size */
#ifndef DEFAULT_ENDP0_SIZE
#define DEFAULT_ENDP0_SIZE          8          // default maximum packet size for endpoint 0
#endif
#ifndef USBHS_MAX_PACKET_SIZE
#define USBHS_MAX_PACKET_SIZE      64
#endif

/*******************************************************************************/
/* Function Declaration */
extern void USBHS_Host_Init( FunctionalState sta );
extern uint8_t USBHSH_CheckRootHubPortStatus( uint8_t status );
extern uint8_t USBHSH_CheckRootHubPortEnable( void );
extern uint8_t USBHSH_CheckRootHubPortSpeed( void );
extern void USBHSH_SetSelfAddr( uint8_t addr );
extern void USBHSH_ResetRootHubPort( uint8_t mode );
extern uint8_t USBHSH_EnableRootHubPort( uint8_t *pspeed );
extern uint8_t USBHSH_Transact( uint8_t endp_pid, uint16_t endp_tog, uint32_t timeout );
extern uint8_t USBHSH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBHSH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf );
extern uint8_t USBHSH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len );
extern uint8_t USBHSH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf );
extern uint8_t USBHSH_SetUsbAddress( uint8_t ep0_size, uint8_t addr );
extern uint8_t USBHSH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val );
extern uint8_t USBHSH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num );
extern uint8_t USBHSH_GetEndpData( uint8_t endp_num, uint16_t *pendp_tog, uint8_t *pbuf, uint16_t *plen );
extern uint8_t USBHSH_SendEndpData( uint8_t endp_num, uint16_t *pendp_tog, uint8_t *pbuf, uint16_t len );

#ifdef __cplusplus
}
#endif

#endif


