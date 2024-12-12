/********************************** (C) COPYRIGHT *******************************
 * File Name          : iap.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/03/15
 * Description        : USB IAP例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef _IAP_H_
#define _IAP_H_

#include "CH58x_common.h"
#include "usb_desc.h"

/* you can change the following definitions below, just keep them same in app and iap. */
#define        APP_CODE_START_ADDR        0x00001000
#define        APP_CODE_END_ADDR          0x00070000

#define     USE_EEPROM_FLAG     0

#define    jumpApp   ((  void  (*)  ( void ))  ((int*)APP_CODE_START_ADDR))


#define FLAG_USER_CALL_IAP   0x55
#define FLAG_USER_CALL_APP   0xaa

/* 存放在DataFlash地址，不能占用蓝牙的位置 */
#define IAP_FLAG_DATAFLASH_ADD               0

/* 存放在DataFlash里的OTA信息 */
typedef struct
{
    unsigned char ImageFlag;            //记录的当前的image标志
    unsigned char Revd[3];
} IAPDataFlashInfo_t;

/* you should not change the following definitions below. */
#define        CMD_IAP_PROM         0x80
#define        CMD_IAP_ERASE        0x81
#define        CMD_IAP_VERIFY       0x82
#define        CMD_IAP_END          0x83

/* usb data length is 64 */
#define     IAP_LEN            64

typedef union _IAP_CMD_STRUCT
{
    struct
    {
        uint8_t    cmd;
        uint8_t    len;
        uint8_t    addr[4];
    } erase;
    struct
    {
    	uint8_t    cmd;
    	uint8_t    len;
    	uint8_t    status[2];
    } end;
    struct
    {
    	uint8_t    cmd;
    	uint8_t    len;
    	uint8_t    addr[4];
    	uint8_t    buf[IAP_LEN - 6];
    } verify;
    struct
    {
    	uint8_t    cmd;
    	uint8_t    len;
    	uint8_t    buf[IAP_LEN - 2];
    } program;
    struct
    {
    	uint8_t    buf[IAP_LEN];
    } other;
} iap_cmd_t;


extern __attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
extern __attribute__ ((aligned(4))) uint8_t USBHS_EP2_Tx_Buf[ DEF_USB_EP2_HS_SIZE ];
extern __attribute__ ((aligned(4))) uint8_t USBHS_EP2_Rx_Buf[ DEF_USB_EP2_HS_SIZE ];
extern uint32_t g_tcnt;

extern volatile uint8_t  USBHS_Endp_Busy[ DEF_UEP_NUM ];

extern void my_memcpy(void *dst, const void *src, uint32_t l);

extern void USB_DevTransProcess(void);

#endif /* _IAP_H_ */
