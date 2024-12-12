/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file of usb_desc.c
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __USB_DESC_H
#define __USB_DESC_H

#ifdef __cplusplus
extern "C" {
#endif
#include "CH58x_common.h"

/* file version */
#define DEF_FILE_VERSION             0x01

#define DEF_USB_VID                  0x1A86
#define DEF_USB_PID                  0x5537

/* USB device descriptor, device serial number(bcdDevice) */
#define DEF_IC_PRG_VER               DEF_FILE_VERSION
/* usb device endpoint size define */
#define DEF_USBD_UEP0_SIZE           64     /* usb hs/fs device end-point 0 size */
/* HS */
#define DEF_USBD_HS_PACK_SIZE        512    /* usb hs device max bluk/int pack size */

#define DEF_USBD_HS_ISO_PACK_SIZE    1024   /* usb hs device max iso pack size */
/* FS */
#define DEF_USBD_FS_PACK_SIZE        64     /* usb fs device max bluk/int pack size */
#define DEF_USBD_FS_ISO_PACK_SIZE    1023   /* usb fs device max iso pack size */
/* LS */
#define DEf_USBD_LS_UEP0_SIZE        8      /* usb ls device end-point 0 size */
#define DEF_USBD_LS_PACK_SIZE        64     /* usb ls device max int pack size */

#define DEF_USB_EP0_SIZE           0x40                                         /* 端点0大小 */
#define DEF_USB_EP1_SIZE           64                                           /* 端点1大小 */
#define DEF_USB_FS_EP2_SIZE        64                                           /* 端点2全速模式大小 */
#define DEF_USB_HS_EP2_SIZE        512                                          /* 端点2高速模式大小 */

/* HS end-point size */
#define DEF_USB_EP1_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP2_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP3_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP4_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP5_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP6_HS_SIZE          DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP7_HS_SIZE          DEF_USBD_HS_PACK_SIZE

/* FS end-point size */
#define DEF_USB_EP1_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP2_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP3_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP4_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP5_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP6_FS_SIZE          DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP7_FS_SIZE          DEF_USBD_FS_PACK_SIZE

#define DEF_USBD_DEVICE_DESC_LEN     ((uint8_t)MyDevDescr[0])
#define DEF_USBD_CONFIG_FS_DESC_LEN  ((uint16_t)MyCfgDescr_FS[2] + (uint16_t)(MyCfgDescr_FS[3] << 8))
#define DEF_USBD_CONFIG_HS_DESC_LEN  ((uint16_t)MyCfgDescr_HS[2] + (uint16_t)(MyCfgDescr_HS[3] << 8))
#define DEF_USBD_REPORT_DESC_LEN     0
#define DEF_USBD_LANG_DESC_LEN       ((uint16_t)MyLangDescr[0])
#define DEF_USBD_MANU_DESC_LEN       ((uint16_t)MyManuInfo[0])
#define DEF_USBD_PROD_DESC_LEN       ((uint16_t)MyProdInfo[0])
#define DEF_USBD_SN_DESC_LEN         ((uint16_t)MySerNumInfo[0])
#define DEF_USBD_QUALFY_DESC_LEN     ((uint16_t)MyQuaDesc[0])
#define DEF_USBD_BOS_DESC_LEN        ((uint16_t)MyBOSDesc[2] + (uint16_t)(MyBOSDesc[3] << 8))
#define DEF_USBD_FS_OTH_DESC_LEN     (DEF_USBD_CONFIG_HS_DESC_LEN)
#define DEF_USBD_HS_OTH_DESC_LEN     (DEF_USBD_CONFIG_FS_DESC_LEN)

extern const uint8_t MyDevDescr[ ];
extern const uint8_t MyCfgDescr_FS[ ];
extern const uint8_t MyCfgDescr_HS[ ];
extern const uint8_t MyLangDescr[ ];
extern const uint8_t MyManuInfo[ ];
extern const uint8_t MyProdInfo[ ];
extern const uint8_t MySerNumInfo[ ];
extern const uint8_t MyQuaDesc[ ];
extern const uint8_t MyBOSDesc[ ];
extern uint8_t TAB_USB_FS_OSC_DESC[ ];
extern uint8_t TAB_USB_HS_OSC_DESC[ ];



extern const uint8_t DEV_STATUS[];
extern const uint8_t HUB_DES[];
extern const uint8_t HUB_STATUS[];

#ifdef __cplusplus
}
#endif

#endif

