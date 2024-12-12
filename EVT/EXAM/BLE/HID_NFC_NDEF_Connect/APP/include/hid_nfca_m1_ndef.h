/********************************** (C) COPYRIGHT *******************************
 * File Name          : hid_nfca_m1_ndef.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/11/15
 * Description        : NFC PICC head file for WCH chips.
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _HID_NFCA_NDEF_H_
#define _HID_NFCA_NDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CONFIG.h"
#include "wch_nfca_picc.h"
#include "wch_nfca_picc_m1.h"

#define HID_NFCA_M1_NDEF_INCLUDE_BLE_NAME               0       /* ÊÇ·ñ°üÀ¨BLEÃû³Æ */

extern void hid_nfca_m1_ndef_init(void);

extern void hid_nfca_m1_ndef_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
