/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_t2t.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/21
 * Description        : NFC PICC type2 Tag.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef _WCH_NFCA_PICC_T2T_H_
#define _WCH_NFCA_PICC_T2T_H_

#include "wch_nfca_picc_bsp.h"
#include "ISO14443-3A.h"

#define WCH_NFCA_PICC_T2T_PAGES_NUM             135     /* T2T“≥√Ê ˝¡ø≈‰÷√ */

#if (WCH_NFCA_PICC_T2T_PAGES_NUM < 48) || (WCH_NFCA_PICC_T2T_PAGES_NUM > 255)
#error "WCH_NFCA_PICC_T2T_PAGES_NUM MUST BIGGER THAN 48 AND SMALL THAN 256."
#endif

typedef enum
{
    NFCA_PICC_T2T_STATE_IDLE = 0,
    NFCA_PICC_T2T_STATE_READY1,
    NFCA_PICC_T2T_STATE_READY2,
    NFCA_PICC_T2T_STATE_ACTIVE,
    NFCA_PICC_T2T_STATE_MAX,
} NFCA_PICC_T2T_STATE_t;

typedef union _nfca_picc_t2t_page_struct
{
    uint32_t data32[1];
    uint16_t data16[2];
    uint8_t data8[4];
} nfca_picc_t2t_page_t;

typedef __attribute__((aligned(4))) struct _nfca_picc_t2t_data_struct
{
    union
    {
        struct
        {
            uint8_t uid_cl1[4];
            uint8_t uid_cl2[4];
        };
        struct
        {
            uint8_t uid_ct;
            uint8_t uid[7];
        };
    };
    union
    {
        nfca_picc_t2t_page_t pages[WCH_NFCA_PICC_T2T_PAGES_NUM];
        struct
        {
            uint8_t serial_num[9];
            uint8_t internal;
            uint8_t lock_bytes[2];
            uint8_t cc[4];
            nfca_picc_t2t_page_t static_data_pages[12];
            nfca_picc_t2t_page_t dynamic_data_pages[WCH_NFCA_PICC_T2T_PAGES_NUM - 19];
            nfca_picc_t2t_page_t reserved_pages[3];
        };
    };
    uint8_t state;
} nfca_picc_t2t_data_t;

extern nfca_picc_t2t_data_t g_nfca_picc_t2t_data;

extern void nfca_picc_t2t_enable(uint8_t *uid);

#endif  /* _WCH_NFCA_PICC_T2T_H_ */
