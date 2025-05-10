/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_m1.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2025/01/21
 * Description        : NFC PICC M1 head file for WCH chips.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef _WCH_NFCA_PICC_M1_H_
#define _WCH_NFCA_PICC_M1_H_

#include "wch_nfca_picc_bsp.h"
#include "ISO14443-3A.h"

typedef enum
{
    NFCA_PICC_M1_STATE_IDLE = 0,
    NFCA_PICC_M1_STATE_READY,
    NFCA_PICC_M1_STATE_ACTIVE,
    NFCA_PICC_M1_STATE_AUTHING,
    NFCA_PICC_M1_STATE_AUTHED_IDLE,
    NFCA_PICC_M1_STATE_WRITING,
    NFCA_PICC_M1_STATE_MAX,
} NFCA_PICC_M1_STATE_t;

typedef struct _nfca_picc_m1_manufacturer_data_struct
{
    uint8_t uid[4];
    uint8_t bcc;
    uint8_t sak;
    uint8_t atqa[2];
    uint8_t data[8];
} nfca_picc_m1_manufacturer_data_t;

typedef struct _nfca_picc_m1_sector_trailer_struct
{
    uint8_t key_a[6];
    uint8_t access_bits[4];
    uint8_t key_b[6];
} nfca_picc_m1_sector_trailer_t;

typedef __attribute__((aligned(4))) struct _nfca_picc_m1_data_struct
{
    union
    {
        uint8_t blocks[64][16];
        struct
        {
            nfca_picc_m1_manufacturer_data_t manufacturer_data;
            uint8_t reserve[63][16];
        };
        struct
        {
            uint8_t data[3][16];
            nfca_picc_m1_sector_trailer_t sector_trailer;
        } sectors[16];
    };

    uint8_t state;

    uint8_t authed_sector;
    uint8_t block_in_use;
    uint8_t key_a_or_b;
} nfca_picc_m1_data_t;

extern nfca_picc_m1_data_t g_nfca_picc_m1_data;

extern void nfca_picc_m1_enable(uint8_t *uid);

#endif  /* _WCH_NFCA_PICC_M1_H_ */
