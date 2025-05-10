/********************************** (C) COPYRIGHT *******************************
 * File Name          : nfc_btssp_t2t.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/12/18
 * Description        : NFC PICC BTSSP T2T header file for WCH chips.
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _NFC_BTSSP_T2T_H_
#define _NFC_BTSSP_T2T_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CONFIG.h"
#include "wch_nfca_picc_t2t.h"

typedef enum
{
    NFC_BTSSP_T2T_INIT_OK = 0,
    NFC_BTSSP_T2T_INIT_ERR_UID,
    NFC_BTSSP_T2T_INIT_ERR_BD_ADDR,
    NFC_BTSSP_T2T_INIT_ERR_LE_ROLE,
} NFC_BTSSP_T2T_INIT_ERR_t;

typedef struct _nfc_btssp_t2t_init_struct
{
    uint8_t *t2t_uid;
    uint8_t *local_name_complete;
    uint8_t *sm_tk;
    uint8_t *le_sc_confirm;
    uint8_t *le_sc_random;
    uint8_t *bd_addr;
    uint8_t *other_adv_data;

    uint8_t bd_addr_type;
    uint8_t le_role;
    uint8_t other_adv_data_len;
} nfc_btssp_t2t_init_t;

extern NFC_BTSSP_T2T_INIT_ERR_t nfc_btssp_t2t_init(nfc_btssp_t2t_init_t *cfg);

extern NFC_BTSSP_T2T_INIT_ERR_t nfc_bt_t2t_init(nfc_btssp_t2t_init_t *cfg);

#ifdef __cplusplus
}
#endif

#endif  /* _NFC_BTSSP_T2T_H_ */
