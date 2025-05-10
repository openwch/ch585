/********************************** (C) COPYRIGHT *******************************
 * File Name          : nfc_btssp_t2t.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/02/11
 * Description        : NFC PICC BTSSP T2T source file for WCH chips.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "nfc_btssp_t2t.h"

#if HAL_SLEEP
#error "如果使能了HAL_SLEEP，则需要在SLEEP.c的CH58x_LowPower函数中，在睡眠之前调用nfca_picc_stop，在唤醒后调用nfca_picc_start。"
#error "修改完成后，可以注释这段话。"
#endif

NFC_BTSSP_T2T_INIT_ERR_t nfc_btssp_t2t_init(nfc_btssp_t2t_init_t *cfg)
{
    uint16_t temp;

    static const uint8_t btssp_ndef_hdr[] = {
        0x03, 0x00,
        0x91, 0x02, 0x0a, 'H', 's', 0x13,
        0xD1, 0x02, 0x04, 'a', 'c', 0x01, 0x01, 0x30, 0x00,
        0x5a, 0x20, 0x4d, 0x01,
        'a', 'p', 'p', 'l', 'i', 'c', 'a', 't', 'i', 'o', 'n', '/', 'v', 'n', 'd', '.',
        'b', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', '.', 'l' ,'e', '.', 'o', 'o', 'b',
        0x30,
    };

    uint8_t *t2t_data = (uint8_t *)&g_nfca_picc_t2t_data.static_data_pages[0];

    if(cfg->t2t_uid == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_UID;
    }

    if(cfg->bd_addr == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_BD_ADDR;
    }

    if(cfg->le_role > 0x0f)
    {
        return NFC_BTSSP_T2T_INIT_ERR_LE_ROLE;
    }

    nfca_picc_stop();

    /* 初始化NFCA PICC */
    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    /* 初始化uid */
    nfca_picc_t2t_enable(cfg->t2t_uid);

    __MCPY((void *)t2t_data, (void *)btssp_ndef_hdr, (void *)(btssp_ndef_hdr + sizeof(btssp_ndef_hdr)));

    t2t_data = t2t_data + sizeof(btssp_ndef_hdr);

    /* GAP_ADTYPE_LE_BD_ADDR */
    *t2t_data++ = 0x08;
    *t2t_data++ = GAP_ADTYPE_LE_BD_ADDR;
    *t2t_data++ = cfg->bd_addr[0];
    *t2t_data++ = cfg->bd_addr[1];
    *t2t_data++ = cfg->bd_addr[2];
    *t2t_data++ = cfg->bd_addr[3];
    *t2t_data++ = cfg->bd_addr[4];
    *t2t_data++ = cfg->bd_addr[5];
    *t2t_data++ = cfg->bd_addr_type;

    /* GAP_ADTYPE_LE_ROLE */
    *t2t_data++ = 0x02;
    *t2t_data++ = GAP_ADTYPE_LE_ROLE;
    *t2t_data++ = cfg->le_role;

    /* GAP_ADTYPE_SM_TK */
    if(cfg->sm_tk != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_SM_TK;
        tmos_memcpy(t2t_data, cfg->sm_tk, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LE_SC_CONFIRMATION_VALUE */
    if(cfg->le_sc_confirm != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_LE_SC_CONFIRMATION_VALUE;
        tmos_memcpy(t2t_data, cfg->le_sc_confirm, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LE_SC_RANDOM_VALUE */
    if(cfg->le_sc_random != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_LE_SC_RANDOM_VALUE;
        tmos_memcpy(t2t_data, cfg->le_sc_random, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LOCAL_NAME_COMPLETE */
    if(cfg->local_name_complete != NULL)
    {
        uint8_t local_name_len;
        local_name_len = tmos_strlen(cfg->local_name_complete);
        if(local_name_len > 0)
        {
            *t2t_data++ = local_name_len + 1;
            *t2t_data++ = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
            tmos_memcpy(t2t_data, cfg->local_name_complete, local_name_len);
            t2t_data = t2t_data + local_name_len;
        }
    }

    if((cfg->other_adv_data != NULL) && (cfg->other_adv_data_len > 0))
    {
        tmos_memcpy(t2t_data, cfg->other_adv_data, cfg->other_adv_data_len);
        t2t_data = t2t_data + cfg->other_adv_data_len;
    }

    /* END */
    *t2t_data++ = 0xfe;
    *t2t_data = 0;

    temp = (uint32_t)t2t_data - (uint32_t)&g_nfca_picc_t2t_data.static_data_pages[0];

    g_nfca_picc_t2t_data.pages[4].data8[1] = temp - 3;
    g_nfca_picc_t2t_data.pages[8].data8[3] = temp - 55;

    nfca_picc_start();

    return NFC_BTSSP_T2T_INIT_OK;
}

NFC_BTSSP_T2T_INIT_ERR_t nfc_bt_t2t_init(nfc_btssp_t2t_init_t *cfg)
{
    uint16_t temp;

    static const uint8_t bt_ndef_hdr[] = {
        0x03, 0x00,
        0xd2, 0x20, 0x12,
        'a', 'p', 'p', 'l', 'i', 'c', 'a', 't', 'i', 'o', 'n', '/', 'v', 'n', 'd', '.',
        'b', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', '.', 'e' ,'p', '.', 'o', 'o', 'b',
    };

    uint8_t *t2t_data = (uint8_t *)&g_nfca_picc_t2t_data.static_data_pages[0];

    if(cfg->t2t_uid == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_UID;
    }

    if(cfg->bd_addr == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_BD_ADDR;
    }

    nfca_picc_stop();

    /* 初始化NFCA PICC */
    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    /* 初始化uid */
    nfca_picc_t2t_enable(cfg->t2t_uid);

    __MCPY((void *)t2t_data, (void *)bt_ndef_hdr, (void *)(bt_ndef_hdr + sizeof(bt_ndef_hdr)));

    t2t_data = t2t_data + sizeof(bt_ndef_hdr);

    t2t_data[1] = 0;
    t2t_data[2] = cfg->bd_addr[0];
    t2t_data[3] = cfg->bd_addr[1];
    t2t_data[4] = cfg->bd_addr[2];
    t2t_data[5] = cfg->bd_addr[3];
    t2t_data[6] = cfg->bd_addr[4];
    t2t_data[7] = cfg->bd_addr[5];

    if(cfg->local_name_complete != NULL)
    {
        uint8_t local_name_len;
        local_name_len = tmos_strlen(cfg->local_name_complete);
        if(local_name_len > 0)
        {
            t2t_data[0] = local_name_len + 10;
            t2t_data[8] = local_name_len + 1;
            t2t_data[9] = 0x09;
            __MCPY((void *)&t2t_data[10], (void *)cfg->local_name_complete, (void *)(cfg->local_name_complete + local_name_len));

            t2t_data = t2t_data + local_name_len + 10;
            *t2t_data++ = 0xfe;
            *t2t_data = 0;
        }
        else
        {
            goto no_name;
        }
    }
    else
    {
no_name:
        t2t_data[0] = 8;
        t2t_data = t2t_data + 8;
        *t2t_data++ = 0xfe;
        *t2t_data = 0;
    }

    temp = (uint32_t)t2t_data - (uint32_t)&g_nfca_picc_t2t_data.static_data_pages[0];

    g_nfca_picc_t2t_data.pages[4].data8[1] = temp - 3;
    g_nfca_picc_t2t_data.pages[5].data8[0] = temp - 38;

    nfca_picc_start();

    return NFC_BTSSP_T2T_INIT_OK;
}
