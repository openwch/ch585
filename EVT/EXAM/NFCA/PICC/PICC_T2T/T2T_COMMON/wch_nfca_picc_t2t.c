/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_t2t.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/21
 * Description        : NFC PICC type2 Tag.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "wch_nfca_picc_t2t.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 0
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

#define CMD_GET_VERSION                 0x60
#define CMD_READ                        0x30
#define CMD_FAST_READ                   0x3A
#define CMD_WRITE                       0xA2
#define CMD_COMPAT_WRITE                0xA0
#define CMD_READ_CNT                    0x39
#define CMD_INCR_CNT                    0xA5
#define CMD_PWD_AUTH                    0x1B
#define CMD_READ_SIG                    0x3C
#define CMD_CHECK_TEARING_EVENT         0x3E
#define CMD_VCSL                        0x4B

#define ACK_NAK_FRAME_SIZE              4
#define ACK_VALUE                       0x0A
#define NAK_INVALID_ARG                 0x00
#define NAK_CRC_ERROR                   0x01
#define NAK_NOT_AUTHED                  0x04
#define NAK_EEPROM_ERROR                0x05
#define NAK_OTHER_ERROR                 0x06

#define BYTE_SWAP(x)                    (((uint8_t)(x)>>4)|((uint8_t)(x)<<4))
#define NO_ACCESS                       0x07

#define NFCA_PICC_T2T_SAK               0x00

static void nfca_picc_t2t_online(void);
static uint16_t nfca_picc_t2t_data_handler(uint16_t bits_num);
static void nfca_picc_t2t_offline(void);

nfca_picc_t2t_data_t g_nfca_picc_t2t_data;

#if IS014443A_FAST_CL
uint16_t cl1_pre_len;
uint8_t iso14443a_cl1_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_CL_FRAME_SIZE)];
uint16_t sak1_pre_len;
uint8_t iso14443a_sak1_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_SAK_FRAME_SIZE)];

uint16_t cl2_pre_len;
uint8_t iso14443a_cl2_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_CL_FRAME_SIZE)];
uint16_t sak2_pre_len;
uint8_t iso14443a_sak2_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_SAK_FRAME_SIZE)];
#endif

/* 回调函数会在中断里调用，所以需要注意和外部程序的线程安全代码处理*/
static nfca_picc_cb_t gs_nfca_picc_t2t_cb =
{
    .online = nfca_picc_t2t_online,
    .data_handler = nfca_picc_t2t_data_handler,
    .offline = nfca_picc_t2t_offline,
};   /* T2T卡数据处理回调 */

__attribute__((section(".highcode")))
static uint16_t nfca_picc_t2t_data_handler(uint16_t bits_num)
{
    uint16_t send_bits = 0;

    if((bits_num == 7) &&
            (((g_picc_data_buf[0] == ISO14443A_CMD_REQA)) || (g_picc_data_buf[0] == ISO14443A_CMD_WUPA)))
    {
        g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_READY1;
        g_picc_data_buf[0] = 0x44;
        g_picc_data_buf[1] = 0x00;
        send_bits = 16;
        ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, 2);
        goto end;
    }

    switch(g_nfca_picc_t2t_data.state)
    {
    case NFCA_PICC_T2T_STATE_READY1:
        if(g_picc_data_buf[0] == ISO14443A_CMD_SELECT_CL1)
        {
            if (ISO14443ASelect(g_picc_data_buf, &send_bits, (uint8_t *)g_nfca_picc_t2t_data.uid_cl1, ISO14443A_SAK_INCOMPLETE) == 0)
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_READY2;
            }
#if IS014443A_FAST_CL ==  0
            if(send_bits != 0)
            {
                ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, send_bits / 8);
            }
#else
            if(send_bits == ISO14443A_SAK_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_sak1_pre_raw_data, sak1_pre_len);
            }
            else if(send_bits == ISO14443A_CL_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_cl1_pre_raw_data, cl1_pre_len);
            }
            else
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
            }
#endif
        }
        else if(g_picc_data_buf[0] == ISO14443A_CMD_HLTA)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
            }
        }
        else
        {
            g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
        }
        break;
    case NFCA_PICC_T2T_STATE_READY2:
        if(g_picc_data_buf[0] == ISO14443A_CMD_SELECT_CL2)
        {
            if (ISO14443ASelect(g_picc_data_buf, &send_bits, (uint8_t *)g_nfca_picc_t2t_data.uid_cl2, NFCA_PICC_T2T_SAK) == 0)
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_ACTIVE;
            }
#if IS014443A_FAST_CL ==  0
            if(send_bits != 0)
            {
                ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, send_bits / 8);
            }
#else
            if(send_bits == ISO14443A_SAK_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_sak2_pre_raw_data, sak2_pre_len);
            }
            else if(send_bits == ISO14443A_CL_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_cl2_pre_raw_data, cl2_pre_len);
            }
            else
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
            }
#endif
        }
        else if(g_picc_data_buf[0] == ISO14443A_CMD_HLTA)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
            }
        }
        else
        {
            g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
        }
        break;
    case NFCA_PICC_T2T_STATE_ACTIVE:

        if(g_picc_data_buf[0] == ISO14443A_CMD_HLTA)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
            }
        }
        else if(g_picc_data_buf[0] == CMD_READ)
        {
            uint8_t page_address;
            uint8_t offset;
            if(g_picc_data_buf[1] < WCH_NFCA_PICC_T2T_PAGES_NUM)
            {
                page_address = g_picc_data_buf[1];
                for(uint8_t i = 0; i < 4; i++)
                {
                    g_picc_data_buf[4 * i] = g_nfca_picc_t2t_data.pages[page_address].data8[0];
                    g_picc_data_buf[4 * i + 1] = g_nfca_picc_t2t_data.pages[page_address].data8[1];
                    g_picc_data_buf[4 * i + 2] = g_nfca_picc_t2t_data.pages[page_address].data8[2];
                    g_picc_data_buf[4 * i + 3] = g_nfca_picc_t2t_data.pages[page_address].data8[3];
                    page_address = (page_address + 1) % WCH_NFCA_PICC_T2T_PAGES_NUM;
                }
                ISO14443AAppendCRCA(g_picc_data_buf, 16);
                ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, 18);
                send_bits = 18 * 8;
            }
            else
            {
                g_picc_data_buf[0] = NAK_INVALID_ARG;
                send_bits = 4;
            }
        }
        else if(g_picc_data_buf[0] == CMD_WRITE)
        {
            uint8_t page_address = g_picc_data_buf[1];
            if((page_address < 135) && (page_address > 3))
            {
                g_nfca_picc_t2t_data.pages[page_address].data8[0] = g_picc_data_buf[2];
                g_nfca_picc_t2t_data.pages[page_address].data8[1] = g_picc_data_buf[3];
                g_nfca_picc_t2t_data.pages[page_address].data8[2] = g_picc_data_buf[4];
                g_nfca_picc_t2t_data.pages[page_address].data8[3] = g_picc_data_buf[5];
                g_picc_data_buf[0] = ACK_VALUE;
            }
            else
            {
                g_picc_data_buf[0] = NAK_INVALID_ARG;
            }
            send_bits = 4;
        }
        else
        {
            g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
        }

        break;
    default:
        break;
    }

end:
    return send_bits;
}

static void nfca_picc_t2t_online(void)
{
    g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
}

static void nfca_picc_t2t_offline(void)
{
    g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
}

void nfca_picc_t2t_enable(uint8_t *uid)
{
    uint8_t data[5];
    uint8_t parity[5];

    g_nfca_picc_t2t_data.state = NFCA_PICC_T2T_STATE_IDLE;
    nfca_picc_register_callback(&gs_nfca_picc_t2t_cb);

    g_nfca_picc_t2t_data.uid_ct = ISO14443A_UID0_CT;
    g_nfca_picc_t2t_data.uid[0] = uid[0];
    g_nfca_picc_t2t_data.uid[1] = uid[1];
    g_nfca_picc_t2t_data.uid[2] = uid[2];
    g_nfca_picc_t2t_data.uid[3] = uid[3];
    g_nfca_picc_t2t_data.uid[4] = uid[4];
    g_nfca_picc_t2t_data.uid[5] = uid[5];
    g_nfca_picc_t2t_data.uid[6] = uid[6];

    g_nfca_picc_t2t_data.serial_num[0] = uid[0];
    g_nfca_picc_t2t_data.serial_num[1] = uid[1];
    g_nfca_picc_t2t_data.serial_num[2] = uid[2];
    g_nfca_picc_t2t_data.serial_num[3] = ISO14443A_UID0_CT ^ uid[0] ^ uid[1] ^ uid[2];
    g_nfca_picc_t2t_data.serial_num[4] = uid[3];
    g_nfca_picc_t2t_data.serial_num[5] = uid[4];
    g_nfca_picc_t2t_data.serial_num[6] = uid[5];
    g_nfca_picc_t2t_data.serial_num[7] = uid[6];
    g_nfca_picc_t2t_data.serial_num[8] = uid[3] ^ uid[4] ^ uid[5] ^ uid[6];
    g_nfca_picc_t2t_data.internal = 0x48;

    g_nfca_picc_t2t_data.lock_bytes[0] = 0;
    g_nfca_picc_t2t_data.lock_bytes[1] = 0;

    g_nfca_picc_t2t_data.cc[0] = 0xe1;
    g_nfca_picc_t2t_data.cc[1] = 0x10;
    g_nfca_picc_t2t_data.cc[2] = 0x3e;
    g_nfca_picc_t2t_data.cc[3] = 0;

    g_nfca_picc_t2t_data.pages[4].data8[0] = 0x03;
    g_nfca_picc_t2t_data.pages[4].data8[1] = 0x00;
    g_nfca_picc_t2t_data.pages[4].data8[2] = 0xfe;
    g_nfca_picc_t2t_data.pages[4].data8[3] = 0x00;

#if IS014443A_FAST_CL
    data[0] = g_nfca_picc_t2t_data.uid_cl1[0];
    data[1] = g_nfca_picc_t2t_data.uid_cl1[1];
    data[2] = g_nfca_picc_t2t_data.uid_cl1[2];
    data[3] = g_nfca_picc_t2t_data.uid_cl1[3];
    data[4] = ISO14443A_CALC_BCC(data);
    ISO14443ACalOddParityBit(data, parity, 5);
    cl1_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_cl1_pre_raw_data, data, parity, ISO14443A_CL_FRAME_SIZE, 0);

    data[0] = ISO14443A_SAK_INCOMPLETE;
    ISO14443AAppendCRCA((void *)data, 1);
    ISO14443ACalOddParityBit(data, parity, (ISO14443A_SAK_FRAME_SIZE / 8));
    sak1_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_sak1_pre_raw_data, data, parity, ISO14443A_SAK_FRAME_SIZE, 0);

    data[0] = g_nfca_picc_t2t_data.uid_cl2[0];
    data[1] = g_nfca_picc_t2t_data.uid_cl2[1];
    data[2] = g_nfca_picc_t2t_data.uid_cl2[2];
    data[3] = g_nfca_picc_t2t_data.uid_cl2[3];
    data[4] = ISO14443A_CALC_BCC(data);
    ISO14443ACalOddParityBit(data, parity, 5);
    cl2_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_cl2_pre_raw_data, data, parity, ISO14443A_CL_FRAME_SIZE, 0);

    data[0] = NFCA_PICC_T2T_SAK;
    ISO14443AAppendCRCA((void *)data, 1);
    ISO14443ACalOddParityBit(data, parity, (ISO14443A_SAK_FRAME_SIZE / 8));
    sak2_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_sak2_pre_raw_data, data, parity, ISO14443A_SAK_FRAME_SIZE, 0);
#endif
}
