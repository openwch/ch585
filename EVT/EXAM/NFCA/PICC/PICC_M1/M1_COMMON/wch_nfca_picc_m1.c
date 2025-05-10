/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_m1.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2025/01/21
 * Description        : NFC PICC M1 head file for WCH chips.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "wch_nfca_picc_m1.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 0
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

/*
 * Mifare Classic卡片命令字
*/
#define PICC_REQIDL                 0x26           
#define PICC_REQALL                 0x52           
#define PICC_ANTICOLL1              0x93           
#define PICC_ANTICOLL2              0x95           
#define PICC_ANTICOLL3              0x97           
#define PICC_AUTHENT1A              0x60           
#define PICC_AUTHENT1B              0x61           
#define PICC_READ                   0x30           
#define PICC_WRITE                  0xA0           
#define PICC_DECREMENT              0xC0           
#define PICC_INCREMENT              0xC1           
#define PICC_RESTORE                0xC2           
#define PICC_TRANSFER               0xB0           
#define PICC_HALT                   0x50           

#define ACK_NAK_FRAME_SIZE          4
#define ACK_VALUE                   0x0A
#define NAK_INVALID_ARG             0x00
#define NAK_CRC_ERROR               0x01
#define NAK_NOT_AUTHED              0x04
#define NAK_EEPROM_ERROR            0x05
#define NAK_OTHER_ERROR             0x06

#define MEM_KEY_A_OFFSET            48
#define MEM_KEY_B_OFFSET            58
#define MEM_KEY_SIZE                6
#define MEM_ACC_GPB_SIZE            4
#define MEM_SECTOR_ADDR_MASK        0xFC
#define MEM_BIGSECTOR_ADDR_MASK     0xF0
#define MEM_BYTES_PER_BLOCK         16
#define MEM_VALUE_SIZE              4

#define BYTE_SWAP(x)                (((uint8_t)(x)>>4)|((uint8_t)(x)<<4))
#define NO_ACCESS                   0x07

#define NFCA_PICC_M1_SAK            0x08

#define ACC_TRAILER_READ_KEYA       0x01
#define ACC_TRAILER_WRITE_KEYA      0x02
#define ACC_TRAILER_READ_ACC        0x04
#define ACC_TRAILER_WRITE_ACC       0x08
#define ACC_TRAILER_READ_KEYB       0x10
#define ACC_TRAILER_WRITE_KEYB      0x20

#define ACC_BLOCK_READ              0x01
#define ACC_BLOCK_WRITE             0x02
#define ACC_BLOCK_INCREMENT         0x04
#define ACC_BLOCK_DECREMENT         0x08

static void nfca_picc_m1_online(void);
static uint16_t nfca_picc_m1_data_handler(uint16_t bits_num);
static void nfca_picc_m1_offline(void);

nfca_picc_m1_data_t g_nfca_picc_m1_data;
nfca_crypto1_cipher_t g_nfca_picc_m1_crypto1_cipher;
nfca_picc_crypto1_auth_t g_nfca_picc_crypto1_auth;

#if IS014443A_FAST_CL
uint16_t cl_pre_len;
uint8_t iso14443a_cl_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_CL_FRAME_SIZE)];
uint16_t sak_pre_len;
uint8_t iso14443a_sak_pre_raw_data[NFCA_PICC_TX_PRE_OUT_LEN(ISO14443A_SAK_FRAME_SIZE)];
#endif

static const uint8_t abTrailerAccessConditions[8][2] = {
    {
        ACC_TRAILER_WRITE_KEYA | ACC_TRAILER_READ_ACC | ACC_TRAILER_WRITE_ACC | ACC_TRAILER_READ_KEYB | ACC_TRAILER_WRITE_KEYB,
        0
    },
    {
        ACC_TRAILER_READ_ACC,
        ACC_TRAILER_WRITE_KEYA | ACC_TRAILER_READ_ACC |  ACC_TRAILER_WRITE_KEYB
    },
    {
        ACC_TRAILER_READ_ACC | ACC_TRAILER_READ_KEYB,
        0
    },
    {
        ACC_TRAILER_READ_ACC,
        ACC_TRAILER_READ_ACC
    },
    {
        ACC_TRAILER_WRITE_KEYA | ACC_TRAILER_READ_ACC | ACC_TRAILER_WRITE_ACC | ACC_TRAILER_READ_KEYB | ACC_TRAILER_WRITE_KEYB,
        0
    },
    {
        ACC_TRAILER_READ_ACC,
        ACC_TRAILER_WRITE_KEYA | ACC_TRAILER_READ_ACC | ACC_TRAILER_WRITE_ACC | ACC_TRAILER_WRITE_KEYB
    },
    {
        ACC_TRAILER_READ_ACC,
        ACC_TRAILER_READ_ACC | ACC_TRAILER_WRITE_ACC
    },
    {
        ACC_TRAILER_READ_ACC,
        ACC_TRAILER_READ_ACC
    },
};

/* 回调函数会在中断里调用，所以需要注意和外部程序的线程安全代码处理*/
static nfca_picc_cb_t gs_nfca_picc_cb_m1 =
{
    .online = nfca_picc_m1_online,
    .data_handler = nfca_picc_m1_data_handler,
    .offline = nfca_picc_m1_offline,
};   /* M1卡数据处理回调 */

__attribute__((always_inline)) RV_STATIC_INLINE uint8_t get_access_condition(uint8_t block)
{
    uint8_t  invsacc0;
    uint8_t  invsacc1;
    uint8_t  acc0 = g_nfca_picc_m1_data.sectors[g_nfca_picc_m1_data.authed_sector].sector_trailer.access_bits[0];
    uint8_t  acc1 = g_nfca_picc_m1_data.sectors[g_nfca_picc_m1_data.authed_sector].sector_trailer.access_bits[1];
    uint8_t  acc2 = g_nfca_picc_m1_data.sectors[g_nfca_picc_m1_data.authed_sector].sector_trailer.access_bits[2];
    uint8_t  res = 0;

    invsacc0 = ~BYTE_SWAP(acc0);
    invsacc1 = ~BYTE_SWAP(acc1);

    if (((invsacc0 ^ acc1) & 0xf0) || ((invsacc0 ^ acc2) & 0x0f) || ((invsacc1 ^ acc2) & 0xf0))
    {
        return (NO_ACCESS);
    }

    if (block < 128)
    {
        block &= 3;
    }
    else
    {
        block &= 15;
        if (block & 15)
        {
            block = 3;
        }
        else if (block <= 4)
        {
            block = 0;
        }
        else if (block <= 9)
        {
            block = 1;
        }
        else
        {
            block = 2;
        }
    }

    acc0 = ~acc0;
    acc1 =  acc2;
    acc2 =  acc2 >> 4;

    if (block)
    {
        acc0 >>= block;
        acc1 >>= block;
        acc2 >>= block;
    }

    res = ((acc2 & 1) << 2) | ((acc1 & 1) << 1) | (acc0 & 1);

    return res;
}

__attribute__((section(".highcode")))
static uint16_t nfca_picc_m1_data_handler(uint16_t bits_num)
{
    uint16_t send_bits = 0;

    if((bits_num == 7) &&
            (((g_picc_data_buf[0] == ISO14443A_CMD_REQA)) || (g_picc_data_buf[0] == ISO14443A_CMD_WUPA)))
    {
        g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_READY;
        g_picc_data_buf[0] = 0x04;
        g_picc_data_buf[1] = 0x00;
        send_bits = 16;
        ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, 2);
        goto end;
    }

    switch(g_nfca_picc_m1_data.state)
    {
    case NFCA_PICC_M1_STATE_READY:
        if(g_picc_data_buf[0] == ISO14443A_CMD_SELECT_CL1)
        {
            if (ISO14443ASelect(g_picc_data_buf, &send_bits, (uint8_t *)g_nfca_picc_m1_data.manufacturer_data.uid, g_nfca_picc_m1_data.manufacturer_data.sak) == 0)
            {
                g_nfca_picc_m1_crypto1_cipher.is_encrypted = 0;
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_ACTIVE;
            }
#if IS014443A_FAST_CL ==  0
            if(send_bits != 0)
            {
                ISO14443ACalOddParityBit(g_picc_data_buf, g_picc_parity_buf, send_bits / 8);
            }
#else
            if(send_bits == ISO14443A_SAK_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_sak_pre_raw_data, sak_pre_len);
            }
            else if(send_bits == ISO14443A_CL_FRAME_SIZE)
            {
                nfca_picc_tx_set_raw_buf(iso14443a_cl_pre_raw_data, cl_pre_len);
            }
            else
            {
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            }
#endif
        }
        else if(g_picc_data_buf[0] == PICC_HALT)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            }
        }
        else
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
        }
        break;
    case NFCA_PICC_M1_STATE_ACTIVE:
        if(g_picc_data_buf[0] == PICC_HALT)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            }
        }
        else if((g_picc_data_buf[0] == PICC_AUTHENT1A) || (g_picc_data_buf[0] == PICC_AUTHENT1B))
        {
            if (ISO14443_CRCA(g_picc_data_buf, 4) == 0)
            {
                uint16_t sector_num;
                uint8_t *key_use;

                sector_num = ((g_picc_data_buf[1] >> 2) & 0x0f) ;
                g_nfca_picc_m1_data.authed_sector = sector_num;

                g_nfca_picc_m1_data.key_a_or_b = g_picc_data_buf[0] & 1;
                if(g_nfca_picc_m1_data.key_a_or_b)
                {
                    key_use = g_nfca_picc_m1_data.sectors[sector_num].sector_trailer.key_b;
                }
                else
                {
                    key_use = g_nfca_picc_m1_data.sectors[sector_num].sector_trailer.key_a;
                }

                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_AUTHING;

                nfca_picc_crypto1_setup(
                        (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                        (uint8_t *)key_use,
                        (uint8_t *)g_nfca_picc_m1_data.manufacturer_data.uid,
                        nfca_picc_rand(),
                        (uint8_t *)g_picc_data_buf,
                        (uint8_t *)g_picc_parity_buf,
                        (nfca_picc_crypto1_auth_t *)&g_nfca_picc_crypto1_auth
                );
                send_bits = 4 * 8;
            }
            else
            {
                g_picc_data_buf[0] = NAK_CRC_ERROR;
                send_bits = ACK_NAK_FRAME_SIZE;
            }
        }
        else if((g_picc_data_buf[0] == PICC_READ) || (g_picc_data_buf[0] == PICC_WRITE) || (g_picc_data_buf[0] == PICC_DECREMENT)
                || (g_picc_data_buf[0] == PICC_INCREMENT) || (g_picc_data_buf[0] == PICC_RESTORE) || (g_picc_data_buf[0] == PICC_TRANSFER))
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            g_picc_data_buf[0] = NAK_NOT_AUTHED;
            send_bits = ACK_NAK_FRAME_SIZE;
        }
        else
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
        }

        break;
    case NFCA_PICC_M1_STATE_AUTHING:
        {
            if(nfca_picc_crypto1_auth(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    (uint8_t *)g_picc_data_buf,
                    (uint8_t *)g_picc_parity_buf,
                    (nfca_picc_crypto1_auth_t *)&g_nfca_picc_crypto1_auth,
                    (uint8_t *)g_picc_data_buf,
                    (uint8_t *)g_picc_parity_buf
            ) == 0)
            {
                send_bits = 4 * 8;
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_AUTHED_IDLE;
                g_nfca_picc_m1_crypto1_cipher.is_encrypted = 1;
            }
            else
            {
                g_nfca_picc_m1_crypto1_cipher.is_encrypted = 0;
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            }
        }
        break;
    case NFCA_PICC_M1_STATE_AUTHED_IDLE:
        if(g_picc_data_buf[0] == PICC_HALT)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            }
            goto end;
        }
        if(bits_num != (4 * 9))
        {
            goto end;
        }
        if(nfca_crypto1_decrypt(
                (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                g_picc_data_buf,
                g_picc_data_buf,
                g_picc_parity_buf,
                32
        ))
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
            goto end;
        }
        if (ISO14443_CRCA(g_picc_data_buf, 4) != 0)
        {
            g_picc_data_buf[0] = NAK_CRC_ERROR;
            send_bits = 4;
            nfca_crypto1_encrypt(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    g_picc_data_buf,
                    g_picc_data_buf,
                    g_picc_parity_buf,
                    send_bits
            );
            goto end;
        }
        if (g_picc_data_buf[0] == PICC_READ)
        {
            uint8_t acc;
            uint8_t read_addr;

            g_nfca_picc_m1_data.authed_sector = g_nfca_picc_m1_data.authed_sector & 0xf;
            read_addr = ((g_picc_data_buf[1] & 0x3) + g_nfca_picc_m1_data.authed_sector * 4);;

            acc = abTrailerAccessConditions[get_access_condition(read_addr)][g_nfca_picc_m1_data.key_a_or_b];
            if((read_addr & 3) == 3)
            {
                g_picc_data_buf[0] = 0;
                g_picc_data_buf[1] = 0;
                g_picc_data_buf[2] = 0;
                g_picc_data_buf[3] = 0;
                g_picc_data_buf[4] = 0;
                g_picc_data_buf[5] = 0;
                g_picc_data_buf[9] = g_nfca_picc_m1_data.blocks[read_addr][9];
                if(acc & ACC_TRAILER_READ_ACC)
                {
                    g_picc_data_buf[6] = g_nfca_picc_m1_data.blocks[read_addr][6];
                    g_picc_data_buf[7] = g_nfca_picc_m1_data.blocks[read_addr][7];
                    g_picc_data_buf[8] = g_nfca_picc_m1_data.blocks[read_addr][8];
                }
                else
                {
                    g_picc_data_buf[6] = 0;
                    g_picc_data_buf[7] = 0;
                    g_picc_data_buf[8] = 0;
                }
                if(acc & ACC_TRAILER_READ_KEYB)
                {
                    g_picc_data_buf[10] = g_nfca_picc_m1_data.blocks[read_addr][10];
                    g_picc_data_buf[11] = g_nfca_picc_m1_data.blocks[read_addr][11];
                    g_picc_data_buf[12] = g_nfca_picc_m1_data.blocks[read_addr][12];
                    g_picc_data_buf[13] = g_nfca_picc_m1_data.blocks[read_addr][13];
                    g_picc_data_buf[14] = g_nfca_picc_m1_data.blocks[read_addr][14];
                    g_picc_data_buf[15] = g_nfca_picc_m1_data.blocks[read_addr][15];
                }
                else
                {
                    g_picc_data_buf[10] = 0;
                    g_picc_data_buf[11] = 0;
                    g_picc_data_buf[12] = 0;
                    g_picc_data_buf[13] = 0;
                    g_picc_data_buf[14] = 0;
                    g_picc_data_buf[15] = 0;
                }
            }
            else
            {
                __MCPY((void *)g_picc_data_buf, (void *)g_nfca_picc_m1_data.blocks[read_addr], (void *)((uint32_t)g_nfca_picc_m1_data.blocks[read_addr] + 16));
            }

            ISO14443AAppendCRCA(g_picc_data_buf, 16);
            send_bits = 18 * 8;
            nfca_crypto1_encrypt(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    g_picc_data_buf,
                    g_picc_data_buf,
                    g_picc_parity_buf,
                    send_bits
            );
        }
        else if ((g_picc_data_buf[0] == PICC_AUTHENT1A) || (g_picc_data_buf[0] == PICC_AUTHENT1B))
        {
            uint16_t sector_num;
            uint8_t *key_use;

            sector_num = ((g_picc_data_buf[1] >> 2) & 0x0f) ;
            g_nfca_picc_m1_data.authed_sector = sector_num;

            g_nfca_picc_m1_data.key_a_or_b = g_picc_data_buf[0] & 1;
            if(g_nfca_picc_m1_data.key_a_or_b)
            {
                key_use = g_nfca_picc_m1_data.sectors[sector_num].sector_trailer.key_b;
            }
            else
            {
                key_use = g_nfca_picc_m1_data.sectors[sector_num].sector_trailer.key_a;
            }

            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_AUTHING;

            nfca_picc_crypto1_setup(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    (uint8_t *)key_use,
                    (uint8_t *)g_nfca_picc_m1_data.manufacturer_data.uid,
                    nfca_picc_rand(),
                    (uint8_t *)g_picc_data_buf,
                    (uint8_t *)g_picc_parity_buf,
                    (nfca_picc_crypto1_auth_t *)&g_nfca_picc_crypto1_auth
            );

            send_bits = 4 * 8;
        }
        else if(g_picc_data_buf[0] == PICC_WRITE)
        {
            g_nfca_picc_m1_data.block_in_use = g_picc_data_buf[1] & 0x0f;
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_WRITING;
            g_picc_data_buf[0] = ACK_VALUE;
            send_bits = ACK_NAK_FRAME_SIZE;
            nfca_crypto1_encrypt(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    g_picc_data_buf,
                    g_picc_data_buf,
                    g_picc_parity_buf,
                    send_bits
            );
        }
        else if(g_picc_data_buf[0] == PICC_HALT)
        {
            if(g_picc_data_buf[1] == 0x00)
            {
                if((g_picc_data_buf[2] == 0x57)
                    && (g_picc_data_buf[3] == 0xcd))
                {
                    g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
                }
                else
                {
                    g_picc_data_buf[0] = NAK_CRC_ERROR;
                    send_bits = ACK_NAK_FRAME_SIZE;
                }
            }
            else
            {
                g_picc_data_buf[0] = NAK_NOT_AUTHED;
                send_bits = ACK_NAK_FRAME_SIZE;
            }
            if(send_bits != 0)
            {
                nfca_crypto1_encrypt(
                        (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                        g_picc_data_buf,
                        g_picc_data_buf,
                        g_picc_parity_buf,
                        send_bits
                );
            }
        }
        else
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
        }
        break;
    case NFCA_PICC_M1_STATE_WRITING:
        if(bits_num == (18 * 9))
        {
            if(nfca_crypto1_decrypt(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    g_picc_data_buf,
                    g_picc_data_buf,
                    g_picc_parity_buf,
                    (18 * 8)
            ))
            {
                g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
                goto end;
            }
            if (ISO14443_CRCA(g_picc_data_buf, 18) == 0)
            {
                /* 写入成功，这里应加上用户自己的写入回调，如果不给写入第0块，增加判断即可 */
                if(g_nfca_picc_m1_data.block_in_use != 0)
                {
                    __MCPY((void *)&g_nfca_picc_m1_data.blocks[g_nfca_picc_m1_data.block_in_use], (void *)g_picc_data_buf, (void *)(g_picc_data_buf + 16));
                }
                g_picc_data_buf[0] = ACK_VALUE;
            }
            else
            {
                g_picc_data_buf[0] = NAK_CRC_ERROR;
            }
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_AUTHED_IDLE;
            send_bits = ACK_NAK_FRAME_SIZE;
            nfca_crypto1_encrypt(
                    (nfca_crypto1_cipher_t *)&g_nfca_picc_m1_crypto1_cipher,
                    g_picc_data_buf,
                    g_picc_data_buf,
                    g_picc_parity_buf,
                    send_bits
            );
        }
        else
        {
            g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
        }
        break;
    default:
        break;
    }

end:
    return send_bits;
}

static void nfca_picc_m1_online(void)
{
    g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
}

static void nfca_picc_m1_offline(void)
{
    g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
}

static const uint8_t block_data1[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t sector_trailer[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x80, 0x69, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void nfca_picc_m1_enable(uint8_t *uid)
{
    uint8_t data[3];
    uint8_t parity[(ISO14443A_CL_UID_SIZE + ISO14443A_CL_BCC_SIZE)];

    g_nfca_picc_m1_data.state = NFCA_PICC_M1_STATE_IDLE;
    nfca_picc_register_callback(&gs_nfca_picc_cb_m1);

    g_nfca_picc_m1_data.manufacturer_data.uid[0] = uid[0];
    g_nfca_picc_m1_data.manufacturer_data.uid[1] = uid[1];
    g_nfca_picc_m1_data.manufacturer_data.uid[2] = uid[2];
    g_nfca_picc_m1_data.manufacturer_data.uid[3] = uid[3];
    g_nfca_picc_m1_data.manufacturer_data.bcc = ISO14443A_CALC_BCC(g_nfca_picc_m1_data.manufacturer_data.uid);
    g_nfca_picc_m1_data.manufacturer_data.sak = NFCA_PICC_M1_SAK;
    g_nfca_picc_m1_data.manufacturer_data.atqa[0] = 0x04;
    g_nfca_picc_m1_data.manufacturer_data.atqa[1] = 0;

    /* 默认的数据 */
    __MCPY((void *)g_nfca_picc_m1_data.blocks[1], (void *)block_data1, (void *)((uint32_t)block_data1 + 16));
    __MCPY((void *)g_nfca_picc_m1_data.blocks[2], (void *)block_data1, (void *)((uint32_t)block_data1 + 16));
    __MCPY((void *)g_nfca_picc_m1_data.blocks[3], (void *)sector_trailer, (void *)((uint32_t)sector_trailer + 16));

    for(uint8_t i = 1; i < 16; i++)
    {
        __MCPY((void *)g_nfca_picc_m1_data.blocks[4 * i], (void *)block_data1, (void *)((uint32_t)block_data1 + 16));
        __MCPY((void *)g_nfca_picc_m1_data.blocks[4 * i + 1], (void *)block_data1, (void *)((uint32_t)block_data1 + 16));
        __MCPY((void *)g_nfca_picc_m1_data.blocks[4 * i + 2], (void *)block_data1, (void *)((uint32_t)block_data1 + 16));
        __MCPY((void *)g_nfca_picc_m1_data.blocks[4 * i + 3], (void *)sector_trailer, (void *)((uint32_t)sector_trailer + 16));
    }

#if IS014443A_FAST_CL
    ISO14443ACalOddParityBit(g_nfca_picc_m1_data.manufacturer_data.uid, parity, (ISO14443A_CL_UID_SIZE + ISO14443A_CL_BCC_SIZE));
    cl_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_cl_pre_raw_data, g_nfca_picc_m1_data.manufacturer_data.uid, parity, ISO14443A_CL_FRAME_SIZE, 0);

    data[0] = g_nfca_picc_m1_data.manufacturer_data.sak;
    ISO14443AAppendCRCA((void *)data, 1);
    ISO14443ACalOddParityBit(data, parity, (ISO14443A_SAK_FRAME_SIZE / 8));
    sak_pre_len = nfca_picc_tx_prepare_raw_buf(iso14443a_sak_pre_raw_data, data, parity, ISO14443A_SAK_FRAME_SIZE, 0);
#endif
}
