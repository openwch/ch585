/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_mifare_classic.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/09/23
 * Description        : WCH Mifare Classic One卡操作库
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "wch_nfca_mifare_classic.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
#define PRINTF(...) PRINT(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

/* The overtime are set according to the MIFARE Classic (1K) Datasheet. */
#define PCD_ANTICOLL_OVER_TIME          10
#define PCD_SELECT_OVER_TIME            10
#define PCD_AUTH_OVER_TIME              5
#define PCD_READ_OVER_TIME              5
#define PCD_WRITE_STEP1_OVER_TIME       5
#define PCD_WRITE_STEP2_OVER_TIME       10
#define PCD_VALUE_STEP1_2_OVER_TIME     5
#define PCD_VALUE_STEP3_OVER_TIME       10

nfca_crypto1_cipher_t g_m1_crypto1_cipher;

/**
 * @brief   Find a card.
 *
 * @param   req_code - 0x52/0x26.
 *          0x52 = find all 14443A-compliant cards in the sensing area.
 *          0x26 = find all 14443A-compliant cards which is not in halt mode in the sensing area.
 *
 * @return  0 = No Card.<BR>
 *          0x0004 = Mifare_One(S50).<BR>
 *          0x0002 = Mifare_One(S70).<BR>
 *          0x0044 = Mifare Ultralight.<BR>
 *          others = maybe other card.<BR>
*/
uint16_t PcdRequest(uint8_t req_code)
{
    uint8_t retry_cnt = 0;
    uint16_t res;
    uint16_t atqa;
    nfca_pcd_controller_state_t status;

    g_nfca_pcd_send_buf[0] = req_code;
    nfca_pcd_set_wait_us(200);

retry:
    if(nfca_pcd_communicate(7, NFCA_PCD_REC_MODE_COLI, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if((status == NFCA_PCD_CONTROLLER_STATE_DONE) || (status == NFCA_PCD_CONTROLLER_STATE_COLLISION))
        {
            if(g_nfca_pcd_recv_bits == (2 * 9))
            {
                if(ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 2))
                {
                    PRINTF("ATQA:0x%04x\r\n", PU16_BUF(g_nfca_pcd_recv_buf)[0]);
                    return PU16_BUF(g_nfca_pcd_recv_buf)[0];
                }
                else
                {
                    PRINTF("ODD BIT ERROR\r\n");
                    PRINTF("data:0x%02x 0x%02x\r\n", PU16_BUF(g_nfca_pcd_recv_buf)[0], PU16_BUF(g_nfca_pcd_recv_buf)[1]);
                    PRINTF("parity:%d %d\r\n", g_nfca_pcd_parity_buf[0], g_nfca_pcd_parity_buf[1]);
                }
            }
            else
            {
                PRINTF("BITS NUM ERROR: %d, 0x%04x\r\n", g_nfca_pcd_recv_bits, PU16_BUF(g_nfca_pcd_recv_buf)[0]);
            }
        }
        else
        {
            PRINTF("STATUS ERROR: %d\r\n", status);
        }

    }
    else
    {
        PRINTF("COMMUNICATE ERROR\r\n");
    }

end:
    return 0;
}

/**
 * @brief   Perform an anti-collision session.
 *
 * @param   cmd - PICC_ANTICOLL1/PICC_ANTICOLL2/PICC_ANTICOLL3.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdAnticoll(uint8_t cmd)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;

    g_nfca_pcd_send_buf[0] = cmd;
    g_nfca_pcd_send_buf[1] = 0x20;

    nfca_pcd_set_wait_ms(PCD_ANTICOLL_OVER_TIME);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 2);

    if (nfca_pcd_communicate(16, NFCA_PCD_REC_MODE_NORMAL, 0) == 0) /* RB_NFCA_PCD_IE_COLI */
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (5 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 5))
                {
                    if (ISO14443A_CHECK_BCC(g_nfca_pcd_recv_buf))
                    {
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_BCC_ERROR;
                        PRINTF("check bcc error\n");
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                    PRINTF("ODD BIT ERROR\r\n");
                }
            }
        }
    }

end:
    return res;
}

/**
 * @brief   select a card.
 *
 * @param   pSnr - 4 bytes card UID.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdSelect(uint8_t cmd, uint8_t *pSnr)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;

    g_nfca_pcd_send_buf[0] = cmd;
    g_nfca_pcd_send_buf[1] = 0x70;
    g_nfca_pcd_send_buf[6] = 0;
    nfca_pcd_set_wait_ms(PCD_SELECT_OVER_TIME);
    for (res = 0; res < 4; res++)
    {
        g_nfca_pcd_send_buf[res + 2] = *(pSnr + res);
        g_nfca_pcd_send_buf[6] ^= *(pSnr + res);
    }
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 7);

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 9);

    if (nfca_pcd_communicate(9 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (3 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 3))
                {
                    if (ISO14443_CRCA((uint8_t *)g_nfca_pcd_recv_buf, 3) == 0)
                    {
                        g_m1_crypto1_cipher.is_encrypted = 0;
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                        PRINTF("check crc error\n");
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                    PRINTF("ODD BIT ERROR\r\n");
                }
            }
        }
    }

end:
    return res;
}

/**
 * @brief   decrement or increment a value on the value block.
 *
 * @param   auth_mode - 0x60 = authenticate A key, 0x61 = authenticate B key.
 * @param   addr - the addr of the block.
 * @param   pValue - 4 bytes value(Little-Endian), for decrement or increment.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pUid)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t tag_rsp[4];
    uint8_t tag_rsp_parity[4];

    PRINTF("Auth %02x, addr: %d\n", auth_mode, addr);

step1:
    g_nfca_pcd_send_buf[0] = auth_mode;
    g_nfca_pcd_send_buf[1] = addr;
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_pcd_set_wait_ms(PCD_AUTH_OVER_TIME);

    if (g_m1_crypto1_cipher.is_encrypted != 0)
    {
        nfca_crypto1_encrypt((nfca_crypto1_cipher_t *)&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4 * 8);
    }
    else
    {
        ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);
    }

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (4 * 9))
            {
                if (nfca_pcd_crypto1_setup(
                            &g_m1_crypto1_cipher,
                            pKey,
                            pUid,
                            g_nfca_pcd_recv_buf,
                            g_nfca_pcd_parity_buf,
                            nfca_pcd_rand(),
                            g_nfca_pcd_send_buf,
                            g_nfca_pcd_parity_buf,
                            tag_rsp,
                            tag_rsp_parity) == 0)
                {
                    goto step2;
                }
                else
                {
                    /* 校验错误 */
                    res = PCD_AUTH_ERROR;
                    PRINTF("Auth err\n");
                }
            }
        }
    }
    goto end;

step2:
    if (nfca_pcd_communicate((8 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (4 * 9))
            {
                if ((tag_rsp[0] == g_nfca_pcd_recv_buf[0])
                        && (tag_rsp[1] == g_nfca_pcd_recv_buf[1])
                        && (tag_rsp[2] == g_nfca_pcd_recv_buf[2])
                        && (tag_rsp[3] == g_nfca_pcd_recv_buf[3])
                        && (tag_rsp_parity[0] == g_nfca_pcd_parity_buf[0])
                        && (tag_rsp_parity[1] == g_nfca_pcd_parity_buf[1])
                        && (tag_rsp_parity[2] == g_nfca_pcd_parity_buf[2])
                        && (tag_rsp_parity[3] == g_nfca_pcd_parity_buf[3]))
                {
                    g_m1_crypto1_cipher.is_encrypted = 1;
                    res = PCD_NO_ERROR;
                }
                else
                {
                    /* 校验错误 */
                    res = PCD_AUTH_ERROR;
                    PRINTF("Auth err\n");
                }
            }
        }
    }
end:
    return res;
}

/**
 * @brief   Read data from the given block.
 *
 * @param   addr - the addr of the block.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdRead(uint8_t addr)
{
    uint16_t res = PCD_COMMUNICATE_ERROR;
    nfca_pcd_controller_state_t status;

    nfca_pcd_set_wait_ms(PCD_READ_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_READ;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (18 * 9))
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 18 * 8) == 0)
                {
                    res = PCD_NO_ERROR;
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   Writes data to the given block.
 *
 * @param   addr - the addr of the block.
 * @param   pData - 16 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdWrite(uint8_t addr, uint8_t *pData)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t i;

step1:
    nfca_pcd_set_wait_ms(PCD_WRITE_STEP1_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_WRITE;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);
    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        goto step2;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

    goto end;

step2:
    nfca_pcd_set_wait_ms(PCD_WRITE_STEP2_OVER_TIME);

    for (i = 0; i < 16; i++)
    {
        g_nfca_pcd_send_buf[i] = pData[i];
    }

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 16);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (18 * 8));

    if (nfca_pcd_communicate((18 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   decrement or increment a value on the value block.
 *
 * @param   dd_mode - 0xC0 = decrement, 0xC1 = increment.
 * @param   addr - the addr of the value block.
 * @param   pValue - 4 bytes value(Little-Endian), for decrement or increment.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdValue(uint8_t dd_mode, uint8_t addr, uint8_t *pValue)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t i;

step1:
    nfca_pcd_set_wait_ms(PCD_VALUE_STEP1_2_OVER_TIME);
    g_nfca_pcd_send_buf[0] = dd_mode;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        PRINT("step1 status: %x\n", status);
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        goto step2;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

    goto end;

step2:
    for (i = 0; i < 4; i++)
    {
        g_nfca_pcd_send_buf[i] = pValue[i];
    }
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 4);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (6 * 8));

    nfca_pcd_communicate((6 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);     /* 本次需要等待超时结束 */

    nfca_pcd_wait_communicate_end();

step3:
    nfca_pcd_set_wait_ms(PCD_VALUE_STEP3_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_TRANSFER;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();
        PRINT("step3 status: %x\n", status);
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   backup a value block to another block.
 *
 * @param   sourceaddr - the addr of the value block need to be backup.
 * @param   goaladdr - the addr of a value block need backup to.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdBakValue(uint8_t sourceaddr, uint8_t goaladdr)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;

step1:
    nfca_pcd_set_wait_ms(PCD_VALUE_STEP1_2_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_RESTORE;
    g_nfca_pcd_send_buf[1] = sourceaddr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        goto step2;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

    goto end;

step2:
    PU32_BUF(g_nfca_pcd_send_buf)[0] = 0;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 4);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (6 * 8));

    nfca_pcd_communicate((6 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);     /* 本次需要等待超时结束 */
    nfca_pcd_wait_communicate_end();

step3:
    nfca_pcd_set_wait_ms(PCD_VALUE_STEP3_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_TRANSFER;
    g_nfca_pcd_send_buf[1] = goaladdr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    nfca_crypto1_encrypt(&g_m1_crypto1_cipher, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, (4 * 8));

    if (nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                if (nfca_crypto1_decrypt(&g_m1_crypto1_cipher, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, ACK_NAK_FRAME_SIZE) == 0)
                {
                    if (g_nfca_pcd_recv_buf[0] == ACK_VALUE)
                    {
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
                    }
                }
                else
                {
                    res = PCD_DECRYPT_ERROR;
                }
            }
            else
            {
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   format a block as a value block which can use decrement or increment cmd.
 *
 * @param   addr - the addr of the data block, which need to be format as a value block.
 * @param   value_from - the pointer to a 4 byte memory (Little-Endian) which need to be saved.
 * @param   adr - the adr of the value block.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdInitValueBlock(uint8_t addr, uint8_t *value_from, uint8_t adr)
{
    uint16_t res;
    uint8_t init_buffer[16];

    for (res = 0; res < 4; res++)
    {
        init_buffer[res] = value_from[res];
        init_buffer[res + 8] = value_from[res];
        init_buffer[res + 4] = ~value_from[res];
    }

    init_buffer[12] = adr;
    init_buffer[13] = ~adr;
    init_buffer[14] = adr;
    init_buffer[15] = ~adr;

    res = PcdWrite(addr, init_buffer);

end:
    return res;
}

/**
 * @brief   read value from a value block,
 *          g_nfca_pcd_recv_buf_u32[0] is the value in the value block,
 *          g_nfca_pcd_recv_buf[12] is the adr in the value block.
 *
 * @param   addr - the addr of the data block, which need to be format as a value block.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdReadValueBlock(uint8_t addr)
{
    uint16_t res;

    res = PcdRead(addr);

    if (res == PCD_NO_ERROR)
    {
        if ((PU32_BUF(g_nfca_pcd_recv_buf)[0] != PU32_BUF(g_nfca_pcd_recv_buf)[2])
                || ((PU32_BUF(g_nfca_pcd_recv_buf)[0] != (~PU32_BUF(g_nfca_pcd_recv_buf)[1])))
                || (g_nfca_pcd_recv_buf[12] != g_nfca_pcd_recv_buf[14])
                || (g_nfca_pcd_recv_buf[13] != g_nfca_pcd_recv_buf[15])
                || ((g_nfca_pcd_recv_buf[12] != (uint8_t)(~g_nfca_pcd_recv_buf[13])))
           )
        {
            res = PCD_VALUE_BLOCK_INVALID;
        }
    }

end:
    return res;
}

/**
 * @brief   set card to halt mode.
 *
 * @param   None.
 *
 * @return  None.
*/
void PcdHalt(void)
{
    nfca_pcd_set_wait_ms(5);

    g_nfca_pcd_send_buf[0] = PICC_HALT;
    g_nfca_pcd_send_buf[1] = 0;

    //ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2); /* 固定为0x57, 0xcd */
    g_nfca_pcd_send_buf[2] = 0x57;
    g_nfca_pcd_send_buf[3] = 0xcd;

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    nfca_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);
    nfca_pcd_wait_communicate_end();
}
