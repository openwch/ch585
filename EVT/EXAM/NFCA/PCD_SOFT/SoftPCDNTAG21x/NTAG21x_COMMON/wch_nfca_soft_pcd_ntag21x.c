/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_soft_pcd_ntag21x.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/11/07
 * Description        : WCH NTAG21x 卡操作库
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "wch_nfca_soft_pcd_ntag21x.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
#define PRINTF(...) PRINT(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

/* The overtime are set according to the NTAG21x Datasheet. */
#define PCD_ANTICOLL_OVER_TIME          10
#define PCD_SELECT_OVER_TIME            10
#define PCD_AUTH_OVER_TIME              5
#define PCD_READ_OVER_TIME              5
#define PCD_WRITE_STEP1_OVER_TIME       5
#define PCD_WRITE_STEP2_OVER_TIME       10
#define PCD_VALUE_STEP1_2_OVER_TIME     5
#define PCD_VALUE_STEP3_OVER_TIME       10

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
    nfca_soft_pcd_set_wait_us(200);

    if(nfca_soft_pcd_communicate(7, NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
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

    nfca_soft_pcd_set_wait_ms(PCD_ANTICOLL_OVER_TIME);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 2);

    if (nfca_soft_pcd_communicate(16, NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

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
            else
            {
                PRINTF("g_nfca_pcd_recv_bits: %d\r\n", g_nfca_pcd_recv_bits);
            }
        }
        else
        {
            PRINTF("status: %d\r\n", status);
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
    nfca_soft_pcd_set_wait_ms(PCD_SELECT_OVER_TIME);
    for (res = 0; res < 4; res++)
    {
        g_nfca_pcd_send_buf[res + 2] = *(pSnr + res);
        g_nfca_pcd_send_buf[6] ^= *(pSnr + res);
    }
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 7);

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 9);

    if (nfca_soft_pcd_communicate(9 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (3 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 3))
                {
                    if (ISO14443_CRCA((uint8_t *)g_nfca_pcd_recv_buf, 3) == 0)
                    {
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
 * @brief   ntag21x get the version.
 *
 * @param   response - pointer to ntag_21x_get_version_res_t.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xGetVersion(ntag_21x_get_version_res_t *response)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;

    g_nfca_pcd_send_buf[0] = PICC_GET_VERSION;
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 1);

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 3);

    if (nfca_soft_pcd_communicate(3 * 8, NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (10 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 10))
                {
                    if (ISO14443_CRCA((uint8_t *)g_nfca_pcd_recv_buf, 10) == 0)
                    {
                        __MCPY(response, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf + 8);
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
 * @brief   Read data from the given addr.
 *
 * @param   addr - the addr of the block.
 * @param   pData - pointer to user buffer to save data.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xRead(uint8_t addr, uint8_t *pData)
{
    uint16_t res = PCD_COMMUNICATE_ERROR;
    nfca_pcd_controller_state_t status;

    nfca_soft_pcd_set_wait_ms(PCD_READ_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_READ;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    if (nfca_soft_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (18 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 18))
                {
                    if(ISO14443_CRCA(g_nfca_pcd_recv_buf, 18) == 0)
                    {
                        __MCPY(pData, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf + 16);
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
            }
            else
            {
                res = PCD_UNKNOWN_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   ntag21x fast read page.
 *
 * @param   start_addr - the addr of the start page.
 * @param   end_addr - the addr of the end page.
 * @param   pData - pointer to user buffer to save data.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xFastRead(uint8_t start_addr, uint8_t end_addr, uint8_t *pData)
{
    uint16_t res = PCD_COMMUNICATE_ERROR;
    nfca_pcd_controller_state_t status;
    uint16_t recv_bytes;

    nfca_soft_pcd_set_wait_ms(PCD_READ_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_FAST_READ;

    if(end_addr < start_addr)
    {
        return PCD_UNKNOWN_ERROR;
    }
    recv_bytes = (end_addr - start_addr + 1) * 4 + 2;    /* data + crc */
    if(recv_bytes > NFCA_PCD_MAX_RECV_NUM)
    {
        return PCD_UNKNOWN_ERROR;   /* 接收缓存不足 */
    }

    g_nfca_pcd_send_buf[1] = start_addr;
    g_nfca_pcd_send_buf[2] = end_addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 3);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 5);

    if (nfca_soft_pcd_communicate((5 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (recv_bytes * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, recv_bytes))
                {
                    if(ISO14443_CRCA(g_nfca_pcd_recv_buf, recv_bytes) == 0)
                    {
                        __MCPY(pData, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf + (recv_bytes - 2));
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
            }
            else
            {
                res = PCD_UNKNOWN_ERROR;
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
 * @param   pData - 4 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xWrite(uint8_t addr, uint8_t *pData)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t i;

    nfca_soft_pcd_set_wait_ms(PCD_WRITE_STEP1_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_WRITE;
    g_nfca_pcd_send_buf[1] = addr;

    g_nfca_pcd_send_buf[2] = pData[0];
    g_nfca_pcd_send_buf[3] = pData[1];
    g_nfca_pcd_send_buf[4] = pData[2];
    g_nfca_pcd_send_buf[5] = pData[3];

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 6);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 8);

    if (nfca_soft_pcd_communicate((8 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
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
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   Writes 4 pages data to the given block.
 *
 * @param   addr - the addr of the block.
 * @param   pData - 16 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xCompWrite(uint8_t addr, uint8_t *pData)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t i;

step1:
    nfca_soft_pcd_set_wait_ms(PCD_WRITE_STEP1_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_COMP_WRITE;
    g_nfca_pcd_send_buf[1] = addr;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    if (nfca_soft_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
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
                res = PCD_FRAME_ERROR;
            }
        }
    }

    goto end;

step2:
    nfca_soft_pcd_set_wait_ms(PCD_WRITE_STEP2_OVER_TIME);

    for (i = 0; i < 16; i++)
    {
        g_nfca_pcd_send_buf[i] = pData[i];
    }

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 16);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 18);

    if (nfca_soft_pcd_communicate((18 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();

        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
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
                res = PCD_FRAME_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   ntag21x read the counter.
 *
 * @param   cnt_data - cnt pointer to a counter buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xReadCNT(uint8_t *cnt_data)
{
    uint16_t res = PCD_COMMUNICATE_ERROR;
    nfca_pcd_controller_state_t status;

    nfca_soft_pcd_set_wait_ms(PCD_READ_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_READ_CNT;
    g_nfca_pcd_send_buf[1] = 0x02;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    if (nfca_soft_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (5 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 5))
                {
                    if(ISO14443_CRCA(g_nfca_pcd_recv_buf, 5) == 0)
                    {
                        cnt_data[0] = g_nfca_pcd_recv_buf[0];
                        cnt_data[1] = g_nfca_pcd_recv_buf[1];
                        cnt_data[2] = g_nfca_pcd_recv_buf[2];
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
            }
            else
            {
                res = PCD_UNKNOWN_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   ntag21x authenticate.
 *
 * @param   pwd - pointer to a pwd buffer.
 * @param   pack - pack pointer to a pack buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xPWDAuth(uint8_t *pwd, uint8_t *pack)
{
    nfca_pcd_controller_state_t status;
    uint16_t res = PCD_COMMUNICATE_ERROR;
    uint8_t tag_rsp[4];
    uint8_t tag_rsp_parity[4];

step1:
    g_nfca_pcd_send_buf[0] = PICC_PWD_AUTH;
    g_nfca_pcd_send_buf[1] = pwd[0];
    g_nfca_pcd_send_buf[2] = pwd[1];
    g_nfca_pcd_send_buf[3] = pwd[2];
    g_nfca_pcd_send_buf[4] = pwd[3];
    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 5);

    nfca_soft_pcd_set_wait_ms(PCD_AUTH_OVER_TIME);

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 7);

    if (nfca_soft_pcd_communicate((7 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (4 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 4))
                {
                    if(ISO14443_CRCA(g_nfca_pcd_recv_buf, 4) == 0)
                    {
                        if((g_nfca_pcd_recv_buf[0] == pack[0]) && (g_nfca_pcd_recv_buf[1] == pack[1]))
                        {
                            res = PCD_NO_ERROR;
                        }
                        else
                        {
                            res = PCD_AUTH_ERROR;
                        }
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
            }
            else
            {
                res = PCD_UNKNOWN_ERROR;
            }
        }
    }

end:
    return res;
}

/**
 * @brief   ntag21x read the signature.
 *
 * @param   sig_data - pointer to 32 bytes sig data buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
uint16_t PcdNTAG21xReadSIG(uint8_t *sig_data)
{
    uint16_t res = PCD_COMMUNICATE_ERROR;
    nfca_pcd_controller_state_t status;

    nfca_soft_pcd_set_wait_ms(PCD_READ_OVER_TIME);

    g_nfca_pcd_send_buf[0] = PICC_READ_SIG;
    g_nfca_pcd_send_buf[1] = 0x00;

    ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2);
    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    if (nfca_soft_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0) == 0)
    {
        status = nfca_soft_pcd_wait_communicate_end();
        if(status == NFCA_PCD_CONTROLLER_STATE_DONE)
        {
            if (g_nfca_pcd_recv_bits == (34 * 9))
            {
                if (ISO14443ACheckOddParityBit(g_nfca_pcd_recv_buf, g_nfca_pcd_parity_buf, 34))
                {
                    if(ISO14443_CRCA(g_nfca_pcd_recv_buf, 34) == 0)
                    {
                        __MCPY(sig_data, g_nfca_pcd_recv_buf, g_nfca_pcd_recv_buf + 32);
                        res = PCD_NO_ERROR;
                    }
                    else
                    {
                        res = PCD_CRC_ERROR;
                    }
                }
                else
                {
                    res = PCD_ODD_PARITY_ERROR;
                }
            }
            else if (g_nfca_pcd_recv_bits == ACK_NAK_FRAME_SIZE)
            {
                res = PICC_NAK_HEAD | g_nfca_pcd_recv_buf[0];
            }
            else
            {
                res = PCD_UNKNOWN_ERROR;
            }

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
    nfca_soft_pcd_set_wait_ms(5);

    g_nfca_pcd_send_buf[0] = PICC_HALT;
    g_nfca_pcd_send_buf[1] = 0;

    //ISO14443AAppendCRCA((uint8_t *)g_nfca_pcd_send_buf, 2); /* 固定为0x57, 0xcd */
    g_nfca_pcd_send_buf[2] = 0x57;
    g_nfca_pcd_send_buf[3] = 0xcd;

    ISO14443ACalOddParityBit((uint8_t *)g_nfca_pcd_send_buf, (uint8_t *)g_nfca_pcd_parity_buf, 4);

    nfca_soft_pcd_communicate((4 * 8), NFCA_PCD_REC_MODE_NORMAL, 0);
    nfca_soft_pcd_wait_communicate_end();
}
