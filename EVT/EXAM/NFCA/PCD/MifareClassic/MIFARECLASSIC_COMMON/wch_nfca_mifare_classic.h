/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_mifare_classic.h
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/09/23
 * Description        : WCH Mifare Classic One卡操作库
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _WCH_NFCA_MIFARE_CLASSIC_H_
#define _WCH_NFCA_MIFARE_CLASSIC_H_

#include "wch_nfca_pcd_bsp.h"
#include "ISO14443-3A.h"

/*
 * Mifare Classic卡片命令字
*/
#define PICC_REQIDL                 0x26               /* 寻天线区内未进入休眠状态 */
#define PICC_REQALL                 0x52               /* 寻天线区内全部卡 */
#define PICC_ANTICOLL1              0x93               /* 防冲撞1 */
#define PICC_ANTICOLL2              0x95               /* 防冲撞2 */
#define PICC_ANTICOLL3              0x97               /* 防冲撞3 */
#define PICC_AUTHENT1A              0x60               /* 验证A密钥 */
#define PICC_AUTHENT1B              0x61               /* 验证B密钥 */
#define PICC_READ                   0x30               /* 读块 */
#define PICC_WRITE                  0xA0               /* 写块 */
#define PICC_DECREMENT              0xC0               /* 扣款 */
#define PICC_INCREMENT              0xC1               /* 充值 */
#define PICC_RESTORE                0xC2               /* 调块数据到缓冲区 */
#define PICC_TRANSFER               0xB0               /* 保存缓冲区中数据 */
#define PICC_HALT                   0x50               /* 休眠 */

#define ACK_NAK_FRAME_SIZE          4
#define ACK_VALUE                   0x0A
#define NAK_INVALID_ARG             0x00
#define NAK_CRC_ERROR               0x01
#define NAK_NOT_AUTHED              0x04
#define NAK_EEPROM_ERROR            0x05
#define NAK_OTHER_ERROR             0x06

#define PCD_ERROR_HEAD              0x0100
#define PICC_NAK_HEAD               0x0200

enum
{
    PCD_NO_ERROR                    = 0,
    PCD_COMMUNICATE_ERROR           = 1,
    PCD_ODD_PARITY_ERROR            = 2,
    PCD_UNKNOWN_ERROR               = 3,

    PCD_OVERTIME_ERROR              = 0x0100,
    PCD_FRAME_ERROR                 = 0x0101,
    PCD_BCC_ERROR                   = 0x0102,
    PCD_CRC_ERROR                   = 0x0103,
    PCD_AUTH_ERROR                  = 0x0104,
    PCD_DECRYPT_ERROR               = 0x0105,
    PCD_VALUE_BLOCK_INVALID         = 0x0106,

    PICC_NAK_INVALID_ARG            = (PICC_NAK_HEAD | NAK_INVALID_ARG),
    PICC_NAK_CRC_ERROR              = (PICC_NAK_HEAD | NAK_CRC_ERROR),
    PICC_NAK_NOT_AUTHED             = (PICC_NAK_HEAD | NAK_NOT_AUTHED),
    PICC_NAK_EEPROM_ERROR           = (PICC_NAK_HEAD | NAK_EEPROM_ERROR),
    PICC_NAK_OTHER_ERROR            = (PICC_NAK_HEAD | NAK_OTHER_ERROR),
};

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
*/
extern uint16_t PcdRequest(uint8_t req_code);

/**
 * @brief   Find a card and select a receive gain.
 *
 * @param   req_code - 0x52/0x26.
 *          0x52 = find all 14443A-compliant cards in the sensing area.
 *          0x26 = find all 14443A-compliant cards which is not in halt mode in the sensing area.
 *
 * @return  0 = No Card.<BR>
 *          0x0004 = Mifare_One(S50).<BR>
 *          0x0002 = Mifare_One(S70).<BR>
 *          0x0044 = Mifare Ultralight.<BR>
*/
extern uint16_t PcdRequestWithCheckGain(uint8_t req_code);

/**
 * @brief   Perform an anti-collision session.
 *
 * @param   cmd - PICC_ANTICOLL1/PICC_ANTICOLL2/PICC_ANTICOLL3.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdAnticoll(uint8_t cmd);

/**
 * @brief   select a card.
 *
 * @param   cmd - PICC_ANTICOLL1/PICC_ANTICOLL2/PICC_ANTICOLL3.
 * @param   pSnr - 4 bytes card UID.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdSelect(uint8_t cmd, uint8_t *pSnr);

/**
 * @brief   decrement or increment a value on the value block.
 *
 * @param   auth_mode - 0x60 = authenticate A key, 0x61 = authenticate B key.
 * @param   addr - the addr of the value block.
 * @param   pValue - 4 bytes value(Little-Endian), for decrement or increment.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdAuthState(uint8_t auth_mode, uint8_t addr, uint8_t *pKey, uint8_t *pUid);

/**
 * @brief   Read data from the given block.
 *
 * @param   addr - the addr of the block.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdRead(unsigned char addr);

/**
 * @brief   Writes data to the given data block.
 *
 * @param   addr - the addr of the value block.
 * @param   pData - 16 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdWrite(unsigned char addr, unsigned char *pData);

/**
 * @brief   decrement or increment a value on the value block.
 *
 * @param   dd_mode - 0xC0 = decrement, 0xC1 = increment.
 * @param   addr - the addr of the value block.
 * @param   pValue - 4 bytes value(Little-Endian), for decrement or increment.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail, see PCD_ERROR_T for details.<BR>
*/
extern uint16_t PcdValue(uint8_t dd_mode, uint8_t addr, uint8_t *pValue);

/**
 * @brief   backup a value block to another block.
 *
 * @param   sourceaddr - the addr of the value block need to be backup.
 * @param   goaladdr - the addr of a value block need backup to.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail, see PCD_ERROR_T for details.<BR>
*/
extern uint16_t PcdBakValue(uint8_t sourceaddr, uint8_t goaladdr);

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
extern uint16_t PcdInitValueBlock(uint8_t addr, uint8_t *value_from, uint8_t value_addr);

/**
 * @brief   read value from a value block,
 *          g_nfc_pcd_signal_data.decode_buf.v32[0] is the value in the value block,
 *          g_nfc_pcd_signal_data.decode_buf.v8[12] is the adr in the value block.
 *
 * @param   addr - the addr of the data block, which need to be format as a value block.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdReadValueBlock(uint8_t addr);

/**
 * @brief   set card to halt mode.
 *
 * @param   None.
 *
 * @return  None.
*/
extern void PcdHalt(void);

#endif /* _NFC_READER_M1_H_ */
