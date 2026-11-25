/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_ntag21x.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/03/02
 * Description        : WCH NTAG21x 卡操作库
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _WCH_NFCA_NTAG21X_H_
#define _WCH_NFCA_NTAG21X_H_

#include "wch_nfca_pcd_bsp.h"
#include "ISO14443-3A.h"

/*
 * NTAG21x卡片命令字
*/
#define PICC_REQIDL                 0x26               /* 寻天线区内未进入休眠状态 */
#define PICC_REQALL                 0x52               /* 寻天线区内全部卡 */
#define PICC_ANTICOLL1              0x93               /* 防冲撞1 */
#define PICC_ANTICOLL2              0x95               /* 防冲撞2 */
#define PICC_ANTICOLL3              0x97               /* 防冲撞3 */

#define PICC_GET_VERSION            0x60
#define PICC_READ                   0x30
#define PICC_FAST_READ              0x3A
#define PICC_WRITE                  0xA2
#define PICC_COMP_WRITE             0xA0
#define PICC_READ_CNT               0x39
#define PICC_PWD_AUTH               0x1B
#define PICC_READ_SIG               0x3C

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

    PICC_NAK_INVALID_ARG            = (PICC_NAK_HEAD | NAK_INVALID_ARG),
    PICC_NAK_CRC_ERROR              = (PICC_NAK_HEAD | NAK_CRC_ERROR),
    PICC_NAK_NOT_AUTHED             = (PICC_NAK_HEAD | NAK_NOT_AUTHED),
    PICC_NAK_EEPROM_ERROR           = (PICC_NAK_HEAD | NAK_EEPROM_ERROR),
    PICC_NAK_OTHER_ERROR            = (PICC_NAK_HEAD | NAK_OTHER_ERROR),
};

typedef __attribute__((aligned(1))) struct _ntag_21x_get_version_response_struct
{
    uint8_t fixed_header;
    uint8_t vendor_id;
    uint8_t product_type;
    uint8_t product_subtype;
    uint8_t major_product_version;
    uint8_t minor_product_version;
    uint8_t storage_size;
    uint8_t protocol_type;
} ntag_21x_get_version_res_t;

#define NTAG213_STORAGE_SIZE                0x0f
#define NTAG215_STORAGE_SIZE                0x11
#define NTAG216_STORAGE_SIZE                0x13

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
 * @brief   select a card.
 *
 * @param   response - pointer to ntag_21x_get_version_res_t.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xGetVersion(ntag_21x_get_version_res_t *response);

/**
 * @brief   Read data from the given addr.
 *
 * @param   addr - the addr of the block.
 * @param   pData - pointer to user buffer to save data.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xRead(uint8_t addr, uint8_t *pData);

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
extern uint16_t PcdNTAG21xFastRead(uint8_t start_addr, uint8_t end_addr, uint8_t *pData);

/**
 * @brief   Writes data to the given block.
 *
 * @param   addr - the addr of the block.
 * @param   pData - 4 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xWrite(uint8_t addr, uint8_t *pData);

/**
 * @brief   Writes 4 pages data to the given block.
 *
 * @param   addr - the addr of the block.
 * @param   pData - 16 bytes data need to write.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xCompWrite(uint8_t addr, uint8_t *pData);

/**
 * @brief   ntag21x read the counter.
 *
 * @param   cnt_data - cnt pointer to a counter buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xReadCNT(uint8_t *cnt_data);

/**
 * @brief   ntag21x authenticate.
 *
 * @param   pwd - pointer to a pwd buffer.
 * @param   pack - pack pointer to a pack buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xPWDAuth(uint8_t *pwd, uint8_t *pack);

/**
 * @brief   ntag21x read the signature.
 *
 * @param   sig_data - pointer to 32 bytes sig data buffer.
 *
 * @return  PCD_NO_ERROR = success.<BR>
 *          others = fail.<BR>
*/
extern uint16_t PcdNTAG21xReadSIG(uint8_t *sig_data);

/**
 * @brief   set card to halt mode.
 *
 * @param   None.
 *
 * @return  None.
*/
extern void PcdHalt(void);

#endif /* _WCH_NFCA_NTAG21X_H_ */
