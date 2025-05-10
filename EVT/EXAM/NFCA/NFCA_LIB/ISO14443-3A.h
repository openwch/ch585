/********************************** (C) COPYRIGHT *******************************
 * File Name          : ISO14443-3A.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2025/01/21
 * Description        : ISO14443-3A相关函数头文件
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef _ISO14443_3A_H_
#define _ISO14443_3A_H_

#include "CH58x_NFCA_LIB.h"

/* NFC PICC ISO14443A快速防冲突，有些情况下，计算速度可能来不及给读卡器回复，使能IS014443A_FAST_CL可以通过提前计算数据的api来加速处理，也可以自行使用函数处理 */
#ifndef IS014443A_FAST_CL                           /* 公用文件，请在工程配置中添加该宏定义进行修改 */
#define IS014443A_FAST_CL                           0
#endif

#define ISO14443A_CMD_REQA                          0x26
#define ISO14443A_CMD_WUPA                          0x52
#define ISO14443A_CMD_SELECT_CL1                    0x93
#define ISO14443A_CMD_SELECT_CL2                    0x95
#define ISO14443A_CMD_SELECT_CL3                    0x97
#define ISO14443A_CMD_HLTA                          0x50

#define ISO14443A_NVB_AC_START                      0x20
#define ISO14443A_NVB_AC_END                        0x70
#define ISO14443A_UID0_RANDOM                       0x08
#define ISO14443A_UID0_CT                           0x88
#define ISO14443A_CL_UID_OFFSET                     0
#define ISO14443A_CL_UID_SIZE                       4
#define ISO14443A_CL_BCC_OFFSET                     4
#define ISO14443A_CL_BCC_SIZE                       1
#define ISO14443A_CL_FRAME_SIZE                     ((ISO14443A_CL_UID_SIZE + ISO14443A_CL_BCC_SIZE) * 8)
#define ISO14443A_SAK_INCOMPLETE                    0x24
#define ISO14443A_SAK_INCOMPLETE_NOT_COMPLIANT      0x04
#define ISO14443A_SAK_COMPLETE_COMPLIANT            0x20
#define ISO14443A_SAK_COMPLETE_NOT_COMPLIANT        0x00

#define ISO14443A_ATQA_FRAME_SIZE_BYTES             (2)
#define ISO14443A_ATQA_FRAME_SIZE                   (2 * 8)
#define ISO14443A_SAK_FRAME_SIZE                    (3 * 8)
#define ISO14443A_HLTA_FRAME_SIZE                   (2 * 8)

/* 计算BCC校验码 */
#define ISO14443A_CALC_BCC(ByteBuffer)              (ByteBuffer[0] ^ ByteBuffer[1] ^ ByteBuffer[2] ^ ByteBuffer[3])

/* 检查BCC校验码 */
#define ISO14443A_CHECK_BCC(B)                      ((B[0] ^ B[1] ^ B[2] ^ B[3]) == B[4])

/*********************************************************************
 * @fn      ISO14443_CRCA
 *
 * @brief   check CRC.
 *
 * @param   Buffer - the data need to check
 *          ByteCount -  the length of data
 *
 * @return  0 if no error.
 */
extern uint16_t ISO14443_CRCA(uint8_t *buf, uint8_t len);

/*********************************************************************
 * @fn      ISO14443AAppendCRCA
 *
 * @brief   add CRC value behind the data.
 *
 * @param   buf - the data need to check
 *          len -  the length of data
 *
 * @return  crc value.
 */
extern uint16_t ISO14443AAppendCRCA(void *buf, uint16_t len);

/*********************************************************************
 * @fn      ISO14443ACheckOddParityBit
 *
 * @brief   check odd parity bit.
 *
 * @param   data - the data need to check
 *          parity - the parity need to check
 *          len -  the length of data
 *
 * @return  1 if the parity is correct, 0 is wrong.
 */
extern uint8_t ISO14443ACheckOddParityBit(uint8_t *data, uint8_t *parity, uint16_t len);

/*********************************************************************
 * @fn      ISO14443ACalOddParityBit
 *
 * @brief   check odd parity bit.
 *
 * @param   data - the data need to check
 *          parity - the parity need to check
 *          len -  the length of data
 *
 * @return  1 if the parity is correct, 0 is wrong.
 */
extern void ISO14443ACalOddParityBit(uint8_t *data, uint8_t *out_parity, uint16_t len);

/*********************************************************************
 * @fn      ISO14443ASelect
 *
 * @brief   卡片选择流程处理
 *
 * @param   data        - 数据指针
 * @param   bit_count   - 数据比特数量指针
 * @param   uid         - 自身的卡片uid数据指针
 * @param   sak         - 自身的卡片sak值
 *
 * @return  1 - 还在卡片选择流程中，0 - 卡片选择流程结束.
 */
extern uint8_t ISO14443ASelect(uint8_t *data, uint16_t *bit_count, uint8_t *uid, uint8_t sak);

#endif  /* _ISO14443_3A_H_ */
