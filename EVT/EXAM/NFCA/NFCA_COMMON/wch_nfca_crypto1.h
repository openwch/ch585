/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_crypto1.h
 * Author             : WCH
 * Version            : V1.3
 * Date               : 2024/11/12
 * Description        : nfc crypto1算法相关函数库 - wch_nfca_crypto1
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef _WCH_NFCA_CRYPTO1_H_
#define _WCH_NFCA_CRYPTO1_H_

#include "stdint.h"

typedef struct _nfca_crypto1_cipher_struct
{
    uint64_t    lfsr    : 48;
    uint8_t     is_encrypted;
} nfca_crypto1_cipher_t;

typedef struct _nfca_picc_crypto1_auth_struct
{
    uint8_t reader_rsp[4];
    uint8_t tag_rsp[4];
} nfca_picc_crypto1_auth_t;

/*
 * look-up table to calculate odd parity bit with 1 byte.
 *
 * OddParityBit(n) = byteParityBitsTable[n];
 * EvenParityBit(n) = 1 - byteParityBitsTable[n];
const uint8_t byteParityBitsTable[256] =
{
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};
 */
extern const uint8_t byteParityBitsTable[256];

/**
 * @brief setup pcd crypto1 cipher.
 *
 * @param[in]   crypto1_cipher      nfca_crypto1_cipher_t for setup.
 * @param[in]   key                 the key of the block which is need authentication, the data length is 6 bytes.
 * @param[in]   tag_uid             uid of tag, the data length is 4 bytes.
 * @param[in]   tag_clg             challenge of tag, the data length is 4 bytes.
 * @param[in]   tag_clg_parity      parity bits of challenge of tag, the data length is 4 bytes.
 * @param[in]   reader_clg_rand     a random number for generate reader_clg.
 * @param[out]  reader_clg          challenge of reader, the data length is 8 bytes.
 * @param[out]  reader_clg_parity   parity bits of challenge of reader, the data length is 8 bytes.
 * @param[out]  tag_rsp             response of tag, the data length is 4 bytes.
 * @param[out]  tag_rsp_parity      parity bits of response of tag, the data length is 4 bytes.
 *
 * @return      0 - success
 *              otherwise - failed.
 */
extern uint8_t nfca_pcd_crypto1_setup(nfca_crypto1_cipher_t *crypto1_cipher, uint8_t *key, uint8_t *tag_uid, uint8_t *tag_clg, uint8_t *tag_clg_parity,
                                        uint32_t reader_clg_rand, uint8_t *reader_clg, uint8_t *reader_clg_parity, uint8_t *tag_rsp, uint8_t *tag_rsp_parity);

/**
 * setup picc crypto1 cipher.
 *
 * @param[in]   crypto1_cipher      nfca_crypto1_cipher_t for setup.
 * @param[in]   key                 the key of the block which is need authentication, the data length is 6 bytes.
 * @param[in]   tag_uid             uid of tag, the data length is 4 bytes.
 * @param[in]   tag_clg_rand        a random number for generate tag_clg.
 * @param[out]  tag_clg             challenge of tag, the data length is 4 bytes.
 * @param[out]  tag_clg_parity      parity bits of challenge of tag, the data length is 4 bytes.
 * @param[out]  picc_crypto1_auth   auth data wait for response from pcd.
 *
 * @return      None.
 */
extern void nfca_picc_crypto1_setup(nfca_crypto1_cipher_t *crypto1_cipher, uint8_t *key, uint8_t *tag_uid, uint32_t tag_clg_rand,
                                        uint8_t *tag_clg, uint8_t *tag_clg_parity, nfca_picc_crypto1_auth_t *picc_crypto1_auth);

/**
 * @brief setup pcd crypto1 cipher.
 *
 * @param[in]   crypto1_cipher      nfca_crypto1_cipher_t for auth.
 * @param[in]   reader_clg          challenge of reader, the data length is 8 bytes.
 * @param[in]   reader_clg_parity   parity bits of challenge of reader, the data length is 8 bytes.
 * @param[out]  reader_rsp_in       response of reader generate by nfca_picc_crypto1_setup, the data length is 4 bytes.
 * @param[out]  tag_rsp_in          response of tag generate by nfca_picc_crypto1_setup, the data length is 4 bytes.
 * @param[in]   tag_rsp_out         response of tag need to send, the data length is 4 bytes.
 * @param[in]   tag_rsp_parity      parity bits of response of tag, the data length is 4 bytes.
 *
 * @return      0 - success
 *              otherwise - failed.
 */
extern uint8_t nfca_picc_crypto1_auth(nfca_crypto1_cipher_t *crypto1_cipher, uint8_t *reader_clg, uint8_t *reader_clg_parity,
                                        nfca_picc_crypto1_auth_t *picc_crypto1_auth, uint8_t *tag_rsp_out, uint8_t *tag_rsp_parity);

/**
 * @brief Encrypt data.
 *
 * @param[in]   crypto1_cipher  nfca_crypto1_cipher_t for encrypt.
 * @param[in]   in              plaintext.
 * @param[out]  out             chiphertext.
 * @param[out]  out_parity      oddparity of data.
 * @param[in]   len             bit length of data.
 */
extern void nfca_crypto1_encrypt(nfca_crypto1_cipher_t *crypto1_cipher, uint8_t *in, uint8_t *out, uint8_t *out_parity, uint8_t len);

/**
 * @brief Decrypt data.
 *
 * @param[in]   crypto1_cipher  nfca_crypto1_cipher_t for decrypt.
 * @param[in]   in              chiphertext.
 * @param[out]  out             plaintext.
 * @param[in]   in_parity       oddparity need to confirm.
 * @param[in]   len             bit length of data.
 *
 * @return      0 - success
 *              otherwise - failed.
 */
extern uint8_t nfca_crypto1_decrypt(nfca_crypto1_cipher_t *crypto1_cipher, uint8_t *in, uint8_t *out, uint8_t *in_parity, uint8_t len);

#endif /* _WCH_NFCA_CRYPTO1_H_ */
