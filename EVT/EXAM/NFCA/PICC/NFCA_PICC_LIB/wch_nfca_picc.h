/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_pcd.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/09/20
 * Description        : nfc picc库头文件
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _WCH_NFCA_PICC_H_
#define _WCH_NFCA_PICC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "wch_nfca_picc_bsp.h"

typedef __attribute__((aligned(4))) struct _nfca_picc_config_struct
{
    uint32_t    *signal_buf;        /* 用于NFC PICC发送和接收原始波形数据的缓冲区 */
    uint8_t     *data_buf;          /* 用于NFC PICC发送和接收数据的缓冲区 */
    uint8_t     *parity_buf;        /* 用于NFC PICC发送和接收数据校验位的缓冲区 */
    uint16_t    signal_buf_len;     /* 用于NFC PICC发送和接收原始波形数据的缓冲区长度 */
    uint16_t    data_buf_len;       /* data_buf和parity_buf长度必须一致 */
} nfca_picc_config_t;

typedef __attribute__((aligned(4))) struct _nfca_picc_callback_struct
{
    void (*online)(void);
    uint16_t (*data_handler)(uint16_t bits_num);
    void (*offline)(void);
} nfca_picc_cb_t;

extern void nfca_picc_lib_init(nfca_picc_config_t *cfg);

extern void nfca_picc_lib_start(void);

extern void nfca_picc_lib_stop(void);

extern void nfca_picc_register_callback(nfca_picc_cb_t *cb);

extern uint16_t nfca_picc_tx_prepare_raw_buf(uint8_t *out, uint8_t *in, uint8_t *parity, uint16_t length, uint8_t offset);

#define NFCA_PICC_TX_PRE_OUT_LEN(LEN)               ((LEN + 7) / 8 * 18)

extern void nfca_picc_tx_set_raw_buf(uint8_t *data, uint16_t length);

extern void nfca_picc_rx_irq_handler(void);

extern void nfca_picc_tx_irq_handler(void);

#define TMR0_NFCA_PICC_CNT_END                      288
#define TMR3_NFCA_PICC_CNT_END                      18

#ifdef __cplusplus
}
#endif

#endif  /* _WCH_NFCA_PICC_H_ */
