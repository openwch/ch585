/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH58x_NFCA_LIB.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2025/01/21
 * Description        : CH585/4 NFC-A头文件
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _CH58x_NFCA_LIB_H_
#define _CH58x_NFCA_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH58x_common.h"

/********************************** CH58x NFC-A PCD ******************************************/
typedef enum
{
    NFCA_PCD_CONTROLLER_STATE_FREE = 0,
    NFCA_PCD_CONTROLLER_STATE_SENDING,
    NFCA_PCD_CONTROLLER_STATE_RECEIVING,
    NFCA_PCD_CONTROLLER_STATE_COLLISION,
    NFCA_PCD_CONTROLLER_STATE_OVERTIME,
    NFCA_PCD_CONTROLLER_STATE_DONE,
    NFCA_PCD_CONTROLLER_STATE_ERR,
} nfca_pcd_controller_state_t;

typedef enum
{
    NFCA_PCD_DRV_CTRL_LEVEL0        = (0x00 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL1        = (0x01 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL2        = (0x02 << 13),
    NFCA_PCD_DRV_CTRL_LEVEL3        = (0x03 << 13),
} NFCA_PCD_DRV_CTRL_Def;

typedef enum
{
    NFCA_PCD_LP_CTRL_0_5_VDD        = (0x00 << 11),
    NFCA_PCD_LP_CTRL_0_6_VDD        = (0x01 << 11),
    NFCA_PCD_LP_CTRL_0_7_VDD        = (0x02 << 11),
    NFCA_PCD_LP_CTRL_0_8_VDD        = (0x03 << 11),
} NFCA_PCD_LP_CTRL_Def;

typedef enum
{
    NFCA_PCD_REC_GAIN_12DB          = (0x00 << 4),
    NFCA_PCD_REC_GAIN_18DB          = (0x01 << 4),
    NFCA_PCD_REC_GAIN_24DB          = (0x02 << 4),
    NFCA_PCD_REC_GAIN_30DB          = (0x03 << 4),
} NFCA_PCD_REC_GAIN_Def;

typedef enum
{
    NFCA_PCD_REC_THRESHOLD_100MV    = (0x00),
    NFCA_PCD_REC_THRESHOLD_150MV    = (0x01),
    NFCA_PCD_REC_THRESHOLD_200MV    = (0x02),
    NFCA_PCD_REC_THRESHOLD_250MV    = (0x03),
} NFCA_PCD_REC_THRESHOLD_Def;

typedef enum
{
    NFCA_PCD_REC_MODE_NONE          = 0,
    NFCA_PCD_REC_MODE_NORMAL        = 1,            /* 接收时不进行冲突检测，尽可能的进行解码 */
    NFCA_PCD_REC_MODE_COLI          = 0x10,         /* 接收时进行冲突检测，可在防冲突流程中启用 */
} NFCA_PCD_REC_MODE_Def;

typedef void (*nfca_pcd_end_cb_t)(void);

typedef struct _nfca_pcd_config_struct
{
    nfca_pcd_end_cb_t pcd_end_cb;
    uint16_t *data_buf;
    uint8_t *send_buf;
    uint8_t *recv_buf;
    uint8_t *parity_buf;

    uint16_t data_buf_size;
    uint16_t send_buf_size;
    uint16_t recv_buf_size;
    uint16_t parity_buf_size;
} nfca_pcd_config_t;

/*******************************************************************************
 * @fn              nfca_pcd_lib_init
 *
 * @brief           nfc-a pcd 初始化
 *
 * @param           cfg - 配置参数指针
 *
 * @return          None.
 */
extern void nfca_pcd_lib_init(nfca_pcd_config_t *cfg);

/*******************************************************************************
 * @fn              nfca_pcd_lib_start
 *
 * @brief           nfc-a pcd开始运行，开始在天线上发送连续波
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_pcd_lib_start(void);

/*******************************************************************************
 * @fn              nfca_pcd_lib_stop
 *
 * @brief           nfc-a pcd停止运行，停止在天线上发送连续波
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_pcd_lib_stop(void);

/*******************************************************************************
 * @fn              nfca_pcd_antenna_on
 *
 * @brief           开始在天线上发送连续波
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_pcd_antenna_on(void);

/*******************************************************************************
 * @fn              nfca_pcd_antenna_off
 *
 * @brief           停止在天线上发送连续波
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_pcd_antenna_off(void);

/*******************************************************************************
 * @fn              nfca_pcd_communicate
 *
 * @brief           nfc-a开始通讯，传输数据
 *
 * @param           data_bits_num - uint16_t，需要发送的数据区bit数量
 * @param           mode - NFCA_PCD_REC_MODE_Def，发送结束后的接收模式
 * @param           offset - uint8_t(0 - 7)，需要发送的第一个位在首字节中的偏移数量
 *
 * @return          0 if success, others failed.
 */
extern uint8_t nfca_pcd_communicate(uint16_t data_bits_num, NFCA_PCD_REC_MODE_Def mode, uint8_t offset);

/*******************************************************************************
 * @fn              nfca_pcd_get_communicate_status
 *
 * @brief           nfc-a获取当前通讯状态
 *
 * @param           None.
 *
 * @return          nfca_pcd_controller_state_t，获取当前通讯状态.
 */
extern nfca_pcd_controller_state_t nfca_pcd_get_communicate_status(void);

/*******************************************************************************
 * @fn              nfca_pcd_get_recv_data_len
 *
 * @brief           获取本次解码出的数据长度
 *
 * @param           None
 *
 * @return          uint16_t - 数据长度.
 */
extern uint16_t nfca_pcd_get_recv_data_len(void);

/*******************************************************************************
 * @fn              nfca_pcd_get_recv_bits
 *
 * @brief           获取本次接收到的bit数量
 *
 * @param           None
 *
 * @return          uint16_t - 接收到的bit数量.
 */
extern uint16_t nfca_pcd_get_recv_bits(void);

/*******************************************************************************
 * @fn              nfca_pcd_set_out_drv
 *
 * @brief           nfc-a 设置天线发射引脚输出档位，默认Level1
 *
 * @param           drv - NFCA_PCD_DRV_CTRL_Def，天线发射引脚输出档位
 *
 * @return          None.
 */
extern void nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_Def drv);

/*******************************************************************************
 * @fn              nfca_pcd_set_recv_gain
 *
 * @brief           nfc-a 设置接收增益，默认18DB
 *
 * @param           gain - NFCA_PCD_REC_GAIN_Def，接收增益
 *
 * @return          None.
 */
extern void nfca_pcd_set_recv_gain(NFCA_PCD_REC_GAIN_Def gain);

/*******************************************************************************
 * @fn              nfca_pcd_set_lp_ctrl
 *
 * @brief           nfc-a 设置天线信号检测档位，默认0.8VDD
 *
 * @param           lp - NFCA_PCD_LP_CTRL_Def，天线信号检测档位
 *
 * @return          None.
 */
extern void nfca_pcd_set_lp_ctrl(NFCA_PCD_LP_CTRL_Def lp);

/*******************************************************************************
 * @fn              nfca_pcd_set_rec_threshold
 *
 * @brief           nfc-a 设置比较门限，默认150mv
 *
 * @param           th - NFCA_PCD_REC_THRESHOLD_Def，解码模拟信号比较门限
 *
 * @return          None.
 */
extern void nfca_pcd_set_rec_threshold(NFCA_PCD_REC_THRESHOLD_Def th);

/*******************************************************************************
 * @fn              nfca_pcd_backup_anactrl
 *
 * @brief           NFC备份模拟配置
 *
 * @param           None.
 *
 * @return          None.
 */
void nfca_pcd_backup_anactrl(void);

/*******************************************************************************
 * @fn              nfca_pcd_restore_anactrl
 *
 * @brief           NFC恢复备份模拟配置
 *
 * @param           None.
 *
 * @return          None.
 */
void nfca_pcd_restore_anactrl(void);

/*******************************************************************************
 * @fn              nfca_pcd_set_wait_ms
 *
 * @brief           NFC设置接收超时时间
 *
 * @param           us - uint16_t，超时时间，单位ms，最大38ms。
 *
 * @return          None.
 */
extern void nfca_pcd_set_wait_ms(uint8_t ms);

/*******************************************************************************
 * @fn              nfca_pcd_set_wait_us
 *
 * @brief           NFC设置接收超时时间
 *
 * @param           us - uint16_t，超时时间，单位us，最大38000us。
 *
 * @return          None.
 */
extern void nfca_pcd_set_wait_us(uint16_t us);

/*******************************************************************************
 * @fn              nfca_pcd_get_lp_status
 *
 * @brief           NFC读取天线信号是否低于设置的检测值
 *
 * @param           None.
 *
 * @return          1低于设定阈值，0不低于设定阈值.
 */
extern uint8_t nfca_pcd_get_lp_status(void);

/*******************************************************************************
 * @fn              NFC_IRQLibHandler
 *
 * @brief           NFC中断处理函数
 *
 * @param           None
 *
 * @return          None.
 */
extern void NFC_IRQLibHandler(void);

/********************************** CH58x NFC-A PICC ******************************************/
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

#define TMR0_NFCA_PICC_CNT_END                      288
#define TMR3_NFCA_PICC_CNT_END                      18
#define NFCA_PICC_TX_PRE_OUT_LEN(LEN)               ((LEN + 7) / 8 * 18)

/*******************************************************************************
 * @fn              nfca_picc_lib_init
 *
 * @brief           nfc-a picc 初始化
 *
 * @param           cfg - 配置参数指针
 *
 * @return          None.
 */
extern void nfca_picc_lib_init(nfca_picc_config_t *cfg);

/*******************************************************************************
 * @fn              nfca_picc_lib_start
 *
 * @brief           nfc-a picc 开始运行
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_picc_lib_start(void);

/*******************************************************************************
 * @fn              nfca_picc_lib_stop
 *
 * @brief           nfc-a picc 停止运行
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_picc_lib_stop(void);

/*******************************************************************************
 * @fn              nfca_picc_register_callback
 *
 * @brief           nfc-a picc 注册数据处理回调
 *
 * @param           cb - nfca_picc_cb_t回调函数结构体指针
 *
 * @return          None.
 */
extern void nfca_picc_register_callback(nfca_picc_cb_t *cb);

/*******************************************************************************
 * @fn              nfca_picc_tx_prepare_raw_buf
 *
 * @brief           nfc-a picc 生成发送原始数据
 *
 * @param           out - 输出数据缓存指针
 * @param           in - 输入数据缓存指针
 * @param           parity - 输入数据校验位缓存指针
 * @param           length - 输入数据长度
 * @param           offset - 输入数据首位在首字节中断的偏移
 *
 * @return          uint16_t - 生成的数据长度.
 */
extern uint16_t nfca_picc_tx_prepare_raw_buf(uint8_t *out, uint8_t *in, uint8_t *parity, uint16_t length, uint8_t offset);

/*******************************************************************************
 * @fn              nfca_picc_tx_set_raw_buf
 *
 * @brief           nfc-a picc 设置发送原始数据
 *
 * @param           data - 数据缓存指针
 * @param           length - 数据长度
 *
 * @return          None.
 */
extern void nfca_picc_tx_set_raw_buf(uint8_t *data, uint16_t length);

/*******************************************************************************
 * @fn              nfca_picc_rx_irq_handler
 *
 * @brief           NFCA PICC 接收中断处理函数
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_picc_rx_irq_handler(void);

/*******************************************************************************
 * @fn              nfca_picc_tx_irq_handler
 *
 * @brief           NFCA PICC 发送中断处理函数
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_picc_tx_irq_handler(void);

/********************************** CH58x NFC-A CRYPTO1 ******************************************/

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

#ifdef __cplusplus
}
#endif

#endif  /* _CH58x_NFCA_LIB_H_ */
