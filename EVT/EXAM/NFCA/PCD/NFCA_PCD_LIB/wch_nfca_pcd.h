/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_pcd.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/08/22
 * Description        : NFC-A PCD库头文件
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _WCH_NFCA_PCD_H_
#define _WCH_NFCA_PCD_H_

#ifdef __cplusplus
extern "C" {
#endif

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
    NFCA_PCD_REC_MODE_COLI          = 0x10,         /* 接收时进行冲突检测 */
} NFCA_PCD_REC_MODE_Def;

typedef struct _nfca_pcd_config_struct
{
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
 * @brief           nfc-a初始化
 *
 * @param           cfg - 配置参数指针
 *
 * @return          None.
 */
extern void nfca_pcd_lib_init(nfca_pcd_config_t *cfg);

/*******************************************************************************
 * @fn              nfca_pcd_lib_start
 *
 * @brief           nfc-a库开始运行，开始在天线上发送连续波
 *
 * @param           None
 *
 * @return          None.
 */
extern void nfca_pcd_lib_start(void);

/*******************************************************************************
 * @fn              nfca_pcd_lib_stop
 *
 * @brief           nfc-a库停止运行，停止在天线上发送连续波
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
 * @brief           nfc-a开始通讯，传输数据
 *
 * @param           None.
 *
 * @return          nfca_pcd_controller_state_t，获取当前通讯状态。.
 */
extern nfca_pcd_controller_state_t nfca_pcd_get_communicate_status(void);


/*******************************************************************************
 * @fn              nfca_pcd_get_recv_data_len
 *
 * @brief           获取本次接收并解码出的数据长度
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
 * @brief           nfc-a设置天线输出阻抗，默认Level1
 *
 * @param           drv - NFCA_PCD_DRV_CTRL_Def，天线发射引脚输出等级
 *
 * @return          None.
 */
extern void nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_Def drv);

/*******************************************************************************
 * @fn              nfca_pcd_set_recv_gain
 *
 * @brief           nfc-a设置接收增益，默认18DB
 *
 * @param           gain - NFCA_PCD_REC_GAIN_Def，接收增益
 *
 * @return          None.
 */
extern void nfca_pcd_set_recv_gain(NFCA_PCD_REC_GAIN_Def gain);

/*******************************************************************************
 * @fn              nfca_pcd_set_lp_ctrl
 *
 * @brief           nfc-a设置天线信号检测档位，默认0.8VDD
 *
 * @param           lp - NFCA_PCD_LP_CTRL_Def，天线信号检测档位
 *
 * @return          None.
 */
extern void nfca_pcd_set_lp_ctrl(NFCA_PCD_LP_CTRL_Def lp);

/*******************************************************************************
 * @fn              nfca_pcd_set_rec_threshold
 *
 * @brief           nfc-a设置比较门限，默认150mv
 *
 * @param           th - NFCA_PCD_REC_THRESHOLD_Def，解码模拟信号比较门限
 *
 * @return          None.
 */
extern void nfca_pcd_set_rec_threshold(NFCA_PCD_REC_THRESHOLD_Def th);

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
 * @brief           NFC读取天线信号是否过低
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

#ifdef __cplusplus
}
#endif

#endif  /* _WCH_NFCA_PCD_H_ */
