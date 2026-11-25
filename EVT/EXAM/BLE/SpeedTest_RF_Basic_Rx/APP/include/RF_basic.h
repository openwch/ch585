/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_basic.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/10
 * Description        : 2.4G 基础模式(RFRole Basic)驱动层接口
 *                      只提供收发原语与状态回调分发，收发节奏策略(带ACK乒乓/
 *                      单向连发等)由应用层通过回调实现
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontrollers manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __RF_BASIC_H
#define __RF_BASIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wchrf.h"

/* PHY 速率 */
#define RFB_PHY_1M              LLE_MODE_PHY_1M
#define RFB_PHY_2M              LLE_MODE_PHY_2M

/* 收发缓冲区长度(RX DMA 缓冲不能小于 264 字节) */
#define RFB_BUF_SIZE            264

/* 单包最大数据长度 */
#define RFB_MAX_DATA_LEN        251

/* 发送结果事件 */
typedef enum
{
    RFB_TX_EVT_DONE = 0,        /* 发送完成 */
    RFB_TX_EVT_TIMEOUT,         /* 发送超时(失败) */
} rfbTxEvt_t;

/* 接收事件 */
typedef enum
{
    RFB_RX_EVT_DATA = 0,        /* 收到数据包 */
    RFB_RX_EVT_CRC_ERR,         /* CRC 校验错误 */
    RFB_RX_EVT_TIMEOUT,         /* RX 等待超时 */
} rfbRxEvt_t;

/*
 * 收发回调均在 RF 中断上下文中调用，只做轻量操作(计数/填包/启动下一包)，
 * 不可在回调中调用阻塞或打印接口。
 * rxBuf 指向驱动层接收缓冲，仅在本次回调执行期间有效：
 *   rxBuf[0] - 对端包头域; rxBuf[1] - 数据长度; rxBuf[2] - 包序号
 * len 即 rxBuf[1]，rssi 为本包信号强度。
 */
typedef void (*rfbTxCb_t)( rfbTxEvt_t evt );
typedef void (*rfbRxCb_t)( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt );

/*
 * 初始化并配置 RF
 * accessAddress - 接入地址
 * crcInit       - CRC 初值
 * phyMode       - PHY 速率: RFB_PHY_1M / RFB_PHY_2M
 * txPower       - 发射功率，如 LL_TX_PWR_4_DBM
 */
void RFB_Config( uint32_t accessAddress, uint32_t crcInit, uint8_t phyMode, int8_t txPower );
void RFB_TxStart( uint8_t *txBuf, uint8_t len, uint8_t channel, rfbTxCb_t cb );  /* 非阻塞启动一次发送，channel 为通信频点 */
void RFB_RxStart( uint8_t channel, uint16_t rxTimeout, rfbRxCb_t cb );           /* 启动一次接收(带超时窗)，rxTimeout 为 N*0.5us(0 表示无超时) */

#ifdef __cplusplus
}
#endif

#endif
