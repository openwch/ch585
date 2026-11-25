/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_PHY.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/09
 * Description        : RF_PHY驱动
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef RF_PHY_H
#define RF_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ================================ 公共 ================================ */

#define RF_PHY_1M           0x01
#define RF_PHY_2M           0x02

#define RF_WHIT_ON          0x01
#define RF_WHIT_OFF         0x02

#define RF_RET_ADDR_ERR     1
#define RF_RET_BUSY         2

extern void RFPhy_Config(uint32_t accessAddress, uint32_t channel,
                         uint32_t crcInit, uint8_t phyMode, uint8_t whitening);

#define RF_TX_MAX_LEN               251     /* RF_Tx 允许的最大长度 */

/* 底层发送结果：0 表示成功 */
typedef enum
{
    RF_TX_OK = 0,           /* 发送完成（BASIC 模式，本机发送结束） */
    RF_TX_ERR_PARAM,        /* 空指针、长度为 0 或超过 251 */
    RF_TX_ERR_ADDR,         /* RF_Tx 返回 1：接入地址非法 */
    RF_TX_ERR_BUSY,         /* RF_Tx 返回 2：射频忙，或上一包未结束 */
    RF_TX_ERR_FAIL,         /* 回调报告 TX_MODE_TX_FAIL */
    RF_TX_ERR_TIMEOUT,      /* 软件等待回调超时 */
} rfTxResult_t;

/* RF中断回调状态标志 */
#define RF_TX_STATE_FINISH    0x00    /* 底层发送数据完成 */
#define RF_TX_STATE_SENDING   0x01    /* 底层发送中,发送未完成 */
#define RF_TX_STATE_TIMEOUT   0x02    /* 底层发送超时 */
#define RF_TX_STATE_IDLE      0xFF    /* 底层发送空闲态 */

extern volatile uint8_t gTx_state;

/* 启动一次发送：返回值只代表是否成功提交给射频 */
extern rfTxResult_t RFPhy_SendPacket(uint8_t *buf, uint8_t len, uint8_t pktType);


#define RF_RX_MAX_LEN               251     /* RF_Rx 允许的最大长度 */
#define RF_RX_BUF_SIZE              (RF_RX_MAX_LEN + 2)   /* RSSI + len + payload */

/* 接收结果：0 表示收到有效数据 / 启动成功 */
typedef enum
{
    RF_RX_OK = 0,           /* 收到有效数据；RFPhy_StartRx 返回时表示接收已启动 */
    RF_RX_ERR_ADDR,         /* RF_Rx 返回 1：接入地址非法 */
    RF_RX_ERR_BUSY,         /* RF_Rx 返回 2：射频忙，或上一次接收还没结束 */
    RF_RX_ERR_CRC,          /* CRC 错误 */
    RF_RX_ERR_TYPE,         /* 匹配类型错误 */
    RF_RX_ERR_LEN,          /* 包长度非法 */
    RF_RX_ERR_NONE,         /* 还没有收到数据 */
} rfRxResult_t;

/* 接收状态 */
#define RF_RX_DRV_IDLE       0x00
#define RF_RX_DRV_RECEIVING  0x01
#define RF_RX_DRV_FINISH     0x02
#define RF_RX_DRV_FAIL       0x03

/* RX 数据回调：在 RF 中断上下文中调用，只做拷贝/置事件等轻量操作，
 * 不可在此回调中直接调用 RF 接收或者发送 API；
 * buf 仅在本次回调执行期间有效，len 为 RSSI + len 域 + payload 的总字节数 */
typedef void (*rfRxCallback_t)(uint8_t *buf, uint8_t len, int8_t rssi, rfRxResult_t status);

/* 启动一次接收并绑定本次数据回调：返回 RF_RX_OK 只表示接收已启动；
 * 收到数据或出错时在 RF 中断上下文中调用 cb，cb 传 NULL 表示本次不上报 */
extern rfRxResult_t RFPhy_StartRx(uint8_t pktType, rfRxCallback_t cb);

#endif /* RF_PHY_H */

#ifdef __cplusplus
}
#endif

