/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/10
 * Description        : RF 接收端应用层配置与接口
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontrollers manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __RF_APP_H
#define __RF_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RF_basic.h"

/* ---------------- 测试配置 ---------------- */
#define RF_USE_ACK          0               /* 1-带ACK模式:收到包后回发短包应答 0-单向接收 */


#define RF_DATA_LEN         251             /* 测试包数据长度 */
#define RF_CHANNEL          20              /* 通信频点 */
#define RF_PHY_MODE         RFB_PHY_2M      /* PHY 速率 */
#define RF_ACCESS_ADDR      0x94826E8E      /* 接入地址 */
#define RF_CRC_INIT         0x555555        /* CRC 初值 */
#define RF_TX_POWER         LL_TX_PWR_4_DBM /* 发射功率 */
#define RF_RX_TIMEOUT       (80*2)          /* ACK 模式回发应答后的接收窗 N*0.5us(80us) */
/* 接收窗口超时：ACK 模式需要超时窗口；单向接收不设超时，避免每 80us 超时重开一次窗 */
#if( RF_USE_ACK )
#define RF_RX_WINDOW        RF_RX_TIMEOUT
#else
#define RF_RX_WINDOW        0
#endif
#define RF_STAT_PERIOD_MS   1000            /* 速率统计打印周期 ms */

/* TMOS 任务事件 */
#define RF_EVT_RX_START     (1<<0)          /* 启动接收 */
#define RF_EVT_STAT         (1<<1)          /* 周期速率统计 */
#define RF_EVT_CMD          (1<<2)          /* 命令包打印 */

extern void RFApp_Init( void );

#ifdef __cplusplus
}
#endif

#endif
