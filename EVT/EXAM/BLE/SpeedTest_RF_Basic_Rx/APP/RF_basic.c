/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_basic.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/10
 * Description        : 2.4G 基础模式(RFRole Basic)驱动层
 *                      库状态回调统一在此解析，再分发给应用层注册的收发回调；
 *                      收发节奏(等ACK/连发/重发)等策略全部由应用层决定
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontrollers manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "RF_basic.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/* 驱动内部记录的最近一次动作，用于超时事件归因 */
typedef enum
{
    RFB_ACTION_TX = 0,          /* 最近一次启动的是发送 */
    RFB_ACTION_RX,              /* 最近一次启动的是接收 */
} rfbAction_t;

static rfbTxCb_t   sTxCb = NULL;
static rfbRxCb_t   sRxCb = NULL;
static rfbAction_t sLastAction = RFB_ACTION_TX;

static rfipTx_t gTxParm = {0};
static rfipRx_t gRxParm = {0};

/* 接收 DMA 缓冲，要求不能小于 264 字节 */
__attribute__((__aligned__(4))) static uint8_t sRxBuf[RFB_BUF_SIZE];

/*******************************************************************************
 * @fn      RFB_StatusCallback
 *
 * @brief   2.4G 库状态回调(中断上下文)，只做状态分发，不做业务决策
 *
 * @param   sta - 中断状态
 *          id  - 保留
 */
__HIGH_CODE
static void RFB_StatusCallback( rfRole_States_t sta, uint8_t id )
{
    if( sta & RF_STATE_RX )
    {
        if( sRxCb ) sRxCb( sRxBuf, sRxBuf[1], RFIP_ReadRssi(), RFB_RX_EVT_DATA );
    }
    else if( sta & RF_STATE_RX_CRCERR )
    {
        if( sRxCb ) sRxCb( NULL, 0, 0, RFB_RX_EVT_CRC_ERR );
    }
    else if( sta & RF_STATE_TX_FINISH )
    {
        if( sTxCb ) sTxCb( RFB_TX_EVT_DONE );
    }
    else if( sta & RF_STATE_TIMEOUT )
    {
        /* 按最近动作归因：发送后等待超时归发送，接收窗超时归接收 */
        if( sLastAction == RFB_ACTION_TX )
        {
            if( sTxCb ) sTxCb( RFB_TX_EVT_TIMEOUT );
        }
        else
        {
            if( sRxCb ) sRxCb( NULL, 0, 0, RFB_RX_EVT_TIMEOUT );
        }
    }
}

/*******************************************************************************
 * @fn      RFB_Config
 *
 * @brief   初始化并配置 2.4G 基础模式收发
 *
 * @param   accessAddress - 接入地址
 *          crcInit       - CRC 初值
 *          phyMode       - PHY 速率
 *          txPower       - 发射功率
 */
void RFB_Config( uint32_t accessAddress, uint32_t crcInit, uint8_t phyMode, int8_t txPower )
{
    rfRoleConfig_t conf = {0};
    rfRoleParam_t  parm = {0};

    /* 库初始化：注册状态回调，四个状态全部上报 */
    conf.TxPower = txPower;
    conf.rfProcessCB = RFB_StatusCallback;
    conf.processMask = RF_STATE_RX | RF_STATE_RX_CRCERR | RF_STATE_TX_FINISH | RF_STATE_TIMEOUT;
    RFRole_BasicInit( &conf );

    /* 射频公共参数(sendInterval/sendTime 沿用官方例程取值，sendTime=0 会导致发送不稳定) */
    parm.accessAddress = accessAddress;
    parm.crcInit = crcInit;
    parm.properties = phyMode;
    parm.sendInterval = 1999*2;
    parm.sendTime = 20*2;
    RFRole_SetParam( &parm );

    /* 发送参数 */
    gTxParm.accessAddress = accessAddress;
    gTxParm.crcInit = crcInit;
    gTxParm.properties = phyMode;
    gTxParm.sendCount = 1;                    

    /* 接收参数 */
    gRxParm.accessAddress = accessAddress;
    gRxParm.crcInit = crcInit;
    gRxParm.properties = phyMode;
    gRxParm.rxDMA = (uint32_t)sRxBuf;
    gRxParm.rxMaxLen = RFB_MAX_DATA_LEN;
}

/*******************************************************************************
 * @fn      RFB_TxStart
 *
 * @brief   启动一次发送。txBuf 需 4 字节对齐且不小于 RFB_BUF_SIZE；
 *
 * @param   txBuf   - 发送缓冲
 *          len     - 数据长度(不大于 RFB_MAX_DATA_LEN)
 *          channel - 通信频点
 *          cb      - 本次发送结果回调
 */
__HIGH_CODE
void RFB_TxStart( uint8_t *txBuf, uint8_t len, uint8_t channel, rfbTxCb_t cb )
{
    sTxCb = cb;
    sLastAction = RFB_ACTION_TX;
    txBuf[1] = len;
    RFIP_SetTxStart( );
    gTxParm.frequency = channel;
    gTxParm.txDMA = (uint32_t)txBuf;
    RFIP_SetTxParm( &gTxParm );
}

/*******************************************************************************
 * @fn      RFB_RxStart
 *
 * @brief   启动一次接收
 *
 * @param   channel   - 通信频点
 *          rxTimeout - RX 等待超时 N*0.5us，0 表示无超时
 *          cb        - 本次接收数据回调
 */
__HIGH_CODE
void RFB_RxStart( uint8_t channel, uint16_t rxTimeout, rfbRxCb_t cb )
{
    sRxCb = cb;
    sLastAction = RFB_ACTION_RX;
    gRxParm.frequency = channel;
    gRxParm.timeOut = rxTimeout;
    RFIP_SetRx( &gRxParm );
}

/******************************** endfile @rf ******************************/
