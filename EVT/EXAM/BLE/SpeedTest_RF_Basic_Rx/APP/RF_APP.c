/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/10
 * Description        : RF 接收端应用层
 *                      收发节奏策略全在本层实现：
 *                      带ACK模式  - 收到包 -> 统计 -> 回发短包应答 -> 回到接收
 *                      单向接收   - 收到包 -> 统计 -> 直接继续接收
 *                      校验错/超时 - 重新开接收窗
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontrollers manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "RF_APP.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
static tmosTaskID sTaskID;

/* ACK 回发缓冲(长度域为 0 的短包) */
__attribute__((__aligned__(4))) static uint8_t TxBuf[RFB_BUF_SIZE];

/* 接收缓冲 */
__attribute__((__aligned__(4))) static uint8_t RxBuf[RFB_BUF_SIZE];

/* 命令包缓冲 */
__attribute__((__aligned__(4))) static uint8_t CmdBuf[RFB_BUF_SIZE];
static volatile uint8_t CmdLen;

/* 速率统计 */
static volatile uint32_t gRxCount;       /* 收到新包数(按序号判重) */
static volatile uint32_t gAckCount;      /* 已回发 ACK 数 */
static volatile int32_t  gRssiSum;       /* RSSI 累计 */

static void AppRxDataCb( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt );
static void AppRxCmdCb( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt );
static void AppTxCb( rfbTxEvt_t evt );

/*******************************************************************************
 * @fn      AppRxSelectCb
 *
 * @brief   按包长选择下一接收窗口绑定的回调
 */
__HIGH_CODE
static rfbRxCb_t AppRxSelectCb( uint8_t len )
{
    return (len == RF_DATA_LEN) ? AppRxDataCb : AppRxCmdCb;
}

/*******************************************************************************
 * @fn      AppRxStats
 *
 * @brief   测试数据包统计(RF中断上下文)
 */
__HIGH_CODE
static void AppRxStats( uint8_t *rxBuf, uint8_t len, int8_t rssi )
{
    static uint8_t lastSeq = 0;          /* 上一包序号，用于判重 */

    gRssiSum += rssi;
    if( rxBuf[2] != lastSeq )
    {
        gRxCount++;                      /* 只统计序号更新的新包 */
    }
    lastSeq = rxBuf[2];
    tmos_memcpy( RxBuf, rxBuf, len );
}

/*******************************************************************************
 * @fn      AppRxDataCb
 *
 * @brief   测试数据包回调(RF中断上下文)
 */
__HIGH_CODE
static void AppRxDataCb( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt )
{
    if( evt == RFB_RX_EVT_DATA )
    {
        AppRxStats( rxBuf, len, rssi );
#if( RF_USE_ACK )
        RFB_TxStart( TxBuf, 0, RF_CHANNEL, AppTxCb );    /* 回发短包作为 ACK */
        gAckCount++;
#else
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxSelectCb( len ) ); /* 继续接收 */
#endif
    }
    else
    {
        /* 校验错 / 接收超时：重新开窗 */
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxDataCb );
    }
}

/*******************************************************************************
 * @fn      AppRxCmdCb
 *
 * @brief   其他长度包回调(RF中断上下文)，转任务打印
 */
__HIGH_CODE
static void AppRxCmdCb( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt )
{
    if( evt == RFB_RX_EVT_DATA )
    {
        if( len == RF_DATA_LEN )
        {
            AppRxStats( rxBuf, len, rssi );
        }
        else
        {
            tmos_memcpy( CmdBuf, rxBuf, len );
            CmdLen = len;
            tmos_set_event( sTaskID, RF_EVT_CMD );
        }
#if( RF_USE_ACK )
        RFB_TxStart( TxBuf, 0, RF_CHANNEL, AppTxCb );
        gAckCount++;
#else
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxSelectCb( len ) );
#endif
    }
    else
    {
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxCmdCb );
    }
}

/*******************************************************************************
 * @fn      AppTxCb
 *
 * @brief   发送结果回调(RF中断上下文)，ACK 回发完成后回到接收
 */
__HIGH_CODE
static void AppTxCb( rfbTxEvt_t evt )
{
    (void)evt;     
    RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxDataCb );
}

/*******************************************************************************
 * @fn      RFApp_ProcessEvent
 *
 * @brief   应用层任务：启动接收、命令包打印、周期统计打印
 */
static uint16_t RFApp_ProcessEvent( tmosTaskID task_id, uint16_t events )
{
    if( events & RF_EVT_RX_START )
    {
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxDataCb );
        return events ^ RF_EVT_RX_START;
    }
    if( events & RF_EVT_CMD )
    {
        uint8_t i;
        PRINT("cmd: len %d,", CmdLen);
        for( i = 0; i < CmdLen; i++ )
        {
            PRINT(" %02x", CmdBuf[i]);
        }
        PRINT("\n");
        return events ^ RF_EVT_CMD;
    }
    if( events & RF_EVT_STAT )
    {
        PRINT("%d\t%d\t%d\tRxSpeed: %d B/s\n", gAckCount, gRxCount,
              gRxCount ? (int)(gRssiSum / (int)gRxCount) : 0, gRxCount * RF_DATA_LEN);
        gRxCount = 0;
        gAckCount = 0;
        gRssiSum = 0;
        return events ^ RF_EVT_STAT;
    }
    return 0;
}

/*******************************************************************************
 * @fn      RFApp_Init
 *
 * @brief   应用层初始化：配置射频、启动接收
 */
void RFApp_Init( void )
{
    sTaskID = TMOS_ProcessEventRegister( RFApp_ProcessEvent );

    RFB_Config( RF_ACCESS_ADDR, RF_CRC_INIT, RF_PHY_MODE, RF_TX_POWER );

    /* 打印演示配置信息 */
    PRINT("\n==============================================\n");
    PRINT("  2.4G RF Basic SpeedTest RX\n");
#if( RF_USE_ACK )
    PRINT("  mode     : ACK\n");
#else
    PRINT("  mode     : RX only (single receive)\n");
#endif
    PRINT("  freq     : %d.%03d MHz\n", 2402 + RF_CHANNEL * 2, 0);
#if (RF_PHY_MODE == RFB_PHY_1M)
    PRINT("  phy      : 1M\n");
#else
    PRINT("  phy      : 2M\n");
#endif
    PRINT("  len      : %d bytes\n", RF_DATA_LEN);
    PRINT("==============================================\n");

    tmos_set_event( sTaskID, RF_EVT_RX_START );
    tmos_start_reload_task( sTaskID, RF_EVT_STAT, MS1_TO_SYSTEM_TIME(RF_STAT_PERIOD_MS) );
}

/******************************** endfile @rf ******************************/
