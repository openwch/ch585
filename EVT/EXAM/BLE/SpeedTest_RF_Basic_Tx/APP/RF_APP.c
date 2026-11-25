/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/10
 * Description        : RF 发射端应用层
 *                      带ACK模式  - 发送完成 -> 等对端回包 -> 收到回包发下一包
 *                                   (回包校验错/等待超时 -> 重发当前包)
 *                      单向连发   - 发送完成 -> 立即填包发送下一包
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

/* 发送缓冲 */
__attribute__((__aligned__(4))) static uint8_t TxBuf[RFB_BUF_SIZE];
/* 接收缓冲 */
__attribute__((__aligned__(4))) static uint8_t RxBuf[RFB_BUF_SIZE];

/* 统计 */
static volatile uint32_t gTxCount;       /* 已发送包数 */
static volatile uint32_t gAckCount;      /* 收到回包(ACK)数 */
static volatile int32_t  gRssiSum;       /* 回包 RSSI 累计 */
static int8_t            gRssi;

static void AppTxCb( rfbTxEvt_t evt );

/*******************************************************************************
 * @fn      AppFillTxPacket
 *
 * @brief   填充下一包，纯应用层数据
 */
__HIGH_CODE
static void AppFillTxPacket( void )
{
    TxBuf[0] = 0x55;                 /* 包头 */
    TxBuf[2]++;                      /* 包序号 */
    TxBuf[3] = gRssi;                /* 携带最近一次回包的 RSSI */
}

/*******************************************************************************
 * @fn      AppRxCb
 *
 * @brief   接收回调(RF中断上下文)。发射端收到包即视为对端回包(ACK)
 */
__HIGH_CODE
static void AppRxCb( uint8_t *rxBuf, uint8_t len, int8_t rssi, rfbRxEvt_t evt )
{
    if( evt == RFB_RX_EVT_DATA )
    {
        /* 收到回包：统计后发下一包 */
        gRssi = rssi;
        gRssiSum += rssi;
        gAckCount++;
        tmos_memcpy( RxBuf, rxBuf, len );
        AppFillTxPacket( );
        RFB_TxStart( TxBuf, RF_DATA_LEN, RF_CHANNEL, AppTxCb );
        gTxCount++;
    }
    else
    {
        /* 回包校验错 / 等待超时：重发当前包 */
        RFB_TxStart( TxBuf, RF_DATA_LEN, RF_CHANNEL, AppTxCb );
    }
}

/*******************************************************************************
 * @fn      AppTxCb
 *
 * @brief   发送结果回调(RF中断上下文)
 */
__HIGH_CODE
static void AppTxCb( rfbTxEvt_t evt )
{
    if( evt == RFB_TX_EVT_DONE )
    {
#if( RF_USE_ACK )
        RFB_RxStart( RF_CHANNEL, RF_RX_WINDOW, AppRxCb );   /* 发送完成，开窗等对端回包 */
#else
        /* 单向连发：立即发下一包 */
        AppFillTxPacket( );
        RFB_TxStart( TxBuf, RF_DATA_LEN, RF_CHANNEL, AppTxCb );
        gTxCount++;
#endif
    }
    else
    {
        /* 发送超时(失败)：重发当前包 */
        RFB_TxStart( TxBuf, RF_DATA_LEN, RF_CHANNEL, AppTxCb );
    }
}

/*******************************************************************************
 * @fn      RFApp_ProcessEvent
 *
 * @brief   应用层任务：周期统计打印
 */
static uint16_t RFApp_ProcessEvent( tmosTaskID task_id, uint16_t events )
{
    if( events & RF_EVT_STAT )
    {
        PRINT("%d\t%d\t%d\tTxSpeed: %d B/s\n", gTxCount, gAckCount,
              gAckCount ? (int)(gRssiSum / (int)gAckCount) : 0, gTxCount * RF_DATA_LEN);
        gTxCount = 0;
        gAckCount = 0;
        gRssiSum = 0;
        return events ^ RF_EVT_STAT;
    }
    return 0;
}

/*******************************************************************************
 * @fn      RFApp_Init
 *
 * @brief   应用层初始化：注册回调、配置射频、启动测速
 */
void RFApp_Init( void )
{
    sTaskID = TMOS_ProcessEventRegister( RFApp_ProcessEvent );

    RFB_Config( RF_ACCESS_ADDR, RF_CRC_INIT, RF_PHY_MODE, RF_TX_POWER );

    /* 打印演示配置信息 */
    PRINT("\n==============================================\n");
    PRINT("  2.4G RF Basic SpeedTest TX\n");
#if( RF_USE_ACK )
    PRINT("  mode     : ACK\n");
#else
    PRINT("  mode     : TX only (single send)\n");
#endif
    PRINT("  freq     : %d.%03d MHz\n", 2402 + RF_CHANNEL * 2, 0);
#if (RF_PHY_MODE == RFB_PHY_1M)
    PRINT("  phy      : 1M\n");
#else
    PRINT("  phy      : 2M\n");
#endif
    PRINT("  len      : %d bytes\n", RF_DATA_LEN);
    PRINT("==============================================\n");

    tmos_start_reload_task( sTaskID, RF_EVT_STAT, MS1_TO_SYSTEM_TIME(RF_STAT_PERIOD_MS) );

    /* 初始化发送的数据并发送首包 */
    tmos_memset( TxBuf, 0x55, sizeof(TxBuf) );
    AppFillTxPacket( );
    RFB_TxStart( TxBuf, RF_DATA_LEN, RF_CHANNEL, AppTxCb );
}

/******************************** endfile @rf ******************************/
