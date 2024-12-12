/********************************** (C) COPYRIGHT *******************************
 * File Name          : rf_basic.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/08/15
 * Description        : 2.4G库基本模式收发测试例程
 *
 *                      功率设置
 *                      RFIP_SetTxPower
 *                      1. 支持-20dBm ~ 4dBm 动态调整
 *
 *                      发送状态切换稳定时间
 *                      RFIP_SetTxDelayTime
 *                      1.如果需要切换通道发送，稳定时间不低于80us
 *
 *                      获取信号强度
 *                      RFIP_ReadRssi
 *                      1.可读取当前数据包的RSSI值
 *
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "rf_test.h"
#include "hal.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
#define  ALIGNED4(x)       ((x+3)/4*4)

tmosTaskID rfTaskID;
rfRoleParam_t gParm;

rfipTx_t gTxParam;
rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];
__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // 接收DMA buf不能小于264字节

#define  MODE_RX     0
#define  MODE_TX     1


#define  WAIT_ACK         0               // 是否使能ACK
#define  TEST_DATA_LEN    4               // 数据长度
#define  TEST_FREQUENCY   16              // 通信频点

#define  TEST_MODE     MODE_TX            // 发送模式
//#define  TEST_MODE     MODE_RX            // 接收模式

#if WAIT_ACK
#define  RF_DEVICE_PERIDOC    4000        // 等待ACK模式的发送速率
#else
#define  RF_DEVICE_PERIDOC    8000        // 无需ACK模式的发送速率
#endif

int8_t gRssi;

int8_t gRssiAverage;
uint32_t gTxCount;
uint32_t gRxCount;

/******************************** 发送相关函数 ********************************/
/**
 * @brief   配置发送的频点
 *
 * @param   channel_num - 需要切换的通道
 *
 * @return  None.
 */

void rf_tx_set_channel( uint8_t channel_num )
{
    gTxParam.frequency = channel_num;
}

/**
 * @brief   配置发送的地址
 *
 * @param   sync_word - 需要配置的接入地址
 *
 * @return  None.
 */

void rf_tx_set_sync_word( uint32_t sync_word )
{
    gTxParam.accessAddress = sync_word;
}

/**
 * @brief   rf发送数据子程序
 *
 * @param   pBuf - 发送的DMA地址
 *
 * @return  None.
 */
__HIGH_CODE
void rf_tx_start( uint8_t *pBuf )
{
    RFIP_SetTxStart( );
    // 配置发送的频点
    gTxParam.frequency = TEST_FREQUENCY;
    // 发送的DMA地址
    gTxParam.txDMA = (uint32_t)pBuf;
//    gTxParam.accessAddress = gParm.accessAddress; // 发送同步字
//    gTxParam.sendCount = 1;  // 发送次数
    RFIP_SetTxParm( &gTxParam );
}

/******************************** 接收相关函数 ********************************/
/**
 * @brief   配置接收的地址
 *
 * @param   sync_word - 需要配置的接入地址
 *
 * @return  None.
 */

void rf_rx_set_sync_word( uint32_t sync_word )
{
    gRxParam.accessAddress = sync_word;
}

/**
 * @brief   配置接收的频点
 *
 * @param   channel_num - 需要切换的通道
 *
 * @return  None.
 */

void rf_rx_set_channel( uint8_t channel_num )
{
    gRxParam.frequency = channel_num;
}

/**
 * @brief   rf接收数据子程序
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_start( void )
{
    // 配置发送的频点
    gRxParam.frequency = TEST_FREQUENCY;
    // 配置接收的超时时间，0则无超时
    gRxParam.timeOut = 0;
//    gRxParam.accessAddress = gParm.accessAddress; // 接收同步字
    RFIP_SetRx( &gRxParam );
}

/**
 * @brief   rf接收数据处理
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void rf_rx_process_data( void )
{
    // 获取信号强度
    gRssi = RFIP_ReadRssi();
    if( !gRssiAverage ) gRssiAverage =  gRssi;
    else gRssiAverage = (gRssi+gRssiAverage)/2;
    gRxCount ++;
//    {
//        uint8_t *pData = (uint8_t *)gRxParam.rxDMA;
//        PRINT("#R %d %d\n",(int8_t)pData[3],(int8_t)gRssi);
//    }
}

/*******************************************************************************
 * @fn      RF_ProcessCallBack
 *
 * @brief   rf中断处理程序
 *
 * @param   sta - 中断状态.
 *          id - 保留
 *
 * @return  None.
 */
__HIGH_CODE
void RF_ProcessCallBack( rfRole_States_t sta,uint8_t id  )
{
    if( sta&RF_STATE_RX )
    {
        rf_rx_process_data();
#if( TEST_MODE == MODE_RX )
#if( WAIT_ACK )
        TxBuf[0] = 0x0d;
        TxBuf[1] = 0;
        rf_tx_start( TxBuf );
#else
        rf_rx_start( );
#endif
#endif
    }
    if( sta&RF_STATE_RX_CRCERR )
    {
#if( TEST_MODE == MODE_RX )
        rf_rx_start( );
#else
        PRINT("nak@crc\n");
#endif
    }
    if( sta&RF_STATE_TX_FINISH )
    {
#if( WAIT_ACK )
        rf_rx_start( );
#endif
        gTxCount ++;
    }
    if( sta&RF_STATE_TIMEOUT )
    {
#if( TEST_MODE ==  MODE_RX )
        PRINT("send error.\n");
        rf_rx_start( );
#endif
    }
}

/*******************************************************************************
 * @fn      RF_ProcessEvent
 *
 * @brief   RF层系统任务处理
 *
 * @param   None.
 *
 * @return  None.
 */
tmosEvents RFRole_ProcessEvent( tmosTaskID task_id, tmosEvents events )
{
    if( events & SYS_EVENT_MSG )
    {
        uint8_t * msgPtr;

        msgPtr = tmos_msg_receive(task_id);
        if( msgPtr )
        {
            /* De-allocate */
             tmos_msg_deallocate( msgPtr );
        }
        return events ^ SYS_EVENT_MSG;
    }
    if( events & RF_START_RX_EVENT )
    {
        rf_rx_start( );
    }
    if( events & RF_TEST_TX_EVENT )
    {
        PRINT("%d\t%d\t%d\n",gTxCount,gRxCount,gRssiAverage );
        gTxCount = 0;
        gRxCount = 0;
        gRssiAverage = 0;
        return events ^ RF_TEST_TX_EVENT;
    }
    if( events & RF_TEST_RX_EVENT )
    {
        PRINT("%d\t%d\t%d\n",gTxCount,gRxCount,gRssiAverage );
        gTxCount = 0;
        gRxCount = 0;
        gRssiAverage = 0;
        return events ^ RF_TEST_RX_EVENT;
    }
    return 0;
}

/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   TMR0中断函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) // TMR0 定时中断
{
    if(TMR0_GetITFlag(TMR0_3_IT_CYC_END))
    {
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
        // 初始化发送的数据
        TxBuf[0] = 0x55;
        TxBuf[1] = TEST_DATA_LEN;
        TxBuf[2]++;
        TxBuf[3] = gRssi;
        rf_tx_start( TxBuf );
    }
}

/*******************************************************************************
 * @fn      RFRole_Init
 *
 * @brief   RF应用层初始化
 *
 * @param   None.
 *
 * @return  None.
 */
void RFRole_Init(void)
{
    rfTaskID = TMOS_ProcessEventRegister( RFRole_ProcessEvent );
    {
        rfRoleConfig_t conf ={0};

        conf.TxPower = LL_TX_POWEER_4_DBM;
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_RX|RF_STATE_RX_CRCERR|RF_STATE_TX_FINISH|RF_STATE_TIMEOUT;
        RFRole_BasicInit( &conf );
    }
    {
        gParm.accessAddress = 0x71762345;
        gParm.crcInit = 0x555555;
        // 配置PHY类型
        gParm.properties = LLE_MODE_PHY_2M;
        // 配置重传间隔，发送次数大于1时有效
        gParm.sendInterval = 1999*2;
        // 配置发送稳定时间
        gParm.sendTime = 10*2;
        RFRole_SetParam( &gParm );
    }
    // TX相关参数，全局变量
    {
        gTxParam.accessAddress = gParm.accessAddress;
        gTxParam.crcInit = gParm.crcInit;
        gTxParam.properties = gParm.properties;
        gTxParam.sendCount = 1;
        gTxParam.txDMA = (uint32_t)TxBuf;
    }
    // RX相关参数，全局变量
    {
        gRxParam.accessAddress = gParm.accessAddress;
        gRxParam.crcInit = gParm.crcInit;
        gRxParam.properties = gParm.properties;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }
    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );
    PRINT("rf role init.id=%d\n",rfTaskID);

#if( TEST_MODE == MODE_RX )
    PRINT("----------------- rx -----------------\n");
    gRssiAverage = 0;
    gTxCount = 0;
    gRxCount = 0;
    tmos_set_event(rfTaskID, RF_START_RX_EVENT );
    tmos_start_reload_task(rfTaskID, RF_TEST_RX_EVENT, MS1_TO_SYSTEM_TIME(1000));
#else
    PRINT("----------------- tx -----------------\n");
    gRssiAverage = 0;
    gTxCount = 0;
    gRxCount = 0;
    tmos_start_reload_task(rfTaskID, RF_TEST_TX_EVENT, MS1_TO_SYSTEM_TIME(1000));
    TMR0_TimerInit( GetSysClock() / RF_DEVICE_PERIDOC );
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
#endif
    PFIC_SetPriority( TMR0_IRQn, 0x80 );
    PFIC_EnableIRQ(TMR0_IRQn);
}

/******************************** endfile @rf ******************************/
