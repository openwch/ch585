/********************************** (C) COPYRIGHT *******************************
 * File Name          : rf_basic.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/11/09
 * Description        : 2.4G库basic模式接收测试例程(RX角色)，单收功能演示
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
tmosTaskID rfTaskID;
rfRoleParam_t gParm;

rfipRx_t gRxParam;

__attribute__((__aligned__(4))) uint8_t RxBuf[264]; // 接收DMA buf不能小于264字节

#define  TEST_DATA_LEN    4               // 数据长度
#define  TEST_FREQUENCY   16              // 通信频点

int8_t gRssi;

volatile int8_t gRssiAverage;
volatile uint32_t gRxCount;

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
    // 配置接收的频点
    gRxParam.frequency = TEST_FREQUENCY;
    // 配置接收的超时时间，0则无超时
    gRxParam.timeOut = 0;
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
        rf_rx_start( );
    }
    if( sta&RF_STATE_RX_CRCERR )
    {
        rf_rx_start( );
    }
    if( sta&RF_STATE_TIMEOUT )
    {
//        PRINT("timeout.\n");
        rf_rx_start( );
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
        return events ^ RF_START_RX_EVENT;
    }
    if( events & RF_TEST_RX_EVENT )
    {
        PRINT("rx:%d rssi:%d\n", gRxCount, gRssiAverage );  //串口打印会影响收发速率
        gRxCount = 0;
        gRssiAverage = 0;
        return events ^ RF_TEST_RX_EVENT;
    }
    return 0;
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

        conf.TxPower = LL_TX_PWR_4_DBM;
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_RX|RF_STATE_RX_CRCERR|RF_STATE_TIMEOUT;
        RFRole_BasicInit( &conf );
    }
    {
        gParm.accessAddress = 0x71762345;
        gParm.crcInit = 0x555555;
        // 配置PHY类型
        gParm.properties = LLE_MODE_PHY_2M;
        RFRole_SetParam( &gParm );
    }
    // RX相关参数，全局变量
    {
        gRxParam.accessAddress = gParm.accessAddress;
        gRxParam.crcInit = gParm.crcInit;
        gRxParam.properties = gParm.properties;
        gRxParam.rxDMA = (uint32_t)RxBuf;
        gRxParam.rxMaxLen = TEST_DATA_LEN;
    }
    PRINT("rf role init.id=%d\n",rfTaskID);

    PRINT("----------------- rx -----------------\n");
    gRssiAverage = 0;
    gRxCount = 0;
    tmos_set_event(rfTaskID, RF_START_RX_EVENT );
    tmos_start_reload_task(rfTaskID, RF_TEST_RX_EVENT, MS1_TO_SYSTEM_TIME(1000));
}

/******************************** endfile @rf ******************************/
