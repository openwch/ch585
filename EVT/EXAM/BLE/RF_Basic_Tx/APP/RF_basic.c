/********************************** (C) COPYRIGHT *******************************
 * File Name          : rf_basic.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2026/11/09
 * Description        : 2.4G库basic模式发送测试例程(TX角色)，单发功能演示
 *
 *                      功率设置
 *                      RFIP_SetTxPower
 *                      1. 支持-20dBm ~ 4dBm 动态调整
 *
 *                      发送状态切换稳定时间
 *                      RFIP_SetTxDelayTime
 *                      1.如果需要切换通道发送，稳定时间不低于80us
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

rfipTx_t gTxParam;

__attribute__((__aligned__(4))) uint8_t TxBuf[64];

#define  TEST_DATA_LEN       4               // 数据长度
#define  TEST_FREQUENCY      16              // 通信频点
#define  RF_DEVICE_PERIOD    8000            // 发送速率

volatile uint32_t gTxCount;

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
    RFIP_SetTxParm( &gTxParam );
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
    if( sta&RF_STATE_TX_FINISH )
    {
        gTxCount ++;
    }
    if( sta&RF_STATE_TIMEOUT )
    {
        PRINT("tx timeout\n");
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
    if( events & RF_TEST_TX_EVENT )
    {
//        PRINT("tx:%d\n", gTxCount );
//        gTxCount = 0;
        return events ^ RF_TEST_TX_EVENT;
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

        conf.TxPower = LL_TX_PWR_4_DBM;
        conf.rfProcessCB = RF_ProcessCallBack;
        conf.processMask = RF_STATE_TX_FINISH;
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
        gParm.sendTime = 20*2;
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
    PFIC_EnableIRQ( BLEB_IRQn );
    PFIC_EnableIRQ( BLEL_IRQn );
    PRINT("rf role init.id=%d\n",rfTaskID);

    PRINT("----------------- tx -----------------\n");
    gTxCount = 0;
    TMR0_TimerInit( GetSysClock() / RF_DEVICE_PERIOD );
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
    PFIC_SetPriority( TMR0_IRQn, 0x80 );
    PFIC_EnableIRQ(TMR0_IRQn);
}

/******************************** endfile @rf ******************************/
