/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/09
 * Description        : RF_PHY例程：2.4G单播发射api接口，周期发送10字节数据
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含（RF_APP.h 提供角色宏，其内部已包含 RF_PHY.h） */
#include "CONFIG.h"
#include "RF_APP.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
uint8_t taskID;
uint8_t TX_DATA[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};

/* RFPhy_Config 配置参数 */
#define RF_CFG_ACCESS_ADDRESS   0x71764129  /* 接入地址：禁止使用0x55555555以及0xAAAAAAAA ( 建议不超过24次位反转，且不超过连续的6个0或1 ) */
#define RF_CFG_CHANNEL          39          /* rf channel(0-39) / frequency (2400000kHz-2483500kHz) */
#define RF_CFG_CRCINIT          0x555555    /* crc initial value */
#define RF_CFG_PHY_MODE          RF_PHY_1M   /* 01: PHY 1M / 02: PHY 2M */
#define RF_CFG_WHITENING        RF_WHIT_OFF /* Whitening 01: On / 02: Off */

/*********************************************************************
 * @fn      RFApp_ProcessEvent
 *
 * @brief   RF 事件处理
 *
 * @param   task_id - 任务ID
 * @param   events  - 事件标志
 *
 * @return  未完成事件
 */
static uint16_t RFApp_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(task_id)) != NULL)
        {
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    if(events & RF_TX_PERIODIC_EVT)
    {
        rfTxResult_t ret;

        ret = RFPhy_SendPacket(TX_DATA, 10, 0xFF);
        if(ret != RF_TX_OK)
        {
            PRINT("tx fail, ret=%d\n", ret);
        }

        if(ret == RF_TX_OK)    TX_DATA[9]++;  // 发送完成后可重新填充数据
        tmos_start_task(taskID, RF_TX_PERIODIC_EVT, 800);
        return events ^ RF_TX_PERIODIC_EVT;
    }
    return 0;
}

/*********************************************************************
 * @fn      RFApp_Init
 *
 * @brief   RF 初始化
 *
 * @return  none
 */
void RFApp_Init(void)
{
    taskID = TMOS_ProcessEventRegister(RFApp_ProcessEvent);

    RFPhy_Config(RF_CFG_ACCESS_ADDRESS, RF_CFG_CHANNEL, RF_CFG_CRCINIT, RF_CFG_PHY_MODE, RF_CFG_WHITENING);

    PRINT("--------TX--------\n");

    { // TX mode
        tmos_set_event( taskID , RF_TX_PERIODIC_EVT );
    }
}

/******************************** endfile @ main ******************************/
