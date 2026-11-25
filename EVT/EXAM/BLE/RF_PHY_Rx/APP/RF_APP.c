/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.c
 * Description        : RF_PHY 应用层
 * Version            : V1.0
 * Date               : 2026/11/09
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "RF_APP.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
static uint8_t taskID;

static uint8_t rx_data[RF_RX_BUF_SIZE];
static volatile uint8_t rx_len = 0;
static volatile int8_t RSSI = 0;
static volatile rfRxResult_t rx_status = RF_RX_ERR_NONE;

/* RFPhy_Config 配置参数 */
#define RF_CFG_ACCESS_ADDRESS   0x71764129  /* 接入地址：禁止使用0x55555555以及0xAAAAAAAA ( 建议不超过24次位反转，且不超过连续的6个0或1 ) */
#define RF_CFG_CHANNEL          2480000     /* rf channel(0-39) / frequency (2400000kHz-2483500kHz) */
#define RF_CFG_CRCINIT          0x555555    /* crc initial value */
#define RF_CFG_PHY_MODE          RF_PHY_1M   /* 01: PHY 1M / 02: PHY 2M */
#define RF_CFG_WHITENING        RF_WHIT_OFF  /* Whitening 01: On / 02: Off */

/*********************************************************************
 * @fn      RFApp_RxCallback
 *
 * @brief   PHY 层数据上报入口，在 RF 中断上下文中调用。
 *          只拷贝数据并置事件，打印等处理放到任务上下文。
 *
 * @param   buf    - 数据buf指针（RSSI + len + payload）
 * @param   len    - 数据总长度
 * @param   status - 接收结果
 *
 * @return  none
 */
static void RFApp_RxCallback(uint8_t *buf, uint8_t len, int8_t rssi, rfRxResult_t status)
{
    if((status == RF_RX_OK) && (buf != NULL) && (len <= sizeof(rx_data)))
    {
        tmos_memcpy(rx_data, buf, len);
        rx_len = len;
        RSSI = rssi;
    }
    else
    {
        rx_len = 0;
    }
    rx_status = status;
    tmos_set_event(taskID, RF_RX_DEAL_EVT);
}

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
    if(events & RF_RX_DEAL_EVT)
    {
        if((rx_status == RF_RX_OK) && (rx_len >= 0))
        {
            uint8_t i;
            uint8_t plen = rx_len;

            PRINT("rx recv, rssi: %d\n", RSSI);
            PRINT("len:%d-", plen);

            for(i = 0; i < plen; i++)
            {
                PRINT("%x ", rx_data[i]);
            }
            PRINT("\n");
        }
        else if(rx_status == RF_RX_ERR_CRC)
        {
            PRINT("crc error\n");
        }
        else if(rx_status == RF_RX_ERR_TYPE)
        {
            PRINT("match type error\n");
        }
        else if(rx_status == RF_RX_ERR_LEN)
        {
            PRINT("rx len err\n");
        }

        if(RFPhy_StartRx(0xFF, RFApp_RxCallback) != RF_RX_OK)
        {
            PRINT("rx start fail\n");
        }
        return events ^ RF_RX_DEAL_EVT;
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

    PRINT("--------RX--------\n");

    { // RX mode
        if(RFPhy_StartRx(0xFF, RFApp_RxCallback) != RF_RX_OK)
        {
            PRINT("rx start fail\n");
        }
    }
}

/******************************** endfile @ main ******************************/
