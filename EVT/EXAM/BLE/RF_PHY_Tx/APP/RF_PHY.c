/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_PHY.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/09
 * Description        : RF_PHY驱动
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
volatile uint8_t gTx_state = 0;

static rfRxCallback_t sRxCb = NULL;                         /* 当前绑定的接收数据回调，由 RFPhy_StartRx 绑定 */             
static void RFPhy_StatusCallback(uint8_t sta, uint8_t crc, uint8_t *rxBuf);

/*********************************************************************
 * @fn      RFPhy_Config
 *
 * @brief   配置并初始化 2.4G RF，channel 0-39 为 BLE 信道，
 *          >=40 视为频率 kHz 走 EX_CHANNEL 模式
 *
 * @return  none
 */
void RFPhy_Config(uint32_t accessAddress, uint32_t channel, uint32_t crcInit, uint8_t phyMode, uint8_t whitening)
{
    uint8_t    state;
    rfConfig_t rf_Config;

    tmos_memset(&rf_Config, 0, sizeof(rfConfig_t));
    rf_Config.accessAddress = accessAddress;
    rf_Config.CRCInit = crcInit;
    if(phyMode == RF_PHY_2M) rf_Config.LLEMode |= LLE_MODE_PHY_2M;
    else  rf_Config.LLEMode &= ~LLE_MODE_PHY_2M;
    if(whitening == RF_WHIT_ON) rf_Config.LLEMode &= ~LLE_WHITENING_OFF;
    else rf_Config.LLEMode |= LLE_WHITENING_OFF;
    if(channel < 40)
    {
       rf_Config.Channel = channel;
       rf_Config.LLEMode &= ~LLE_MODE_EX_CHANNEL;
    }
    else
    {
        rf_Config.Frequency = channel;
        rf_Config.LLEMode |= LLE_MODE_BASIC | LLE_MODE_EX_CHANNEL; // 使能 LLE_MODE_EX_CHANNEL 表示 选择 rf_Config.Frequency 作为通信频点
    }
    rf_Config.rfStatusCB = RFPhy_StatusCallback;
    rf_Config.RxMaxlen = 251;
    state = RF_Config(&rf_Config);
    PRINT("rf 2.4g init: %x\n", state);

    /* 打印传入的配置参数 */
    PRINT("\n==============================================\n");
    PRINT("  addr     : %08x\n", accessAddress);
    PRINT("  channel  : %d\n", channel);
    PRINT("  crcInit  : %06x\n", crcInit);
    PRINT("  phyMode  : %d\n", phyMode);
    PRINT("  whiten   : %d\n", whitening);
    PRINT("==============================================\n");
    gTx_state = RF_TX_STATE_IDLE;
}

/*********************************************************************
 * @fn      RFPhy_StatusCallback
 *
 * @brief   RF 状态回调，此函数在中断中调用。注意：不可在此函数中直接调用RF接收或者发送API，需要使用事件的方式调用
 *          在此回调中直接使用或调用函数涉及到的变量需注意，此函数在中断中调用。
 *
 * @param   sta     - 状态类型
 * @param   crc     - crc校验结果
 * @param   rxBuf   - 数据buf指针
 *
 * @return  none
 */
static void RFPhy_StatusCallback(uint8_t sta, uint8_t crc, uint8_t *rxBuf)
{
    uint8_t len;
    int8_t rssi;
    switch(sta)
    {
        case TX_MODE_TX_FINISH:
        {
            gTx_state = RF_TX_STATE_FINISH;
            break;
        }
        case TX_MODE_TX_TIMEOUT:
        {
            gTx_state = RF_TX_STATE_TIMEOUT;
            break;
        }

        case RX_MODE_RX_DATA:
            if(crc == 0)
            {
                rssi = rxBuf[0];
                len = rxBuf[1];
                if((len + 2U) <= RF_RX_BUF_SIZE)
                {
                    if(sRxCb)  sRxCb(rxBuf+2, len, rssi, RF_RX_OK);
                }
                else
                {
                    if(sRxCb)  sRxCb(NULL, 0, 0, RF_RX_ERR_LEN);
                }
            }
            else
            {
                if(sRxCb)  sRxCb(NULL, 0, 0, (crc & (1 << 1)) ? RF_RX_ERR_TYPE : RF_RX_ERR_CRC);
            }
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      RFPhy_WaitTxDone
 *
 * @brief   手动模式等待发送完成，自动模式等待发送-接收完成，必须在RAM中等待，等待时可以执行用户代码，但需要注意执行的代码必须运行在RAM中，否则影响发送
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline))
static void RFPhy_WaitTxDone(void)
{
    uint32_t i=0;
    while(gTx_state == RF_TX_STATE_SENDING)
    {
        i++;
        __nop();
        __nop();
        // 约5ms超时
        if(i>(FREQ_SYS/1000))
        {
            gTx_state = RF_TX_STATE_TIMEOUT;
        }
    }
}

/*********************************************************************
 * @fn      RFPhy_SendPacket
 *
 * @brief   启动一次驱动发送。
 *
 * @param   none.
 *
 * @return  none.
 */
rfTxResult_t RFPhy_SendPacket(uint8_t *buf, uint8_t len, uint8_t pktType)
{
    bStatus_t ret;

    if((buf == NULL) || (len == 0) || (len > RF_TX_MAX_LEN))
    {
        return RF_TX_ERR_PARAM;     /* 参数异常 */
    }

    RF_Shut();
    gTx_state = RF_TX_STATE_SENDING;
    ret = RF_Tx(buf, len, pktType, 0xFF);
    if(ret != SUCCESS)
    {
        gTx_state = RF_TX_STATE_IDLE;
        return (ret == RF_RET_ADDR_ERR) ? RF_TX_ERR_ADDR : RF_TX_ERR_BUSY;
    }

    RFPhy_WaitTxDone();
    if(gTx_state == RF_TX_STATE_TIMEOUT)  return RF_TX_ERR_FAIL;

    return RF_TX_OK;
}


/*********************************************************************
 * @fn      RFPhy_StartRx
 *
 * @brief   启动一次接收并绑定本次数据回调。返回 RF_RX_OK 只说明接收已启动，
 *          收到数据或出错时在 RF 中断上下文中回调 cb。
 *
 * @param   pktRxType - 接收包类型匹配，0xFF 收所有匹配类型
 * @param   cb        - 本次接收绑定的数据回调，NULL 表示本次不上报
 *
 * @return  RF_RX_OK - 已启动；其他 - 未启动成功
 */
rfRxResult_t RFPhy_StartRx(uint8_t pktType, rfRxCallback_t cb)
{
    bStatus_t ret;

    RF_Shut();
    sRxCb = cb;                     /* 绑定本次接收的处理函数 */

    ret = RF_Rx(NULL, 0, pktType, 0xFF);
    if(ret != SUCCESS)
    {
        return (ret == 1) ? RF_RX_ERR_ADDR : RF_RX_ERR_BUSY;
    }

    return RF_RX_OK;
}


/******************************** endfile @ rf ******************************/
