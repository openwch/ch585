/********************************** (C) COPYRIGHT *******************************
* File Name          : rf_test.h
* Author             : WCH
* Version            : V1.0
* Date               : 2022/03/10
* Description        : 
*******************************************************************************/

#ifndef __RF_TEST_H
#define __RF_TEST_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "wchrf.h"
#include "CH58x_common.h"
#include "RingMem.h"

#define RF_TEST_SLEEP_EVENT      (1<<0)
#define RF_START_RX_EVENT        (1<<1)

#define RF_STOP_BOUND_EVENT      (1<<11)
#define RF_START_BOUND_EVENT     (1<<12)

#define RF_TEST_RX_EVENT         (1<<13)
#define RF_TEST_TX_EVENT         (1<<14)

#define  PHY_MODE      LLE_MODE_PHY_2M

#define  R32_PA_CTRL      (*((PUINT32V)0x4000D05C))
#define  R32_LNA_CTRL1    (*((PUINT32V)0x4000D048))
#define  R32_LNA_CTRL2    (*((PUINT32V)0x4000D04C))

#define  UART_REC_LEN               10u
#define  DTM_CMD_END_TEST           {0x01, 0x1F, 0x20, 0x00}
#define  DTM_CMD_RESET              {0x01, 0x03, 0x0C, 0x00}
#define  DTM_CMD_RX_1M              {0x01, 0x1D, 0x20, 0x01}
#define  DTM_CMD_TX_1M              {0x01, 0x1E, 0x20, 0x03}
#define  DTM_CMD_RX_2M              {0x01, 0x33}
#define  DTM_CMD_TX_2M              {0x01, 0x34}
#define  DTM_CMD_SET_POWER          {0x01, 0x01, 0xFF, 0x01}
#define  DTM_CMD_SET_PF             {0x01, 0x02, 0xFF, 0x01}
#define  DTM_CMD_SET_PA             {0x01, 0x11, 0xFF, 0x03}
#define  DTM_CMD_SET_LNA            {0x01, 0x12, 0xFF, 0x03}
#define  DTM_CMD_SINGLE_CARRIER     {0x01, 0x03, 0xFF, 0x01}

#define  DTM_CMD_BUF_LEN            23u
#define  TX_WAITTIME                (20u * 2u)
#define  DTM_AA_VAL                 0x71764129
#define  DTM_CRC_INIT_VAL           0x555555

#define SRG_CAST(a) ( (*((volatile unsigned long *)(a))) )

extern uint8_t usbRingMemBuff[1024];
extern RingMemParm usbRingParm;

typedef enum {
    dtm_tx = 0,
    dtm_rx,
    dtm_init = 0xff
}DTM_MODE_DEF;

void RFRole_Init(void);
uint8_t Choose_CH( uint8_t cch );
void UART_Process_Data(void);
void PRBS9_Get( uint8_t *pData, uint16_t len );
void PRBS15_Get( uint8_t *pData, uint16_t len );
void TX_DATA( uint8_t* buf, uint8_t len );
void DtmProcess(void);

#ifdef __cplusplus
}
#endif

#endif
