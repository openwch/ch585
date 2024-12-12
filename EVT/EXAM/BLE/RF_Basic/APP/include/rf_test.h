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


#define RF_TEST_SLEEP_EVENT      (1<<0)
#define RF_START_RX_EVENT        (1<<1)

#define RF_STOP_BOUND_EVENT      (1<<11)
#define RF_START_BOUND_EVENT     (1<<12)

#define RF_TEST_RX_EVENT         (1<<13)
#define RF_TEST_TX_EVENT         (1<<14)


#define  PHY_MODE      LLE_MODE_PHY_2M


typedef struct
{
    uint32_t errContinue;
    uint32_t errCount;
    uint32_t txCount;
    uint32_t rxCount;
    uint8_t testCount;
    uint8_t testData;
    int8_t  rssi;
    int8_t  rssiMax;
    int8_t  rssiMin;
    uint8_t boundEst;
    uint8_t boundConnect;
} testBound_t;

void RFRole_Init(void);
void RF_LowPower( uint32_t time);
void RF_ProcessRx( void );

#ifdef __cplusplus
}
#endif

#endif
