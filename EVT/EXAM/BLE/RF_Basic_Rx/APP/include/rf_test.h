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

void RFRole_Init(void);

#ifdef __cplusplus
}
#endif

#endif
