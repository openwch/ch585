/********************************** (C) COPYRIGHT *******************************
 * File Name          : RF_APP.h
 * Description        : RF_PHY 应用 API 层（BASIC 单收模式）
 * Version            : V1.0
 * Date               : 2026/11/09
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef RF_PHY_API_H
#define RF_PHY_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RF_PHY.h"

#define RF_RX_DEAL_EVT         1

extern void RFApp_Init(void);

#ifdef __cplusplus
}
#endif

#endif
