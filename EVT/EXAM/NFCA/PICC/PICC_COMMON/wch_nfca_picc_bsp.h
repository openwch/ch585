/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_bsp.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/10/29
 * Description        : nfc picc收发控制底层
 *********************************************************************************
 * Copyright (c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef _WCH_NFCA_PICC_BSP_H_
#define _WCH_NFCA_PICC_BSP_H_

#include "CH58x_common.h"
#include "wch_nfca_picc_config.h"
#include "CH58x_NFCA_LIB.h"

extern uint8_t g_picc_data_buf[PICC_DATA_BUF_LEN];

extern uint8_t g_picc_parity_buf[PICC_DATA_BUF_LEN];

extern void nfca_picc_init(void);

extern void nfca_picc_start(void);

extern void nfca_picc_stop(void);

extern uint32_t nfca_picc_rand(void);

#endif /* _WCH_NFCA_PCD_BSP_H_ */
