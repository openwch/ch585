/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidkbd.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/10
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef HIDKBD_H
#define HIDKBD_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Task Events
#define START_DEVICE_EVT                0x0001
#define START_PARAM_UPDATE_EVT          0x0002
#define START_PHY_UPDATE_EVT            0x0004
#define START_SPECIAL_KEY_TIMEOUT_EVT   0x0008
#define START_TOUCH_KEY_PROCESS_EVT     0x0010

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern volatile uint8_t bleConnectState; // 0:没有连接; 1:已连接
extern volatile uint8_t advState; // 0:停止广播; 1:正在广播

/*
 * Task Initialization for the BLE Application
 */
extern void HidEmu_Init(void);

/*
 * Task Event Processor for the BLE Application
 */
extern uint16_t HidEmu_ProcessEvent(uint8_t task_id, uint16_t events);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
