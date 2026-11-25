/********************************** (C) COPYRIGHT *******************************
 * File Name          : hidmobilecontroller.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器应用头文件
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef HIDMOBILECONTROLLER_H
#define HIDMOBILECONTROLLER_H


#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
#include "CH58x_common.h"

/*********************************************************************
 * CONSTANTS
 */

// Task Events
#define HIDEMU_START_DEVICE_EVT            0x0001
#define START_REPORT_EVT                   0x0002
#define START_PARAM_UPDATE_EVT             0x0004
#define KEY_SCAN_EVT                       0x0008
#define START_SLEEP_EVT                    0x0010
#define SAVE_AN_CONFIG_EVT                 0x0020
#define LED_BLINK_EVT                      0x0080
/*********************************************************************
 * MACROS
 */
enum {
    REMOTE_KEY_UP = 0,
    REMOTE_KEY_DOWN,
    REMOTE_KEY_LEFT,
    REMOTE_KEY_RIGHT,
    REMOTE_KEY_MIDDLE,
    REMOTE_KEY_PHOTO,
    REMOTE_KEY_MAX,
};
/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * Task ID
 */
extern uint8_t hidEmuTaskId;

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


#endif /* HIDMOBILECONTROLLER_H */
