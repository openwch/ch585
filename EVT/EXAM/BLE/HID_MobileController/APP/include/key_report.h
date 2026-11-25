/********************************** (C) COPYRIGHT *******************************
 * File Name          : key_report.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器HID报文数据
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef KEY_REPORT_H
#define KEY_REPORT_H

/*********************************************************************
 * INCLUDES
 */
#include "CH58x_common.h"

/*********************************************************************
 * CONSTANTS
 */
/* 安卓单点触摸报文第一个字节：按下/松开 */
#define ANDROID_FINGER_DOWN     0x07
#define ANDROID_FINGER_UP       0x00
//#define ANDROID_FINGER_MOVE     0x05

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern const uint8_t KEY_UP_ANDROID[81];
extern const uint8_t KEY_DOWN_ANDROID[81];
extern const uint8_t KEY_RIGHT_ANDROID[90];
extern const uint8_t KEY_LEFT_ANDROID[90];

extern uint8_t KEY_MIDDLE_ANDROID[9];
extern uint8_t KEY_CAMERA_ANDROID[9];

/* 音量加减报文 */
extern const uint8_t KEY_VOLUME[6];

#endif /* KEY_REPORT_H */
