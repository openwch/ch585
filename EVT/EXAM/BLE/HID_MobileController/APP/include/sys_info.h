/********************************** (C) COPYRIGHT *******************************
 * File Name          : sys_info.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器配置信息存储
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef SYS_INFO_H
#define SYS_INFO_H

/*********************************************************************
 * INCLUDES
 */
#include "CH58x_common.h"

/*********************************************************************
 * TYPEDEFS
 */
/* 拍照按键坐标配置 */
typedef struct an_camera_pointer_struct {
    uint8_t left_pointer[2];    /* 水平方向坐标（左右键调节） */
    uint8_t down_pointer[2];    /* 垂直方向坐标（上下键调节） */
} __attribute__((__packed__)) an_camera_pointer_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern an_camera_pointer_t g_an_camera_pointer;

/*********************************************************************
 * FUNCTIONS
 */
extern void sys_info_init(void);

extern void save_android_config(void);

#endif /* SYS_INFO_H */
