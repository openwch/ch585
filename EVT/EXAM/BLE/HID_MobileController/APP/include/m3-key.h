/********************************** (C) COPYRIGHT *******************************
 * File Name          : m3-key.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器按键扫描
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#ifndef M3_KEY_H
#define M3_KEY_H

/*********************************************************************
 * INCLUDES
 */
#include "CH58x_common.h"
#include "CONFIG.h"
#include "RingMem.h"
#include "key_report.h"

/*********************************************************************
 * CONSTANTS
 */
/* 按键事件编码，低4位：0-松开，1-按下，2-长按 */
enum
{
    UP_KEY_UP = 0xd0,
    UP_KEY_DOWN = 0xd1,
    UP_KEY_LONG = 0xd2,

    DOWN_KEY_UP = 0x40,
    DOWN_KEY_DOWN = 0x41,
    DOWN_KEY_LONG = 0x42,

    LEFT_KEY_UP = 0xe0,
    LEFT_KEY_DOWN = 0xe1,

    RIGHT_KEY_UP = 0xc0,
    RIGHT_KEY_DOWN = 0xc1,

    MIDDLE_KEY_UP = 0xf0,
    MIDDLE_KEY_DOWN = 0xf1,

    CAMERA_KEY_UP = 0x50,
    CAMERA_KEY_DOWN = 0x51,
};

/* 按键消抖时间（扫描次数），扫描周期为BTN_CHECK_PERIOD毫秒 */
#define KEY_FILTER_TIME                     2

/* 长按时间与连发速度（扫描次数） */
#define KEY_LONG_TIME                       (2000 / BTN_CHECK_PERIOD)   /* 2s判定长按 */
#define KEY_REPEAT_SPEED                    (500 / BTN_CHECK_PERIOD)    /* 长按后0.5s连发一次 */

/* 按键控制结构 */
typedef struct
{
    uint8_t  State;         /* 按键状态：0松开 1按下 2长按 */
    uint16_t Count;         /* 消抖计数 */
    uint16_t LongCount;     /* 长按计数 */
    uint16_t LongTime;      /* 长按阈值，0=禁用长按 */
    uint16_t RepeatCount;   /* 连发计数 */
    uint16_t RepeatSpeed;   /* 连发周期，0=禁用连发 */
    uint16_t PinIndex;      /* GPIOA引脚号(0~15)，低电平表示按下 */
    uint8_t  DownCode;      /* 按下事件码 */
    uint8_t  UpCode;        /* 松开事件码 */
    uint8_t  LongCode;      /* 长按事件码 */
} KEY_T;

/*********************************************************************
 * GLOBAL VARIABLES
 */
/* 按键事件环形缓冲区（RingMem，已用空间见CurrentLen） */
extern RingMemParm_t m3_keys_ringmem;

/*********************************************************************
 * FUNCTIONS
 */
extern void m3_key_init(void);

extern void m3_key_start(void);

extern void m3_key_stop(void);

/* 按键检测扫描，需每个BTN_CHECK_PERIOD毫秒周期调用 */
extern void bsp_key_scan(void);

/* 键值放入按键FIFO */
extern void bsp_put_key(uint8_t code);

extern uint8_t is_any_btn_down(void);

#endif /* M3_KEY_H */
