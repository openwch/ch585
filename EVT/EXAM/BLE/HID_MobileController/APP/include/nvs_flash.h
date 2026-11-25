/********************************** (C) COPYRIGHT *******************************
 * File Name          : nvs_flash.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/03
 * Description        : 参数存储（flash）头文件：固定地址整块擦写+校验和，无磨损均衡
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#ifndef NVS_FLASH_H
#define NVS_FLASH_H

#include "CONFIG.h"

#ifndef SUCCESS
#define SUCCESS	0
#endif

/*********************************************************************
 * CONSTANTS
 */
#define NVS_FLASH_INFO_ADDRESS     0x2000
#define NVS_FLASH_ERASE_SIZE       0x1000

#define CHECK_VAL_A                0x5A
#define CHECK_VAL_B                0xA5

/*********************************************************************
 * TYPEDEFS
 */
typedef struct __PACKED
{
    uint8_t   check_val_A;
    uint8_t   check_val_B;
    uint8_t   left_pointer[2];      /* 拍照按键水平坐标（第1字节为低字节） */
    uint8_t   down_pointer[2];      /* 拍照按键垂直坐标（第1字节为低字节） */
    uint8_t   check_sum;
} nvs_flash_info_t;

/*********************************************************************
 * Global Variables
 */
/* 4字节对齐：EEPROM_WRITE/READ要求缓冲区4字节对齐 */
extern nvs_flash_info_t nvs_flash_info;

/*********************************************************************
 * FUNCTIONS
 */
void nvs_flash_init(void);

uint8_t nvs_flash_store(void);

/*********************************************************************
*********************************************************************/

#endif
