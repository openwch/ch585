/********************************** (C) COPYRIGHT *******************************
 * File Name          : sys_info.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器配置信息（拍照按键坐标）
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "sys_info.h"
#include "key_report.h"
#include "nvs_flash.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
an_camera_pointer_t g_an_camera_pointer;

/*********************************************************************
 * @fn      sys_info_init
 *
 * @brief   配置信息初始化：从flash读取拍照按键坐标配置，读取失败时nvs_flash内部装载默认值；
 *          参数范围非法时同样回退默认值
 *
 * @param   none
 *
 * @return  none
 */
void sys_info_init(void)
{
    /* 从flash读取配置（校验标志位/校验和不符时自动装载默认值并写回flash） */
    nvs_flash_init();

    /* 点位不合法（含全 0xff 的空白 flash），使用默认参数 */
    if(((nvs_flash_info.left_pointer[0] % 0x20) != 0x15) || (nvs_flash_info.left_pointer[1] > 3) ||
            ((nvs_flash_info.down_pointer[0] % 0x20) != 0x10) || (nvs_flash_info.down_pointer[1] > 3))
    {
        PRINT("android x/y config err, use default\n");
        nvs_flash_info.left_pointer[0] = KEY_CAMERA_ANDROID[1];
        nvs_flash_info.left_pointer[1] = KEY_CAMERA_ANDROID[2];
        nvs_flash_info.down_pointer[0] = KEY_CAMERA_ANDROID[3];
        nvs_flash_info.down_pointer[1] = KEY_CAMERA_ANDROID[4];
        nvs_flash_store();
    }

    /* 同步到RAM缓存（应用层调节/上报使用） */
    g_an_camera_pointer.left_pointer[0] = nvs_flash_info.left_pointer[0];
    g_an_camera_pointer.left_pointer[1] = nvs_flash_info.left_pointer[1];
    g_an_camera_pointer.down_pointer[0] = nvs_flash_info.down_pointer[0];
    g_an_camera_pointer.down_pointer[1] = nvs_flash_info.down_pointer[1];

    /* 同步到HID上报报文 */
    KEY_CAMERA_ANDROID[1] = nvs_flash_info.left_pointer[0];
    KEY_CAMERA_ANDROID[2] = nvs_flash_info.left_pointer[1];
    KEY_CAMERA_ANDROID[3] = nvs_flash_info.down_pointer[0];
    KEY_CAMERA_ANDROID[4] = nvs_flash_info.down_pointer[1];
}

/*********************************************************************
 * @fn      save_android_config
 *
 * @brief   保存拍照按键坐标配置到flash（固定地址整块擦写+校验和）
 *
 * @param   none
 *
 * @return  none
 */
void save_android_config(void)
{
    nvs_flash_info.left_pointer[0] = g_an_camera_pointer.left_pointer[0];
    nvs_flash_info.left_pointer[1] = g_an_camera_pointer.left_pointer[1];
    nvs_flash_info.down_pointer[0] = g_an_camera_pointer.down_pointer[0];
    nvs_flash_info.down_pointer[1] = g_an_camera_pointer.down_pointer[1];

    if(nvs_flash_store() == SUCCESS)
    {
        PRINT("an save OK\n");
    }
    else
    {
        PRINT("an save Err\n");
    }
}
