/********************************** (C) COPYRIGHT *******************************
 * File Name          : nvs_flash.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/11/03
 * Description        : 参数存储（flash）：固定地址整块擦写+校验和
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CONFIG.h"
#include "HAL.h"
#include "nvs_flash.h"
#include "key_report.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
/* 4字节对齐：EEPROM_WRITE/READ要求缓冲区4字节对齐 */
nvs_flash_info_t nvs_flash_info __attribute__((aligned(4)));

/*********************************************************************
 * @fn      nvs_flash_padding_check_sum
 *
 * @brief   计算校验和并填入结构体（前sizeof-1字节累加和）
 *
 * @return  none
 */
static void nvs_flash_padding_check_sum(void)
{
    uint8_t i;
    uint8_t *p;
    p = (uint8_t *)&nvs_flash_info;
    nvs_flash_info.check_sum = 0;
    for(i = 0; i < sizeof(nvs_flash_info_t) - 1; i++)
    {
        nvs_flash_info.check_sum += p[i];
    }
}

/*********************************************************************
 * @fn      nvs_flash_check_check_sum
 *
 * @brief   校验结构体的校验和
 *
 * @return  TRUE - 校验通过  FALSE - 校验失败
 */
static uint8_t nvs_flash_check_check_sum(void)
{
    uint8_t i;
    uint8_t *p;
    uint8_t temp = 0;
    p = (uint8_t *)&nvs_flash_info;
    for(i = 0; i < sizeof(nvs_flash_info_t) - 1; i++)
    {
        temp += p[i];
    }
    if(temp != nvs_flash_info.check_sum)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/*********************************************************************
 * @fn      nvs_flash_store
 *
 * @brief   保存参数到flash：填校验和 -> 整块擦除 -> 写入
 *
 * @return  SUCCESS - 成功  !SUCCESS - 失败
 */
uint8_t nvs_flash_store(void)
{
    nvs_flash_padding_check_sum();

    if(EEPROM_ERASE(NVS_FLASH_INFO_ADDRESS, NVS_FLASH_ERASE_SIZE) != 0)
    {
        return !SUCCESS;
    }
    if(EEPROM_WRITE(NVS_FLASH_INFO_ADDRESS, &nvs_flash_info, sizeof(nvs_flash_info_t)) != 0)
    {
        return !SUCCESS;
    }
    return SUCCESS;
}

/*********************************************************************
 * @fn      nvs_flash_init
 *
 * @brief   初始化其他功能之前调用，确保使用的是flash中的数据；
 *          读取失败（校验标志位/校验和不符）时装载默认参数并写回flash
 *
 * @return  none
 */
void nvs_flash_init(void)
{
    EEPROM_READ(NVS_FLASH_INFO_ADDRESS, &nvs_flash_info, sizeof(nvs_flash_info_t));
    if((nvs_flash_info.check_val_A != CHECK_VAL_A) || (nvs_flash_info.check_val_B != CHECK_VAL_B) ||
            (nvs_flash_check_check_sum() == FALSE))
    {
        /* flash内无有效参数：使用默认参数（与KEY_CAMERA_ANDROID初始值一致）并写回flash */
        nvs_flash_info.check_val_A = CHECK_VAL_A;
        nvs_flash_info.check_val_B = CHECK_VAL_B;
        nvs_flash_info.left_pointer[0] = KEY_CAMERA_ANDROID[1];
        nvs_flash_info.left_pointer[1] = KEY_CAMERA_ANDROID[2];
        nvs_flash_info.down_pointer[0] = KEY_CAMERA_ANDROID[3];
        nvs_flash_info.down_pointer[1] = KEY_CAMERA_ANDROID[4];
        nvs_flash_store();
        PRINT("nvs_flash init: use default\n");
    }
    else
    {
        PRINT("nvs_flash init: read ok\n");
    }
}

/******************************** endfile @ nvs_flash ******************************/
