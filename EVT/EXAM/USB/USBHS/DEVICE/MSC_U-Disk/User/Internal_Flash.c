/********************************** (C) COPYRIGHT *******************************
 * File Name          : Internal_Flash.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : Internal Flash program
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "CH58x_common.h"
#include "Internal_Flash.h"
#include "ISP585.h"

void IFlash_Prog_4096(uint32_t address,uint32_t *pbuf)
{
    if (address < IFLASH_UDISK_START_ADDR || (address + (INTERNAL_FLASH_PAGE_SIZE - 1)) > IFLASH_UDISK_END_ADDR )
    {
        PRINT("Error Address %x\n",address);
        return;
    }
    address &= 0x00FFFFFF;
    PFIC_DisableAllIRQ();
    FLASH_ROM_ERASE(address, INTERNAL_FLASH_PAGE_SIZE);
    FLASH_ROM_WRITE(address, (uint8_t*)pbuf, INTERNAL_FLASH_PAGE_SIZE);
    PFIC_EnableAllIRQ();
}
