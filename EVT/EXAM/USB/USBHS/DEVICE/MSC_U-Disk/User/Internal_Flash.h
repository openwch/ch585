/********************************** (C) COPYRIGHT *******************************
 * File Name          : Internal_Flash.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file for Internal_Flash.c
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef USER_INTERNAL__FLASH_C_
#define USER_INTERNAL__FLASH_C_

#ifdef __cplusplus
 extern "C" {
#endif 

#define INTERNAL_FLASH_PAGE_SIZE  4096
#define IFLASH_UDISK_START_ADDR   0x10000
#define IFLASH_UDISK_END_ADDR     0x20000
#define IFLASH_UDISK_SIZE         (IFLASH_UDISK_END_ADDR - IFLASH_UDISK_START_ADDR + 1 )

extern void IFlash_Prog_4096(uint32_t address,uint32_t *pbuf);

#ifdef __cplusplus
}
#endif

#endif /* USER_INTERNAL__FLASH_C_ */
