/********************************** (C) COPYRIGHT *******************************
 * File Name          : SPI_FLASH.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file for SPI_FLASH.c
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif 

/******************************************************************************/
/* header files */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "CH58x_common.h"
/******************************************************************************/
/* SPI Pins */
#define PIN_FLASH_CS_LOW( )         GPIOA_ResetBits(GPIO_Pin_12)
#define PIN_FLASH_CS_HIGH( )        GPIOA_SetBits(GPIO_Pin_12)

/******************************************************************************/
/* SPI Serial Flash OPERATION INSTRUCTIONS */
#define CMD_FLASH_READ             0x03                                         /* Read Memory at 25 MHz */
#define CMD_FLASH_SECTOR_ERASE     0x20                                         /* Erase 4 KByte of memory array */
#define CMD_FLASH_BYTE_PROG        0x02                                         /* To Program One Data Byte */
#define CMD_FLASH_RDSR             0x05                                         /* Read-Status-Register */
#define CMD_FLASH_EWSR             0x50                                         /* Enable-Write-Status-Register */
#define CMD_FLASH_WREN             0x06                                         /* Write-Enable */
#define CMD_FLASH_WRDI             0x04                                         /* Write-Disable */
#define CMD_FLASH_JEDEC_ID         0x9F                                         /* JEDEC ID read */

/******************************************************************************/
#define DEF_DUMMY_BYTE             0xFF

/******************************************************************************/
/* FLASH Parameter Definition */
#define SPI_FLASH_SectorSize       4096
#define SPI_FLASH_PageSize         256
#define SPI_FLASH_PerWritePageSize 256

/******************************************************************************/
/* SPI FLASH Type */
#define DEF_TYPE_W25XXX            0                                            /* W25XXX */

/******************************************************************************/
/* SPI FLASH Type Define */

/* W25XXX*/
#define W25X10_FLASH_ID            0xEF3011                                     /* 1M bit */
#define W25X20_FLASH_ID            0xEF3012                                     /* 2M bit */
#define W25X40_FLASH_ID            0xEF3013                                     /* 4M bit */
#define W25X80_FLASH_ID            0xEF4014                                     /* 8M bit */
#define W25Q16_FLASH_ID1           0xEF3015                                     /* 16M bit */
#define W25Q16_FLASH_ID2           0xEF4015                                     /* 16M bit */
#define W25Q32_FLASH_ID1           0xEF4016                                     /* 32M bit */
#define W25Q32_FLASH_ID2           0xEF6016                                     /* 32M bit */
#define W25Q64_FLASH_ID1           0xEF4017                                     /* 64M bit */
#define W25Q64_FLASH_ID2           0xEF6017                                     /* 64M bit */
#define W25Q128_FLASH_ID1          0xEF4018                                     /* 128M bit */
#define W25Q128_FLASH_ID2          0xEF6018                                     /* 128M bit */
#define W25Q256_FLASH_ID1          0xEF4019                                     /* 256M bit */
#define W25Q256_FLASH_ID2          0xEF6019                                     /* 256M bit */

/******************************************************************************/
/* Variable Definition */
extern volatile uint8_t  Flash_Type;                                            /* FLASH chip: 0: W25XXX series   */
extern volatile uint32_t Flash_ID;                                              /* FLASH ID */
extern volatile uint32_t Flash_Sector_Count;                                    /* FLASH sector number */
extern volatile uint16_t Flash_Sector_Size;                                     /* FLASH sector size */

/******************************************************************************/
/* external functions */
/**
 * @brief   SPI-Flash related pins and SPI0 initialization
 */
extern void FLASH_Port_Init( void );
/**
 * @brief   send one byte to the SPI-Flash through SPI0
 *
 * @param   byte - the byte to be sent
 *
 * @return  the byte received at the same time
 */
extern uint8_t SPI_FLASH_SendByte( uint8_t byte );
/**
 * @brief   read one byte from the SPI-Flash through SPI0
 *
 * @return  the byte received
 */
extern uint8_t SPI_FLASH_ReadByte( void );
/**
 * @brief   read the JEDEC ID of the SPI-Flash (0x9F)
 *
 * @return  the 24-bit flash ID
 */
extern uint32_t FLASH_ReadID( void );
/**
 * @brief   SPI-Flash write enable (0x06)
 */
extern void FLASH_WriteEnable( void );
/**
 * @brief   SPI-Flash write disable (0x04)
 */
extern void FLASH_WriteDisable( void );
/**
 * @brief   read status register 1 of the SPI-Flash (0x05)
 *
 * @return  the status register value
 */
extern uint8_t FLASH_ReadStatusReg( void );
/**
 * @brief   check the SPI-Flash type, set the U-disk sector number
 */
extern void FLASH_IC_Check( void );
/**
 * @brief   erase one 4K byte sector of the SPI-Flash (0x20)
 *
 * @param   address - sector address
 */
extern void FLASH_Erase_Sector( uint32_t address );
/**
 * @brief   start a continuous block read of the SPI-Flash (0x03)
 *
 * @param   address - start address
 */
extern void FLASH_RD_Block_Start( uint32_t address );
/**
 * @brief   continuous block read, must follow FLASH_RD_Block_Start()
 *
 * @param   pbuf - pointer to the receive buffer
 * @param   len - number of bytes to read
 */
extern void FLASH_RD_Block( uint8_t *pbuf, uint32_t len );
/**
 * @brief   end the continuous block read of the SPI-Flash
 */
extern void FLASH_RD_Block_End( void );
/**
 * @brief   write a block of data to the W25XXX series SPI-Flash
 *
 * @param   pbuf - pointer to the data to be written
 * @param   address - start address
 * @param   len - number of bytes to write
 */
extern void W25XXX_WR_Block( uint8_t *pbuf, uint32_t address, uint32_t len );

#ifdef __cplusplus
}
#endif

#endif
