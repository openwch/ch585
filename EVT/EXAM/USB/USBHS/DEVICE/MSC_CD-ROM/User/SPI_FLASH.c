/********************************** (C) COPYRIGHT *******************************
 * File Name          : SPI_FLAH.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : SPI FLASHChip operation file
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/******************************************************************************/
/* Header Files */
#include <SPI_FLASH.h>
#include <SW_CDROM.h>
#include "CH58x_spi.h"

/******************************************************************************/
volatile uint8_t   Flash_Type = 0x00;                                           /* FLASH chip: 0: W25XXXseries */
volatile uint32_t  Flash_ID = 0x00;                                             /* FLASH ID */
volatile uint32_t  Flash_Sector_Count = 0x00;                                   /* FLASH sector number */
volatile uint16_t  Flash_Sector_Size = 0x00;                                    /* FLASH sector size */

/*******************************************************************************
* Function Name  : FLASH_Port_Init
* Description    : FLASH chip operation related pins and hardware initialization
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_Port_Init( void )
{
    GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    SPI0_MasterDefInit();
}

/*******************************************************************************
 * @fn      SPI0_MasterTransRecv
 *
 * @brief   Continuously send/receive multiple bytes
 *
 * @param   pbuf: The first address of the data content to be sent
 *
 * @return  none
 */
void SPI0_MasterTransRecv(uint8_t *ptbuf, uint8_t *prbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    R16_SPI0_TOTAL_CNT = sendlen;
    R8_SPI0_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    SPI0_ClearITFlag(RB_SPI_IF_CNT_END);

    while (sendlen)
    {
        if (R8_SPI0_FIFO_COUNT == 0)
        {
            R8_SPI0_FIFO = *ptbuf;
            while (R8_SPI0_FIFO_COUNT != 0);
            ptbuf++;
            *prbuf = R8_SPI0_BUFFER;
            prbuf++;
            sendlen--;
        }
    }
}

/*********************************************************************
 * @fn      SPI1_ReadWriteByte
 *
 * @brief   SPI1 read or write one byte.
 *
 * @param   TxData - write one byte data.
 *
 * @return  Read one byte data.
 */
uint8_t SPI0_ReadWriteByte(uint8_t TxData)
{
    uint8_t recdata = 0;
    SPI0_MasterTransRecv(&TxData, &recdata, 1);
    return recdata;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : SPI send a byte
* Input          : byte: byte to send
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t SPI_FLASH_SendByte( uint8_t byte )
{
    return SPI0_ReadWriteByte( byte );
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : SPI receive a byte
* Input          : None
* Output         : None
* Return         : byte received
*******************************************************************************/
uint8_t SPI_FLASH_ReadByte( void )
{
    return SPI0_ReadWriteByte( 0xFF );
}

/*******************************************************************************
* Function Name  : FLASH_ReadID
* Description    : Read FLASH ID
* Input          : None
* Output         : None
* Return         : chip id
*******************************************************************************/
uint32_t FLASH_ReadID( void )
{
    uint32_t dat;
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_JEDEC_ID );
    dat = ( uint32_t )SPI_FLASH_SendByte( DEF_DUMMY_BYTE ) << 16;
    dat |= ( uint32_t )SPI_FLASH_SendByte( DEF_DUMMY_BYTE ) << 8;
    dat |= SPI_FLASH_SendByte( DEF_DUMMY_BYTE );
    PIN_FLASH_CS_HIGH( );

    return( dat );
}

/*******************************************************************************
* Function Name  : FLASH_WriteEnable
* Description    : FLASH Write Enable
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_WriteEnable( void )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_WREN );
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : FLASH_WriteDisable
* Description    : FLASH Write Disable
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_WriteDisable( void )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_WRDI );
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : FLASH_ReadStatusReg
* Description    : FLASH Read Status
* Input          : None
* Output         : None
* Return         : status
*******************************************************************************/
uint8_t FLASH_ReadStatusReg( void )
{
    uint8_t  status;

    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_RDSR );
    status = SPI_FLASH_ReadByte( );
    PIN_FLASH_CS_HIGH( );
    return( status );
}

/*******************************************************************************
* Function Name  : FLASH_Erase_Sector
* Description    : FLASH Erase Sector
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/ 
void FLASH_Erase_Sector( uint32_t address )
{
    uint8_t  temp;
    FLASH_WriteEnable( );
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_SECTOR_ERASE );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
    PIN_FLASH_CS_HIGH( );
    do
    {
        temp = FLASH_ReadStatusReg( );    
    }while( temp & 0x01 );
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block_Start
* Description    : FLASH start block read
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/  
void FLASH_RD_Block_Start( uint32_t address )
{
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_READ );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block
* Description    : FLASH read block
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/  
void FLASH_RD_Block( uint8_t *pbuf, uint32_t len )
{
    while( len-- )                                                             
    {
        *pbuf++ = SPI_FLASH_ReadByte( );
    }
}

/*******************************************************************************
* Function Name  : FLASH_RD_Block_End
* Description    : FLASH end block read
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_RD_Block_End( void )
{
    PIN_FLASH_CS_HIGH( );
}

/*******************************************************************************
* Function Name  : W25XXX_WR_Page
* Description    : Flash page program
* Input          : address
*                  len
*                  *pbuf
* Output         : None
* Return         : None
*******************************************************************************/
void W25XXX_WR_Page( uint8_t *pbuf, uint32_t address, uint32_t len )
{
    uint8_t  temp;
    FLASH_WriteEnable( );
    PIN_FLASH_CS_LOW( );
    SPI_FLASH_SendByte( CMD_FLASH_BYTE_PROG );
    SPI_FLASH_SendByte( (uint8_t)( address >> 16 ) );
    SPI_FLASH_SendByte( (uint8_t)( address >> 8 ) );
    SPI_FLASH_SendByte( (uint8_t)address );
    if( len > SPI_FLASH_PerWritePageSize )
    {
        len = SPI_FLASH_PerWritePageSize;
    }
    while( len-- )
    {
        SPI_FLASH_SendByte( *pbuf++ );
    }
    PIN_FLASH_CS_HIGH( );
    do
    {
        temp = FLASH_ReadStatusReg( );    
    }while( temp & 0x01 );
}

/*******************************************************************************
* Function Name  : W25XXX_WR_Block
* Description    : W25XXX block write 
* Input          : address
*                  len
*                  *pbuf
* Output         : None
* Return         : None
*******************************************************************************/
void W25XXX_WR_Block( uint8_t *pbuf, uint32_t address, uint32_t len )
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = address % SPI_FLASH_PageSize;
    count = SPI_FLASH_PageSize - Addr;
    NumOfPage =  len / SPI_FLASH_PageSize;
    NumOfSingle = len % SPI_FLASH_PageSize;

    if( Addr == 0 )
    {
        if( NumOfPage == 0 )
        {
            W25XXX_WR_Page( pbuf, address, len );
        }
        else
        {
            while( NumOfPage-- )
            {
                W25XXX_WR_Page( pbuf, address, SPI_FLASH_PageSize );
                address +=  SPI_FLASH_PageSize;
                pbuf += SPI_FLASH_PageSize;
            }
            W25XXX_WR_Page( pbuf, address, NumOfSingle );
        }
    }
    else
    {
        if( NumOfPage == 0 )
        {
            if( NumOfSingle > count )
            {
                temp = NumOfSingle - count;
                W25XXX_WR_Page( pbuf, address, count );
                address +=  count;
                pbuf += count;
                W25XXX_WR_Page( pbuf, address, temp );
            }
            else
            {
                W25XXX_WR_Page( pbuf, address, len );
            }
        }
        else
        {
            len -= count;
            NumOfPage =  len / SPI_FLASH_PageSize;
            NumOfSingle = len % SPI_FLASH_PageSize;
            W25XXX_WR_Page( pbuf, address, count );
            address +=  count;
            pbuf += count;
            while( NumOfPage-- )
            {
                W25XXX_WR_Page( pbuf, address, SPI_FLASH_PageSize );
                address += SPI_FLASH_PageSize;
                pbuf += SPI_FLASH_PageSize;
            }
            if( NumOfSingle != 0 )
            {
                W25XXX_WR_Page( pbuf, address, NumOfSingle );
            }
        }
    }
}



/*******************************************************************************
* Function Name  : FLASH_IC_Check
* Description    : check flash type
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IC_Check( void )
{
    uint32_t count;

    /* Read FLASH ID */    
    Flash_ID = FLASH_ReadID( );
    PRINT("Flash_ID: %08x\n",(uint32_t)Flash_ID);

    Flash_Type = 0x00;                                                                
    Flash_Sector_Count = 0x00;
    Flash_Sector_Size = 0x00;

    switch( Flash_ID )
    {
        /* W25XXX */
        case W25X10_FLASH_ID:                                                   /* 0xEF3011-----1M bit */
            count = 1;
            break;

        case W25X20_FLASH_ID:                                                   /* 0xEF3012-----2M bit */
            count = 2;
            break;

        case W25X40_FLASH_ID:                                                   /* 0xEF3013-----4M bit */
            count = 4;
            break;

        case W25X80_FLASH_ID:                                                   /* 0xEF4014-----8M bit */
            count = 8;
            break;

        case W25Q16_FLASH_ID1:                                                  /* 0xEF3015-----16M bit */
        case W25Q16_FLASH_ID2:                                                  /* 0xEF4015-----16M bit */
            count = 16;
            break;

        case W25Q32_FLASH_ID1:                                                  /* 0xEF4016-----32M bit */
        case W25Q32_FLASH_ID2:                                                  /* 0xEF6016-----32M bit */
            count = 32;
            break;

        case W25Q64_FLASH_ID1:                                                  /* 0xEF4017-----64M bit */
        case W25Q64_FLASH_ID2:                                                  /* 0xEF6017-----64M bit */
            count = 64;
            break;

        case W25Q128_FLASH_ID1:                                                 /* 0xEF4018-----128M bit */
        case W25Q128_FLASH_ID2:                                                 /* 0xEF6018-----128M bit */
            count = 128;
            break;

        case W25Q256_FLASH_ID1:                                                 /* 0xEF4019-----256M bit */
        case W25Q256_FLASH_ID2:                                                 /* 0xEF6019-----256M bit */
            count = 256;
            break;
        default:
            if( ( Flash_ID != 0xFFFFFFFF ) && ( Flash_ID != 0x00000000 ) )
            {
                count = 16;
            }
            else
            {
                count = 0x00;
            }
            break;
    }
    count = ( (uint32_t)count * 1024 ) * ( (uint32_t)1024 / 8 );

    if( count )
    {
        Flash_Sector_Count = count / DEF_CDROM_SECTOR_SIZE;
        Flash_Sector_Size = DEF_CDROM_SECTOR_SIZE;
    }
    else
    {
        PRINT("External Flash not connected\r\n");
        while(1);
    }
}

