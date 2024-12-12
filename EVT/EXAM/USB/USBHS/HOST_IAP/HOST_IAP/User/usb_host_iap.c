/********************************** (C) COPYRIGHT  *******************************
 * File Name          : usb_host_iap.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : IAP
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "usb_host_iap.h"

#define    jumpApp   ((  void  (*)  ( void ))  ((int*)DEF_APP_CODE_START_ADDR))
/* Variable */
__attribute__((aligned(4))) uint8_t  IAPLoadBuffer[ DEF_MAX_IAP_BUFFER_LEN ];
__attribute__((aligned(4))) uint8_t  Com_Buffer[ DEF_COM_BUF_LEN ];     // even address , used for host enumcation and udisk operation
__attribute__((aligned(4))) uint8_t  DevDesc_Buf[ 18 ];                 // Device Descriptor Buffer
volatile   uint32_t  IAP_Load_Addr_Offset;
volatile   uint32_t  IAP_WriteIn_Length;
volatile   uint32_t  IAP_WriteIn_Count;
struct   _ROOT_HUB_DEVICE RootHubDev[ DEF_TOTAL_ROOT_HUB ];
struct   __HOST_CTL HostCtl[ DEF_TOTAL_ROOT_HUB * DEF_ONE_USB_SUP_DEV_TOTAL ];

/* Verify Code */
const uint8_t Verify_Code[ DEF_VERIFY_CODE_LEN ] =
{ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

/* Flash Operation Key */
volatile uint32_t Flash_Operation_Key0;
volatile uint32_t Flash_Operation_Key1;

/*********************************************************************
 * @fn      FLASH_ReadByte
 *
 * @brief   Read Flash In byte(8 bits)
 *
 * @return  8bits data readout
 */
uint8_t FLASH_ReadByte(uint32_t address)
{
    uint32_t data=0;
    FLASH_ROM_READ(address,&data,1);
    return (uint8_t)data;
}

/*********************************************************************
 * @fn      FLASH_ReadWord
 *
 * @brief   Read Flash In byte(8 bits)
 *
 * @return  8bits data readout
 */
uint32_t FLASH_ReadWord(uint32_t address)
{
    uint32_t data=0;
    FLASH_ROM_READ(address,&data,1);
    return data;
}

/*********************************************************************
 * @fn      IAP_Flash_Read
 *
 * @brief   Read Flash In bytes(8 bits),Specify length & address,
 *          With address protection and program runaway protection.
 *
 * @return  0: Operation Success
 *          See notes for other errors
 */
uint8_t IAP_Flash_Read(uint32_t address, uint8_t* buff, uint32_t length)
{
    uint32_t i;
    uint32_t read_addr;
    uint32_t read_len;
    read_addr = address;
    read_len = length;

    /* Verify Keys, No flash operation if keys are not correct */
    if((Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0) || (Flash_Operation_Key1 != DEF_FLASH_OPERATION_KEY_CODE_1))
    {
        /* ERR: Risk of code running away */
        return 0xFF;
    }

    /* Verify Address, No flash operation if the address is out of range */
    if(((read_addr >= DEF_APP_CODE_START_ADDR) && (read_addr <= DEF_APP_CODE_END_ADDR)) || ((read_addr >= DEF_VERIFY_CODE_START_ADDR) && (read_addr <= DEF_VERIFY_CODE_END_ADDR)))
    {
        /* Available Data */
        for(i = 0; i < read_len; i++)
        {
            buff[i] = *(__IO uint8_t*)read_addr;//Read one word of data at the specified address
            read_addr++;
        }
#if 0
        PRINT("i        %d\n", i);
        PRINT("read_len %d\n", read_len);
#endif
        if(i != read_len)
        {
            /* Incorrect read length */
            return 0xFD;
        }
    }
    else
    {
        /* address Out Of Range */
        return 0xFE;
    }

    return 0;
}

/*********************************************************************
 * @fn      mFLASH_ProgramPage_Fast
 *
 * @brief   Fast programming, input variable addresses and data buffers,
 *          no longer than one page at a time.
 *          The content of the functions can be written according to the
 *          chip flash fast programming process by yourself
 *
 * @return  0: Operation Success
 *          See notes for other errors
 */
uint8_t mFLASH_ProgramPage_Fast(uint32_t addr, uint32_t* buffer)
{
    /* Modify the code here according to the flash programming process of the chip */
    FLASH_ROM_WRITE(addr, (uint8_t*)buffer, DEF_FLASH_PAGE_SIZE);
    return 0;
}

/*********************************************************************
 * @fn      IAP_Flash_Write
 *
 * @brief   Write Flash In bytes(8 bits),Specify length & address,
 *          Based On Fast Flash Operation,
 *          With address protection and program runaway protection.
 *
 * @return  0: Operation Success
 *          See notes for other errors
 */
uint8_t IAP_Flash_Write(uint32_t address, uint8_t* buff, uint32_t length)
{
    uint32_t i, j;
    uint32_t write_addr;
    uint32_t write_len;
    uint16_t write_len_once;
    uint32_t write_cnts;
    volatile uint32_t page_cnts;
    volatile uint32_t page_addr;
    uint8_t temp_buf[DEF_FLASH_PAGE_SIZE];

    /* Set initial value */
    write_addr = address;
    page_addr = address;
    write_len = length;
    if((write_len % DEF_FLASH_PAGE_SIZE) == 0)
    {
        page_cnts = write_len / DEF_FLASH_PAGE_SIZE;
    }
    else
    {
        page_cnts = (write_len / DEF_FLASH_PAGE_SIZE) + 1;
    }
    write_cnts = 0;

    /* Verify Keys, No flash operation if keys are not correct */
    if((Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0) || (Flash_Operation_Key1 != DEF_FLASH_OPERATION_KEY_CODE_1))
    {
        /* ERR: Risk of code running away */
        return 0xFF;
    }

    /* Verify Address, No flash operation if the address is out of range */
    if(((write_addr >= DEF_APP_CODE_START_ADDR) && (write_addr <= DEF_APP_CODE_END_ADDR)) || ((write_addr >= DEF_VERIFY_CODE_START_ADDR) && (write_addr <= DEF_VERIFY_CODE_END_ADDR)))
    {
        for(i = 0; i < page_cnts; i++)
        {
            /* Verify Keys, No flash operation if keys are not correct */
            if(Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0)
            {
                /* ERR: Risk of code running away */
                return 0xFF;
            }
            /* Determine if the length of the written packet is less than the page size */
            /* If it is less than, the original content of the memory needs to be read out first and modified before the operation can be performed */
            write_len_once = DEF_FLASH_PAGE_SIZE;
            FLASH_ROM_ERASE(address, DEF_FLASH_PAGE_SIZE);
            if(write_len < DEF_FLASH_PAGE_SIZE)
            {
                IAP_Flash_Read(page_addr, temp_buf, DEF_FLASH_PAGE_SIZE);

                for(j = 0; j < write_len; j++)
                {
                    temp_buf[j] = buff[DEF_FLASH_PAGE_SIZE * i + j];
                }
                write_len_once = write_len;
                mFLASH_ProgramPage_Fast(page_addr, (uint32_t*)&temp_buf[0]);
            }
            else
            {
                mFLASH_ProgramPage_Fast(page_addr, (uint32_t*)&buff[DEF_FLASH_PAGE_SIZE * i]);
            }
            page_addr += DEF_FLASH_PAGE_SIZE;
            write_len -= write_len_once;
            write_cnts++;
        }
        if(i != write_cnts)
        {
            /* Incorrect read length */
            return 0xFD;
        }
    }
    else
    {
        /* address Out Of Range */
        return 0xFE;
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_Flash_Erase
 *
 * @brief   Erase Flash In page(256 bytes),Specify length & address,
 *          Based On Fast Flash Operation,
 *          With address protection and program runaway protection.
 *
 * @return  0: Operation Success
 *          See notes for other errors
 */
uint8_t IAP_Flash_Erase(uint32_t address, uint32_t length)
{
    uint32_t i;
    uint32_t erase_addr;
    uint32_t erase_len;
    volatile uint32_t page_cnts;
    volatile uint32_t page_addr;

    /* Set initial value */
    erase_addr = address;
    page_addr = address;
    erase_len = length;
    if((erase_len % DEF_FLASH_PAGE_SIZE) == 0)
    {
        page_cnts = erase_len / DEF_FLASH_PAGE_SIZE;
    }
    else
    {
        page_cnts = (erase_len / DEF_FLASH_PAGE_SIZE) + 1;
    }

    /* Verify Keys, No flash operation if keys are not correct */
    if((Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0) || (Flash_Operation_Key1 != DEF_FLASH_OPERATION_KEY_CODE_1))
    {
        /* ERR: Risk of code running away */
        return 0xFF;
    }
    /* Verify Address, No flash operation if the address is out of range */
    if(((erase_addr >= DEF_APP_CODE_START_ADDR) && (erase_addr <= DEF_APP_CODE_END_ADDR)) || ((erase_addr >= DEF_VERIFY_CODE_START_ADDR) && (erase_addr <= DEF_VERIFY_CODE_END_ADDR)))
    {
        for(i = 0; i < page_cnts; i++)
        {
            /* Verify Keys, No flash operation if keys are not correct */
            if(Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0)
            {
                /* ERR: Risk of code running away */
                return 0xFF;
            }
            FLASH_ROM_ERASE(page_addr, DEF_FLASH_PAGE_SIZE);
            page_addr += DEF_FLASH_PAGE_SIZE;
        }
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_Flash_Verify
 *
 * @brief   verify Flash In bytes(8 bits),Specify length & address,
 *          With address protection and program runaway protection.
 *
 * @return  0: Operation Success
 *          See notes for other errors
 */
uint32_t IAP_Flash_Verify(uint32_t address, uint8_t* buff, uint32_t length)
{
    uint32_t i;
    uint32_t read_addr;
    uint32_t read_len;

    /* Set initial value */
    read_addr = address;
    read_len = length;

    /* Verify Keys, No flash operation if keys are not correct */
    if((Flash_Operation_Key0 != DEF_FLASH_OPERATION_KEY_CODE_0) || (Flash_Operation_Key1 != DEF_FLASH_OPERATION_KEY_CODE_1))
    {
        /* ERR: Risk of code running away */
        return 0xFFFFFFFF;
    }

    /* Verify Address, No flash operation if the address is out of range */
    if(((read_addr >= DEF_APP_CODE_START_ADDR) && (read_addr <= DEF_APP_CODE_END_ADDR)) || ((read_addr >= DEF_VERIFY_CODE_START_ADDR) && (read_addr <= DEF_VERIFY_CODE_END_ADDR)))
    {
        for(i = 0; i < read_len; i+=4 )
        {
            if( FLASH_ReadWord(read_addr) != *(uint32_t*)(buff+i) )
            {
                /* To prevent 0-length errors, the returned position +1 */
                return i + 1;
            }
            read_addr += 4;
        }
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_Flash_Program
 *
 * @brief   IAP programming code, including write and verify,
 *          Specify length & address, Based On Fast Flash Operation,
 *          With address protection and program runaway protection.
 *
 * @return  ret : The meaning of 'ret' can be found in the notes of the
 *          corresponding function.
 */
uint32_t IAP_Flash_Program(uint32_t address, uint8_t* buff, uint32_t length)
{
    uint32_t ret;
    /* IAP Write */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Write(address, buff, length);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        PRINT("write error\n");
        return ret;
    }
    /* IAP Verify */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Verify(address, buff, length);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        PRINT("verify error\n");
        return ret;
    }
    return ret;
}

/*********************************************************************
 * @fn      IAP_VerifyCode_Write
 *
 * @brief   IAP VerifyCode Write in, including write and verify,
 *          Based On Fast Flash Operation.
 *          With address protection and program runaway protection.
 *          If an operation is unsuccessful,the code will repeat the
 *          operation up to 5 times and if it is still unsuccessful,
 *          an error code will be returned.
 *
 * @return  ret : The meaning of 'ret' can be found in the notes of the
 *          corresponding function.
 */
uint32_t IAP_VerifyCode_Write(void)
{
    uint32_t ret;
    uint8_t  rty_cnts;
    rty_cnts = 0;

IAP_VERIFYCODE_RTY:
    /* Verify Code Write-in */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Write(DEF_VERIFY_CODE_START_ADDR, (uint8_t*)Verify_Code, DEF_VERIFY_CODE_LEN);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        return ret;
    }

    /* Verify Code Check */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Verify(DEF_VERIFY_CODE_START_ADDR, (uint8_t*)Verify_Code, DEF_VERIFY_CODE_LEN);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        rty_cnts++;
        if(rty_cnts >= 5)
        {
            return ret;
        }
        goto IAP_VERIFYCODE_RTY;
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_VerifyCode_Check
 *
 * @brief   Check IAP VerifyCode,
 *          With address protection and program runaway protection.
 *
 * @return  ret : The meaning of 'ret' can be found in the notes of the
 *          corresponding function.
 */
uint32_t IAP_VerifyCode_Check(void)
{
    uint32_t ret;
    /* Verify Code Check */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Verify(DEF_VERIFY_CODE_START_ADDR, (uint8_t*)Verify_Code, DEF_VERIFY_CODE_LEN);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        return ret;
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_VerifyCode_Erase
 *
 * @brief   Erase IAP VerifyCode, Based On Fast Flash Operation.
 *          With address protection and program runaway protection.
 *
 * @return  ret : The meaning of 'ret' can be found in the notes of the
 *          corresponding function.
 */
uint8_t  IAP_VerifyCode_Erase(void)
{
    uint8_t ret;

    /* Verify Code Erase */
    Flash_Operation_Key1 = DEF_FLASH_OPERATION_KEY_CODE_1;
    ret = IAP_Flash_Erase(DEF_VERIFY_CODE_START_ADDR, DEF_VERIFY_CODE_MAXLEN);
    Flash_Operation_Key1 = 0;
    if(ret != 0)
    {
        return ret;
    }

    return 0;
}

/*********************************************************************
 * @fn      IAP_Jump_APP
 *
 * @brief   Start the Operation of jumping to user application
 *
 * @return  none
 */
void IAP_Jump_APP(void)
{
    PRINT("User Code!\r\n");

    /* Jump Code */
    jumpApp();
}


/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   Checking the operation status, displaying the error code and stopping if there is an error
 *          input : iError - Error code input
 *
 * @return  none
 */
void mStopIfError( uint8_t iError )
{
    if ( iError == ERR_SUCCESS )
    {
        /* operation success, return */
        return;
    }
    /* Display the errors */
    PRINT( "Error:%02x\r\n", iError );
    /* After encountering an error, you should analyze the error code and CHRV3DiskStatus status, for example,
     * call CHRV3DiskReady to check whether the current USB disk is connected or not,
     * if the disk is disconnected then wait for the disk to be plugged in again and operate again,
     * Suggested steps to follow after an error:
     *     1, call CHRV3DiskReady once, if successful, then continue the operation, such as Open, Read/Write, etc.
     *     2?If CHRV3DiskReady is not successful, then the operation will be forced to start from the beginning.
     */
    while(1)
    {  }
}

/*********************************************************************
 * @fn      IAP_Check_Verify_Code
 *
 * @brief   Check IAP Verify-Code if exists, if exists then jump to APP Code,
 *          else if stay in IAP code
 *
 * @return  none
 */
void IAP_Check_Verify_Code(void)
{
    uint8_t ret;
    PRINT( "Check If Verify-Code Exists.\r\n" );
    ret = IAP_VerifyCode_Check( );
    if( ret == 0x00 )
    {
        PRINT( "Verify-Code Exists.\r\n" );
        /* Jump User Application */
        IAP_Jump_APP( );
    }
    PRINT( "Verify-Code Not Exists, Goto UDisk Operation.\r\n" );
}

/*********************************************************************
 * @fn      IAP_Initialization
 *
 * @brief   IAP process Initialization, include usb-host initialization
 *          usb libs initialization, iap-related values Initialization
 *          IAP verify-code inspection
 *
 * @return  none
 */
void IAP_Initialization( void )
{    
    /* IAP verify-code inspection */
    Flash_Operation_Key0 = DEF_FLASH_OPERATION_KEY_CODE_0;
    IAP_Check_Verify_Code( );

    /* USB Host Initialization */
    PRINT( "USB Host & UDisk Lib Initialization. \r\n" );

#if DEF_USB_PORT_HS_EN
    PRINT( "USBHS Host Init\r\n" );
    USBHS_Host_Init( ENABLE );
    memset( &RootHubDev[ DEF_USB_PORT_HS ].bStatus, 0, sizeof( struct _ROOT_HUB_DEVICE ) );
    memset( &HostCtl[ DEF_USB_PORT_HS ].InterfaceNum, 0, sizeof( struct __HOST_CTL ) );
#endif


    /* USB Libs Initialization */
    PRINT( "UDisk library Initialization. \r\n" );
    CHRV3LibInit( );

    /* IAP-related variable initialization  */
    IAP_Load_Addr_Offset = 0;
    IAP_WriteIn_Length = 0;
    IAP_WriteIn_Count = 0;
}

/*********************************************************************
 * @fn      USBH_CheckRootHubPortStatus
 *
 * @brief   Check status of USB port.
 *
 * @para    index: USB host port
 *
 * @return  The current status of the port.
 */
uint8_t USBH_CheckRootHubPortStatus( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {
        return s;
    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        s = USBHSH_CheckRootHubPortStatus( RootHubDev[ usb_port ].bStatus );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_ResetRootHubPort
 *
 * @brief   Reset USB port.
 *
 * @para    index: USB host port
 *          mod: Reset host port operating mode.
 *               0 -> reset and wait end
 *               1 -> begin reset
 *               2 -> end reset
 *
 * @return  none
 */
void USBH_ResetRootHubPort( uint8_t usb_port, uint8_t mode )
{
    if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        USBHSH_ResetRootHubPort( mode );
#endif
    }
}

/*********************************************************************
 * @fn      USBH_EnableRootHubPort
 *
 * @brief   Enable USB host port.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_EnableRootHubPort( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {

    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        s = USBHSH_EnableRootHubPort( &RootHubDev[ usb_port ].bSpeed );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_GetDeviceDescr
 *
 * @brief   Get the device descriptor of the USB device.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_GetDeviceDescr( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {
        return s;
    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        s = USBHSH_GetDeviceDescr( &RootHubDev[ usb_port ].bEp0MaxPks, DevDesc_Buf );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_SetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_SetUsbAddress( uint8_t usb_port )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {
        return s;
    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        RootHubDev[ usb_port ].bAddress = (uint8_t)( DEF_USB_PORT_HS + USB_DEVICE_ADDR );
        s = USBHSH_SetUsbAddress( RootHubDev[ usb_port ].bEp0MaxPks, RootHubDev[ usb_port ].bAddress );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_GetConfigDescr
 *
 * @brief   Get the configuration descriptor of the USB device.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_GetConfigDescr( uint8_t usb_port, uint16_t *pcfg_len )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {
        return s;
    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        s = USBHSH_GetConfigDescr( RootHubDev[ usb_port ].bEp0MaxPks, Com_Buffer, DEF_COM_BUF_LEN, pcfg_len );
#endif
    }
    return s;
}

/*********************************************************************
 * @fn      USBFSH_SetUsbConfig
 *
 * @brief   Set USB configuration.
 *
 * @para    index: USB host port
 *
 * @return  none
 */
uint8_t USBH_SetUsbConfig( uint8_t usb_port, uint8_t cfg_val )
{
    uint8_t s = ERR_USB_UNSUPPORT;

    if( usb_port == DEF_USB_PORT_FS )
    {
        return s;
    }
    else if( usb_port == DEF_USB_PORT_HS )
    {
#if DEF_USB_PORT_HS_EN
        s = USBHSH_SetUsbConfig( RootHubDev[ usb_port ].bEp0MaxPks, cfg_val );
#endif
    }

    return s;
}

/*********************************************************************
 * @fn      USBH_EnumRootDevice
 *
 * @brief   Generally enumerate a device connected to host port.
 *
 * @para    index: USB host port
 *
 * @return  Enumeration result
 */
uint8_t USBH_EnumRootDevice( uint8_t usb_port )
{
    uint8_t  s;
    uint8_t  enum_cnt;
    uint8_t  cfg_val;
    uint16_t i;
    uint16_t len;

    PRINT( "Enum:\r\n" );

    enum_cnt = 0;
ENUM_START:
    /* Delay and wait for the device to stabilize */
    DelayMs( 100 );
    enum_cnt++;
    DelayMs( 8 << enum_cnt );

    /* Reset the USB device and wait for the USB device to reconnect */
    USBH_ResetRootHubPort( usb_port, 0 );
    for( i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++ )
    {
        if( USBH_EnableRootHubPort( usb_port ) == ERR_SUCCESS )
        {
            i = 0;
            s++;
            if( s > 6 )
            {
                break;
            }
        }
        DelayMs( 1 );
    }
    if( i )
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return ERR_USB_DISCON;
    }

    /* Get USB device device descriptor */
    PRINT("Get DevDesc: ");
    s = USBH_GetDeviceDescr( usb_port );
    if( s == ERR_SUCCESS )
    {
        /* Print USB device device descriptor */
#if DEF_DEBUG_PRINTF
        for( i = 0; i < 18; i++ )
        {
            PRINT( "%02x ", DevDesc_Buf[ i ] );
        }
        PRINT("\n");
#endif
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        PRINT( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return DEF_DEV_DESCR_GETFAIL;
    }

    /* Set the USB device address */
    PRINT("Set DevAddr: ");
    s = USBH_SetUsbAddress( usb_port );
    if( s == ERR_SUCCESS )
    {
        PRINT( "OK\n" );
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        PRINT( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return DEF_DEV_ADDR_SETFAIL;
    }
    DelayMs( 5 );

    /* Get the USB device configuration descriptor */
    PRINT("Get CfgDesc: ");
    s = USBH_GetConfigDescr( usb_port, &len );
    if( s == ERR_SUCCESS )
    {
        cfg_val = ( (PUSB_CFG_DESCR)Com_Buffer )->bConfigurationValue;

        /* Print USB device configuration descriptor  */
#if DEF_DEBUG_PRINTF
        for( i = 0; i < len; i++ )
        {
            PRINT( "%02x ", Com_Buffer[ i ] );
        }
        PRINT("\n");
#endif
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        PRINT( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return DEF_CFG_DESCR_GETFAIL;
    }

    /* Set USB device configuration value */
    PRINT("Set Cfg: ");
    s = USBH_SetUsbConfig( usb_port, cfg_val );
    if( s == ERR_SUCCESS )
    {
        PRINT( "OK\n" );
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        PRINT( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return ERR_USB_UNSUPPORT;
    }

    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      IAP_USBH_PreDeal
 *
 * @brief   usb host preemption operations, 
  *         including detecting device insertion and enumerating device information 
 *
 * @return  none
 */
uint8_t IAP_USBH_PreDeal( void )
{
    uint8_t usb_port;
    uint8_t index;
    uint8_t ret;
    usb_port = DEF_USB_PORT_HS;
    ret = USBH_CheckRootHubPortStatus( usb_port );
    if( ret == ROOT_DEV_CONNECTED )
    {
        PRINT("USB Dev In.\n");
        USBH_CheckRootHubPortStatus( usb_port );
        RootHubDev[ usb_port ].bStatus = ROOT_DEV_CONNECTED; // Set connection status_
        RootHubDev[ usb_port ].DeviceIndex = usb_port * DEF_ONE_USB_SUP_DEV_TOTAL;

        /* Enumerate root device */
        ret = USBH_EnumRootDevice( usb_port );
        if( ret == ERR_SUCCESS )
        {
            PRINT( "USB Port %02x Device Enumeration Succeed\r\n", usb_port );
            RootHubDev[ usb_port ].bStatus = ROOT_DEV_SUCCESS;
            return DEF_IAP_SUCCESS;
        }
        else
        {
            PRINT( "USB Port %02x Device Enumeration ERR %02x.\r\n", usb_port, ret );
            RootHubDev[ usb_port ].bStatus = ROOT_DEV_FAILED;
            return DEF_IAP_ERR_ENUM;
        }
    }
    else if( ret == ROOT_DEV_DISCONNECT )
    {
        PRINT("USB Port %02x Device Out.\r\n", usb_port );
        /* Clear parameters */
        index = RootHubDev[ usb_port ].DeviceIndex;
        memset( &RootHubDev[ usb_port ].bStatus, 0, sizeof( struct _ROOT_HUB_DEVICE ) );
        memset( &HostCtl[ index ].InterfaceNum, 0, sizeof( struct __HOST_CTL ) );
        return DEF_IAP_ERR_DETECT;
    }
    return DEF_IAP_DEFAULT;
}

/*********************************************************************
 * @fn      IAP_Main_Deal
 *
 * @brief   IAP Transaction Processing
 *
 * @return  none
 */
void IAP_Main_Deal( void )
{
    uint32_t totalcount, t;
    uint16_t i, ret;
    uint8_t  *pCodeStr;
    static   uint8_t   op_flag = 0;

    /* Detect USB Device & Enumeration processing */
    ret = IAP_USBH_PreDeal( );
    if( ret == DEF_IAP_SUCCESS )
    {
        /* Wait for uDisk Ready */
        CHRV3DiskStatus = DISK_USB_ADDR;
        for ( i = 0; i != 10; i ++ )
        {
              PRINT( "Wait Disk Ready...\r\n" );
              ret = CHRV3DiskReady( );
              if ( ret == ERR_SUCCESS )
              {
                  /* Disk Ready */
                  PRINT( "Disk Ready Code:%02x.\r\n", ret );
                  PRINT( "CHRV3DiskStatus:%02x\n", CHRV3DiskStatus);
                  op_flag = 1;
                  break;
              }
              else
              {
                  PRINT("Not Ready Code :%02x.\r\n", ret);
                  PRINT("CHRV3DiskStatus:%02x.\n", CHRV3DiskStatus);
              }
              DelayMs( 50 );
          }
    }

    if( (CHRV3DiskStatus >= DISK_MOUNTED) && op_flag )
    {
        op_flag = 0;
        /* Make sure the flash operation is correct */
        Flash_Operation_Key0 = DEF_FLASH_OPERATION_KEY_CODE_0;
        /* Set the name of the file to be manipulated to "/HOST_APP.BIN"  */
        strcpy( (char *)mCmdParam.Open.mPathName, DEF_IAP_FILE_NAME );
        /* open file */
        ret = CHRV3FileOpen( );
        /* file or directory not found */
        if ( ret == ERR_MISS_DIR || ret == ERR_MISS_FILE )
        {
            /* list all file and stay in IAP code */
            PRINT( "APP file not Found, Stay In IAP.\r\n" );
            pCodeStr = "/*";
            PRINT( "List all files %s\r\n", pCodeStr );
            for ( i = 0; i < 10000; i ++ )                          //Search up to the first 10,000 files, practically no limit
            {
                strcpy( (char *)mCmdParam.Open.mPathName, (char *)pCodeStr );       //Search for filenames, * is a wildcard, for all files or subdirectories
                ret = strlen( (char *)mCmdParam.Open.mPathName );
                mCmdParam.Open.mPathName[ ret ] = 0xFF;             //Replace the terminator with the search number according to the length of the string, from 0 to 254,if it is 0xFF that is 255, then the search number is in the CHRV3vFileSize variable
                CHRV3vFileSize = i;                                 //Specify the serial number of the search/enumeration
                ret = CHRV3FileOpen( );                             //Open a file, if the file name contains a wildcard *, the file is searched for and not opened
                if ( ret == ERR_MISS_FILE )                         //The only difference between CHRV3FileEnum and CHRV3FileOpen is that when the latter returns ERR_FOUND_NAME, then the former returns ERR_SUCCESS.
                {
                    break;                                          //No more matching files, no more matching file names
                }
                if ( ret == ERR_FOUND_NAME )                        //Search for a file name matching the wildcard, the file name and its full path are in the command buffer
                {
                    PRINT( "  match file %04d#: %s\r\n", (unsigned int)i, mCmdParam.Open.mPathName );//Display the serial number and the name of the matching file or subdirectory searched
                    continue;                                       //Continue searching for the next matching file name, the next search will add 1 to the serial number
                }
                else
                {
                    mStopIfError( ret );
                    break;
                }
            }
        }
        /* Found file, start IAP processing */
        else
        {
            PRINT( "File Found, Start IAP Process\r\n" );
            /* Read File Size */
            totalcount = CHRV3vFileSize;
            PRINT( "File size in bytes: %d.\r\n", totalcount );
            /* Make sure the flash operation is correct */
            Flash_Operation_Key0 = DEF_FLASH_OPERATION_KEY_CODE_0;
            IAP_Load_Addr_Offset = 0;
            IAP_WriteIn_Length = 0;
            IAP_WriteIn_Count = 0;
            /* Binary file read & iap write in */
            while ( totalcount )
            {
                /* If the file is large and cannot be read at once, you can call CH103ByteRead again to continue reading, and the file pointer will move backward automatically.*/
                /* MAX_PATH_LEN: the maximum path length, including all slash separators and decimal point separators and path terminator 00H */
                if ( totalcount > (MAX_PATH_LEN-1) )
                {
                    t = MAX_PATH_LEN-1;                            //The length of a single read/write cannot exceed sizeof( mCmdParam.Other.mBuffer ) if the remaining data is large.
                }
                else
                {
                    t = totalcount;                                //Last remaining bytes
                }
                mCmdParam.ByteRead.mByteCount = t;                 //Request to read out tens of bytes of data
                mCmdParam.ByteRead.mByteBuffer= &Com_Buffer[0];
                ret = CHRV3ByteRead( );                            //Read the data block in bytes, the length of a single read/write cannot exceed MAX_BYTE_IO, and the second call is followed by a backward read.
                mStopIfError( ret );
                totalcount -= mCmdParam.ByteRead.mByteCount;       //Counting, subtracting the number of characters that have actually been read out
                for(i=0;i<mCmdParam.ByteRead.mByteCount;i++)
                {
                  IAPLoadBuffer[IAP_WriteIn_Length]=mCmdParam.ByteRead.mByteBuffer[i];
                  IAP_WriteIn_Length++;
                  /* The whole package part of the IAP user file */
                  if( IAP_WriteIn_Length == DEF_MAX_IAP_BUFFER_LEN )
                  {
                      /* Write Data In Flash */
                      ret = IAP_Flash_Program( DEF_APP_CODE_START_ADDR+IAP_Load_Addr_Offset, IAPLoadBuffer, IAP_WriteIn_Length );
                      mStopIfError( ret );
                      IAP_Load_Addr_Offset += DEF_MAX_IAP_BUFFER_LEN;
                      IAP_WriteIn_Count += IAP_WriteIn_Length;
                      IAP_WriteIn_Length = 0;
                  }
                }
                if ( mCmdParam.ByteRead.mByteCount < t )            //The actual number of characters read is less than the number of characters requested, which means that the end of the file has been reached.
                {
                    PRINT( "\r\nFile End.\r\n" );
                    break;
                }
            }
            /* Close the file be operated now */
            CHRV3FileClose( );
            /* Disposal of remaining package length  */
            ret = IAP_Flash_Program( DEF_APP_CODE_START_ADDR+IAP_Load_Addr_Offset, IAPLoadBuffer, IAP_WriteIn_Length );
            mStopIfError( ret );
            IAP_WriteIn_Count += IAP_WriteIn_Length;
            /* Check actual write length and file length */
            PRINT( "\r\nFileSze : %d,%d.\r\n", (int)CHRV3vFileSize, IAP_WriteIn_Count );
            if( CHRV3vFileSize == IAP_WriteIn_Count )
            {
                ret = IAP_VerifyCode_Write( );
                if( ret != ERR_SUCCESS )
                {
                    /* IAP Verify code error, stay in IAP */
                    PRINT( "Verify-Code ERR %02x\r\n", ret );
                }
                PRINT( "\r\nIAP End.\r\n" );

                /* Jump User Application */
                IAP_Jump_APP( );
            }
            else
            {
                /* IAP length checksum error */
                PRINT( "IAP length checksum ERR. \r\n" );
            }
        }
    }
}

