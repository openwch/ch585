/********************************** (C) COPYRIGHT *******************************
 * File Name          : iap.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2022/03/15
 * Description        : USBHS IAP例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "iap.h"
#include "usb_desc.h"

#undef pSetupReqPak     /* 解决和外设库头文件冲突  */
#define pSetupReqPak          ((PUSB_SETUP_REQ)EP0_Databuf)

/* Endpoint tx busy flag */
volatile uint8_t  USBHS_Endp_Busy[ DEF_UEP_NUM ];

/* General */
#define pUSBHS_SetupReqPak            ((PUSB_SETUP_REQ)USBHS_EP0_Buf)

/* Global */
const uint8_t    *pUSBHS_Descr;

/* test mode */
volatile uint8_t  USBHS_Test_Flag;

/* Setup Request */
volatile uint8_t  USBHS_SetupReqCode;
volatile uint8_t  USBHS_SetupReqType;
volatile uint16_t USBHS_SetupReqValue;
volatile uint16_t USBHS_SetupReqIndex;
volatile uint16_t USBHS_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBHS_DevConfig;
volatile uint8_t  USBHS_DevAddr;
volatile uint16_t USBHS_DevMaxPackLen;
volatile uint8_t  USBHS_DevSpeed;
volatile uint8_t  USBHS_DevSleepStatus;
volatile uint8_t  USBHS_DevEnumStatus;

/* USB SPEED TYPE */
#define USBHS_SPEED_TYPE_MASK         ((uint8_t)(0x03))
#define USBHS_SPEED_LOW               ((uint8_t)(0x02))
#define USBHS_SPEED_FULL              ((uint8_t)(0x00))
#define USBHS_SPEED_HIGH              ((uint8_t)(0x01))

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_Tx_Buf[ DEF_USB_EP2_HS_SIZE];
__attribute__ ((aligned(4))) uint8_t USBHS_EP2_Rx_Buf[ DEF_USB_EP2_HS_SIZE];

#define DevEP0SIZE  64
// 设备描述符
const uint8_t MyDevDescr[] =
{
    0x12, 0x01, 0x10, 0x01, 0xFF, 0x80, 0x55,
    DevEP0SIZE, 0x48, 0x43, 0xe0, 0x55,      // 厂商ID和产品ID
    0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
};
// 配置描述符
const uint8_t MyCfgDescr[] =
{
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32, 0x09, 0x04, 0x00, 0x00,
    0x02, 0xFF, 0x80, 0x55, 0x00, 0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00
};
// 语言描述符
const uint8_t MyLangDescr[] =
{ 0x04, 0x03, 0x09, 0x04 };
// 厂家信息
const uint8_t MyManuInfo[] =
{ 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };
// 产品信息
const uint8_t MyProdInfo[] =
{ 0x0C, 0x03, 'C', 0, 'H', 0, '5', 0, '8', 0, 'x', 0 };
// 上下传输函数声明
void myDevEP2_OUT_Deal(uint8_t l);
void myDevEP2_IN_Deal(uint8_t s);

//USB相关变量
uint8_t DevConfig;
uint8_t SetupReqCode;
uint16_t SetupReqLen;
const uint8_t *pDescr;

//升级相关变量
__attribute__((aligned(4)))   uint8_t g_write_buf[256 + 64]; //每次满256字节再写flash，提升速度
volatile uint16_t g_buf_write_ptr = 0;
volatile uint32_t g_flash_write_ptr = 0;
uint32_t g_tcnt;
__attribute__((aligned(4))) iap_cmd_t g_iap_cmd;

/*********************************************************************
 * @fn      USB_DevTransProcess
 *
 * @brief   IAP USB主循环,程序放ram中运行，提升速度.
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void USB_DevTransProcess(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len, i;
    uint8_t endp_num;

    intflag = R8_USB2_INT_FG;
    intst = R8_USB2_INT_ST;

    if( intflag & USBHS_UDIF_TRANSFER )
    {
        g_tcnt = 0; //USB有数据，清空超时计数

        endp_num = intst & USBHS_UDIS_EP_ID_MASK;
        if( !(R8_USB2_INT_ST & USBHS_UDIS_EP_DIR )) // SETUP/OUT Transaction
        {
            switch( endp_num )
            {
                case   DEF_UEP0:
                    if( R8_U2EP0_RX_CTRL & USBHS_UEP_R_SETUP_IS )
                    {
                        /* Store All Setup Values */
                        USBHS_SetupReqType  = pUSBHS_SetupReqPak->bRequestType;
                        USBHS_SetupReqCode  = pUSBHS_SetupReqPak->bRequest;
                        USBHS_SetupReqLen   = pUSBHS_SetupReqPak->wLength;
                        USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
                        USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;

                        len = 0;
                        errflag = 0;
                        if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* usb non-standard request processing */
                            errflag = 0xFF;
                        }
                        else
                        {
                            /* usb standard request processing */
                            switch( USBHS_SetupReqCode )
                            {
                                /* get device/configuration/string/report/... descriptors */
                                case USB_GET_DESCRIPTOR:
                                    switch( (uint8_t)(USBHS_SetupReqValue>>8) )
                                    {
                                        /* get usb device descriptor */
                                        case USB_DESCR_TYP_DEVICE:
                                            pUSBHS_Descr = MyDevDescr;
                                            len = DEF_USBD_DEVICE_DESC_LEN;
                                            break;

                                        /* get usb configuration descriptor */
                                        case USB_DESCR_TYP_CONFIG:
                                            /* Query current usb speed */
                                            if( R8_USB2_MIS_ST & USBHS_UDMS_HS_MOD )
                                            {
                                                /* High speed mode */
                                                USBHS_DevSpeed = USBHS_SPEED_HIGH;
                                                USBHS_DevMaxPackLen = DEF_USBD_HS_PACK_SIZE;
                                            }
                                            else
                                            {
                                                /* Full speed mode */
                                                USBHS_DevSpeed = USBHS_SPEED_FULL;
                                                USBHS_DevMaxPackLen = DEF_USBD_FS_PACK_SIZE;
                                            }

                                            /* Load usb configuration descriptor by speed */
                                            if( USBHS_DevSpeed == USBHS_SPEED_HIGH )
                                            {
                                                /* High speed mode */
                                                pUSBHS_Descr = MyCfgDescr;
                                                len = DEF_USBD_CONFIG_HS_DESC_LEN;
                                            }
                                            else
                                            {
                                                /* Full speed mode */
                                                pUSBHS_Descr = MyCfgDescr;
                                                len = DEF_USBD_CONFIG_FS_DESC_LEN;
                                            }
                                            pUSBHS_Descr = MyCfgDescr;
                                            len = DEF_USBD_CONFIG_HS_DESC_LEN;
                                            break;

                                        /* get usb string descriptor */
                                        case USB_DESCR_TYP_STRING:
                                            switch( (uint8_t)(USBHS_SetupReqValue&0xFF) )
                                            {
                                                /* Descriptor 0, Language descriptor */
                                                case DEF_STRING_DESC_LANG:
                                                    pUSBHS_Descr = MyLangDescr;
                                                    len = DEF_USBD_LANG_DESC_LEN;
                                                    break;

                                                /* Descriptor 1, Manufacturers String descriptor */
                                                case DEF_STRING_DESC_MANU:
                                                    pUSBHS_Descr = MyManuInfo;
                                                    len = DEF_USBD_MANU_DESC_LEN;
                                                    break;

                                                /* Descriptor 2, Product String descriptor */
                                                case DEF_STRING_DESC_PROD:
                                                    pUSBHS_Descr = MyProdInfo;
                                                    len = DEF_USBD_PROD_DESC_LEN;
                                                    break;

                                                default:
                                                    errflag = 0xFF;
                                                    break;
                                            }
                                            break;
                                        default :
                                            errflag = 0xFF;
                                            break;
                                    }

                                    /* Copy Descriptors to Endp0 DMA buffer */
                                    if( USBHS_SetupReqLen>len )
                                    {
                                        USBHS_SetupReqLen = len;
                                    }
                                    len = (USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                                    my_memcpy( USBHS_EP0_Buf, pUSBHS_Descr, len );
                                    pUSBHS_Descr += len;
                                    break;

                                /* Set usb address */
                                case USB_SET_ADDRESS:
                                    USBHS_DevAddr = (uint16_t)(USBHS_SetupReqValue&0xFF);
                                    break;

                                /* Get usb configuration now set */
                                case USB_GET_CONFIGURATION:
                                    USBHS_EP0_Buf[0] = USBHS_DevConfig;
                                    if ( USBHS_SetupReqLen > 1 )
                                    {
                                        USBHS_SetupReqLen = 1;
                                    }
                                    break;

                                /* Set usb configuration to use */
                                case USB_SET_CONFIGURATION:
                                    USBHS_DevConfig = (uint8_t)(USBHS_SetupReqValue&0xFF);
//                                    USBHS_DevEnumStatus = 0x01;
                                    break;

                                /* Clear or disable one usb feature */
                                case USB_CLEAR_FEATURE:
                                    if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                                    {
                                        /* Set End-point Feature */
                                        if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                                        {
                                            /* Clear End-point Feature */
                                            switch( (uint8_t)(USBHS_SetupReqIndex&0xFF) )
                                            {
                                                case (DEF_UEP2 | DEF_UEP_OUT):
                                                        /* Set End-point 2 OUT ACK */
                                                        R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
                                                    break;

                                                case (DEF_UEP2 | DEF_UEP_IN):
                                                        /* Set End-point 2 IN NAK */
                                                        R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
                                                    break;
                                                default:
                                                    errflag = 0xFF;
                                                    break;
                                            }
                                        }
                                        else
                                        {
                                            errflag = 0xFF;
                                        }

                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                    break;

                                /* set or enable one usb feature */
                                case USB_SET_FEATURE:
                                    break;

                                /* This request allows the host to select another setting for the specified interface  */
                                case USB_GET_INTERFACE:
                                    USBHS_EP0_Buf[0] = 0x00;
                                    if ( USBHS_SetupReqLen > 1 )
                                    {
                                        USBHS_SetupReqLen = 1;
                                    }
                                    break;

                                case USB_SET_INTERFACE:
                                    break;

                                /* host get status of specified device/interface/end-points */
                                case USB_GET_STATUS:
                                    USBHS_EP0_Buf[0] = 0x00;
                                    USBHS_EP0_Buf[1] = 0x00;
                                    if ( USBHS_SetupReqLen > 2 )
                                    {
                                        USBHS_SetupReqLen = 2;
                                    }
                                    break;

                                default:
                                    errflag = 0xFF;
                                    break;
                            }
                        }

                        /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                        if( errflag == 0xFF )
                        {
                            /* if one request not support, return stall */
                            R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
                            R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
                        }
                        else
                        {
                            /* end-point 0 data Tx/Rx */
                            if( USBHS_SetupReqType & DEF_UEP_IN )
                            {
                                /* tx */
                                len = (USBHS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                                USBHS_SetupReqLen -= len;
                                R16_U2EP0_T_LEN = len;
                                R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                            }
                            else
                            {
                                /* rx */
                                if( USBHS_SetupReqLen == 0 )
                                {
                                    R16_U2EP0_T_LEN = 0;
                                    R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                                }
                                else
                                {
                                    R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                                }
                            }
                        }
                    }
                    /* end-point 0 data out interrupt */
                    else
                    {
                        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_NAK; // clear
                        if( USBHS_SetupReqLen == 0 )
                        {
                            R16_U2EP0_T_LEN  = 0;
                            R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                        }
                    }
                    R8_U2EP0_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;

                /* end-point 2 data out interrupt */
                case  DEF_UEP2:
                    if ( R8_U2EP2_RX_CTRL & USBHS_UEP_R_TOG_MATCH )
                    {
                        uint8_t l;
                        /* Write In Buffer */
                        R8_U2EP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                        l = R16_U2EP2_RX_LEN;
                        my_memcpy(g_iap_cmd.other.buf, USBHS_EP2_Rx_Buf, l);
                        myDevEP2_OUT_Deal(l);
                    }
                    R8_U2EP2_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;

               default:
                   errflag = 0xFF;
                break;
            }
        }

        else
        {
          /* data-in stage processing */
            switch ( endp_num )
            {
                /* end-point 0 data in interrupt */
                case  DEF_UEP0:
                    if( USBHS_SetupReqLen == 0 )
                    {
                        R8_U2EP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                    }
                    if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                    {
                        /* Non-standard request endpoint 0 Data upload */
                    }
                    else
                    {
                        /* Standard request endpoint 0 Data upload */
                        switch( USBHS_SetupReqCode )
                        {
                            case USB_GET_DESCRIPTOR:
                                len = USBHS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReqLen;
                                my_memcpy(USBHS_EP0_Buf, pUSBHS_Descr, len);
                                USBHS_SetupReqLen -= len;
                                pUSBHS_Descr += len;
                                R16_U2EP0_T_LEN = len;
                                R8_U2EP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                                R8_U2EP0_TX_CTRL = ( R8_U2EP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK; // clear
                                break;

                            case USB_SET_ADDRESS:
                                R8_USB2_DEV_AD = USBHS_DevAddr;
                                break;

                            default:
                                R16_U2EP0_T_LEN = 0;
                                break;
                        }
                    }

                    /* test mode */
                    if( USBHS_Test_Flag & 0x80 )
                    {
//                        USB_TestMode_Deal( );
                    }
                    R8_U2EP0_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                /* end-point 2 data in interrupt */
                case DEF_UEP2:
                    R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                    R8_U2EP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                    USBHS_Endp_Busy[ DEF_UEP2 ] &= ~DEF_UEP_BUSY;
                    R8_U2EP2_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                default :
                    break;
            }
        }
    }
    else if( R8_USB2_INT_FG & USBHS_UDIF_LINK_RDY )
    {

#ifdef  SUPPORT_USB_HSI

            USB_HSI->CAL_CR |= HSI_CAL_EN | HSI_CAL_VLD;
            USB_HSI->CAL_CR &= ~HSI_CAL_RST;

#endif
            R8_USB2_INT_FG = USBHS_UDIF_LINK_RDY;

    }
    else if( intflag & USBHS_UDIF_SUSPEND )
    {
        R8_USB2_INT_FG = USBHS_UDIF_SUSPEND;
        /* usb suspend interrupt processing */
        if ( R8_USB2_MIS_ST & RB_UMS_SUSPEND  )
        {
            USBHS_DevSleepStatus |= 0x02;
            if( USBHS_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBHS_DevSleepStatus &= ~0x02;
        }
    }
    else if( intflag & USBHS_UDIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBHS_DevConfig = 0;
        USBHS_DevAddr = 0;
        USBHS_DevSleepStatus = 0;
        USBHS_DevEnumStatus = 0;

        R8_USB2_DEV_AD = 0;

        R16_U2EP_TX_EN = RB_EP0_EN |  RB_EP2_EN ;
        R16_U2EP_RX_EN = RB_EP0_EN |  RB_EP2_EN ;

        R32_U2EP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
        R32_U2EP2_MAX_LEN  = DEF_USB_EP2_HS_SIZE;

        R32_U2EP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;
        R32_U2EP2_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Rx_Buf;
        R32_U2EP2_TX_DMA = (uint32_t)(uint8_t *)USBHS_EP2_Tx_Buf;

        R16_U2EP0_T_LEN  = 0;
        R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
        R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

        R16_U2EP2_T_LEN  = 0;
        R8_U2EP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
        R8_U2EP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

        /* Clear End-points Busy Status */
        for( i = 0; i < DEF_UEP_NUM; i++ )
        {
            USBHS_Endp_Busy[ i ] = 0;
        }

        R8_USB2_INT_FG = USBHS_UDIF_BUS_RST;
    }
    else
    {
        /* other interrupts */
        R8_USB2_INT_FG = intflag;
    }
}

/*********************************************************************
 * @fn      myDevEP2_OUT_Deal
 *
 * @brief   IAP USB数据处理函数，放入ram中运行提升速度.
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void myDevEP2_OUT_Deal(uint8_t l)
{
    /* 用户可自定义 */
    uint8_t s = 0;
    uint32_t addr;
    switch (g_iap_cmd.other.buf[0])
    {
    case CMD_IAP_PROM:
        if (g_iap_cmd.program.len == 0)
        {
            if (g_buf_write_ptr != 0)
            {
            	g_buf_write_ptr = ((g_buf_write_ptr + 3) & (~3)); //四字节对齐
                s = FLASH_ROM_WRITE(g_flash_write_ptr, (PUINT32)g_write_buf, g_buf_write_ptr);
                g_buf_write_ptr = 0;
            }
        }
        else
        {
            my_memcpy(g_write_buf + g_buf_write_ptr, g_iap_cmd.program.buf, g_iap_cmd.program.len);
            g_buf_write_ptr += g_iap_cmd.program.len;
            if (g_buf_write_ptr >= 256)
            {
                s = FLASH_ROM_WRITE(g_flash_write_ptr, (PUINT32)g_write_buf, 256);
                g_flash_write_ptr += 256;
                g_buf_write_ptr = g_buf_write_ptr - 256;    //超出的长度
                my_memcpy(g_write_buf, g_write_buf + 256, g_buf_write_ptr); //保存剩下的iap_cmd.program.buf + g_iap_cmd.program.len - g_buf_write_ptr
            }
        }
        myDevEP2_IN_Deal(s);
        break;
    case CMD_IAP_ERASE:
    	//这里可以添加地址判断，也可以直接擦除指定位置
    	addr = (g_iap_cmd.erase.addr[0]
				| (uint32_t) g_iap_cmd.erase.addr[1] << 8
				| (uint32_t) g_iap_cmd.erase.addr[2] << 16
				| (uint32_t) g_iap_cmd.erase.addr[3] << 24);
    	if(addr == APP_CODE_START_ADDR)
    	{
			s = FLASH_ROM_ERASE(APP_CODE_START_ADDR, APP_CODE_END_ADDR - APP_CODE_START_ADDR);
			g_buf_write_ptr = 0;    //计数清零
			g_flash_write_ptr = APP_CODE_START_ADDR;
    	}
    	else
    	{
    		s = 0xfe;
		}
        myDevEP2_IN_Deal(s);
        break;
    case CMD_IAP_VERIFY:
		my_memcpy(g_write_buf, g_iap_cmd.verify.buf, g_iap_cmd.verify.len);
		addr = (g_iap_cmd.verify.addr[0]
				| (uint32_t) g_iap_cmd.verify.addr[1] << 8
				| (uint32_t) g_iap_cmd.verify.addr[2] << 16
				| (uint32_t) g_iap_cmd.verify.addr[3] << 24);
		s = FLASH_ROM_VERIFY(addr, g_write_buf, g_iap_cmd.verify.len);
        myDevEP2_IN_Deal(s);
        break;
    case CMD_IAP_END:
        /*结束升级，复位USB，跳转到app*/
        R8_USB2_CTRL = USBHS_UD_RST_SIE;
        R16_PIN_CONFIG &= ~RB_PIN_USB2_EN;
        DelayMs(10);
        jumpApp();
        break;

    default:
        myDevEP2_IN_Deal(0xfe);
        break;
    }
    R8_U2EP2_RX_CTRL = (R8_U2EP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
}

/*******************************************************************************
 * Function Name  : myDevEP2_IN_Deal
 * Description    : 端点2数据上传
 * Input          : l: 上传数据长度(<64B)
 * Return         : None
 *******************************************************************************/
__HIGH_CODE
void myDevEP2_IN_Deal(uint8_t s)
{
    uint8_t endp_en;
    endp_en =  R16_U2EP_TX_EN;
    if( (USBHS_Endp_Busy[ DEF_UEP2 ] & DEF_UEP_BUSY) == 0x00 )
    {
        USBHS_EP2_Tx_Buf[64] = s;
        USBHS_EP2_Tx_Buf[65] = 0;
        /* Set end-point busy */
        USBHS_Endp_Busy[ DEF_UEP2 ] |= DEF_UEP_BUSY;
        /* end-point n response tx ack */
        USBHSD_UEP_TLEN( DEF_UEP2 ) = 2;
        USBHSD_UEP_TXCTRL( DEF_UEP2 ) = (USBHSD_UEP_TXCTRL( DEF_UEP2 ) &= ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
    }
}

/*********************************************************************
 * @fn      my_memcpy
 *
 * @brief   数据拷贝函数,程序放ram中运行，提升速度
 *
 * @param   None.
 *
 * @return  None.
 */
__HIGH_CODE
void my_memcpy(void *dst, const void *src, uint32_t l)
{
    uint32_t len = l;
    PUINT8 pdst = (PUINT8) dst;
    PUINT8 psrc = (PUINT8) src;
    while (len)
    {
        *pdst++ = *psrc++;
        len--;
    }
}
