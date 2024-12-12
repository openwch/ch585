/********************************** (C) COPYRIGHT *******************************
* File Name          : ch585_usbhs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : This file provides all the USBHS firmware functions.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <ch585_usbhs_device.h>
#include "CH58x_common.h"
#include "usb_desc.h"

/******************************************************************************/
/* Variable Definition */
/* test mode */
volatile uint8_t  USBHS_Test_Flag;
__attribute__ ((aligned(4))) uint8_t IFTest_Buf[ 53 ] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
    0xFE,//26
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,//37
    0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD,//44
    0xFC, 0x7E, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0x7E//53
};

/* Global */
const uint8_t    *pUSBHS_Descr;

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

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP3_Rx_Buf[ DEF_USB_EP3_HS_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP5_Rx_Buf[ DEF_USB_EP5_HS_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP4_Tx_Buf[ DEF_USB_EP4_HS_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP6_Tx_Buf[ DEF_USB_EP6_HS_SIZE ];

/* Endpoint tx busy flag */
volatile uint8_t  USBHS_Endp_Busy[ DEF_UEP_NUM ];

/* Ring buffer */
RING_BUFF_COMM  RingBuffer_Comm;
__attribute__ ((aligned(4))) uint8_t Data_Buffer[DEF_RING_BUFFER_SIZE];

/*********************************************************************
 * @fn      USB_TestMode_Deal
 *
 * @brief   Eye Diagram Test Function Processing.
 *
 * @return  none
 *
 */
void USB_TestMode_Deal( void )
{
#if TEST_ENABLE==0x01
    /* start test */
    USBHS_Test_Flag &= ~0x80;

    if( USBHS_SetupReqIndex == 0x0100 )
    {
        /* Test_J */
        R8_USB2_TEST_MODE &= ~TEST_MASK;
        R8_USB2_TEST_MODE |= USBHS_UD_TEST_J;
    }
    else if( USBHS_SetupReqIndex == 0x0200 )
    {
        /* Test_K */
        R8_USB2_TEST_MODE &= ~TEST_MASK;
        R8_USB2_TEST_MODE |= USBHS_UD_TEST_K;
    }
    else if( USBHS_SetupReqIndex == 0x0300 )
    {
        /* Test_SE0_NAK */
        R8_USB2_TEST_MODE &= ~TEST_MASK;
        R8_USB2_TEST_MODE |= USBHS_UD_TEST_SE0NAK;
    }
    else if( USBHS_SetupReqIndex == 0x0400 )
    {
        /* Test_Packet */
        R8_USB2_TEST_MODE &= ~TEST_MASK;

        R32_U2EP4_TX_DMA = (uint32_t)(&IFTest_Buf[ 0 ]);
        R16_U2EP4_T_LEN = 53;
        R8_U2EP4_TX_CTRL = USBHS_UEP_T_RES_ACK;
        R8_USB2_TEST_MODE |= USBHS_UD_TEST_PKT;
    }
    R8_USB2_TEST_MODE |= USBHS_UD_TEST_EN;
#endif
}

/*********************************************************************
 * @fn      USBHS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHS_Device_Endp_Init ( void )
{
    uint8_t i = 0;

    R16_U2EP_TX_EN = RB_EP0_EN | RB_EP1_EN | RB_EP4_EN | RB_EP6_EN;
    R16_U2EP_RX_EN = RB_EP0_EN | RB_EP1_EN | RB_EP3_EN | RB_EP5_EN;

    R32_U2EP0_MAX_LEN = DEF_USBD_UEP0_SIZE;
    R32_U2EP1_MAX_LEN = DEF_USB_EP1_HS_SIZE;
    R32_U2EP2_MAX_LEN = DEF_USB_EP2_HS_SIZE;
    R32_U2EP3_MAX_LEN = DEF_USB_EP3_HS_SIZE;
    R32_U2EP4_MAX_LEN = DEF_USB_EP4_HS_SIZE;
    R32_U2EP5_MAX_LEN = DEF_USB_EP5_HS_SIZE;
    R32_U2EP6_MAX_LEN = DEF_USB_EP6_HS_SIZE;

    R32_U2EP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;

    R32_U2EP1_RX_DMA = (uint32_t)(uint8_t *)Data_Buffer;
    R32_U2EP3_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP3_Rx_Buf;
    R32_U2EP5_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP5_Rx_Buf;


    R32_U2EP1_TX_DMA = (uint32_t)(uint8_t *)Data_Buffer;
    R32_U2EP4_TX_DMA = (uint32_t)(uint8_t *)USBHS_EP4_Tx_Buf;
    R32_U2EP6_TX_DMA = (uint32_t)(uint8_t *)USBHS_EP6_Tx_Buf;

    R16_U2EP0_T_LEN = 0;
    R8_U2EP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    R8_U2EP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

    R8_U2EP1_RX_CTRL = USBHS_UEP_R_RES_ACK;
    R8_U2EP3_RX_CTRL = USBHS_UEP_R_RES_ACK;
    R8_U2EP5_RX_CTRL = USBHS_UEP_R_RES_ACK;

    R16_U2EP1_T_LEN = 0;
    R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;

    R16_U2EP4_T_LEN = 0;
    R8_U2EP4_TX_CTRL = USBHS_UEP_T_RES_NAK;

    R16_U2EP6_T_LEN = 0;
    R8_U2EP6_TX_CTRL = USBHS_UEP_T_RES_NAK;

    /* Clear End-points Busy Status */
    for( i = 0; i < DEF_UEP_NUM; i++ )
    {
        USBHS_Endp_Busy[ i ] = 0;
    }
}

/*********************************************************************
 * @fn      USBHS_Device_Init
 *
 * @brief   Initializes USB high-speed device.
 *
 * @return  none
 */
void USBHS_Device_Init ( FunctionalState sta )
{
    if( sta )
    {
        R8_USBHS_PLL_CTRL = USBHS_PLL_EN;
        R16_PIN_CONFIG |= RB_PIN_USB2_EN;

        R8_USB2_CTRL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;            
        R8_USB2_INT_EN = USBHS_UDIE_BUS_RST | USBHS_UDIE_SUSPEND | USBHS_UDIE_BUS_SLEEP | USBHS_UDIE_LPM_ACT | USBHS_UDIE_TRANSFER | USBHS_UDIE_LINK_RDY;      
        USBHS_Device_Endp_Init();
        R8_USB2_BASE_MODE = USBHS_UD_SPEED_HIGH;
        R8_USB2_CTRL = USBHS_UD_DEV_EN | USBHS_UD_DMA_EN | USBHS_UD_LPM_EN | USBHS_UD_PHY_SUSPENDM;
        PFIC_EnableIRQ( USB2_DEVICE_IRQn );
    }
    else
    {
        R8_USBHS_PLL_CTRL &= ~USBHS_PLL_EN;
        R32_PIN_CONFIG &= ~RB_PIN_USB2_EN;

        R8_USB2_CTRL |= USBHS_UD_RST_SIE;
        R8_USB2_CTRL &= ~USBHS_UD_RST_SIE;
        PFIC_DisableIRQ( USB2_DEVICE_IRQn );
    }
}

/*********************************************************************
 * @fn      USBHS_Endp_DataUp
 *
 * @brief   usbhd-hs device data upload
 *          input: endp  - end-point numbers
 *                 *pubf - data buffer
 *                 len   - load data length
 *                 mod   - 0: DEF_UEP_DMA_LOAD 1: DEF_UEP_CPY_LOAD
 *
 * @return  none
 */
uint8_t USBHS_Endp_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod )
{
    uint8_t endp_en;

    /* DMA config, endp_ctrl config, endp_len config */
    if( (endp>=DEF_UEP1) && (endp<=DEF_UEP15) )
    {
        endp_en =  R16_U2EP_TX_EN;
        if( endp_en & USBHSD_UEP_TX_EN( endp ) )
        {
            if( (USBHS_Endp_Busy[ endp ] & DEF_UEP_BUSY) == 0x00 )
            {

                /* end-point buffer mode is single buffer */
                if( mod == DEF_UEP_DMA_LOAD )
                {
                    USBHSD_UEP_TXDMA( endp ) = (uint32_t)pbuf;
                }
                else if( mod == DEF_UEP_CPY_LOAD )
                {
                    memcpy( USBHSD_UEP_TXBUF(endp), pbuf, len );
                }
                else
                {
                    return 1;
                }

                /* Set end-point busy */
                USBHS_Endp_Busy[ endp ] |= DEF_UEP_BUSY;
                /* end-point n response tx ack */
                USBHSD_UEP_TLEN( endp ) = len;
                USBHSD_UEP_TXCTRL( endp ) = (USBHSD_UEP_TXCTRL( endp ) &= ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
    return 0;
}

/*********************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   This function handles USBHS exception.
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void USB2_DEVICE_IRQHandler( void )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len, i;
    uint8_t endp_num;

    intflag = R8_USB2_INT_FG;
    intst = R8_USB2_INT_ST;

    if( intflag & USBHS_UDIF_TRANSFER )
    {
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
                                                pUSBHS_Descr = MyCfgDescr_HS;
                                                len = DEF_USBD_CONFIG_HS_DESC_LEN;
                                            }
                                            else
                                            {
                                                /* Full speed mode */
                                                pUSBHS_Descr = MyCfgDescr_FS;
                                                len = DEF_USBD_CONFIG_FS_DESC_LEN;
                                            }
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

                                                /* Descriptor 3, Serial-number String descriptor */
                                                case DEF_STRING_DESC_SERN:
                                                    pUSBHS_Descr = MySerNumInfo;
                                                    len = DEF_USBD_SN_DESC_LEN;
                                                    break;

                                                default:
                                                    errflag = 0xFF;
                                                    break;
                                            }
                                            break;

                                        /* get usb device qualify descriptor */
                                        case USB_DESCR_TYP_QUALIF:
                                            pUSBHS_Descr = MyQuaDesc;
                                            len = DEF_USBD_QUALFY_DESC_LEN;
                                            break;

                                        /* get usb BOS descriptor */
                                        case USB_DESCR_TYP_BOS:
                                            /* USB 2.00 DO NOT support BOS descriptor */
                                            errflag = 0xFF;
                                            break;

                                        /* get usb other-speed descriptor */
                                        case USB_DESCR_TYP_SPEED:
                                            if( USBHS_DevSpeed == USBHS_SPEED_HIGH )
                                            {
                                                /* High speed mode */
                                                memcpy( &TAB_USB_HS_OSC_DESC[ 2 ], &MyCfgDescr_FS[ 2 ], DEF_USBD_CONFIG_FS_DESC_LEN - 2 );
                                                pUSBHS_Descr = ( uint8_t * )&TAB_USB_HS_OSC_DESC[ 0 ];
                                                len = DEF_USBD_CONFIG_FS_DESC_LEN;
                                            }
                                            else if( USBHS_DevSpeed == USBHS_SPEED_FULL )
                                            {
                                                /* Full speed mode */
                                                memcpy( &TAB_USB_FS_OSC_DESC[ 2 ], &MyCfgDescr_HS[ 2 ], DEF_USBD_CONFIG_HS_DESC_LEN - 2 );
                                                pUSBHS_Descr = ( uint8_t * )&TAB_USB_FS_OSC_DESC[ 0 ];
                                                len = DEF_USBD_CONFIG_HS_DESC_LEN;
                                            }
                                            else
                                            {
                                                errflag = 0xFF;
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
                                    memcpy( USBHS_EP0_Buf, pUSBHS_Descr, len );
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
                                    USBHS_DevEnumStatus = 0x01;
                                    break;

                                /* Clear or disable one usb feature */
                                case USB_CLEAR_FEATURE:
                                    if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                                    {
                                        /* clear one device feature */
                                        if((uint8_t)(USBHS_SetupReqValue&0xFF) == 0x01)
                                        {
                                            /* clear usb sleep status, device not prepare to sleep */
                                            USBHS_DevSleepStatus &= ~0x01;
                                        }
                                        else
                                        {
                                            errflag = 0xFF;
                                        }
                                    }
                                    else if ( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                                    {
                                        /* Set End-point Feature */
                                        if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                                        {
                                            /* Clear End-point Feature */
                                            switch( (uint8_t)(USBHS_SetupReqIndex&0xFF) )
                                            {
                                                case (DEF_UEP1 | DEF_UEP_OUT):
                                                        /* Set End-point 1 OUT ACK */
                                                        R8_U2EP1_RX_CTRL = USBHS_UEP_R_RES_ACK;
                                                    break;

                                                case (DEF_UEP1 | DEF_UEP_IN):
                                                        /* Set End-point 1 IN NAK */
                                                        R8_U2EP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
                                                    break;

                                                case (DEF_UEP3 | DEF_UEP_OUT):
                                                        /* Set End-point 3 OUT ACK */
                                                        R8_U2EP3_RX_CTRL = USBHS_UEP_R_RES_ACK;
                                                    break;

                                                case (DEF_UEP4 | DEF_UEP_IN):
                                                        /* Set End-point 4 IN NAK */
                                                        R8_U2EP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
                                                    break;

                                                case (DEF_UEP5 | DEF_UEP_OUT):
                                                        /* Set End-point 5 OUT ACK */
                                                        R8_U2EP5_RX_CTRL = USBHS_UEP_R_RES_ACK;
                                                    break;

                                                case (DEF_UEP6 | DEF_UEP_IN):
                                                        /* Set End-point 6 IN NAK */
                                                        R8_U2EP6_TX_CTRL = USBHS_UEP_T_RES_NAK;
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
                                    if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                                    {
                                        /* Set Device Feature */
                                        if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                        {
                                            if (((USBHS_DevSpeed == USBHS_SPEED_HIGH) && (MyCfgDescr_HS[7] & 0x20)) ||
                                                ((USBHS_DevSpeed == USBHS_SPEED_FULL) && (MyCfgDescr_FS[7] & 0x20)))
                                            {
                                                /* Set Wake-up flag, device prepare to sleep */
                                                USBHS_DevSleepStatus |= 0x01;
                                            }
                                            else
                                            {
                                                errflag = 0xFF;
                                            }
                                        }
                                        else if( (uint8_t)(USBHS_SetupReqValue&0xFF) == 0x02 )
                                        {
                                            /* test mode deal */
                                            if( ( USBHS_SetupReqIndex == 0x0100 ) ||
                                                ( USBHS_SetupReqIndex == 0x0200 ) ||
                                                ( USBHS_SetupReqIndex == 0x0300 ) ||
                                                ( USBHS_SetupReqIndex == 0x0400 ) )
                                            {
                                                /* Set the flag and wait for the status to be uploaded before proceeding with the actual operation */
                                                USBHS_Test_Flag |= 0x80;
                                            }
                                        }
                                        else
                                        {
                                            errflag = 0xFF;
                                        }
                                    }
                                    else if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                                    {
                                        /* Set End-point Feature */
                                        if( (uint8_t)(USBHS_SetupReqValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                                        {
                                            /* Set end-points status stall */
                                            switch((uint8_t)(USBHS_SetupReqIndex&0xFF) )
                                            {
                                                case (DEF_UEP1 | DEF_UEP_OUT):
                                                        /* Set End-point 1 OUT STALL */
                                                        R8_U2EP1_RX_CTRL = ( R8_U2EP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                                                    break;

                                                case (DEF_UEP1 | DEF_UEP_IN):
                                                    /* Set End-point 1 IN STALL */
                                                        R8_U2EP1_TX_CTRL = ( R8_U2EP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                                                    break;

                                                case (DEF_UEP3 | DEF_UEP_OUT):
                                                        /* Set End-point 3 OUT STALL */
                                                        R8_U2EP3_RX_CTRL = ( R8_U2EP3_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                                                    break;

                                                case (DEF_UEP4 | DEF_UEP_IN):
                                                        /* Set End-point 4 IN STALL */
                                                        R8_U2EP4_TX_CTRL = ( R8_U2EP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                                                    break;

                                                case (DEF_UEP5 | DEF_UEP_OUT):
                                                        /* Set End-point 5 OUT STALL */
                                                        R8_U2EP5_RX_CTRL = ( R8_U2EP5_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                                                    break;

                                                case (DEF_UEP6 | DEF_UEP_IN):
                                                        /* Set End-point 6 IN STALL */
                                                        R8_U2EP6_TX_CTRL = ( R8_U2EP6_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                                                    break;

                                                default:
                                                    errflag = 0xFF;
                                                    break;
                                            }
                                        }
                                    }
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
                                    if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                                    {
                                        switch( (uint8_t)( USBHS_SetupReqIndex & 0xFF ) )
                                        {
                                            case (DEF_UEP1 | DEF_UEP_OUT):
                                                 if( ( (R8_U2EP1_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                                                 {
                                                     USBHS_EP0_Buf[ 0 ] = 0x01;
                                                 }

                                            case (DEF_UEP1 | DEF_UEP_IN):
                                                if( ( (R8_U2EP1_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                                {
                                                    USBHS_EP0_Buf[ 0 ] = 0x01;
                                                }
                                                break;

                                            case (DEF_UEP2 | DEF_UEP_IN):
                                                if( ( (R8_U2EP2_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                                {
                                                    USBHS_EP0_Buf[ 0 ] = 0x01;
                                                }
                                                break;

                                            case (DEF_UEP3 | DEF_UEP_OUT):
                                                 if( ( (R8_U2EP3_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                                                 {
                                                     USBHS_EP0_Buf[ 0 ] = 0x01;
                                                 }
                                                 break;

                                            case (DEF_UEP4 | DEF_UEP_IN):
                                                if( ( (R8_U2EP4_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                                {
                                                    USBHS_EP0_Buf[ 0 ] = 0x01;
                                                }
                                                break;

                                            case (DEF_UEP5 | DEF_UEP_OUT):
                                                if( ( (R8_U2EP5_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                                                {
                                                    USBHS_EP0_Buf[ 0 ] = 0x01;
                                                }
                                                break;

                                            case (DEF_UEP6 | DEF_UEP_IN):
                                                if( ( (R8_U2EP6_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                                {
                                                    USBHS_EP0_Buf[ 0 ] = 0x01;
                                                }
                                                break;

                                            default:
                                                 errflag = 0xFF;
                                                 break;
                                        }
                                    }
                                    else if( ( USBHS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                                    {
                                          if( USBHS_DevSleepStatus & 0x01 )
                                          {
                                              USBHS_EP0_Buf[ 0 ] = 0x02;
                                          }
                                    }

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
                        len = R16_U2EP0_RX_LEN;

                        /* if any processing about rx, set it here */
                        if ( ( USBHS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            USBHS_SetupReqLen = 0;
                            /* Non-standard request end-point 0 Data download */
                            if( USBHS_SetupReqCode == CDC_SET_LINE_CODING )
                            {

                            }
                        }
                        else
                        {
                            /* Standard request end-point 0 Data download */
                        }

                        if( USBHS_SetupReqLen == 0 )
                        {
                            R16_U2EP0_T_LEN  = 0;
                            R8_U2EP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                        }
                    }
                    R8_U2EP0_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;
                /* end-point 1 data out interrupt */
                case  DEF_UEP1:
                    if ( R8_U2EP1_RX_CTRL & USBHS_UEP_R_TOG_MATCH )
                    {
                        /* Write In Buffer */
                        R8_U2EP1_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                        RingBuffer_Comm.PackLen[RingBuffer_Comm.LoadPtr] = R16_U2EP1_RX_LEN;
                        RingBuffer_Comm.LoadPtr ++;
                        if(RingBuffer_Comm.LoadPtr == DEF_Ring_Buffer_Max_Blks)
                        {
                            RingBuffer_Comm.LoadPtr = 0;
                        }
                        R32_U2EP1_RX_DMA = (uint32_t)(&Data_Buffer[(RingBuffer_Comm.LoadPtr) * DEF_USBD_HS_PACK_SIZE]);
                        RingBuffer_Comm.RemainPack ++;
                        if(RingBuffer_Comm.RemainPack >= DEF_Ring_Buffer_Max_Blks-DEF_RING_BUFFER_REMINE)
                        {
                            R8_U2EP1_RX_CTRL = ((R8_U2EP1_RX_CTRL) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
                            RingBuffer_Comm.StopFlag = 1;
                        }
                        else
                        {
                            R8_U2EP1_RX_CTRL = ((R8_U2EP1_RX_CTRL) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
                        }
                    }
                    R8_U2EP1_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;

                /* end-point 3 data out interrupt */
                case  DEF_UEP3:
                    if ( R8_U2EP3_RX_CTRL & USBHS_UEP_R_TOG_MATCH )
                    {
                        len = (uint16_t)(R16_U2EP3_RX_LEN);
                        R8_U2EP3_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                        R8_U2EP3_RX_CTRL = ((R8_U2EP3_RX_CTRL) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
                        for(i=0; i<len; i++)
                        {
                            USBHS_EP4_Tx_Buf[i] = ~USBHS_EP3_Rx_Buf[i];
                        }
                        R16_U2EP4_T_LEN = len;
                        R8_U2EP4_TX_CTRL = (R8_U2EP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                    }
                    R8_U2EP3_RX_CTRL &= ~USBHS_UEP_R_DONE;
                   break;

                /* end-point 5 data out interrupt */
                case  DEF_UEP5:
                if ( R8_U2EP5_RX_CTRL & USBHS_UEP_R_TOG_MATCH )
                {
                    len = (uint16_t)(R16_U2EP5_RX_LEN);
                    R8_U2EP5_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                    R8_U2EP5_RX_CTRL = ((R8_U2EP5_RX_CTRL) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
                    for(i=0; i<len; i++)
                    {
                        USBHS_EP6_Tx_Buf[i] = ~USBHS_EP5_Rx_Buf[i];
                    }
                    R16_U2EP6_T_LEN = len;
                    R8_U2EP6_TX_CTRL = (R8_U2EP6_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                }
                R8_U2EP5_RX_CTRL &= ~USBHS_UEP_R_DONE;
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
                                memcpy(USBHS_EP0_Buf, pUSBHS_Descr, len);
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
                        USB_TestMode_Deal( );
                    }
                    R8_U2EP0_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                /* end-point 1 data in interrupt */
                case DEF_UEP1:
                    R8_U2EP1_TX_CTRL = (R8_U2EP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                    R8_U2EP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                    USBHS_Endp_Busy[ DEF_UEP1 ] &= ~DEF_UEP_BUSY;
                    R8_U2EP1_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                /* end-point 4 data in interrupt */
                case DEF_UEP4:
                    R8_U2EP4_TX_CTRL = (R8_U2EP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                    R8_U2EP4_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                    USBHS_Endp_Busy[ DEF_UEP4 ] &= ~DEF_UEP_BUSY;
                    R8_U2EP3_RX_CTRL = (R8_U2EP3_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
                    R8_U2EP4_TX_CTRL &= ~USBHS_UEP_T_DONE;
                    break;

                /* end-point 6 data in interrupt */
                case DEF_UEP6:
                    R8_U2EP6_TX_CTRL = (R8_U2EP6_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                    R8_U2EP6_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                    USBHS_Endp_Busy[ DEF_UEP6 ] &= ~DEF_UEP_BUSY;
                    R8_U2EP5_RX_CTRL = (R8_U2EP5_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
                    R8_U2EP6_TX_CTRL &= ~USBHS_UEP_T_DONE;
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
        USBHS_Device_Endp_Init( );
        R8_USB2_INT_FG = USBHS_UDIF_BUS_RST;
    }
    else
    {
        /* other interrupts */
        R8_USB2_INT_FG = intflag;
    }
}

