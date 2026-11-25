/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_device.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : USB full-speed (USBFS) device layer of the U-disk demo.
 *                      The low level driver is CH58x_usbdev.c from the SDK.
 *                      End-point allocation:
 *                          EP0 : control (64 byte)
 *                          EP2 : bulk IN  (0x82, 64 byte)
 *                          EP3 : bulk OUT (0x03, 64 byte)
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "usb_device.h"
#include "SW_UDISK.h"

/*******************************************************************************/
/* Variable Definition */

/* SetUp Request Values */
const uint8_t *pUSB_Descr;

/* Setup Request */
volatile uint8_t  USB_SetupReqCode;
volatile uint8_t  USB_SetupReqType;
volatile uint16_t USB_SetupReqValue;
volatile uint16_t USB_SetupReqIndex;
volatile uint16_t USB_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USB_DevConfig;
volatile uint8_t  USB_DevAddr;
volatile uint8_t  USB_DevSleepStatus;
volatile uint8_t  USB_DevEnumStatus;

/* Endpoint Buffer */
__attribute__((aligned(4))) uint8_t UDisk_In_Buf[ DEF_USBD_FS_PACK_SIZE ];    //EP2 IN
__attribute__((aligned(4))) uint8_t UDisk_Out_Buf[ DEF_USBD_FS_PACK_SIZE ];   //EP3 OUT

/* Endpoint tx busy flag */
volatile uint8_t  USB_Endp_Busy[ DEF_UEP_NUM ];

/*********************************************************************
 * @fn      USB_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints for the U-disk:
 *          EP2 is used as bulk IN (tx only), EP3 as bulk OUT (rx only).
 *          The endpoint buffer start addresses are UEP2_DMA / UEP3_DMA.
 *
 * @return  none
 */
void USB_Device_Endp_Init(void)
{
    uint8_t i;

    R8_UEP2_3_MOD = RB_UEP2_TX_EN | RB_UEP3_RX_EN;

    R32_UEP2_DMA = (uint32_t)(uint8_t *)UDisk_In_Buf;
    R32_UEP3_DMA = (uint32_t)(uint8_t *)UDisk_Out_Buf;

    /* manual toggle (RB_UEP_AUTO_TOG cleared), the toggle is handled in USB_DevTransProcess */
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

    /* Clear End-points Busy Status */
    for(i = 0; i < DEF_UEP_NUM; i++)
    {
        USB_Endp_Busy[i] = DEF_UEP_FREE;
    }
}

/*********************************************************************
 * @fn      USB_Endp_DataUp
 *
 * @brief   full-speed device bulk IN data upload
 *          input: endp  - end-point numbers
 *                 *pbuf - data buffer
 *                 len   - load data length
 *                 mod   - 0: DEF_UEP_DMA_LOAD 1: DEF_UEP_CPY_LOAD
 *
 * @return  0 - upload started, 1 - end-point busy or not available
 */
uint8_t USB_Endp_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod)
{
    if(endp != DEF_UEP2)
    {
        return 1;
    }
    if(USB_Endp_Busy[DEF_UEP2] == DEF_UEP_BUSY)
    {
        return 1;
    }

    if(mod == DEF_UEP_DMA_LOAD)
    {
        /* DMA mode */
        R32_UEP2_DMA = (uint32_t)pbuf;
    }
    else if(mod == DEF_UEP_CPY_LOAD)
    {
        /* copy mode, the buffer address is fixed */
        R32_UEP2_DMA = (uint32_t)(uint8_t *)UDisk_In_Buf;
        memcpy(UDisk_In_Buf, pbuf, len);
    }
    else
    {
        return 1;
    }

    /* tx length, response ack, set end-point busy */
    R8_UEP2_T_LEN = len;
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    USB_Endp_Busy[DEF_UEP2] = DEF_UEP_BUSY;

    return 0;
}

/*********************************************************************
 * @fn      USB_DevTransProcess
 *
 * @brief   USB transfer process function, executed in the USB interrupt.
 *
 * @return  none
 */
void USB_DevTransProcess(void)
{
    uint8_t len;
    uint8_t intflag, intst, errflag = 0;

    intflag = R8_USB_INT_FG;
    intst = R8_USB_INT_ST;
    if(intflag & RB_UIF_TRANSFER)
    {
        if((intst & MASK_UIS_TOKEN) != MASK_UIS_TOKEN) // not idle
        {
            /* data-in stage processing */
            switch(intst & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
            {
                /* end-point 0 data in interrupt */
                case UIS_TOKEN_IN | DEF_UEP0:
                    if(USB_SetupReqLen == 0)
                    {
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
                    }
                    if((USB_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
                    {
                        /* Non-standard request endpoint 0 Data upload */
                    }
                    else
                    {
                        switch(USB_SetupReqCode)
                        {
                            case USB_GET_DESCRIPTOR:
                                len = USB_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USB_SetupReqLen;
                                memcpy(pEP0_DataBuf, pUSB_Descr, len);
                                USB_SetupReqLen -= len;
                                pUSB_Descr += len;
                                R8_UEP0_T_LEN = len;
                                R8_UEP0_CTRL ^= RB_UEP_T_TOG; // toggle
                                break;

                            case USB_SET_ADDRESS:
                                R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | USB_DevAddr;
                                break;

                            default:
                                break;
                        }
                    }
                    break;

                /* end-point 2 data in interrupt (bulk IN, U-disk data upload) */
                case UIS_TOKEN_IN | DEF_UEP2:
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
                    R8_UEP2_CTRL ^= RB_UEP_T_TOG;
                    USB_Endp_Busy[DEF_UEP2] = DEF_UEP_FREE;
                    UDISK_In_EP_Deal();
                    break;

                /* end-point 0 data out interrupt */
                case UIS_TOKEN_OUT | DEF_UEP0:
                    len = R8_USB_RX_LEN;
                    if(intst & RB_UIS_TOG_OK)
                    {
                        if((USB_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
                        {
                            /* Non-standard request end-point 0 Data download */
                        }
                        else
                        {
                            /* Standard request end-point 0 Data download */
                        }
                        if(USB_SetupReqLen == 0)
                        {
                            R8_UEP0_T_LEN = 0;
                            R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK | RB_UEP_T_TOG;
                        }
                    }
                    break;

                /* end-point 3 data out interrupt (bulk OUT, U-disk data download) */
                case UIS_TOKEN_OUT | DEF_UEP3:
                    if(intst & RB_UIS_TOG_OK)   // packets with wrong data toggle are dropped
                    {
                        len = R8_USB_RX_LEN;
                        R8_UEP3_CTRL ^= RB_UEP_R_TOG;
                        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
                        UDISK_Out_EP_Deal(UDisk_Out_Buf, len);
                        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
                    }
                    break;
                default:
                    break;
            }
            R8_USB_INT_FG = RB_UIF_TRANSFER;
        }

        /* setup packet processing */
        if(intst & RB_UIS_SETUP_ACT)
        {
            R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
            /* Store All Setup Values */
            USB_SetupReqType = pUSB_SetupReqPak->bRequestType;
            USB_SetupReqCode = pUSB_SetupReqPak->bRequest;
            USB_SetupReqLen = pUSB_SetupReqPak->wLength;
            USB_SetupReqValue = pUSB_SetupReqPak->wValue;
            USB_SetupReqIndex = pUSB_SetupReqPak->wIndex;
            len = 0;
            errflag = 0;
            if((USB_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
            {
                /* usb non-standard request processing */
                if((USB_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS)
                {
                    if(USB_SetupReqCode == CMD_UDISK_GET_MAX_LUN)
                    {
                        /* bulk-only mass storage: only one LUN, return 0 */
                        pEP0_DataBuf[0] = 0;
                        pUSB_Descr = (uint8_t *)pEP0_DataBuf;
                        len = 1;
                    }
                    else if(USB_SetupReqCode == CMD_UDISK_RESET)
                    {
                        /* UDisk Reset */
                        Udisk_Sense_Key = 0x00;
                        Udisk_Sense_ASC = 0x00;
                        Udisk_CSW_Status = 0x00;
                        Udisk_Transfer_Status = 0x00;
                        UDISK_Transfer_DataLen = 0x00;
                    }
                    else
                    {
                        /* other class request is not supported */
                        errflag = 0xFF;
                    }
                }
                else
                {
                    errflag = 0xFF;
                }
            }
            else /* standard request */
            {
                switch(USB_SetupReqCode)
                {
                    case USB_GET_DESCRIPTOR:
                    {
                        switch((uint8_t)(USB_SetupReqValue >> 8))
                        {
                            case USB_DESCR_TYP_DEVICE:
                                pUSB_Descr = MyDevDescr;
                                len = DEF_USBD_DEVICE_DESC_LEN;
                                break;

                            case USB_DESCR_TYP_CONFIG:
                                pUSB_Descr = MyCfgDescr;
                                len = DEF_USBD_CONFIG_DESC_LEN;
                                break;

                            case USB_DESCR_TYP_STRING:
                                switch((uint8_t)(USB_SetupReqValue & 0xFF))
                                {
                                    /* Descriptor 0, Language descriptor */
                                    case DEF_STRING_DESC_LANG:
                                        pUSB_Descr = MyLangDescr;
                                        len = DEF_USBD_LANG_DESC_LEN;
                                        break;

                                    /* Descriptor 1, Manufacturers String descriptor */
                                    case DEF_STRING_DESC_MANU:
                                        pUSB_Descr = MyManuInfo;
                                        len = DEF_USBD_MANU_DESC_LEN;
                                        break;

                                    /* Descriptor 2, Product String descriptor */
                                    case DEF_STRING_DESC_PROD:
                                        pUSB_Descr = MyProdInfo;
                                        len = DEF_USBD_PROD_DESC_LEN;
                                        break;

                                    /* Descriptor 3, Serial-number String descriptor */
                                    case DEF_STRING_DESC_SERN:
                                        pUSB_Descr = MySerNumInfo;
                                        len = DEF_USBD_SN_DESC_LEN;
                                        break;

                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                                break;

                            default:
                                errflag = 0xFF;
                                break;
                        }
                        if(USB_SetupReqLen > len)
                        {
                            USB_SetupReqLen = len;  // actual total upload length
                        }
                        len = (USB_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USB_SetupReqLen;
                        memcpy(pEP0_DataBuf, pUSB_Descr, len);
                        pUSB_Descr += len;
                    }
                    break;

                    /* Set usb address */
                    case USB_SET_ADDRESS:
                        USB_DevAddr = (uint8_t)(USB_SetupReqValue & 0xFF);
                        break;

                    case USB_GET_CONFIGURATION:
                        pEP0_DataBuf[0] = USB_DevConfig;
                        if(USB_SetupReqLen > 1)
                        {
                            USB_SetupReqLen = 1;
                        }
                        break;

                    case USB_SET_CONFIGURATION:
                        USB_DevConfig = (uint8_t)(USB_SetupReqValue & 0xFF);
                        USB_DevEnumStatus = 0x01;
                        break;

                    case USB_CLEAR_FEATURE:
                        if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                        {
                            /* clear one device feature */
                            if((uint8_t)(USB_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP)
                            {
                                /* clear usb sleep status, device not prepare to sleep */
                                USB_DevSleepStatus &= ~0x01;
                            }
                        }
                        else if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                        {
                            /* Clear End-point Feature */
                            if((uint8_t)(USB_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT)
                            {
                                switch((uint8_t)(USB_SetupReqIndex & 0xFF))
                                {
                                    case (DEF_UEP_IN | DEF_UEP2):
                                        /* Set End-point 2 IN NAK */
                                        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
                                        if(Udisk_Transfer_Status & DEF_UDISK_CSW_UP_FLAG)
                                        {
                                            UDISK_Up_CSW();
                                        }
                                        break;

                                    case (DEF_UEP_OUT | DEF_UEP3):
                                        /* Set End-point 3 OUT ACK */
                                        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
                                        if(Udisk_Transfer_Status & DEF_UDISK_CSW_UP_FLAG)
                                        {
                                            UDISK_Up_CSW();
                                        }
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
                        if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                        {
                            /* Set Device Feature */
                            if((uint8_t)(USB_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP)
                            {
                                if(MyCfgDescr[7] & 0x20)
                                {
                                    /* Set Wake-up flag, device prepare to sleep */
                                    USB_DevSleepStatus |= 0x01;
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
                        }
                        else if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                        {
                            /* Set End-point Feature */
                            if((uint8_t)(USB_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT)
                            {
                                /* Set end-points status stall */
                                switch((uint8_t)(USB_SetupReqIndex & 0xFF))
                                {
                                    case (DEF_UEP_IN | DEF_UEP2):
                                        /* Set End-point 2 IN STALL */
                                        R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_STALL;
                                        break;

                                    case (DEF_UEP_OUT | DEF_UEP3):
                                        /* Set End-point 3 OUT STALL */
                                        R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_STALL;
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

                    case USB_GET_INTERFACE:
                        pEP0_DataBuf[0] = 0x00;
                        if(USB_SetupReqLen > 1)
                        {
                            USB_SetupReqLen = 1;
                        }
                        break;

                    case USB_SET_INTERFACE:
                        break;

                    /* host get status of specified device/interface/end-points */
                    case USB_GET_STATUS:
                        pEP0_DataBuf[0] = 0x00;
                        pEP0_DataBuf[1] = 0x00;
                        if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                        {
                            if(USB_DevSleepStatus & 0x01)
                            {
                                pEP0_DataBuf[0] = 0x02;
                            }
                        }
                        else if((USB_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                        {
                            switch((uint8_t)(USB_SetupReqIndex & 0xFF))
                            {
                                case (DEF_UEP_IN | DEF_UEP2):
                                    if((R8_UEP2_CTRL & MASK_UEP_T_RES) == UEP_T_RES_STALL)
                                    {
                                        pEP0_DataBuf[0] = 0x01;
                                    }
                                    break;

                                case (DEF_UEP_OUT | DEF_UEP3):
                                    if((R8_UEP3_CTRL & MASK_UEP_R_RES) == UEP_R_RES_STALL)
                                    {
                                        pEP0_DataBuf[0] = 0x01;
                                    }
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

                        if(USB_SetupReqLen > 2)
                        {
                            USB_SetupReqLen = 2;
                        }
                        break;

                    default:
                        errflag = 0xFF;
                        break;
                }
            }

            /* if the request is not supported, response STALL */
            if(errflag == 0xFF)
            {
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
            }
            else
            {
                if(USB_SetupReqType & DEF_UEP_IN)   /* upload */
                {
                    len = (USB_SetupReqLen > DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USB_SetupReqLen;
                    USB_SetupReqLen -= len;
                    R8_UEP0_T_LEN = len;
                    R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                }
                else
                {
                    /* download */
                    if(USB_SetupReqLen == 0)
                    {
                        R8_UEP0_T_LEN = 0;
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                    else
                    {
                        R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
                    }
                }
            }

            R8_USB_INT_FG = RB_UIF_TRANSFER;
        }
    }
    else if(intflag & RB_UIF_BUS_RST)
    {
        /* usb reset interrupt processing */
        USB_DevConfig = 0;
        USB_DevAddr = 0;
        USB_DevSleepStatus = 0;
        USB_DevEnumStatus = 0;

        R8_USB_DEV_AD = 0;
        USB_Device_Endp_Init();
        R8_USB_INT_FG |= RB_UIF_BUS_RST;
    }
    else if(intflag & RB_UIF_SUSPEND)
    {
        /* usb suspend interrupt processing */
        R8_USB_INT_FG = RB_UIF_SUSPEND;
        if(R8_USB_MIS_ST & RB_UMS_SUSPEND)
        {
            USB_DevSleepStatus |= 0x02;
            if(USB_DevSleepStatus == 0x03)
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USB_DevSleepStatus &= ~0x02;
        }
    }
    else
    {
        /* other interrupts */
        R8_USB_INT_FG = intflag;
    }
}

/*********************************************************************
 * @fn      USB_IRQHandler
 *
 * @brief   USB interrupt service routine
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
    USB_DevTransProcess();
}
