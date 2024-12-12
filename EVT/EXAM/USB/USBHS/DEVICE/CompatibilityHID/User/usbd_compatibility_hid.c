/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_compatibility_hid.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        :
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <ch585_usbhs_device.h>
#include "usbd_compatibility_hid.h"

__attribute__ ((aligned(4))) uint8_t UART2_Rx_Buf[UART_REV_BUFFLEN];  // UART2 Rx Buffer
__attribute__ ((aligned(4))) uint8_t HID_Report_Buffer[DEF_USBD_HS_PACK_SIZE];              // HID Report Buffer
volatile uint8_t HID_Set_Report_Flag = SET_REPORT_DEAL_OVER;               // HID SetReport flag
volatile uint16_t Data_Pack_Max_Len = 0;                                   // UART data packet length in hid packet
volatile uint16_t Head_Pack_Len = 0;                                       // UART head packet( valid data length ) length in hid packet
volatile uint16_t Uart_Input_Ptr;      //Circular buffer write pointer
volatile uint16_t Uart_RecLen;         //The number of bytes remaining to be fetched in the current buffer
volatile uint16_t USB_RecLen;          //Data received by USB endpoint
volatile uint8_t  UploadPoint_Busy;    //Upload whether the endpoint is busy
volatile uint16_t Uart_Timeout_Count;  //Timeout processing calculation time

/*********************************************************************
 * @fn      TMR2_IRQHandler
 *
 * @brief   TIM2 IRQ handler
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR2_IRQHandler(void)
{
    if( TMR2_GetITFlag(RB_TMR_IF_CYC_END) != RESET )
    {
        /* Clear interrupt flag */
        TMR2_ClearITFlag(RB_TMR_IF_CYC_END);
        Uart_Timeout_Count++;
    }
}

/*******************************************************************************
 * @fn        UART2_IRQHandler
 *
 * @brief     CDC serial port interrupt function
 *
 * @return    None
 */
__INTERRUPT
__HIGH_CODE
void UART2_IRQHandler(void)
{
    UINT8 i,rec_length;
    UINT8 recbuff[7] = {0};

    switch( UART2_GetITFlag() )
    {
        case UART_II_LINE_STAT:         //Line status error
            PRINT("error:%x\n",R8_UART2_LSR);
            break;

        case UART_II_RECV_RDY:          //Data reaches the trigger point
            for(rec_length = 0; rec_length < 7; rec_length++)
            {
                UART2_Rx_Buf[Uart_Input_Ptr++] = UART2_RecvByte();
                if(Uart_Input_Ptr >= UART_REV_BUFFLEN )
                {
                    Uart_Input_Ptr = 0;
                }
            }
            Uart_RecLen += rec_length;
            Uart_Timeout_Count = 0;

            break;
        case UART_II_RECV_TOUT:         //Receive timeout
            rec_length = UART2_RecvString(recbuff);
            for(i = 0; i < rec_length ; i++)
            {
                UART2_Rx_Buf[Uart_Input_Ptr++] = recbuff[i];
                if(Uart_Input_Ptr>=UART_REV_BUFFLEN )
                {
                    Uart_Input_Ptr = 0;
                }
            }
            Uart_RecLen += i;
            Uart_Timeout_Count = 0;
            break;

        case UART_II_THR_EMPTY:         //Send buffer empty
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      UART2_Init
 *
 * @brief   Uart2 total initialization
 *          baudrate : Serial port 2 default baud rate
 *
 * @return  none
 */
void USART2_Init( uint32_t baudrate )
{
    /* 配置串口2：先配置IO口模式，再配置串口 */
    GPIOA_SetBits(GPIO_Pin_7);
    GPIOA_ModeCfg(GPIO_Pin_6, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART2_BaudRateCfg(baudrate);
    R8_UART2_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN; // FIFO打开，触发点4字节
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    R8_UART2_IER = RB_IER_TXD_EN;
    R8_UART2_DIV = 1;

    UART2_ByteTrigCfg(UART_7BYTE_TRIG);
    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART2_IRQn);

    Data_Pack_Max_Len = 0;
    Head_Pack_Len = 0;
    Uart_Input_Ptr = 0;
    Uart_RecLen = 0;
    USB_RecLen = 0;
    UploadPoint_Busy = 0;
    Uart_Timeout_Count = 0;

}

/*********************************************************************
 * @fn      UART2_Rx_Service
 *
 * @brief   UART2 rx service routine that sends the data received by
 *          uart2 via USB-HID.
 *
 * @return  none
 */
void UART2_Rx_Service( void )
{
    uint16_t usb_sendlenth = 0;         //USB upload data length
    static uint16_t uart_output_ptr = 0;//Loop buffer fetch pointer
    if( UploadPoint_Busy == 0 )
    {
        usb_sendlenth = Uart_RecLen;
        if(usb_sendlenth > 0)
        {
            if( (usb_sendlenth >= USBHS_UART_REV_LEN - Head_Pack_Len) || Uart_Timeout_Count > DEF_UART2_TOUT_TIME)
            {
                if( uart_output_ptr + usb_sendlenth > UART_REV_BUFFLEN - Head_Pack_Len )
                {
                    usb_sendlenth = UART_REV_BUFFLEN - Head_Pack_Len - uart_output_ptr;
                }
                if( usb_sendlenth > USBHS_UART_REV_LEN - Head_Pack_Len )
                {
                    usb_sendlenth = USBHS_UART_REV_LEN - Head_Pack_Len;
                }

                Uart_RecLen -= usb_sendlenth;
                memset(USBHS_EP2_Tx_Buf,0,DEF_USBD_HS_PACK_SIZE);
                memcpy(USBHS_EP2_Tx_Buf+Head_Pack_Len,&UART2_Rx_Buf[uart_output_ptr],usb_sendlenth);
                if( Head_Pack_Len == 1 )
                {
                    USBHS_EP2_Tx_Buf[0] = usb_sendlenth+Head_Pack_Len;
                }
                else
                {
                    USBHS_EP2_Tx_Buf[0] = (uint8_t)(usb_sendlenth & 0xff);
                    USBHS_EP2_Tx_Buf[1] = (uint8_t)(usb_sendlenth >> 8);
                }

                uart_output_ptr += usb_sendlenth;
                if(uart_output_ptr >= UART_REV_BUFFLEN )
                {
                    uart_output_ptr = 0;
                }

                UploadPoint_Busy = 1;
                R16_U2EP2_T_LEN = usb_sendlenth+Head_Pack_Len;
                R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                Uart_Timeout_Count = 0;
            }
        }
    }
}

/*********************************************************************
 * @fn      UART2_Tx_Service
 *
 * @brief   UART2 tx service routine that sends the data received by
 *          USB-HID via uart2.
 *
 * @return  none
 */
void UART2_Tx_Service( void )
{
    static uint16_t uartsendlen = 0;

    if( USB_RecLen )
    {
        if( Head_Pack_Len == 1 )
        {
            uartsendlen = USBHS_EP1_Rx_Buf[0];
        }
        else
        {
            uartsendlen = USBHS_EP1_Rx_Buf[0] | (USBHS_EP1_Rx_Buf[1]<<8);
        }
        if( uartsendlen >= Data_Pack_Max_Len )
        {
            uartsendlen = Data_Pack_Max_Len;
        }
        UART2_SendString(&USBHS_EP1_Rx_Buf[Head_Pack_Len],uartsendlen);
        USB_RecLen = 0;
        R8_U2EP1_RX_CTRL = ( R8_U2EP1_RX_CTRL &~ USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_ACK;
    }
}

/*********************************************************************
 * @fn      HID_Set_Report_Deal
 *
 * @brief   print hid set report data
 *
 * @return  none
 */
void HID_Set_Report_Deal( void )
{
    uint16_t i;
    if (HID_Set_Report_Flag == SET_REPORT_WAIT_DEAL)
    {
        PRINT("Set Report:\n");
        for (i = 0; i < USBHS_DevMaxPackLen; ++i)
        {
            PRINT("%02x ",HID_Report_Buffer[i]);
        }
        PRINT("\n");
        HID_Set_Report_Flag = SET_REPORT_DEAL_OVER;
        R16_U2EP0_T_LEN = 0;
        R8_U2EP0_TX_CTRL = (R8_U2EP0_TX_CTRL & USBHS_UEP_T_TOG_MASK) | USBHS_UEP_T_RES_ACK;
    }
}

