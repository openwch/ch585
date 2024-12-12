/********************************** (C) COPYRIGHT *******************************
* File Name          : UART.C
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/07/31
* Description        : uart serial port related initialization and processing
*******************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "UART.h"

/*******************************************************************************/
/* Variable Definition */

/* Global */
/* The following are serial port transmit and receive related variables and buffers */
volatile CDC_CTL CDC;

__attribute__ ((aligned(4))) uint8_t  UART2_Tx_Buf[ UART_REV_BUFFLEN ];  /* Serial port 2 transmit data buffer */
__attribute__ ((aligned(4))) uint8_t  UART2_Rx_Buf[ UART_REV_BUFFLEN ];  /* Serial port 2 receive data buffer */

/*********************************************************************
 * @fn      UART2_ParaInit
 *
 * @brief   CDC2 parameters initialization
 *          mode = 0 : Used in usb modify initialization
 *          mode = 1 : Used in default initializations
 * @return  none
 */
void UART2_ParaInit( uint8_t mode )
{
    CDC.USB_Output_Ptr = 0;
    CDC.Uart_Output_Ptr = 0;
    CDC.Uart_Input_Ptr = 0;
    CDC.Uart_RecLen = 0;
    CDC.UploadPoint_Busy = 0;
    CDC.DownloadPoint_Busy = 1;
    CDC.Uart_Timeout_Count = 0;

    if( mode )
    {
        CDC.Com_Cfg[ 0 ] = (uint8_t)( DEF_UARTx_BAUDRATE );
        CDC.Com_Cfg[ 1 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 8 );
        CDC.Com_Cfg[ 2 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 16 );
        CDC.Com_Cfg[ 3 ] = (uint8_t)( DEF_UARTx_BAUDRATE >> 24 );
        CDC.Com_Cfg[ 4 ] = DEF_UARTx_STOPBIT;
        CDC.Com_Cfg[ 5 ] = DEF_UARTx_PARITY;
        CDC.Com_Cfg[ 6 ] = DEF_UARTx_DATABIT;
        CDC.Com_Cfg[ 7 ] = DEF_UARTx_RX_TIMEOUT;
    }
}

/*********************************************************************
 * @fn      UART2_Init
 *
 * @brief   CDC2 total initialization
 *          mode     : See the useage of UART2_ParaInit( mode )
 *          baudrate : Serial port 2 default baud rate
 *          stopbits : Serial port 2 default stop bits
 *          parity   : Serial port 2 default parity
 *
 * @return  none
 */
void UART2_Init( uint8_t mode, uint32_t baudrate, uint8_t stopbits, uint8_t parity )
{
    /* 配置串口2：先配置IO口模式，再配置串口 */
    GPIOA_SetBits(GPIO_Pin_7);
    GPIOA_ModeCfg(GPIO_Pin_6, GPIO_ModeIN_PU);      // RXD-配置上拉输入
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // TXD-配置推挽输出，注意先让IO口输出高电平
    UART2_BaudRateCfg(baudrate);
    R8_UART2_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    R8_UART2_IER = RB_IER_TXD_EN;
    R8_UART2_DIV = 1;

    if(stopbits == 1)
       R8_UART2_LCR |= RB_LCR_STOP_BIT ;
    /* Check digit (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space); */
    if(parity == 1)
        R8_UART2_LCR |= (RB_LCR_PAR_EN) ;
    else if(parity == 2)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x10) ;
    else if(parity == 3)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x20) ;
    else if(parity == 4)
        R8_UART2_LCR |= (RB_LCR_PAR_EN | 0x30) ;

    UART2_ByteTrigCfg(UART_7BYTE_TRIG);
    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART2_IRQn);

    UART2_ParaInit( mode );
}

/*********************************************************************
 * @fn      UART2_USB_Init
 *
 * @brief   CDC2 initialization in usb interrupt
 *
 * @return  none
 */
void UART2_USB_Init( void )
{
    uint32_t baudrate;
    uint8_t  stopbits;
    uint8_t  parity;

    baudrate = ( uint32_t )( CDC.Com_Cfg[ 3 ] << 24 ) + ( uint32_t )( CDC.Com_Cfg[ 2 ] << 16 );
    baudrate += ( uint32_t )( CDC.Com_Cfg[ 1 ] << 8 ) + ( uint32_t )( CDC.Com_Cfg[ 0 ] );
    stopbits = CDC.Com_Cfg[ 4 ];
    parity = CDC.Com_Cfg[ 5 ];

    UART2_Init( 0, baudrate, stopbits, parity );

    /* restart usb receive  */
    R32_U2EP2_RX_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
    R8_U2EP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
    R8_U2EP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
}

/*********************************************************************
 * @fn      UART2_DataTx_Deal
 *
 * @brief   CDC2 data transmission processing
 *
 * @return  none
 */
void UART2_DataTx_Deal( void )
{
    if( CDC.USB_RecLen )
    {
        UART2_SendString(&UART2_Tx_Buf[CDC.USB_Output_Ptr++],1);
        CDC.USB_RecLen--;
    }

    if( CDC.DownloadPoint_Busy == 0 )
    {
        if( CDC.USB_RecLen == 0 )
        {
            CDC.USB_Output_Ptr = 0;
            CDC.DownloadPoint_Busy = 1;
            R32_U2EP2_RX_DMA = (uint32_t)(uint8_t *)&UART2_Tx_Buf[ 0 ];
            R8_U2EP2_RX_CTRL = ( R8_U2EP2_RX_CTRL &~ USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_ACK;
        }
    }
}

/*********************************************************************
 * @fn      UART2_DataRx_Deal
 *
 * @brief   CDC2 data receiving processing
 *
 * @return  none
 */
void UART2_DataRx_Deal( void )
{
    uint16_t usb_sendlenth = 0;         //USB upload data length
    if( CDC.UploadPoint_Busy == 0 )
    {
        usb_sendlenth = CDC.Uart_RecLen;
        if(usb_sendlenth > 0)
        {
            if( (usb_sendlenth >= (USBHS_DevMaxPackLen - 1) || CDC.Uart_Timeout_Count > DEF_UARTx_RX_TIMEOUT) /*&& CDC.DownloadPoint_Busy == 0*/)
            {
                if( CDC.Uart_Output_Ptr + usb_sendlenth > UART_REV_BUFFLEN )
                {
                    usb_sendlenth = UART_REV_BUFFLEN - CDC.Uart_Output_Ptr;
                }
                if(usb_sendlenth > (USBHS_DevMaxPackLen - 1) )
                {
                    usb_sendlenth = (USBHS_DevMaxPackLen - 1);
                }

                CDC.Uart_RecLen -= usb_sendlenth;
                memcpy(USBHS_EP2_Tx_Buf,&UART2_Rx_Buf[CDC.Uart_Output_Ptr],usb_sendlenth);
                CDC.Uart_Output_Ptr += usb_sendlenth;
                if(CDC.Uart_Output_Ptr >= UART_REV_BUFFLEN )
                {
                    CDC.Uart_Output_Ptr = 0;
                }

                CDC.UploadPoint_Busy = 1;

                R16_U2EP2_T_LEN = usb_sendlenth;
                R8_U2EP2_TX_CTRL = (R8_U2EP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;

                CDC.Uart_Timeout_Count = 0;
            }
        }
    }
}

/*********************************************************************
 * @fn      TMR2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR2_IRQHandler( void )
{
    if( TMR2_GetITFlag(RB_TMR_IF_CYC_END) != RESET )
    {
        /* Clear interrupt flag */
        TMR2_ClearITFlag(RB_TMR_IF_CYC_END);

        /* uart timeout counts */
        CDC.Uart_Timeout_Count++;
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
            printf("error:%x\n",R8_UART2_LSR);
            break;

        case UART_II_RECV_RDY:          //Data reaches the trigger point
            for(rec_length = 0; rec_length < 7; rec_length++)
            {
                UART2_Rx_Buf[CDC.Uart_Input_Ptr++] = UART2_RecvByte();
                if(CDC.Uart_Input_Ptr >= UART_REV_BUFFLEN )
                {
                    CDC.Uart_Input_Ptr = 0;
                }
            }
            CDC.Uart_RecLen += rec_length;
            CDC.Uart_Timeout_Count = 0;

            break;
        case UART_II_RECV_TOUT:         //Receive timeout
            rec_length = UART2_RecvString(recbuff);
            for(i = 0; i < rec_length ; i++)
            {
                UART2_Rx_Buf[CDC.Uart_Input_Ptr++] = recbuff[i];
                if(CDC.Uart_Input_Ptr>=UART_REV_BUFFLEN )
                {
                    CDC.Uart_Input_Ptr = 0;
                }
            }
            CDC.Uart_RecLen += i;
            CDC.Uart_Timeout_Count = 0;
            break;

        case UART_II_THR_EMPTY:         //Send buffer empty
            break;

        default:
            break;
    }
}

