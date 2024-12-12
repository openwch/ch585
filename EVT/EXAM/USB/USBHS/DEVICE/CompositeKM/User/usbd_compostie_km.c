/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_composite_km.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : USB keyboard and mouse processing.
*********************************************************************************
* Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*******************************************************************************/
/* Header Files */
#include <ch585_usbhs_device.h>
#include <usbd_composite_km.h>

/*******************************************************************************/
/* Variable Definition */

/* Mouse */
volatile uint8_t  MS_Scan_Done = 0x00;                                          // Mouse Movement Scan Done
volatile uint16_t MS_Scan_Result = 0x0F00;                                      // Mouse Movement Scan Result
uint8_t  MS_Data_Pack[ 4 ] = { 0x00 };                                          // Mouse IN Data Packet

/* Keyboard */
volatile uint8_t  KB_Scan_Done = 0x00;                                          // Keyboard Keys Scan Done
volatile uint16_t KB_Scan_Result = 0x000F;                                      // Keyboard Keys Current Scan Result
volatile uint16_t KB_Scan_Last_Result = 0x000F;                                 // Keyboard Keys Last Scan Result
uint8_t  KB_Data_Pack[ 8 ] = { 0x00 };                                          // Keyboard IN Data Packet
volatile uint8_t  KB_LED_Last_Status = 0x00;                                    // Keyboard LED Last Result
volatile uint8_t  KB_LED_Cur_Status = 0x00;                                     // Keyboard LED Current Result

/* USART */
volatile uint8_t  USART_Recv_Dat = 0x00;
volatile uint8_t  USART_Send_Flag = 0x00;
volatile uint8_t  USART_Send_Cnt = 0x00;

typedef enum
{
    NoREADY = 0,
    READY = !NoREADY
} ErrorStatus;

/*********************************************************************
 * @fn      TMR3_IRQHandler
 *
 * @brief   This function handles TIM3 global interrupt request.
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR3_IRQHandler( void )
{
    if( TMR3_GetITFlag(RB_TMR_IF_CYC_END) != RESET )
    {
        /* Clear interrupt flag */
        TMR3_ClearITFlag(RB_TMR_IF_CYC_END);

        /* Handle keyboard scan */
        KB_Scan( );

        /* Handle mouse scan */
        MS_Scan( );

        /* Start timing for uploading the key value received from USART2 */
        if( USART_Send_Flag )
        {
            USART_Send_Cnt++;
        }
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
    GPIOA_SetBits(GPIO_Pin_7);
    GPIOA_ModeCfg(GPIO_Pin_6, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    UART2_BaudRateCfg(baudrate);
    R8_UART2_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART2_LCR = RB_LCR_WORD_SZ;
    R8_UART2_IER = RB_IER_TXD_EN;
    R8_UART2_DIV = 1;

    UART2_ByteTrigCfg(UART_7BYTE_TRIG);
    UART2_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART2_IRQn);
}

/*********************************************************************
 * @fn      UART2_IRQHandler
 *
 * @brief   This function handles UART2 global interrupt request.
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void UART2_IRQHandler( void )
{
    if( UART2_GetITFlag() ==  UART_II_RECV_RDY )
    {
        /* Save the key value received from USART2 */
        USART_Recv_Dat = UART2_RecvByte() & 0xFF;
    }
}

/*********************************************************************
 * @fn      USART2_Receive_Handle
 *
 * @brief   This function handles the key value received from USART2.
 *
 * @return  none
 */
void USART2_Receive_Handle( void )
{
    uint8_t status;
    static uint8_t flag = 0x00;

    if( flag == 0 )
    {
        /* Store the received specified key value into the keyboard data buffer */
        if( ( USART_Recv_Dat == DEF_KEY_CHAR_A ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_W ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_S ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_D ) )
        {
            memset( KB_Data_Pack, 0x00, sizeof( KB_Data_Pack ) );
            KB_Data_Pack[ 2 ] = USART_Recv_Dat;
            flag = 1;
        }
    }
    else if( flag == 1 )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Enable timing for uploading the key value */
            USART_Send_Cnt = 0;
            USART_Send_Flag = 1;
            flag = 2;
        }
    }
    else if( flag == 2 )
    {
        /* Delay 10ms to ensure that the key value is successfully uploaded,
         * and prepare the data packet indicating the key release.
         */
        if( USART_Send_Cnt >= 50 )
        {
            USART_Send_Flag = 0;
            memset( KB_Data_Pack, 0x00, sizeof( KB_Data_Pack ) );
            flag = 3;
        }
    }
    else if( flag == 3 )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        /* Clear variables for next reception */
        if( status == READY )
        {
            USART_Recv_Dat = 0;
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      KB_Scan_Init
 *
 * @brief   Initialize IO for keyboard scan.
 *
 * @return  none
 */
void KB_Scan_Init( void )
{
    GPIOB_ModeCfg(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_ModeIN_PU);
}

/*********************************************************************
 * @fn      KB_Sleep_Wakeup_Cfg
 *
 * @brief   Configure keyboard wake up mode.
 *
 * @return  none
 */
void KB_Sleep_Wakeup_Cfg( void )
{
    GPIOB_ITModeCfg(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_ITMode_FallEdge);
}

/*********************************************************************
 * @fn      KB_Scan
 *
 * @brief   Perform keyboard scan.
 *
 * @return  none
 */
void KB_Scan( void )
{
    static uint16_t scan_cnt = 0;
    static uint16_t scan_result = 0;

    scan_cnt++;
    if( ( scan_cnt % 10 ) == 0 )
    {
        scan_cnt = 0;

        /* Determine whether the two scan results are consistent */
        if( scan_result == ( GPIOB_ReadPortPin( 0x000F ) ) )
        {
            KB_Scan_Done = 1;
            KB_Scan_Result = scan_result;
        }
    }
    else if( ( scan_cnt % 5 ) == 0 )
    {
        /* Save the first scan result */
        scan_result = ( GPIOB_ReadPortPin( 0x000F ) );
    }
}

/*********************************************************************
 * @fn      KB_Scan_Handle
 *
 * @brief   Handle keyboard scan data.
 *
 * @return  none
 */
void KB_Scan_Handle( void )
{
    uint8_t i, j;
    uint8_t status;
    static uint8_t key_cnt = 0x00;
    static uint8_t flag = 0x00;

    if( KB_Scan_Done )
    {
        KB_Scan_Done = 0;

        if( KB_Scan_Result != KB_Scan_Last_Result )
        {
            for( i = 0; i < 4; i++ )
            {
                /* Determine that there is at least one key is pressed or released */
                if( ( KB_Scan_Result & ( 1 << i ) ) != ( KB_Scan_Last_Result & ( 1 << i ) ) )
                {
                    if( ( KB_Scan_Result & ( 1 << i ) ) )           // Key press
                    {
                        if( i == 0 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_W )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 1 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_A )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 2 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_S )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 3 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_D )
                                {
                                    break;
                                }
                            }
                        }

                        if( j == 8 )
                        {
                            KB_Data_Pack[ 5 ] = 0;
                        }
                        else
                        {
                            memcpy( &KB_Data_Pack[ j ], &KB_Data_Pack[ j + 1 ], ( 8 - j - 1 ) );
                            KB_Data_Pack[ 7 ] = 0;
                        }
                        key_cnt--;
                    }
                    else                                            // Key release
                    {
                        if( i == 0 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_W;
                        }
                        else if( i == 1 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_A;
                        }
                        else if( i == 2 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_S;
                        }
                        else if( i == 3 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_D;
                        }
                        key_cnt++;
                    }
                }
            }

            /* Copy the keyboard data to the buffer of endpoint 1 and set the data uploading flag */
            KB_Scan_Last_Result = KB_Scan_Result;
            flag = 1;
        }
    }

    if( flag )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Clear flag after successful loading */
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      KB_LED_Handle
 *
 * @brief   Handle keyboard lighting.
 *
 * @return  none
 */
void KB_LED_Handle( void )
{
    if( KB_LED_Cur_Status != KB_LED_Last_Status )
    {
        if( ( KB_LED_Cur_Status & 0x01 ) != ( KB_LED_Last_Status & 0x01 ) )
        {
            if( KB_LED_Cur_Status & 0x01 )
            {
                PRINT("Turn on the NUM LED\r\n");
            }
            else
            {
                PRINT("Turn off the NUM LED\r\n");
            }
        }
        if( ( KB_LED_Cur_Status & 0x02 ) != ( KB_LED_Last_Status & 0x02 ) )
        {
            if( KB_LED_Cur_Status & 0x02 )
            {
                PRINT("Turn on the CAPS LED\r\n");
            }
            else
            {
                PRINT("Turn off the CAPS LED\r\n");
            }
        }
        if( ( KB_LED_Cur_Status & 0x04 ) != ( KB_LED_Last_Status & 0x04 ) )
        {
            if( KB_LED_Cur_Status & 0x04 )
            {
                PRINT("Turn on the SCROLL LED\r\n");
            }
            else
            {
                PRINT("Turn off the SCROLL LED\r\n");
            }
        }
        KB_LED_Last_Status = KB_LED_Cur_Status;
    }
}

/*********************************************************************
 * @fn      MS_Scan_Init
 *
 * @brief   Initialize IO for mouse scan.
 *
 * @return  none
 */
void MS_Scan_Init( void )
{
    GPIOA_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11, GPIO_ModeIN_PU);
}

/*********************************************************************
 * @fn      MS_Sleep_Wakeup_Cfg
 *
 * @brief   Configure mouse wake up mode.
 *
 * @return  none
 */
void MS_Sleep_Wakeup_Cfg( void )
{
    GPIOA_ITModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11, GPIO_ITMode_FallEdge);
}

/*********************************************************************
 * @fn      MS_Scan
 *
 * @brief   Perform mouse scan.
 *
 * @return  none
 */
void MS_Scan( void )
{
    static uint16_t scan_cnt = 0;
    static uint16_t scan_result = 0;

    scan_cnt++;
    if( scan_cnt >= 2 )
    {
        scan_cnt = 0;

        /* Determine whether the two scan results are consistent */
        if( scan_result == ( GPIOA_ReadPort( ) & 0x0F00 ) )
        {
            MS_Scan_Result = scan_result;
            MS_Scan_Done = 1;
        }
    }
    else if( scan_cnt >= 1 )
    {
        /* Save the first scan result */
        scan_result = ( GPIOA_ReadPort( ) & 0x0F00 );
    }
}

/*********************************************************************
 * @fn      MS_Scan_Handle
 *
 * @brief   Handle mouse scan data.
 *
 * @return  none
 */
void MS_Scan_Handle( void )
{
    uint8_t i;
    uint8_t status;
    static uint8_t flag = 0x00;

    if( MS_Scan_Done )
    {
        MS_Scan_Done = 0;

        memset( MS_Data_Pack, 0x00, sizeof( MS_Data_Pack ) );
        for( i = 8; i < 12; i++ )
        {
            /* Determine that the mouse is moved */
            if( ( MS_Scan_Result & ( 1 << i ) ) == 0 )
            {
                if( i == 8 )
                {
                    MS_Data_Pack[ 1 ] += 0x02;
                }
                else if( i == 9 )
                {
                    MS_Data_Pack[ 1 ] += 0xFE;
                }
                else if( i == 10 )
                {
                    MS_Data_Pack[ 2 ] += 0x02;
                }
                else if( i == 11 )
                {
                    MS_Data_Pack[ 2 ] += 0xFE;
                }

                /* Set the data uploading flag */
                flag = 1;
            }
        }
    }

    if( flag )
    {
        /* Load mouse data to endpoint 2 */
        status = USBHS_Endp_DataUp( DEF_UEP2, MS_Data_Pack, sizeof( MS_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Clear flag after successful loading */
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      MCU_Sleep_Wakeup_Operate
 *
 * @brief   Perform sleep operation
 *
 * @return  none
 */
void MCU_Sleep_Wakeup_Operate( void )
{
    PRINT( "Sleep\r\n" );
    PFIC_DisableAllIRQ( );

    R8_USB2_CTRL &= ~USBHS_UD_PHY_SUSPENDM;
    GPIOA_ClearITFlagBit(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
    GPIOB_ClearITFlagBit(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    __WFE( );

    R8_USB2_CTRL |= USBHS_UD_PHY_SUSPENDM;

    if( GPIOA_ReadITFlagBit( (GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11) ) != RESET )
    {
        GPIOA_ClearITFlagBit(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
        USBHS_Send_Resume( );
    }
    else if ( GPIOB_ReadITFlagBit( (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3) ) != RESET )
    {
        GPIOB_ClearITFlagBit(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
        USBHS_Send_Resume( );
    }

    PFIC_EnableAllIRQ( );
    PRINT( "Wake\r\n" );
}
