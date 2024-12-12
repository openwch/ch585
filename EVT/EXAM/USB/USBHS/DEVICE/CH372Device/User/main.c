/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
/*
 *@Note
  Example routine to emulate a custom USB device (CH372 device).
  This routine demonstrates the use of a USBHS Device to emulate a custom device, the CH372,
  with endpoints 1/3/5 downlinking data and uploading via endpoints 1/4/6 respectively
  Endpoint 1 uploads and downlinks via ring buffer with no data reversal, endpoints 3/4, and endpoints 5/6 copy and upload.
  The device can be operated using Bushund or other upper computer software.
  Note: This routine needs to be demonstrated in conjunction with the host software.

  If the USB is set to high-speed, an external crystal oscillator is recommended for the clock source.
*/

#include <ch585_usbhs_device.h>
#include "string.h"
#include "CH58x_common.h"
#include <stdio.h>

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   µ÷ÊÔ³õÊ¼»¯
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
    uint8_t ret;
    SetSysClock( CLK_SOURCE_HSE_PLL_62_4MHz );
    DebugInit();
    PRINT( "CH372Device Running On USBHS Controller\n" );
    USBHS_Device_Init(ENABLE);
    PFIC_EnableIRQ( USB2_DEVICE_IRQn );

    while(1)
    {
        /* Determine if enumeration is complete, perform data transfer if completed */
        if(USBHS_DevEnumStatus)
        {
            /* Data Transfer */
            if(RingBuffer_Comm.RemainPack)
            {
                ret = USBHS_Endp_DataUp(DEF_UEP1, &Data_Buffer[(RingBuffer_Comm.DealPtr) * DEF_USBD_HS_PACK_SIZE], RingBuffer_Comm.PackLen[RingBuffer_Comm.DealPtr], DEF_UEP_DMA_LOAD);
                if(ret == 0)
                {
                    PFIC_DisableIRQ(USB2_DEVICE_IRQn);
                    RingBuffer_Comm.RemainPack--;
                    RingBuffer_Comm.DealPtr++;
                    if(RingBuffer_Comm.DealPtr == DEF_Ring_Buffer_Max_Blks)
                    {
                        RingBuffer_Comm.DealPtr = 0;
                    }
                    PFIC_EnableIRQ(USB2_DEVICE_IRQn);
                }
            }

            /* Monitor whether the remaining space is available for further downloads */
            if(RingBuffer_Comm.RemainPack < (DEF_Ring_Buffer_Max_Blks - DEF_RING_BUFFER_RESTART))
            {
                if(RingBuffer_Comm.StopFlag)
                {
                    RingBuffer_Comm.StopFlag = 0;
                    R8_U2EP1_RX_CTRL = (R8_U2EP1_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
                }
            }
        }
    }
}
