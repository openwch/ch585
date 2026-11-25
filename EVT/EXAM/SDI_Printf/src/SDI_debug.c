/********************************** (C) COPYRIGHT *******************************
 * File Name          : SDI_debug.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : SDI打印功能,使用此功能需要
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"
#include "SDI_debug.h"

#define DEBUG_DATA0_ADDRESS  ((volatile uint32_t*)0xE0000340)
#define DEBUG_DATA1_ADDRESS  ((volatile uint32_t*)0xE0000344)

/*********************************************************************
 * @fn      SDI_Printf_Enable
 *
 * @brief   Initializes the SDI printf Function.
 *
 * @param   None
 *
 * @return  None
 */
void SDI_Printf_Enable(void)
{
    *(DEBUG_DATA0_ADDRESS) = 0;
    DelayMs(1);
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
__attribute__((used))
int _write(int fd, char *buf, int size)
{
    int i = 0;
    (void)fd;
    int writeSize = size;
    do
    {
        /**
         * data0 + data1 = 8 bytes
         * data0 The lowest byte storage length, the maximum is 7
         *
         */
        while( (*(DEBUG_DATA0_ADDRESS) != 0u))
        {

        }

        if(writeSize>7)
        {
            *(DEBUG_DATA1_ADDRESS) =      \
              ((uint8_t)(buf[i+3])) |     \
              ((uint8_t)(buf[i+4]) << 8) |  \
              ((uint8_t)(buf[i+5]) << 16) | \
              ((uint8_t)(buf[i+6]) << 24);
            *(DEBUG_DATA0_ADDRESS) =      \
              (7u) |                      \
              ((uint8_t)(buf[i]) << 8) |    \
              ((uint8_t)(buf[i+1]) << 16) | \
              ((uint8_t)(buf[i+2]) << 24);

            i += 7;
            writeSize -= 7;
        }
        else
        {
            uint8_t tmp = (uint8_t)writeSize;
            *(DEBUG_DATA1_ADDRESS) =                       \
                ((tmp > 3 ? (uint8_t)buf[i+3] : 0)) |      \
                ((tmp > 4 ? (uint8_t)buf[i+4] : 0) << 8) | \
                ((tmp > 5 ? (uint8_t)buf[i+5] : 0) << 16) |\
                ((tmp > 6 ? (uint8_t)buf[i+6] : 0) << 24);
            *(DEBUG_DATA0_ADDRESS) =
                (tmp) | ((uint8_t)buf[i] << 8) |           \
                ((tmp > 1 ? (uint8_t)buf[i+1] : 0) << 16) |\
                ((tmp > 2 ? (uint8_t)buf[i+2] : 0) << 24);
            writeSize = 0;
        }

    } while (writeSize);

    return size;
}
