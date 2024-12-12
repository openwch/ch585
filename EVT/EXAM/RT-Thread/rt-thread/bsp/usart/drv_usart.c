 /********************************** (C) COPYRIGHT *******************************
 * File Name          : drv_usart.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/08/13
 * Description
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "drv_usart.h"
#include "rtthread.h"
#include "rtdevice.h"
#include <string.h>

#define RT_UART_RINGBUF_SIZE        128
static struct rt_ringbuffer rt_uart_ringbuf;
ALIGN(RT_ALIGN_SIZE) uint8_t rt_uart_buffer[RT_UART_RINGBUF_SIZE];

int usart_init(void)
{
    rt_ringbuffer_init(&rt_uart_ringbuf, rt_uart_buffer, RT_UART_RINGBUF_SIZE);

    /* 配置串口0：先配置IO口模式，再配置串口 */
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);

    UART0_BaudRateCfg(115200);
    R8_UART0_FCR = (0 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART0_LCR = RB_LCR_WORD_SZ;
    R8_UART0_IER = RB_IER_TXD_EN;
    R8_UART0_DIV = 1;

    UART0_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART0_IRQn);

    return 0;
}
INIT_BOARD_EXPORT(usart_init);//add to board auto initial



// output
void rt_hw_console_output(const char *str)
{
    rt_size_t i = 0, size = 0;
    char a = '\r';

    size = rt_strlen(str);
    for (i = 0; i < size; i++)
    {
        if (*(str + i) == '\n')
        {
            while(R8_UART0_TFC == UART_FIFO_SIZE);
            R8_UART0_THR = a;   //if end of '\n', then add '\r'
        }
        while(R8_UART0_TFC == UART_FIFO_SIZE);
        R8_UART0_THR = *(str + i);
    }
}


//input
char rt_hw_console_getchar(void)
{
    /* Note: the initial value of ch must < 0 */
    rt_uint8_t ch;

    if(rt_ringbuffer_getchar(&rt_uart_ringbuf, &ch) != 0)
    {
        return ch;
    }
    else
    {
        rt_thread_mdelay(10);
        return -1;
    }
    return ch;
}

__HIGH_CODE
void UART0_IRQHandler(void)
{
    uint8_t v;
    switch (UART0_GetITFlag())
    {
    case UART_II_LINE_STAT:
        v = UART0_GetLinSTA();
        break;
    case UART_II_RECV_RDY:
    case UART_II_RECV_TOUT:
    {
        while (R8_UART0_RFC)
        {
            v = R8_UART0_RBR;
            rt_ringbuffer_putchar(&rt_uart_ringbuf, v);
        }
        break;
    }
    default:
        break;
    }
}
