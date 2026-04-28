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

#define DEBUG_UART_REMAP 1
#define DEBUG_UART_NUM 1
#define DEBUG_UART_BAUDRATE 921600

#define _UARTx_IRQn(x) UART##x##_IRQn
#define UARTx_IRQn(x) _UARTx_IRQn (x)

#define _UARTx_IRQHandler(x) UART##x##_IRQHandler
#define UARTx_IRQHandler(x) _UARTx_IRQHandler (x)

#define _UARTx_INTCfg(x) UART##x##_INTCfg
#define UARTx_INTCfg(x) _UARTx_INTCfg (x)

#define _UARTx_BaudRateCfg(x) UART##x##_BaudRateCfg
#define UARTx_BaudRateCfg(x) _UARTx_BaudRateCfg (x)

#define _R8_UARTx_TFC(x) R8_UART##x##_TFC
#define R8_UARTx_TFC(x) _R8_UARTx_TFC (x)

#define _R8_UARTx_THR(x) R8_UART##x##_THR
#define R8_UARTx_THR(x) _R8_UARTx_THR (x)

#define _R8_UARTx_RFC(x) R8_UART##x##_RFC
#define R8_UARTx_RFC(x) _R8_UARTx_RFC (x)

#define _R8_UARTx_RBR(x) R8_UART##x##_RBR
#define R8_UARTx_RBR(x) _R8_UARTx_RBR (x)

#define _UARTx_GetITFlag(x) UART##x##_GetITFlag()
#define UARTx_GetITFlag(x) _UARTx_GetITFlag (x)

#define _UARTx_GetLinSTA(x) UART##x##_GetLinSTA()
#define UARTx_GetLinSTA(x) _UARTx_GetLinSTA (x)

#define _UARTx_ByteTrigCfg(x) UART##x##_ByteTrigCfg
#define UARTx_ByteTrigCfg(x) _UARTx_ByteTrigCfg (x)

#define RT_UART_RINGBUF_SIZE 128
static struct rt_ringbuffer rt_uart_ringbuf;
ALIGN (RT_ALIGN_SIZE)
uint8_t rt_uart_buffer[RT_UART_RINGBUF_SIZE];

int usart_init (void) {
    rt_ringbuffer_init (&rt_uart_ringbuf, rt_uart_buffer, RT_UART_RINGBUF_SIZE);

    /* 配置串口0：先配置IO口模式，再配置串口 */
#if (DEBUG_UART_NUM == 0)
    GPIOA_SetBits (GPIO_Pin_14);
    GPIOPinRemap (ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg (GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg (GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
#elif (DEBUG_UART_NUM == 1)
#if DEBUG_UART_REMAP
    GPIOB_SetBits (GPIO_Pin_13);
    GPIOPinRemap (ENABLE, RB_PIN_UART1);
    GPIOB_ModeCfg (GPIO_Pin_12, GPIO_ModeIN_PU);       // RXD-配置上拉输入
    GPIOB_ModeCfg (GPIO_Pin_13, GPIO_ModeOut_PP_5mA);  // TXD-配置推挽输出，注意先让IO口输出高电平
#else
    GPIOA_SetBits (GPIO_Pin_9);
    GPIOA_ModeCfg (GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ModeCfg (GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
#endif
#elif (DEBUG_UART_NUM == 2)
#if DEBUG_UART_REMAP
    GPIOB_SetBits (GPIO_Pin_23);
    GPIOPinRemap (ENABLE, RB_PIN_UART2);
    GPIOB_ModeCfg (GPIO_Pin_22, GPIO_ModeIN_PU);       // RXD-配置上拉输入
    GPIOB_ModeCfg (GPIO_Pin_23, GPIO_ModeOut_PP_5mA);  // TXD-配置推挽输出，注意先让IO口输出高电平
#else
    GPIOA_SetBits (GPIO_Pin_7);
    GPIOA_ModeCfg (GPIO_Pin_6, GPIO_ModeIN_PU);
    GPIOA_ModeCfg (GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
#endif
#elif (DEBUG_UART_NUM == 3)
#if DEBUG_UART_REMAP
    GPIOB_SetBits (GPIO_Pin_21);
    GPIOPinRemap (ENABLE, RB_PIN_UART3);
    GPIOB_ModeCfg (GPIO_Pin_20, GPIO_ModeIN_PU);       // RXD-配置上拉输入
    GPIOB_ModeCfg (GPIO_Pin_21, GPIO_ModeOut_PP_5mA);  // TXD-配置推挽输出，注意先让IO口输出高电平
#else
    GPIOA_SetBits (GPIO_Pin_5);
    GPIOA_ModeCfg (GPIO_Pin_4, GPIO_ModeIN_PU);
    GPIOA_ModeCfg (GPIO_Pin_5, GPIO_ModeOut_PP_5mA);
#endif
#endif


#if (DEBUG_UART_NUM == 0)
    UART0_BaudRateCfg (115200);
    R8_UART0_FCR = (0 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART0_LCR = RB_LCR_WORD_SZ;
    R8_UART0_IER = RB_IER_TXD_EN;
    R8_UART0_DIV = 1;
#elif (DEBUG_UART_NUM == 1)
    UART1_DefInit();
#elif (DEBUG_UART_NUM == 2)
    UART2_DefInit();
#elif (DEBUG_UART_NUM == 3)
    UART3_DefInit();
#endif
    UARTx_BaudRateCfg (DEBUG_UART_NUM) (DEBUG_UART_BAUDRATE);
    UARTx_ByteTrigCfg (DEBUG_UART_NUM) (UART_1BYTE_TRIG);
    UARTx_INTCfg (DEBUG_UART_NUM) (ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
    PFIC_EnableIRQ (UARTx_IRQn (DEBUG_UART_NUM));

    return 0;
}

INIT_BOARD_EXPORT (usart_init);  // add to board auto initial

// output
void rt_hw_console_output (const char *str) {
    rt_size_t i = 0, size = 0;
    char a = '\r';

    size = rt_strlen (str);
    for (i = 0; i < size; i++) {
        if (*(str + i) == '\n') {
            while (R8_UARTx_TFC (DEBUG_UART_NUM) == UART_FIFO_SIZE);
            R8_UARTx_THR (DEBUG_UART_NUM) = a;  // if end of '\n', then add '\r'
        }
        while (R8_UARTx_TFC (DEBUG_UART_NUM) == UART_FIFO_SIZE);
        R8_UARTx_THR (DEBUG_UART_NUM) = *(str + i);
    }
}

// input
char rt_hw_console_getchar (void) {
    /* Note: the initial value of ch must < 0 */
    rt_uint8_t ch;

    if (rt_ringbuffer_getchar (&rt_uart_ringbuf, &ch) != 0) {
        return ch;
    } else {
        rt_thread_mdelay (10);
        return -1;
    }
    return ch;
}

__HIGH_CODE
void UARTx_IRQHandler (DEBUG_UART_NUM) (void) {
    uint8_t v;
    switch (UARTx_GetITFlag (DEBUG_UART_NUM)) {
    case UART_II_LINE_STAT:
        v = UARTx_GetLinSTA (DEBUG_UART_NUM);
        break;
    case UART_II_RECV_RDY:
    case UART_II_RECV_TOUT: {
        while (R8_UARTx_RFC (DEBUG_UART_NUM)) {
            v = R8_UARTx_RBR (DEBUG_UART_NUM);
            rt_ringbuffer_putchar (&rt_uart_ringbuf, v);
        }
        break;
    }
    default:
        break;
    }
}
