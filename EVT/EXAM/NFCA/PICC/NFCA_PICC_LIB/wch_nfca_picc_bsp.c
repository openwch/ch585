/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_picc_bsp.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/10/29
 * Description        : nfc picc收发控制层
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "wch_nfca_picc.h"
#include "wch_nfca_picc_bsp.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

/*
 * NFC PICC功能将占用TIMER0和TIMER3进行数据的捕获和发送处理，
 *  所以在使用PICC功能时，TIMER0和TIMER3无法被其他功能使用。
 */

static uint32_t gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_data_buf[PICC_DATA_BUF_LEN];
__attribute__((aligned(4))) uint8_t g_picc_parity_buf[PICC_DATA_BUF_LEN];

void nfca_picc_init(void)
{
    nfca_picc_config_t cfg;

    /* 对于EVT板NFC CTR引脚，如果分时使用卡和读卡器，NFC CTR引脚和PA7短接，在开始PICC之前，需要将该引脚置为模拟输入 */
    /* 如果只使用卡模式，NFC CTR引脚无需和PA7短接，则无需初始化该引脚，注释下面的代码。 */
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeIN_Floating);
    GPIOADigitalCfg(DISABLE, GPIO_Pin_7);

    /* CH585F和CH584F的PA9和PB9在芯片内部短接，所以F系列需要将下面PA9的代码取消注释 */
//    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeIN_Floating);
//    GPIOADigitalCfg(DISABLE, GPIO_Pin_9);

    GPIOB_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_16 | GPIO_Pin_17, GPIO_ModeIN_Floating);

    /* 关闭GPIO数字输入功能 */
    R32_PIN_IN_DIS |= ((GPIO_Pin_8 | GPIO_Pin_9) << 16);        /* 关闭GPIOB中GPIO_Pin_8和GPIO_Pin_9的数字输入功能 */
    R16_PIN_CONFIG |= ((GPIO_Pin_16 | GPIO_Pin_17) >> 8);       /* 关闭GPIOB中GPIO_Pin_16和GPIO_Pin_17的数字输入功能 */

    cfg.signal_buf = gs_picc_signal_buf;
    cfg.signal_buf_len = PICC_SIGNAL_BUF_LEN;

    cfg.parity_buf = g_picc_parity_buf;
    cfg.data_buf = g_picc_data_buf;
    cfg.data_buf_len = PICC_DATA_BUF_LEN;

    nfca_picc_lib_init(&cfg);
}

void nfca_picc_start(void)
{
    nfca_picc_lib_start();

    /* TIMER3 发送PWM数据 发送使用DMA进行 */
    R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

#if NFCA_PICC_RSP_POLAR != 0
    R8_TMR3_CTRL_MOD = (High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#else
    R8_TMR3_CTRL_MOD = (Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56;
#endif

    R32_TMR3_CNT_END = TMR3_NFCA_PICC_CNT_END;
    R32_TMR3_FIFO = 0;
    R32_TMR3_DMA_END = R32_TMR3_DMA_NOW + 0x100;
    R8_TMR3_INT_FLAG = RB_TMR_IF_DMA_END;
    R8_TMR3_INTER_EN = RB_TMR_IE_DMA_END;

#if NFCA_PICC_RSP_POLAR != 0
    R8_TMR3_CTRL_MOD = ((High_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#else
    R8_TMR3_CTRL_MOD = ((Low_Level << 4) | (PWM_Times_4 << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);
#endif

    /* TIMER0 捕获接收数据  */
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR0_CTRL_MOD = RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56;
    R32_TMR0_CNT_END = TMR0_NFCA_PICC_CNT_END;

    /* 清除中断标志位 */
    R32_TMR0_DMA_END = R32_TMR0_DMA_NOW + 0x100;
    R8_TMR0_INT_FLAG = RB_TMR_IF_DMA_END;

    /* 配置DMA缓冲区地址 */
    R32_TMR0_DMA_BEG = (uint32_t)gs_picc_signal_buf;
    R32_TMR0_DMA_END = (uint32_t)&(gs_picc_signal_buf[PICC_SIGNAL_BUF_LEN]);
    /* DMA使用循环模式 */
    R8_TMR0_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;

    /* 使能中断，先只使能DATA_ACT中断，当进入中断时，必然是接收到了有效数据 */
    R8_TMR0_INT_FLAG = RB_TMR_IF_DATA_ACT;
    R8_TMR0_INTER_EN = RB_TMR_IE_DATA_ACT;

    /* 开始计数 */
    R8_TMR0_CTRL_MOD = (RB_TMR_MODE_IN | (RiseEdge_To_RiseEdge << 6) | RB_TMR_FREQ_13_56 | RB_TMR_COUNT_EN);

    PFIC_EnableIRQ(TMR3_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
}

void nfca_picc_stop(void)
{
    PFIC_DisableIRQ(TMR0_IRQn);
    PFIC_DisableIRQ(TMR3_IRQn);

    R8_TMR0_CTRL_DMA = 0;
    R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR3_CTRL_DMA = 0;
    R8_TMR3_CTRL_MOD = RB_TMR_ALL_CLEAR;

    nfca_picc_lib_stop();
}

/*********************************************************************
 * @fn      nfca_picc_rand
 *
 * @brief   nfc-a picc读卡器随机数生成函数
 *
 * @param   none
 *
 * @return  随机数
 */
__attribute__((section(".highcode")))
uint32_t nfca_picc_rand(void)
{
    /* 需要自行实现产生随机数的回调 */
    /* 和蓝牙一起使用时可以使用返回tmos_rand() */
    return 0;
}

/* TIMER0 中断函数必须是最高优先级，不可以被打断，运行期间关闭中断不可以超过100us。  */
/* 由于NFC协议规定，通讯开始时的ATQA回复信号必须在读卡器发送结束后的90us左右，所以如果关闭中断超过90us，
 * 可能导致设备无法成功通讯，有的读卡器设备对时间要求低，就没有这个问题。一般手机读卡要求高。 */
/* 由于信号频繁，且函数处理流程复杂，所以该中断函数在接收数据期间会一直停留在中断中，
 * 在处理完数据开始发送数据后，退出中断，停止接收。等到TMR3中断中会重新打开接收。 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR0_IRQHandler(void)
{
    nfca_picc_rx_irq_handler();
}

/* TIMER3 中断函数必须是最高优先级，不被打断，运行期间关闭中断不可以超过100us。  */
/* 由于NFC协议规定，从机回复结束后，在最小时间80us左右主机可能会有下一步回复，所以如果关闭中断超过80us，而读卡器发送信号很快，则无法成功接收 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR3_IRQHandler(void)
{
    nfca_picc_tx_irq_handler();
}
