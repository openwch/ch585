/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : SPI0例程：普通模式(8bit)数据收发，单字节/FIFO多字节/DMA方式，宏切换主机/从机
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* 主从模式定义 */
#define SPI_MASTER_MODE     1
#define SPI_SLAVE_MODE      0

/* 主从模式选择 */
#define SPI_MODE            SPI_MASTER_MODE      // 切换模式

/* ---- 硬件引脚配置：PA12-SCS PA13-SCK PA14-MOSI PA15-MISO ---- */
#define SPI_CS_PIN      GPIO_Pin_12
#define SPI_SCK_PIN     GPIO_Pin_13
#define SPI_MOSI_PIN    GPIO_Pin_14
#define SPI_MISO_PIN    GPIO_Pin_15

__attribute__((aligned(4))) uint8_t spiTxBuf[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) uint8_t spiRxBuf[16];

void DebugInit(void)                                  // SPI0与例程惯用串口打印引脚冲突，所以此例程改用UART1 输出打印
{
    GPIOB_SetBits(GPIO_Pin_13);
    GPIOPinRemap(ENABLE, RB_PIN_UART1);
    GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_13, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
}

int main(void)
{
    uint8_t i;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if(SPI_MODE == SPI_MASTER_MODE)
    /* ---- 主机模式 ---- */
    PRINT("1.spi0 master mode send data ...\n");
    DelayMs(100);

    GPIOA_SetBits(SPI_CS_PIN);
    GPIOA_ModeCfg(SPI_CS_PIN | SPI_SCK_PIN | SPI_MOSI_PIN, GPIO_ModeOut_PP_5mA);
    SPI0_MasterDefInit();

    // 单字节发送
    GPIOA_ResetBits(SPI_CS_PIN);
    SPI0_MasterSendByte(0x55);
    GPIOA_SetBits(SPI_CS_PIN);
    DelayMs(1);
    GPIOA_ResetBits(SPI_CS_PIN);
    i = SPI0_MasterRecvByte();
    GPIOA_SetBits(SPI_CS_PIN);
    DelayMs(2);
    PRINT("receive %x\n", i);

    // FIFO 连续发送
    GPIOA_ResetBits(SPI_CS_PIN);
    SPI0_MasterTrans(spiTxBuf, 8);
    GPIOA_SetBits(SPI_CS_PIN);
    DelayMs(2);
    GPIOA_ResetBits(SPI_CS_PIN);
    SPI0_MasterRecv(spiRxBuf, 8);
    GPIOA_SetBits(SPI_CS_PIN);
    DelayMs(2);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiRxBuf[i]);
    }
    PRINT("\n");

    // DMA 连续发送
    GPIOA_ResetBits(SPI_CS_PIN);
    SPI0_MasterDMATrans(spiTxBuf, 12);
    GPIOA_SetBits(SPI_CS_PIN);
    DelayMs(2);
    GPIOA_ResetBits(SPI_CS_PIN);
    SPI0_MasterDMARecv(spiRxBuf, 12);
    GPIOA_SetBits(SPI_CS_PIN);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiRxBuf[i]);
    }
    PRINT("\n");
#else
    /* ---- 从机模式 ---- */
    PRINT("1.spi0 slave mode \n");
    GPIOA_ModeCfg(SPI_CS_PIN | SPI_SCK_PIN | SPI_MOSI_PIN | SPI_MISO_PIN, GPIO_ModeIN_PU);
    SPI0_SlaveInit();

    // 单字节接收与回发
    i = SPI0_SlaveRecvByte();
    SPI0_SlaveSendByte(~i);
    PRINT("receive %x\n", i);

    // FIFO 连续接收与回发
    SPI0_SlaveRecv(spiRxBuf, 8);
    SPI0_SlaveTrans(spiRxBuf, 8);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiRxBuf[i]);
    }
    PRINT("\n");

    // DMA 连续接收与回发
    SPI0_SlaveDMARecv(spiRxBuf, 12);
    SPI0_SlaveDMATrans(spiRxBuf, 12);
    PRINT("DMA recv ");
    for(i = 0; i < 12; i++)
    {
        PRINT(" %x", spiRxBuf[i]);
    }
    PRINT("\n");
#endif

    PRINT("END ...\n");
    while(1);
    return 0;
}
