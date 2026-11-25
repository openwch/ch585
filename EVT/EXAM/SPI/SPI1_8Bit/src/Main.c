/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/13
 * Description        : SPI1例程：主机模式(8bit)数据收发，单字节/FIFO多字节(仅CH585支持SPI1)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

/* ---- 硬件引脚配置：PA12-SCS PA0-SCK PA1-MOSI(PA1/MOSI与PA0/SCK按SPI1引脚映射) ---- */
#define SPI1_CS_PIN      GPIO_Pin_12
#define SPI1_SCK_PIN     GPIO_Pin_0
#define SPI1_MOSI_PIN    GPIO_Pin_1

__attribute__((aligned(4))) uint8_t spiTxBuf[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
__attribute__((aligned(4))) uint8_t spiRxBuf[16];

void DebugInit(void)                                  // SPI1例程改用UART1 输出打印
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

    /* ---- 主机模式(仅CH585支持SPI1) ---- */
    PRINT("1.spi1 master mode send data ...\n");
    DelayMs(100);

    GPIOA_SetBits(SPI1_CS_PIN);
    GPIOA_ModeCfg(SPI1_CS_PIN | SPI1_SCK_PIN | SPI1_MOSI_PIN, GPIO_ModeOut_PP_5mA);
    SPI1_MasterDefInit();

    // 单字节发送
    GPIOA_ResetBits(SPI1_CS_PIN);
    SPI1_MasterSendByte(0x55);
    GPIOA_SetBits(SPI1_CS_PIN);
    DelayMs(1);
    GPIOA_ResetBits(SPI1_CS_PIN);
    i = SPI1_MasterRecvByte();
    GPIOA_SetBits(SPI1_CS_PIN);
    DelayMs(2);
    PRINT("receive %x\n", i);

    // FIFO 连续发送
    GPIOA_ResetBits(SPI1_CS_PIN);
    SPI1_MasterTrans(spiTxBuf, 8);
    GPIOA_SetBits(SPI1_CS_PIN);
    DelayMs(2);
    GPIOA_ResetBits(SPI1_CS_PIN);
    SPI1_MasterRecv(spiRxBuf, 8);
    GPIOA_SetBits(SPI1_CS_PIN);
    DelayMs(2);
    PRINT("FIFO recv ");
    for(i = 0; i < 8; i++)
    {
        PRINT(" %x", spiRxBuf[i]);
    }
    PRINT("\n");

    PRINT("END ...\n");
    while(1);
    return 0;
}
