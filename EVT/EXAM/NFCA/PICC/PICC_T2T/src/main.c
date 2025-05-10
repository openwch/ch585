/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2025/01/21
 * Description        : NFC PICC type2 Tag测试例程
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CH58x_common.h"
#include "wch_nfca_picc_bsp.h"
#include "wch_nfca_picc_t2t.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
    #define PRINTF(...) PRINT(__VA_ARGS__)
#else
    #define PRINTF(...) do {} while (0)
#endif

/*********************************************************************
 * GLOBAL TYPEDEFS
 */

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    UINT16 x;
//    uint8_t uid[7] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde};
//    uint8_t uid[7] = {0x04, 0xae, 0x38, 0xe2, 0xb5, 0x4c, 0x80};  /* nxp */
//    uint8_t uid[7] = {0x1D, 0xae, 0x38, 0xe2, 0xb5, 0x4c, 0x80};  /* 复旦微 */
    uint8_t uid[7] = {0x00, 0xae, 0x38, 0xe2, 0xb5, 0x4c, 0x80};    /* 未知厂商 */

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_78MHz);

#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif

    PRINT("Program build on: %s, %s\n", __DATE__, __TIME__);

    PRINT("NFCA PICC T2T START\n");

    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    nfca_picc_t2t_enable(uid);

    nfca_picc_start();

    PRINT("wait pcd\n");

    while(1);

    nfca_picc_stop();

}

/******************************** endfile @ main ******************************/
