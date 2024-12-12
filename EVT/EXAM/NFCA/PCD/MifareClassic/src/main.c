/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/11/14
 * Description        : NFC PCD Mifare Classic测试例程
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CH58x_common.h"
#include "wch_nfca_mifare_classic.h"
#include "wch_nfca_pcd_bsp.h"

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
uint8_t default_key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t picc_uid[4];

/*********************************************************************
 * @fn      sys_get_vdd
 *
 * @brief   系统电压检测
 *
 * @param   none
 *
 * @return  检测的ADC值
 */
uint16_t sys_get_vdd(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_VBAT;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (0 << 4);    /* 使用-12dB模式 */
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT |= (3 << 4);                                 /* 7个Tadc */
    R8_ADC_CONVERT |= RB_ADC_START;
    while (R8_ADC_CONVERT & RB_ADC_START);
    adc_data = R16_ADC_DATA;

    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;
    return (adc_data);
}

/*********************************************************************
 * @fn      nfca_pcd_test
 *
 * @brief   nfc-a pcd读卡器测试函数
 *
 * @param   none
 *
 * @return  none
 */
void nfca_pcd_test(void)
{
    uint16_t res;
    uint16_t adc_vdd;
    int vdd_value;

    adc_vdd = sys_get_vdd();
    vdd_value = ADC_VoltConverSignalPGA_MINUS_12dB(adc_vdd);
    PRINTF("vdd_value: %d\n", vdd_value);
    if(vdd_value > 3400)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL0);
        PRINTF("LV0\n");
    }
    else if(vdd_value > 3000)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL1);
        PRINTF("LV1\n");
    }
    else if(vdd_value > 2600)
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL2);
        PRINTF("LV2\n");
    }
    else
    {
        nfca_pcd_set_out_drv(NFCA_PCD_DRV_CTRL_LEVEL3);
        PRINTF("LV3\n");
    }

    while(1)
    {
        nfca_pcd_start();

#if 1   /* 置1先进行超低功耗检卡，对天线信号幅度影响小的设备可能会无法唤醒 */
        if(nfca_pcd_lpcd_check() == 0)
        {
            PRINTF("NO CARD\n");
            goto next_loop;
        }
        PRINTF("CARD DETECT\n");
#endif
        mDelaymS(5);   /* 手机等模拟卡设备需要长时间的连续波唤醒其卡功能，普通实体卡 1 ms 即可 */

#if NFCA_PCD_USE_NFC_CTR_PIN
        nfca_pcd_ctr_handle();  /* 对天线信号进行检测，使用NFC CTR引脚控制幅度 */
#endif

        res = PcdRequest(PICC_REQALL);
        if(res == 0x0004)
        {
            res = PcdAnticoll(PICC_ANTICOLL1);
            if (res == PCD_NO_ERROR)
            {
                picc_uid[0] = g_nfca_pcd_recv_buf[0];
                picc_uid[1] = g_nfca_pcd_recv_buf[1];
                picc_uid[2] = g_nfca_pcd_recv_buf[2];
                picc_uid[3] = g_nfca_pcd_recv_buf[3];
                PRINTF("uid: %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1], picc_uid[2], picc_uid[3]);

                res = PcdSelect(PICC_ANTICOLL1, picc_uid);
                if (res == PCD_NO_ERROR)
                {
                    PRINTF("\nselect OK, SAK:%02x\n", g_nfca_pcd_recv_buf[0]);

#if 1   /* 读取前4个块数据测试 */
                    res = PcdAuthState(PICC_AUTHENT1A, 0, default_key, picc_uid);
                    if (res != PCD_NO_ERROR)
                    {
                        goto nfc_exit;
                    }

                    for (uint8_t i = 0; i < 4; i++)
                    {
                        res = PcdRead(i);
                        if (res != PCD_NO_ERROR)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }
                        PRINTF("block %02d: ", i);
                        for (uint8_t j = 0; j < 16; j++)
                        {
                            PRINTF("%02x ", g_nfca_pcd_recv_buf[j]);
                        }
                        PRINTF("\n");
                    }

#if 0   /* 值块读取和初始化测试 */

                    res = PcdReadValueBlock(1);
                    if (res == PCD_VALUE_BLOCK_INVALID)
                    {
                        PRINTF("not a value block, init it.");
                        uint32_t vdata = 100;
                        res = PcdInitValueBlock(1, (uint8_t *)&vdata, 2);
                        if (res != PCD_NO_ERROR)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }
                    }
                    else if (res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n", res);
                        goto nfc_exit;
                    }
                    else
                    {
                        PRINTF("value:%d, adr:%d\n", PU32_BUF(g_nfca_pcd_recv_buf)[0], g_nfca_pcd_recv_buf[12]);
                    }

#endif  /* 值块读取和初始化测试 */

#if 0   /* 值块扣款和备份测试 */
                    PRINTF("PcdValue\n");
                    uint32_t di_data = 1;
                    res = PcdValue(PICC_DECREMENT, 1, (uint8_t *)&di_data);
                    if(res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n",res);
                        goto nfc_exit;
                    }
                    PRINTF("PcdBakValue\n");
                    res = PcdBakValue(1,2);
                    if(res != PCD_NO_ERROR)
                    {
                        PRINTF("ERR: 0x%x\n",res);
                        goto nfc_exit;
                    }

#endif  /* 值块扣款和备份测试 */

#endif  /* 读取前4个块数据测试 */

#if 1   /* 所有扇区读取测试 */
                    for (uint8_t l = 1; l < 16; l++)
                    {
                        res = PcdAuthState(PICC_AUTHENT1A, 4 * l, default_key, picc_uid);
                        if (res)
                        {
                            PRINTF("ERR: 0x%x\n", res);
                            goto nfc_exit;
                        }

                        PRINTF("read:\n");
                        for (uint8_t i = 0; i < 3; i++)
                        {
                            res = PcdRead(i + 4 * l);
                            if (res)
                            {
                                PRINTF("ERR: 0x%x\n", res);
                                goto nfc_exit;
                            }
                            PRINTF("block %02d: ", i + 4 * l);
                            for (uint8_t j = 0; j < 16; j++)
                            {
                                PRINTF("%02x ", g_nfca_pcd_recv_buf[j]);
                            }
                            PRINTF("\n");
                        }
                    }
#endif  /* 所有扇区读取测试 */

nfc_exit:
                    PcdHalt();
                }
            }
        }
next_loop:
        nfca_pcd_stop();
        mDelaymS(500);
    }
}

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
    PWR_DCDCCfg(ENABLE);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif

    PRINT("NFCA PCD START\n");

    nfca_pcd_init();

    nfca_pcd_lpcd_calibration();    /* 低功耗检卡ADC值校准，和天线等相关，可在生产时进行检测。 */

    nfca_pcd_test();

    while(1);
}

/******************************** endfile @ main ******************************/
