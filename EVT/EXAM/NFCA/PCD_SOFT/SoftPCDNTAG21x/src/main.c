/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/03/02
 * Description        : NFC PCD NTAG21x操作例程
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CH58x_common.h"
#include "wch_nfca_soft_pcd_ntag21x.h"
#include "wch_nfca_soft_pcd_bsp.h"

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
uint8_t picc_uid[7];

ntag_21x_get_version_res_t ntag_21x_ver;
uint8_t ntag_21x_sig_data[32];
uint8_t ntag_page_data[128];
uint8_t ntag_cnt_buf[3];

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

#if 0   /* 这段代码为电压检测并根据电压设置输出档位，非必要代码，可以置0屏蔽或者直接删除。 */
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
#endif

    nfca_pcd_set_recv_gain(NFCA_PCD_REC_GAIN_24DB);     /* 软件解码抗干扰更好，可以默认增益使用24DB */

    while(1)
    {
        nfca_pcd_start();

#if 1   /* 置1先进行超低功耗检卡，对天线信号幅度影响小的设备可能会无法唤醒，检测距离会比通讯距离更近。 */
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

        if(res == 0x0044)   /* NTAG21x卡片 */
        {
            res = PcdAnticoll(PICC_ANTICOLL1);
            if (res == PCD_NO_ERROR)
            {
                if(g_nfca_pcd_recv_buf[0] == 0x88)
                {
                    picc_uid[0] = g_nfca_pcd_recv_buf[1];
                    picc_uid[1] = g_nfca_pcd_recv_buf[2];
                    picc_uid[2] = g_nfca_pcd_recv_buf[3];

                    res = PcdSelect(PICC_ANTICOLL1, g_nfca_pcd_recv_buf);
                    if (res == PCD_NO_ERROR)
                    {
                        res = PcdAnticoll(PICC_ANTICOLL2);
                        if (res == PCD_NO_ERROR)
                        {
                            picc_uid[3] = g_nfca_pcd_recv_buf[0];
                            picc_uid[4] = g_nfca_pcd_recv_buf[1];
                            picc_uid[5] = g_nfca_pcd_recv_buf[2];
                            picc_uid[6] = g_nfca_pcd_recv_buf[3];
                            PRINTF("uid: %02x %02x %02x %02x %02x %02x %02x\n", picc_uid[0], picc_uid[1],
                                    picc_uid[2], picc_uid[3], picc_uid[4], picc_uid[5], picc_uid[6]);

                            res = PcdSelect(PICC_ANTICOLL2, g_nfca_pcd_recv_buf);
                            if (res == PCD_NO_ERROR)
                            {
                                /* 防冲突第二层选中成功 */
                                PRINTF("SELECT OK, SAK: %02x\n", g_nfca_pcd_recv_buf[0]);

                                res = PcdNTAG21xGetVersion(&ntag_21x_ver);
                                if (res == PCD_NO_ERROR)
                                {
                                    PRINTF("GetVersion OK\n");

                                    PRINTF("fixed_header: 0x%02x\n", ntag_21x_ver.fixed_header);
                                    PRINTF("vendor_id: 0x%02x\n", ntag_21x_ver.vendor_id);
                                    PRINTF("product_type: 0x%02x\n", ntag_21x_ver.product_type);
                                    PRINTF("product_subtype: 0x%02x\n", ntag_21x_ver.product_subtype);
                                    PRINTF("major_product_version: 0x%02x\n", ntag_21x_ver.major_product_version);
                                    PRINTF("minor_product_version: 0x%02x\n", ntag_21x_ver.minor_product_version);
                                    PRINTF("storage_size: 0x%02x\n", ntag_21x_ver.storage_size);
                                    PRINTF("protocol_type: 0x%02x\n", ntag_21x_ver.protocol_type);

                                    PRINTF("\nCARD CHIP: ");
                                    if(ntag_21x_ver.storage_size == NTAG213_STORAGE_SIZE)
                                    {
                                        PRINTF("NTAG213\n");
                                    }
                                    else if(ntag_21x_ver.storage_size == NTAG215_STORAGE_SIZE)
                                    {
                                        PRINTF("NTAG215\n");
                                    }
                                    else if(ntag_21x_ver.storage_size == NTAG216_STORAGE_SIZE)
                                    {
                                        PRINTF("NTAG216\n");
                                    }
                                    else
                                    {
                                        PRINTF("UNKNOWN\n");
                                    }

                                    res = PcdNTAG21xReadSIG(ntag_21x_sig_data);
                                    if (res == PCD_NO_ERROR)
                                    {
                                        PRINT("SIG:");
                                        for(uint8_t i = 0; i < 32; i++)
                                        {
                                            PRINT(" 0x%02x", ntag_21x_sig_data[i]);
                                        }
                                        PRINT("\n");

                                        res = PcdNTAG21xRead(0, ntag_page_data);
                                        if(res == PCD_NO_ERROR)
                                        {
                                            PRINT("PAGE 0 - 3:");
                                            for(uint8_t i = 0; i < 16; i++)
                                            {
                                                PRINT(" 0x%02x", ntag_page_data[i]);
                                            }
                                            PRINT("\n");
                                        }
                                        res = PcdNTAG21xFastRead(4, 35, ntag_page_data);
                                        if(res == PCD_NO_ERROR)
                                        {
                                            PRINT("PAGE 4 - 35:");
                                            for(uint8_t i = 0; i < 128; i++)
                                            {
                                                PRINT(" 0x%02x", ntag_page_data[i]);
                                            }
                                            PRINT("\n");
                                        }

                                        res = PcdNTAG21xReadCNT(ntag_cnt_buf);
                                        if(res == PCD_NO_ERROR)
                                        {
                                            PRINT("cnt buf:");
                                            for(uint8_t i = 0; i < 3; i++)
                                            {
                                                PRINT(" 0x%02x", ntag_cnt_buf[i]);
                                            }
                                            PRINT("\n");
                                        }
                                        else
                                        {
                                            PRINT("PcdNTAG21xReadCNT: 0x%04x\n", res);
                                        }
                                    }
                                }

                                PcdHalt();
                            }
                        }
                        else
                        {
                            PRINTF("ERROR ANTICOLL2\n");
                        }
                    }
                    else
                    {
                        PRINTF("CARD PcdSelect error: %d\n", res);
                    }
                }
                else
                {
                    PRINTF("ERROR UID0\n");
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
    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(CLK_SOURCE_HSE_PLL_62_4MHz);

#ifdef DEBUG
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_15, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
#endif

    PRINT("Program build on: %s, %s\n", __DATE__, __TIME__);

    PRINT("NFCA SOFT PCD NTAG21x START\n");

    nfca_soft_pcd_init();

    nfca_pcd_lpcd_calibration();    /* 低功耗检卡ADC值校准，和天线等相关，可在生产时进行检测。 */

    nfca_pcd_test();

    while(1);
}

/******************************** endfile @ main ******************************/
