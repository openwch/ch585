/********************************** (C) COPYRIGHT *******************************
 * File Name          : wch_nfca_pcd_bsp.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2024/11/14
 * Description        : NFC-A PCD BSP底层接口
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "wch_nfca_pcd_bsp.h"

/* 每个文件单独debug打印的开关，置0可以禁止本文件内部打印 */
#define DEBUG_PRINT_IN_THIS_FILE 1
#if DEBUG_PRINT_IN_THIS_FILE
#define PRINTF(...) PRINT(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

__attribute__((aligned(4))) static uint16_t gs_nfca_pcd_data_buf[NFCA_PCD_DATA_BUF_SIZE];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_send_buf[((NFCA_PCD_MAX_SEND_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_recv_buf[((NFCA_PCD_MAX_RECV_NUM + 3) & 0xfffc)];
__attribute__((aligned(4))) uint8_t g_nfca_pcd_parity_buf[NFCA_PCD_MAX_PARITY_NUM];

static uint16_t gs_lpcd_adc_base_value;
static uint16_t gs_lpcd_adc_filter_buf[8];
uint16_t g_nfca_pcd_recv_buf_len;
uint32_t g_nfca_pcd_recv_bits;

#if NFCA_PCD_USE_NFC_CTR_PIN

void nfca_pcd_ctr_init(void)
{
    GPIOA_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);
    GPIOA_ResetBits(GPIO_Pin_7);
    nfca_pcd_set_lp_ctrl(NFCA_PCD_LP_CTRL_0_5_VDD);
}

__always_inline static inline void nfca_pcd_ctr_on(void)
{
    /* 输出使能，输出低，不可输出高，天线峰峰值分压3分之一 */
    R32_PA_DIR |= (GPIO_Pin_7);
}

__always_inline static inline void nfca_pcd_ctr_off(void)
{
    /* 输出禁止，模拟输入，天线峰峰值几乎不分压 */
    R32_PA_DIR &= ~(GPIO_Pin_7);
}

void nfca_pcd_ctr_handle(void)
{
    if(nfca_pcd_get_lp_status())
    {
        /* 峰峰值过低 */
        PRINTF("LP\n");
        nfca_pcd_ctr_off();
    }
    else
    {
        nfca_pcd_ctr_on();
    }
}

#endif

/*********************************************************************
 * @fn      nfca_pcd_init
 *
 * @brief   nfc-a pcd读卡器初始化
 *
 * @param   none
 *
 * @return  none
 */
void nfca_pcd_init(void)
{
    nfca_pcd_config_t cfg;

    /* NFC引脚初始化为模拟输入模式 */
    GPIOB_ModeCfg(GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_16 | GPIO_Pin_17, GPIO_ModeIN_Floating);

    /* 关闭GPIO数字输入功能 */
    R32_PIN_IN_DIS |= ((GPIO_Pin_8 | GPIO_Pin_9) << 16);        /* 关闭GPIOB中GPIO_Pin_8和GPIO_Pin_9的数字输入功能 */
    R16_PIN_CONFIG |= ((GPIO_Pin_16 | GPIO_Pin_17) >> 8);       /* 关闭GPIOB中GPIO_Pin_16和GPIO_Pin_17的数字输入功能 */

    /* CH584F和CH585F内部PA9和PB9短接，需要将PA9也设置为模拟输入并关闭数字功能，M封装注释下面的两句代码，F封装取消注释 */
//    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeIN_Floating);
//    R32_PIN_IN_DIS |= GPIO_Pin_9;

    cfg.data_buf = gs_nfca_pcd_data_buf;
    cfg.data_buf_size = NFCA_PCD_DATA_BUF_SIZE;

    cfg.send_buf = g_nfca_pcd_send_buf;
    cfg.send_buf_size = NFCA_PCD_MAX_SEND_NUM;

    cfg.recv_buf = g_nfca_pcd_recv_buf;
    cfg.recv_buf_size = NFCA_PCD_MAX_RECV_NUM;

    cfg.parity_buf = g_nfca_pcd_parity_buf;
    cfg.parity_buf_size = NFCA_PCD_MAX_PARITY_NUM;

    /* 将数据区指针传入给NFC库内BUFFER指针 */
    nfca_pcd_lib_init(&cfg);

#if NFCA_PCD_USE_NFC_CTR_PIN
    nfca_pcd_ctr_init();
#endif

}

/*********************************************************************
 * @fn      nfca_pcd_start
 *
 * @brief   nfc-a pcd读卡器功能开始运行
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void nfca_pcd_start(void)
{
#if NFCA_PCD_USE_NFC_CTR_PIN
    nfca_pcd_ctr_on();
#endif
    nfca_pcd_lib_start();
    PFIC_ClearPendingIRQ(NFC_IRQn);
    PFIC_EnableIRQ(NFC_IRQn);
}

/*********************************************************************
 * @fn      nfca_pcd_stop
 *
 * @brief   nfc-a pcd读卡器功能停止运行
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void nfca_pcd_stop(void)
{
    nfca_pcd_lib_stop();
    PFIC_DisableIRQ(NFC_IRQn);
}

/*********************************************************************
 * @fn      nfca_pcd_wait_communicate_end
 *
 * @brief   nfc-a pcd读卡器等待通讯结束
 *
 * @param   none
 *
 * @return  nfca_pcd_controller_state_t 通讯状态
 */
nfca_pcd_controller_state_t nfca_pcd_wait_communicate_end(void)
{
    nfca_pcd_controller_state_t status;
    uint32_t overtimes;

    overtimes = 0;

    /* 可以在蓝牙任务中改写该函数，用任务查询结束状态，示例为死等 */
    while (1)
    {
        status = nfca_pcd_get_communicate_status();
        if (status != 0)
        {
            break;
        }

        if (overtimes > (NFCA_PCD_WAIT_MAX_MS * 10))
        {
            /* 软件超时时间， */
            break;
        }

        mDelayuS(100);
        overtimes++;
    }

    g_nfca_pcd_recv_buf_len = nfca_pcd_get_recv_data_len();
    g_nfca_pcd_recv_bits = nfca_pcd_get_recv_bits();

    return status;
}

/*********************************************************************
 * @fn      nfca_pcd_rand
 *
 * @brief   nfc-a pcd读卡器随机数生成函数
 *
 * @param   none
 *
 * @return  nfca_pcd_controller_state_t 通讯状态
 */
uint32_t nfca_pcd_rand(void)
{
    /* 需要自行实现产生随机数的回调 */
    /* 和蓝牙一起使用时可以使用返回tmos_rand() */
    return 0;
}

/*********************************************************************
 * @fn      nfca_adc_get_ant_signal
 *
 * @brief   nfca 检测天线信号强度
 *
 * @param   none
 *
 * @return  检测的ADC值
 */
__HIGH_CODE
uint16_t nfca_adc_get_ant_signal(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;
    uint32_t adc_data_all;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB采样 ADC_PGA_1_4*/
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT &= ~(3 << 4);  /* 4个Tadc */

    adc_data_all = 0;

    for(uint8_t i = 0; i < 2; i++)
    {
        R8_ADC_CONVERT |= RB_ADC_START;
        while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
        adc_data_all = adc_data_all + R16_ADC_DATA;
    }

    adc_data = adc_data_all / 2;

    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;
    return (adc_data);
}

/*********************************************************************
 * @fn      nfca_pcd_lpcd_calibration
 *
 * @brief   nfca pcd低功耗检卡校准
 *
 * @param   none
 *
 * @return  none
 */
void nfca_pcd_lpcd_calibration(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint32_t adc_all;
    uint16_t adc_max, adc_min, adc_value;
    uint8_t i;

    /* 中值滤波 */
    adc_all = 0;
    adc_max = 0;
    adc_min = 0xffff;

    nfca_pcd_start();
    mDelayuS(200);      /* 内部信号建立需要200us才趋于稳定 */

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    /* adc配置保存 */
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_8_or_4 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB采样 ADC_PGA_1_4*/
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT &= ~(3 << 4);  /* 4个Tadc */

    for(i = 0; i < 10; i++)
    {
        R8_ADC_CONVERT |= RB_ADC_START;
        while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
        adc_value = R16_ADC_DATA;

        if(adc_value > adc_max)
        {
            adc_max = adc_value;
        }
        if(adc_value < adc_min)
        {
            adc_min = adc_value;
        }
        adc_all = adc_all + adc_value;
    }

    /* adc配置恢复 */
    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;

    adc_all = adc_all - adc_max - adc_min;

    gs_lpcd_adc_base_value = adc_all >> 3;

    PRINTF("gs_lpcd_adc_base_value:%d\n", gs_lpcd_adc_base_value);

    for(i = 0; i < 8; i++)
    {
        gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_base_value;
    }

    nfca_pcd_stop();
}

/*********************************************************************
 * @fn      nfca_pcd_lpcd_adc_filter_buf_add
 *
 * @brief   nfca pcd低功耗检卡adc值滤波器处理
 *
 * @param   lpcd_adc - 需要添加的adc值
 *
 * @return  uint16_t - 新的低功耗检卡的ADC阈值基准
 */
__HIGH_CODE
static uint16_t nfca_pcd_lpcd_adc_filter_buf_add(uint16_t lpcd_adc)
{
    uint32_t lpcd_adc_all = 0;
    uint8_t i;
    for(uint8_t i = 0; i < 7; i++)
    {
        gs_lpcd_adc_filter_buf[i] = gs_lpcd_adc_filter_buf[i + 1];
        lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[i];
    }
    gs_lpcd_adc_filter_buf[7] = lpcd_adc;
    lpcd_adc_all = lpcd_adc_all + gs_lpcd_adc_filter_buf[7];
    lpcd_adc_all = (lpcd_adc_all >> 3);
    return (uint16_t)lpcd_adc_all;
}

/*******************************************************************************
 * @fn              nfca_pcd_lpcd_check
 *
 * @brief           nfc-a pcd lpcd ADC检卡
 *
 * @param           None.
 *
 * @return          1 有卡，0无卡.
 */
__HIGH_CODE
uint8_t nfca_pcd_lpcd_check(void)
{
    uint32_t adc_value_diff;
    uint16_t adc_value;
    uint8_t res = 0;

    adc_value = nfca_adc_get_ant_signal();
    PRINTF("adc_value:%d\n", adc_value);
    if(adc_value > gs_lpcd_adc_base_value)
    {
        adc_value_diff = adc_value - gs_lpcd_adc_base_value;

    }
    else
    {
        adc_value_diff = gs_lpcd_adc_base_value - adc_value;
    }
    adc_value_diff = (adc_value_diff * 1000) / gs_lpcd_adc_base_value;

    if(adc_value > gs_lpcd_adc_base_value)
    {

#if 0
        /*
         * 此处可以考虑不判断变大也触发，
         * 手机等同时做读卡器和卡的设备靠近时，会导致值增加。
         * 当手机检测到天线有波形后，切换为卡模式，会降低天线信号。
         */
        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_PERMIL)
        {
            res = 1;
        }
#endif

        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL)
        {
            /* 阈值更新应当是缓慢的，根据检测间隔酌情修改该值，千分之二内基本为误差 */
            adc_value = ((uint32_t)gs_lpcd_adc_base_value * (1000 + NFCA_PCD_LPCD_THRESHOLD_MAX_LIMIT_PERMIL) / 1000);
        }
    }
    else
    {
        if(adc_value_diff >= NFCA_PCD_LPCD_THRESHOLD_PERMIL)
        {
            res = 1;
        }
#if 1
        /* 每次限幅为1 */
        if(adc_value < gs_lpcd_adc_base_value)
        {
            adc_value = gs_lpcd_adc_base_value - 1;
        }
#else
        /* 按千分比限幅 */
        if(adc_value_diff > NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL)
        {
            /* 阈值更新应当是缓慢的，根据检测间隔酌情修改该值，千分之二内基本为误差 */
            adc_value = ((uint32_t)gs_lpcd_adc_base_value * (1000 - NFCA_PCD_LPCD_THRESHOLD_MIN_LIMIT_PERMIL) / 1000);
        }
#endif
    }

    gs_lpcd_adc_base_value = nfca_pcd_lpcd_adc_filter_buf_add(adc_value);
    return res;
}

/*******************************************************************************
 * @fn              NFC_IRQHandler
 *
 * @brief           nfc-a 中断函数，PCD模式下必须保证800us内可以进入一次中断
 *
 * @param           None.
 *
 * @return          1 有卡，0无卡.
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void NFC_IRQHandler(void)
{
    NFC_IRQLibHandler();
}
