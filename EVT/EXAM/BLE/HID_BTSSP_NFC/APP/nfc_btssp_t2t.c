/********************************** (C) COPYRIGHT *******************************
 * File Name          : nfc_btssp_t2t.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2025/05/27
 * Description        : NFC PICC BTSSP T2T source file for WCH chips.
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "nfc_btssp_t2t.h"

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)

#define NFC_BTSSP_T2T_SLEEP_CHECK_PERIOD_MS         200     /* 天线检测周期，有读卡器在周围时，不睡眠。 */
#define NFC_BTSSP_T2T_SLEEP_CHECK_CNT_LIMIT         5       /* 连续几次满足条件才切换可以睡眠状态 */
#define NFC_BTSSP_T2T_NO_SLEEP_CHECK_CNT_LIMIT      2       /* 连续几次满足条件才切换不可以睡眠状态 */
#define NFC_BTSSP_T2T_SLEEP_LIMIT_PERMIL            15      /* 千分比*/

uint8_t nfc_btssp_t2t_sleep_flag;
static uint8_t nfc_btssp_t2t_sleep_taskid;
static uint16_t gs_adc_base_value;
static uint8_t gs_adc_cnt;

uint16_t picc_get_ant_adc(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;   // CH_INTE_NFC CH_EXTIN_11
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_4_or_2 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB采样 ADC_PGA_1_4*/
    R8_ADC_CONVERT &= ~RB_ADC_PGA_GAIN2;
    R8_ADC_CONVERT &= ~(3 << 4);  /* 4个Tadc */

    R8_ADC_CONVERT |= RB_ADC_START;
    while (R8_ADC_CONVERT & (RB_ADC_START | RB_ADC_EOC_X));
    adc_data = R16_ADC_DATA;

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
void nfca_picc_ant_calibration(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint32_t adc_all;
    uint16_t adc_max, adc_min, adc_value;
    uint8_t i;

    /* 中值滤波 */
    adc_all = 0;
    adc_max = 0;
    adc_min = 0xffff;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;

    /* adc配置保存 */
    R8_TKEY_CFG &= ~RB_TKEY_PWR_ON;
    R8_ADC_CHANNEL = CH_INTE_NFC;
    R8_ADC_CFG = RB_ADC_POWER_ON | RB_ADC_BUF_EN | (SampleFreq_4_or_2 << 6) | (ADC_PGA_1_4 << 4);   /* -12DB采样 ADC_PGA_1_4*/
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

    gs_adc_base_value = adc_all >> 3;

    gs_adc_cnt = 0;
    PRINT("gs_adc_base_value: %d\n", gs_adc_base_value);
}

uint16_t nfc_btssp_t2t_ProcessEvent(uint8_t task_id, uint16_t events)
{
    uint16_t adc_value;
    uint32_t adc_value_diff;

    if(events & NFC_BTSSP_T2T_SLEEP_CHECK_EVT)
    {
        uint8_t *pMsg;

        adc_value = picc_get_ant_adc();
        if(adc_value > gs_adc_base_value)
        {
            /* 检测值大于阈值，需要检测是否设置为不可以睡眠 */
            adc_value_diff = adc_value - gs_adc_base_value;
        }
        else
        {
            /* 检测值小于阈值，需要检测是否设置为可以睡眠 */
            adc_value_diff = gs_adc_base_value - adc_value;
        }
        adc_value_diff = (adc_value_diff * 1000) / gs_adc_base_value;

        PRINT("ant_adc: %d, diff: %d\n", adc_value, adc_value_diff);
        if(nfc_btssp_t2t_sleep_flag)
        {
            /* 如果为1，可以睡眠。检测是否读卡器靠近，切换为不可以睡眠 */
            if((adc_value_diff > NFC_BTSSP_T2T_SLEEP_LIMIT_PERMIL) && (adc_value > gs_adc_base_value))
            {
                gs_adc_cnt++;
                if(gs_adc_cnt >= NFC_BTSSP_T2T_NO_SLEEP_CHECK_CNT_LIMIT)
                {
                    PRINT("can't sleep\n");
                    nfc_btssp_t2t_sleep_flag = 0;
                    PFIC_EnableIRQ(TMR0_IRQn);      /* 继续接收 */
                    gs_adc_cnt = 0;
                }
            }
            else
            {
                gs_adc_cnt = 0;
            }
        }
        else
        {
            /* 如果为0，不可以睡眠，检测是否读卡器离开，切换为可以睡眠 */
            if(adc_value_diff < NFC_BTSSP_T2T_SLEEP_LIMIT_PERMIL)
            {
                gs_adc_cnt++;
                if(gs_adc_cnt >= NFC_BTSSP_T2T_SLEEP_CHECK_CNT_LIMIT)
                {
                    PRINT("can sleep\n");
                    nfc_btssp_t2t_sleep_flag = 1;
                    PFIC_DisableIRQ(TMR0_IRQn);     /* 此时不再接收了，不睡眠的时候接收 */
                    gs_adc_cnt = 0;
                }
            }
            else
            {
                gs_adc_cnt = 0;
            }
        }
        // return unprocessed events
        return (events ^ NFC_BTSSP_T2T_SLEEP_CHECK_EVT);
    }

    if(events & NFC_BTSSP_T2T_START_CHECK_EVT)
    {
        nfca_picc_ant_calibration();

        if(gs_adc_base_value > 1000)
        {
            /** 启动校准成功，开始轮询检测 */
            tmos_start_reload_task(nfc_btssp_t2t_sleep_taskid, NFC_BTSSP_T2T_SLEEP_CHECK_EVT, MS1_TO_SYSTEM_TIME(NFC_BTSSP_T2T_SLEEP_CHECK_PERIOD_MS));
        }
        else
        {
            /** 启动校准失败，重新校准 */
            tmos_start_task(nfc_btssp_t2t_sleep_taskid, NFC_BTSSP_T2T_START_CHECK_EVT, MS1_TO_SYSTEM_TIME(10));
        }

        // return unprocessed events
        return (events ^ NFC_BTSSP_T2T_START_CHECK_EVT);
    }

    return 0;
}

#endif

NFC_BTSSP_T2T_INIT_ERR_t nfc_btssp_t2t_init(nfc_btssp_t2t_init_t *cfg)
{
    uint16_t temp;

    static const uint8_t btssp_ndef_hdr[] = {
        0x03, 0x00,
        0x91, 0x02, 0x0a, 'H', 's', 0x13,
        0xD1, 0x02, 0x04, 'a', 'c', 0x01, 0x01, 0x30, 0x00,
        0x5a, 0x20, 0x4d, 0x01,
        'a', 'p', 'p', 'l', 'i', 'c', 'a', 't', 'i', 'o', 'n', '/', 'v', 'n', 'd', '.',
        'b', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', '.', 'l' ,'e', '.', 'o', 'o', 'b',
        0x30,
    };

    uint8_t *t2t_data = (uint8_t *)&g_nfca_picc_t2t_data.static_data_pages[0];

    if(cfg->t2t_uid == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_UID;
    }

    if(cfg->bd_addr == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_BD_ADDR;
    }

    if(cfg->le_role > 0x0f)
    {
        return NFC_BTSSP_T2T_INIT_ERR_LE_ROLE;
    }

    nfca_picc_stop();

    /* 初始化NFCA PICC */
    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    /* 初始化uid */
    nfca_picc_t2t_enable(cfg->t2t_uid);

    __MCPY((void *)t2t_data, (void *)btssp_ndef_hdr, (void *)(btssp_ndef_hdr + sizeof(btssp_ndef_hdr)));

    t2t_data = t2t_data + sizeof(btssp_ndef_hdr);

    /* GAP_ADTYPE_LE_BD_ADDR */
    *t2t_data++ = 0x08;
    *t2t_data++ = GAP_ADTYPE_LE_BD_ADDR;
    *t2t_data++ = cfg->bd_addr[0];
    *t2t_data++ = cfg->bd_addr[1];
    *t2t_data++ = cfg->bd_addr[2];
    *t2t_data++ = cfg->bd_addr[3];
    *t2t_data++ = cfg->bd_addr[4];
    *t2t_data++ = cfg->bd_addr[5];
    *t2t_data++ = cfg->bd_addr_type;

    /* GAP_ADTYPE_LE_ROLE */
    *t2t_data++ = 0x02;
    *t2t_data++ = GAP_ADTYPE_LE_ROLE;
    *t2t_data++ = cfg->le_role;

    /* GAP_ADTYPE_SM_TK */
    if(cfg->sm_tk != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_SM_TK;
        tmos_memcpy(t2t_data, cfg->sm_tk, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LE_SC_CONFIRMATION_VALUE */
    if(cfg->le_sc_confirm != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_LE_SC_CONFIRMATION_VALUE;
        tmos_memcpy(t2t_data, cfg->le_sc_confirm, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LE_SC_RANDOM_VALUE */
    if(cfg->le_sc_random != NULL)
    {
        *t2t_data++ = 0x11;
        *t2t_data++ = GAP_ADTYPE_LE_SC_RANDOM_VALUE;
        tmos_memcpy(t2t_data, cfg->le_sc_random, 16);
        t2t_data = t2t_data + 16;
    }

    /* GAP_ADTYPE_LOCAL_NAME_COMPLETE */
    if(cfg->local_name_complete != NULL)
    {
        uint8_t local_name_len;
        local_name_len = tmos_strlen(cfg->local_name_complete);
        if(local_name_len > 0)
        {
            *t2t_data++ = local_name_len + 1;
            *t2t_data++ = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
            tmos_memcpy(t2t_data, cfg->local_name_complete, local_name_len);
            t2t_data = t2t_data + local_name_len;
        }
    }

    if((cfg->other_adv_data != NULL) && (cfg->other_adv_data_len > 0))
    {
        tmos_memcpy(t2t_data, cfg->other_adv_data, cfg->other_adv_data_len);
        t2t_data = t2t_data + cfg->other_adv_data_len;
    }

    /* END */
    *t2t_data++ = 0xfe;
    *t2t_data = 0;

    temp = (uint32_t)t2t_data - (uint32_t)&g_nfca_picc_t2t_data.static_data_pages[0];

    g_nfca_picc_t2t_data.pages[4].data8[1] = temp - 3;
    g_nfca_picc_t2t_data.pages[8].data8[3] = temp - 55;

    nfca_picc_start();

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    nfc_btssp_t2t_sleep_flag = 0;
    nfc_btssp_t2t_sleep_taskid = TMOS_ProcessEventRegister(nfc_btssp_t2t_ProcessEvent);
    if(nfc_btssp_t2t_sleep_taskid != 0xFF)
    {
        /* 校准需要等待硬件工作100ms左右稳定内部电平 */
        tmos_start_task(nfc_btssp_t2t_sleep_taskid, NFC_BTSSP_T2T_START_CHECK_EVT, MS1_TO_SYSTEM_TIME(100));
    }
#endif

    return NFC_BTSSP_T2T_INIT_OK;
}

NFC_BTSSP_T2T_INIT_ERR_t nfc_bt_t2t_init(nfc_btssp_t2t_init_t *cfg)
{
    uint16_t temp;

    static const uint8_t bt_ndef_hdr[] = {
        0x03, 0x00,
        0xd2, 0x20, 0x12,
        'a', 'p', 'p', 'l', 'i', 'c', 'a', 't', 'i', 'o', 'n', '/', 'v', 'n', 'd', '.',
        'b', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', '.', 'e' ,'p', '.', 'o', 'o', 'b',
    };

    uint8_t *t2t_data = (uint8_t *)&g_nfca_picc_t2t_data.static_data_pages[0];

    if(cfg->t2t_uid == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_UID;
    }

    if(cfg->bd_addr == NULL)
    {
        return NFC_BTSSP_T2T_INIT_ERR_BD_ADDR;
    }

    nfca_picc_stop();

    /* 初始化NFCA PICC */
    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    /* 初始化uid */
    nfca_picc_t2t_enable(cfg->t2t_uid);

    __MCPY((void *)t2t_data, (void *)bt_ndef_hdr, (void *)(bt_ndef_hdr + sizeof(bt_ndef_hdr)));

    t2t_data = t2t_data + sizeof(bt_ndef_hdr);

    t2t_data[1] = 0;
    t2t_data[2] = cfg->bd_addr[0];
    t2t_data[3] = cfg->bd_addr[1];
    t2t_data[4] = cfg->bd_addr[2];
    t2t_data[5] = cfg->bd_addr[3];
    t2t_data[6] = cfg->bd_addr[4];
    t2t_data[7] = cfg->bd_addr[5];

    if(cfg->local_name_complete != NULL)
    {
        uint8_t local_name_len;
        local_name_len = tmos_strlen(cfg->local_name_complete);
        if(local_name_len > 0)
        {
            t2t_data[0] = local_name_len + 10;
            t2t_data[8] = local_name_len + 1;
            t2t_data[9] = 0x09;
            __MCPY((void *)&t2t_data[10], (void *)cfg->local_name_complete, (void *)(cfg->local_name_complete + local_name_len));

            t2t_data = t2t_data + local_name_len + 10;
            *t2t_data++ = 0xfe;
            *t2t_data = 0;
        }
        else
        {
            goto no_name;
        }
    }
    else
    {
no_name:
        t2t_data[0] = 8;
        t2t_data = t2t_data + 8;
        *t2t_data++ = 0xfe;
        *t2t_data = 0;
    }

    temp = (uint32_t)t2t_data - (uint32_t)&g_nfca_picc_t2t_data.static_data_pages[0];

    g_nfca_picc_t2t_data.pages[4].data8[1] = temp - 3;
    g_nfca_picc_t2t_data.pages[5].data8[0] = temp - 38;

    nfca_picc_start();

#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    nfc_btssp_t2t_sleep_flag = 0;
    nfc_btssp_t2t_sleep_taskid = TMOS_ProcessEventRegister(nfc_btssp_t2t_ProcessEvent);
    if(nfc_btssp_t2t_sleep_taskid != 0xFF)
    {
        /* 校准需要等待硬件工作100ms左右稳定内部电平，可以考虑手动校准后存下来。 */
        tmos_start_task(nfc_btssp_t2t_sleep_taskid, NFC_BTSSP_T2T_START_CHECK_EVT, MS1_TO_SYSTEM_TIME(100));
    }
#endif

    return NFC_BTSSP_T2T_INIT_OK;
}

#if HAL_SLEEP
#error "如果使能了HAL_SLEEP，则需要在SLEEP.c的CH58x_LowPower函数中，在睡眠之前调用nfca_picc_stop，在唤醒后调用nfca_picc_start。"
#error "参考本文件最下方注释的代码，修改完成后，可以注释本段话。"
#endif

/*
 *
#include "nfc_btssp_t2t.h"

__HIGH_CODE
uint32_t CH58x_LowPower(uint32_t time)
{
#if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    volatile uint32_t i;
    uint32_t time_tign, time_sleep, time_curr;
    unsigned long irq_status;

    if(nfc_btssp_t2t_sleep_flag == 0)
    {
        return 2;
    }

    // 提前唤醒
    if (time <= WAKE_UP_RTC_MAX_TIME) {
        time_tign = time + (RTC_MAX_COUNT - WAKE_UP_RTC_MAX_TIME);
    } else {
        time_tign = time - WAKE_UP_RTC_MAX_TIME;
    }

    SYS_DisableAllIrq(&irq_status);
    time_curr = RTC_GetCycle32k();
    // 检测睡眠时间
    if (time_tign < time_curr) {
        time_sleep = time_tign + (RTC_MAX_COUNT - time_curr);
    } else {
        time_sleep = time_tign - time_curr;
    }

    // 若睡眠时间小于最小睡眠时间或大于最大睡眠时间，则不睡眠
    if ((time_sleep < SLEEP_RTC_MIN_TIME) ||
        (time_sleep > SLEEP_RTC_MAX_TIME)) {
        SYS_RecoverIrq(irq_status);
        return 2;
    }

    nfca_picc_stop();
    RTC_SetTignTime(time_tign);
#if(DEBUG == Debug_UART0) // 使用其他串口输出打印信息需要修改这行代码
    while((R8_UART0_LSR & RB_LSR_TX_ALL_EMP) == 0)
    {
        __nop();
    }
#endif
    // LOW POWER-sleep模式
    if(!RTCTigFlag)
    {
        uint8_t x32Mpw;
        LowPower_Sleep_WFE(RB_PWR_RAM32K | RB_PWR_RAM96K | RB_PWR_EXTEND);

        // 切换32M电流
        x32Mpw = R8_XT32M_TUNE;
        x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150%额定电流
        sys_safe_access_enable();
        R8_XT32M_TUNE = x32Mpw;
        sys_safe_access_disable();

        if(!(R8_RTC_FLAG_CTRL&RB_RTC_TRIG_FLAG)) //非RTC唤醒
        {
            // 注意此时32M还需等待稳定，也可执行一些时钟要求不高的代码
            DelayUs(1400);
            SetSysClock( SYSCLK_FREQ );
            SYS_RecoverIrq(irq_status);
            nfca_picc_start();
            return 0;
        }

        R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
        RTC_SetTignTime(time);
        sys_safe_access_enable();
        R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_KEEP;
        sys_safe_access_disable();
        if(!RTCTigFlag)
        {
            LowPower_Halt_WFE();
        }
        // 恢复时钟
        SetSysClock( SYSCLK_FREQ );
        R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
        SYS_RecoverIrq(irq_status);
        HSECFG_Current(HSE_RCur_100); // 降为额定电流(低功耗函数中提升了HSE偏置电流)
        nfca_picc_start();
        return 0;
    }
    SYS_RecoverIrq(irq_status);
    nfca_picc_start();
#endif
    return 3;
}

 *
 */
