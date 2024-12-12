/********************************** (C) COPYRIGHT *******************************
 * File Name          : hid_nfca_m1_ndef.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/11/15
 * Description        : NFC PICC head file for WCH chips.
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "hid_nfca_m1_ndef.h"
#include "hidconsumer.h"
#include "string.h"

#if HAL_SLEEP

#define HID_NFCA_M1_NDEF_EVT        1
tmosTaskID hid_nfca_m1_ndef_taskid;
tmosEvents hid_nfca_m1_ndef_event(tmosTaskID task_id, tmosEvents events)
{
    return 0;
}
#endif

void hid_nfca_m1_ndef_init(void)
{
    uint8_t uid[4] = {0x12, 0x34, 0x56, 0x78};      /* 默认UID，可以自行修改 */

#if HID_NFCA_M1_NDEF_INCLUDE_BLE_NAME
    uint8_t bleAttDeviceName[GAP_DEVICE_NAME_LEN];  /* 如果使用的DEVICE NAME长度不可以超出GAP_DEVICE_NAME_LEN，如果超过可以不增加BLE NAME字段 */
    uint8_t bleAttDeviceNameLen;
#endif

    /* M1只有716字节可以写入NDEF信息 */
    static const uint8_t sector_trailer0_data[] = {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0x78, 0x77, 0x88, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static const uint8_t sector_trailers_data[] = {0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7, 0x7f, 0x07, 0x88, 0x40, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    static const uint8_t data1[] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1 };
    static const uint8_t data2[] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1 };

    static const uint8_t data4[] = {0x03, 0x35, 0xD2, 0x20, 0x12, 0x61, 0x70, 0x70, 0x6C, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E };
    static const uint8_t data5[] = {0x2F, 0x76, 0x6E, 0x64, 0x2E, 0x62, 0x6C, 0x75, 0x65, 0x74, 0x6F, 0x6F, 0x74, 0x68, 0x2E, 0x65 };
    static const uint8_t data6[] = {0x70, 0x2E, 0x6F, 0x6F, 0x62};

    nfca_picc_stop();

    /* 初始化NFCA PICC */
    nfca_picc_init();
    PRINT("nfca_picc_init ok\n");

    /* 初始化uid */
    nfca_picc_m1_enable(uid);

    __MCPY((void *)&g_nfca_picc_m1_data.sectors[0].sector_trailer, (void *)sector_trailer0_data, (void *)(sector_trailer0_data + 16));
    for(uint8_t i = 1; i < 16; i++)
    {
        __MCPY((void *)&g_nfca_picc_m1_data.sectors[i].sector_trailer, (void *)sector_trailers_data, (void *)(sector_trailers_data + 16));
    }

    __MCPY((void *)&g_nfca_picc_m1_data.blocks[1], (void *)data1, (void *)(data1 + 16));
    __MCPY((void *)&g_nfca_picc_m1_data.blocks[2], (void *)data2, (void *)(data2 + 16));
    __MCPY((void *)&g_nfca_picc_m1_data.blocks[4], (void *)data4, (void *)(data4 + 16));
    __MCPY((void *)&g_nfca_picc_m1_data.blocks[5], (void *)data5, (void *)(data5 + 16));
    __MCPY((void *)&g_nfca_picc_m1_data.blocks[6], (void *)data6, (void *)(data6 + 5));

    g_nfca_picc_m1_data.blocks[6][6] = 0;
    g_nfca_picc_m1_data.blocks[6][7] = hid_static_addr[0];
    g_nfca_picc_m1_data.blocks[6][8] = hid_static_addr[1];
    g_nfca_picc_m1_data.blocks[6][9] = hid_static_addr[2];
    g_nfca_picc_m1_data.blocks[6][10] = hid_static_addr[3];
    g_nfca_picc_m1_data.blocks[6][11] = hid_static_addr[4];
    g_nfca_picc_m1_data.blocks[6][12] = hid_static_addr[5];

#if HID_NFCA_M1_NDEF_INCLUDE_BLE_NAME

    GGS_GetParameter(GGS_DEVICE_NAME_ATT, (void *)bleAttDeviceName);
    bleAttDeviceNameLen = tmos_strlen(bleAttDeviceName);

    g_nfca_picc_m1_data.blocks[6][5] = 8 + 2 + bleAttDeviceNameLen;   /* 长度 */

    if(bleAttDeviceNameLen > 0)
    {
        PRINT("bleAttDeviceNameLen:%d\n", bleAttDeviceNameLen);
        g_nfca_picc_m1_data.blocks[6][13] = 1 + bleAttDeviceNameLen;
        g_nfca_picc_m1_data.blocks[6][14] = 0x09;

        g_nfca_picc_m1_data.blocks[6][15] = bleAttDeviceName[0];

        bleAttDeviceNameLen--;

        if(bleAttDeviceNameLen <= 15)
        {
            for(uint8_t i = 0; i < bleAttDeviceNameLen; i++)
            {
                g_nfca_picc_m1_data.blocks[8][i] = bleAttDeviceName[i + 1];
            }
            g_nfca_picc_m1_data.blocks[8][bleAttDeviceNameLen] = 0xfe;
        }
        else
        {
            for(uint8_t i = 0; i < 16; i++)
            {
                g_nfca_picc_m1_data.blocks[8][i] = bleAttDeviceName[i + 1];
            }
            bleAttDeviceNameLen = bleAttDeviceNameLen - 16;
            if(bleAttDeviceNameLen > 0)
            {
                for(uint8_t i = 0; i < bleAttDeviceNameLen; i++)
                {
                    g_nfca_picc_m1_data.blocks[9][i] = bleAttDeviceName[i + 17];
                }
                g_nfca_picc_m1_data.blocks[9][bleAttDeviceNameLen] = 0xfe;
            }
            else
            {
                g_nfca_picc_m1_data.blocks[9][0] = 0xfe;
            }
        }
    }
    else
#endif
    {
        g_nfca_picc_m1_data.blocks[6][5] = 8;   /* 长度 */
        g_nfca_picc_m1_data.blocks[6][13] = 0xfe;
    }

    nfca_picc_start();

    /* 使用中，蓝牙不可以睡眠，睡眠之前必须要调用nfca_picc_stop，睡眠唤醒后再调用nfca_picc_start */
#if HAL_SLEEP
    hid_nfca_m1_ndef_taskid = TMOS_ProcessEventRegister(hid_nfca_m1_ndef_event);
    tmos_start_reload_task(hid_nfca_m1_ndef_taskid, HID_NFCA_M1_NDEF_EVT, 2);
#endif

}

void hid_nfca_m1_ndef_deinit(void)
{
    nfca_picc_stop();
#if HAL_SLEEP
    hid_nfca_m1_ndef_taskid = TMOS_ProcessEventRegister(hid_nfca_m1_ndef_event);
    tmos_stop_task(hid_nfca_m1_ndef_taskid, HID_NFCA_M1_NDEF_EVT);
    tmos_clear_event(hid_nfca_m1_ndef_taskid, HID_NFCA_M1_NDEF_EVT);
#endif
}


