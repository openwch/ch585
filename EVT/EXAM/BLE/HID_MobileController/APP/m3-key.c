/********************************** (C) COPYRIGHT *******************************
 * File Name          : m3-key.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2026/10/30
 * Description        : 蓝牙移动端遥控器按键扫描
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "m3-key.h"
#include "hidmobilecontroller.h"
#include "HAL.h"

/*********************************************************************
 * MACROS
 */
/* 按键事件环形缓冲区大小（RingMem） */
#define   M3_KEYS_RINGMEM_SIZE   32

/*********************************************************************
 * GLOBAL VARIABLES
 */
RingMemParm_t m3_keys_ringmem;
uint8_t m3_keys_ringmem_buf[M3_KEYS_RINGMEM_SIZE];

/* 按键参数表（字段顺序：State/Count/LongCount/LongTime/RepeatCount/RepeatSpeed/PinIndex/DownCode/UpCode/LongCode）
 * 下键(PA4)与上键(PA13)支持长按与连发，其余按键只有按下与松开事件 */
static KEY_T s_key_list[] =
{
    {0, 0, 0, KEY_LONG_TIME, 0, KEY_REPEAT_SPEED, 4, DOWN_KEY_DOWN, DOWN_KEY_UP, DOWN_KEY_LONG},    /* 下键 */
    {0, 0, 0, 0, 0, 0, 5, CAMERA_KEY_DOWN, CAMERA_KEY_UP, 0},                                       /* 拍照键 */
    {0, 0, 0, 0, 0, 0, 12, RIGHT_KEY_DOWN, RIGHT_KEY_UP, 0},                                        /* 右键 */
    {0, 0, 0, KEY_LONG_TIME, 0, KEY_REPEAT_SPEED, 13, UP_KEY_DOWN, UP_KEY_UP, UP_KEY_LONG},         /* 上键 */
    {0, 0, 0, 0, 0, 0, 8, LEFT_KEY_DOWN, LEFT_KEY_UP, 0},                                           /* 左键 */
    {0, 0, 0, 0, 0, 0, 9, MIDDLE_KEY_DOWN, MIDDLE_KEY_UP, 0},                                       /* 中间键 */
};

typedef struct
{
    KEY_T *p_stbtn;         /* 按键状态数组 */
    uint8_t num_elements;   /* 按键数量 */
} key_button_cfg_t;

static key_button_cfg_t s_key_cfg =
{
    s_key_list,
    sizeof(s_key_list) / sizeof(s_key_list[0]),
};

/*********************************************************************
 * @fn      bsp_put_key
 *
 * @brief   键值放入按键FIFO
 *
 * @param   code - 按键事件码
 *
 * @return  none
 */
void bsp_put_key(uint8_t code)
{
    if (RingMemWrite(&m3_keys_ringmem, &code, 1) != SUCCESS)
    {
        PRINT("KEY FIFO full, drop:%02x\n", code);
    }
    PRINT("KEY:%02x\n", code);
}

/*********************************************************************
 * @fn      bsp_DetectKey
 *
 * @brief   按键检测：短按键软件消抖，按下报DownCode、松开报UpCode；
 *          LongTime非0的按键支持长按报LongCode，长按后按RepeatSpeed连发DownCode
 *
 * @param   p - 按键配置
 *
 * @return  none
 */
static void bsp_DetectKey(key_button_cfg_t *p)
{
    KEY_T *pBtn;
    uint16_t keyData;

    if (p == NULL)
    {
        return;
    }

    /* 上拉输入，引脚为低电平表示按键按下 */
    keyData = GPIOA_ReadPort();

    for (uint8_t i = 0; i < p->num_elements; i++)
    {
        pBtn = &p->p_stbtn[i];
        if ((keyData & (1 << pBtn->PinIndex)) == 0) /* 按键按下 */
        {
            /*短按键，软件处理消抖*/
            if (pBtn->Count < KEY_FILTER_TIME)
            {
                pBtn->Count = KEY_FILTER_TIME;
            }
            else if (pBtn->Count < 2 * KEY_FILTER_TIME)
            {
                pBtn->Count++;
            }
            else
            {
                if (pBtn->State == 0)
                {
                    pBtn->State = 1;
                    /* 发送按钮按下的消息 */
                    bsp_put_key(pBtn->DownCode);
                }
            }

            /*处理长按键*/
            if (pBtn->LongTime > 0)
            {
                if (pBtn->LongCount < pBtn->LongTime)
                {
                    /* 发送按钮长按下的消息 */
                    if (++pBtn->LongCount == pBtn->LongTime)
                    {
                        pBtn->State = 2;
                        /* 键值放入按键FIFO */
                        bsp_put_key(pBtn->LongCode);
                    }
                }
                else
                {
                    if (pBtn->RepeatSpeed > 0)
                    {
                        if (++pBtn->RepeatCount >= pBtn->RepeatSpeed)
                        {
                            pBtn->RepeatCount = 0;
                            /* 长按键后，每隔RepeatSpeed个扫描周期发送1个按键 */
                            bsp_put_key(pBtn->DownCode);
                        }
                    }
                }
            }
        }
        else
        {
            if (pBtn->Count > KEY_FILTER_TIME)
            {
                pBtn->Count = KEY_FILTER_TIME;
            }
            else if (pBtn->Count != 0)
            {
                pBtn->Count--;
            }
            else
            {
                if (pBtn->State)
                {
                    pBtn->State = 0;
                    /* 松开按键消抖后 发送按钮弹起的消息 */
                    bsp_put_key(pBtn->UpCode);
                }
            }

            pBtn->LongCount = 0;
            pBtn->RepeatCount = 0;
        }
    }
}

/*********************************************************************
 * @fn      bsp_key_scan
 *
 * @brief   按键检测扫描，由应用任务KEY_SCAN_EVT按BTN_CHECK_PERIOD毫秒周期调用
 *
 * @param   none
 *
 * @return  none
 */
void bsp_key_scan(void)
{
    bsp_DetectKey(&s_key_cfg);
}

/*********************************************************************
 * @fn      is_any_btn_down
 *
 * @brief   判断当前是否有任意按键处于按下状态
 *
 * @param   none
 *
 * @return  0 - 无按键按下
 *          1 - 有按键按下
 */
uint8_t is_any_btn_down(void)
{
    for (uint8_t i = 0; i < s_key_cfg.num_elements; i++)
    {
        if (s_key_list[i].State != 0)
        {
            return 1;
        }
    }
    return 0;
}

/*********************************************************************
 * @fn      m3_key_init
 *
 * @brief   按键检测初始化：按键引脚配置为上拉输入，允许下降沿中断（先关闭，
 *          睡眠前再使能，下降沿唤醒）
 *
 * @param   none
 *
 * @return  none
 */
void m3_key_init(void)
{
    GPIOA_SetBits(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_5);
    GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_ModeIN_PU);

    GPIOA_ITModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_5, GPIO_ITMode_FallEdge); //需要睡眠时再启用中断，下降沿唤醒
    PFIC_DisableIRQ(GPIO_A_IRQn); //需要睡眠时再启用中断，下降沿唤醒

}

/*********************************************************************
 * @fn      m3_key_start
 *
 * @brief   启动按键检测：复位全部按键状态与事件缓存，启动按键扫描节拍任务
 *          （连接成功后调用）
 *
 * @param   none
 *
 * @return  none
 */
void m3_key_start(void)
{
    /* 复位全部按键状态 */
    for (uint8_t i = 0; i < s_key_cfg.num_elements; i++)
    {
        s_key_list[i].State = 0;
        s_key_list[i].Count = 0;
        s_key_list[i].LongCount = 0;
        s_key_list[i].RepeatCount = 0;
    }
    RingMemInit(&m3_keys_ringmem, m3_keys_ringmem_buf, M3_KEYS_RINGMEM_SIZE, NULL);

    /* 开启扫描任务 */
    tmos_start_reload_task(hidEmuTaskId, KEY_SCAN_EVT, MS1_TO_SYSTEM_TIME(BTN_CHECK_PERIOD));
}

/*********************************************************************
 * @fn      m3_key_stop
 *
 * @brief   停止按键检测：停止按键扫描节拍任务并清空事件缓存（断开连接后调用）
 *
 * @param   none
 *
 * @return  none
 */
void m3_key_stop(void)
{
    tmos_stop_task(hidEmuTaskId, KEY_SCAN_EVT);
    tmos_clear_event(hidEmuTaskId, KEY_SCAN_EVT);
    RingMemInit(&m3_keys_ringmem, m3_keys_ringmem_buf, M3_KEYS_RINGMEM_SIZE, NULL);
}

/*********************************************************************
 * @fn      GPIOA_IRQHandler
 *
 * @brief   GPIOA中断函数，睡眠期间按键按下唤醒
 *
 * @param   none
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void)
{
    GPIOA_ClearITFlagBit(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_5);
}
