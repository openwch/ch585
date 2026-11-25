/********************************** (C) COPYRIGHT *******************************
* File Name : TouchKey_CFG.h
* Author             : WCH
* Version            : V1.0
* Date               : 2023/10/17
* Description        : 触摸按键参数配置头文件
* ********************************************************************************
* Copyright(c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention : This software(modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __TOUCH_KEY_CFG_H__
#define __TOUCH_KEY_CFG_H__
#ifdef __cplusplus
extern "C" {
#endif

typedef enum _TKY_QUEUE_ID
{
TKY_QUEUE_0 = 0, 
TKY_QUEUE_1, 
TKY_QUEUE_2, 
TKY_QUEUE_3, 
TKY_QUEUE_4, 
TKY_QUEUE_5, 
TKY_QUEUE_6, 
TKY_QUEUE_7, 
TKY_QUEUE_8, 
TKY_QUEUE_9, 
TKY_QUEUE_10,
TKY_QUEUE_11,
TKY_QUEUE_12,
TKY_QUEUE_13,
TKY_QUEUE_14,
TKY_QUEUE_15,

/* Add new above this */
TKY_QUEUE_END
} TKY_QUEUE_ID;

#define TKY_FILTER_MODE     			            3
#define TKY_FILTER_GRADE     			            2
#define TKY_BASE_REFRESH_ON_PRESS     			    0
#define TKY_BASE_UP_REFRESH_DOUBLE     			    10
#define TKY_BASE_DOWN_REFRESH_SLOW     			    0
#define TKY_BASE_REFRESH_SAMPLE_NUM     			50
#define TKY_SHIELD_EN     			                1
#define TKY_SINGLE_PRESS_MODE     			        2
#define TKY_TOUCH_QUEUE_NUM                         8
#define TKY_SLEEP_QUEUE_NUM                         8
#define TKY_MAX_QUEUE_NUM     			            (TKY_TOUCH_QUEUE_NUM+TKY_SLEEP_QUEUE_NUM)
#define TKY_MAX_NOISE_CH_COUNT     			        (TKY_TOUCH_QUEUE_NUM - 1)
#define TKY_CX_CH_IDX                               13


#define GEN_TKY_CH_INIT(qNum,chNum,hyswin,chBaseline,maxvar,level) \
    {\
     .queueNum=qNum,            /*该通道在测试队列的序号*/\
     .channelNum=chNum,         /*该通道对应的ADC通道标号*/\
     .hysteresisWindow=hyswin,  /*迟滞比较窗口*/\
     .baseLine = chBaseline,    /*参考基线*/\
     .maxVar=maxvar,            /*参考最大变化量*/\
     .sensitivityLevel=level    /*灵敏度1~10*/\
    }

#define TKY_CHS_INIT                                           \
        GEN_TKY_CH_INIT( TKY_QUEUE_0,  0      , 0, 2274, 227, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_1,  10     , 0, 2277, 186, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_2,  8      , 0, 2265, 218, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_3,  6      , 0, 2293, 210, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_4,  1      , 0, 2275, 269, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_5,  9      , 0, 2288, 223, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_6,  7      , 0, 2276, 243, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_7,  5      , 0, 2261, 253, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_8,  (1<<0 ), 0, 2274, 227, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_9,  (1<<10 ),0, 2277, 186, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_10, (1<<8),  0, 2265, 218, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_11, (1<<6 ), 0, 2293, 210, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_12, (1<<1 ), 0, 2275, 269, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_13, (1<<9 ), 0, 2288, 223, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_14, (1<<7 ), 0, 2276, 243, 4),\
        GEN_TKY_CH_INIT( TKY_QUEUE_15, (1<<5 ), 0, 2261, 253, 4),\
        /* Add new above this */

//***********************************************************
#ifdef __cplusplus
}
#endif

#endif /* __TOUCH_KEY_CFG_H__ */
