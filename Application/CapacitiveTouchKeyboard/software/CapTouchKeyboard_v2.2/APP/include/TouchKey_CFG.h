/********************************** (C) COPYRIGHT *******************************
* File Name : TouchKey_CFG.h
* Author             : WCH
* Version            : V1.0
* Date               : 2023/10/17
* Description        : TouchKey parameter configuration header file
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

/* Add new above this */
TKY_QUEUE_END
} TKY_QUEUE_ID;

#define TKY_FILTER_MODE     			            3
#define TKY_FILTER_GRADE     			            4
#define TKY_BASE_REFRESH_ON_PRESS     			    0
#define TKY_BASE_UP_REFRESH_DOUBLE     			    20
#define TKY_BASE_DOWN_REFRESH_SLOW     			    0
#define TKY_BASE_REFRESH_SAMPLE_NUM     			100
#define TKY_SHIELD_EN     			                1
#define TKY_SINGLE_PRESS_MODE     			        0
#define TKY_MAX_QUEUE_NUM     			            14

#define GEN_TKY_CH_INIT(qNum,chNum,chTime,disChTime,slpmode,chBaseline,trs,trs2) \
    {\
     .queueNum=qNum,.channelNum=chNum,.chargeTime=chTime,.disChargeTime=disChTime,\
     .sleepStatus=slpmode,\
     .baseLine = chBaseline,\
     .threshold=trs,\
     .threshold2=trs2\
    }

#define TKY_CHS_INIT \
				GEN_TKY_CH_INIT(TKY_QUEUE_0, 2, 1,3,0,3382,230,130),\
				GEN_TKY_CH_INIT(TKY_QUEUE_1,11, 2,3,0,3223,200,100),\
				GEN_TKY_CH_INIT(TKY_QUEUE_2,12, 2,3,0,3621,200,100),\
				GEN_TKY_CH_INIT(TKY_QUEUE_3,13, 2,3,0,3600,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_4, 9, 1,3,0,3360,180,80),\
				GEN_TKY_CH_INIT(TKY_QUEUE_5,10, 2,3,0,3233,180,80),\
				GEN_TKY_CH_INIT(TKY_QUEUE_6, 1, 2,3,0,3604,180,80),\
				GEN_TKY_CH_INIT(TKY_QUEUE_7, 0, 2,3,0,3214,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_8, 8, 2,3,0,3284,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_9, 7, 2,3,0,3221,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_10,6, 2,3,0,3611,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_11,5, 2,3,0,3649,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_12,4, 2,3,0,3611,150,50),\
				GEN_TKY_CH_INIT(TKY_QUEUE_13,3, 2,3,0,3649,180,80)

        /* Add new above this */

//***********************************************************
#ifdef __cplusplus
}
#endif

#endif /* __TOUCH_KEY_CFG_H__ */
