#ifndef APP_TMOS_H_
#define APP_TMOS_H_

#include "CH58x_common.h"

#define DEALDATA_EVT         0x0001
#define WAKEUP_DATA_DEAL_EVT 0x0002
#define DEBUG_PRINT_EVENT    0x0004
#define TKY_KEEPALIVE_EVENT  0x0008

#define TOUCH_DECIMAL_POINT_PRECISION       (100)

/************************TOUCH_KEY_DEFINE****************************/
#define TOUCH_KEY_ELEMENTS                  (2)
#define TOUCH_KEY_CHS                       0,1

/************************WHEEL_SLIDER_DEFINE****************************/
#define TOUCH_WHEEL_ELEMENTS                (3)
#define TOUCH_WHEEL_RESOLUTION              (120)
#define TOUCH_WHEEL_CHS                     2,3,4

/************************LINE_SLIDER_DEFINE****************************/
#define TOUCH_SLIDER_ELEMENTS               (3)
#define TOUCH_SLIDER_RESOLUTION             (120)
#define TOUCH_SLIDER_CHS                    5,6,7

/************************TOUCH_PAD_DEFINE****************************/
#define TKY_THRESHOLD1  60  //阈值上限
#define TKY_THRESHOLD2  30  //阈值下限

#define CBUF_NUM            80
#define TBUF_NUM            40       //计算的触摸个数
#define FILTER_NUM          5        //滤除的较大值和较小值的个数

#define TKY_THRESHOLD_MAX   120      //触摸阈值上限
#define TOUCH_WINDOW        30       //触摸窗口
#define TOUCH_DROP_RATE     0.15     //触摸衰减比率


#define ENLARGE_FACTOR_X  16        //X坐标扩展
#define ENLARGE_FACTOR_Y  32        //Y坐标扩展

#define ROW_NUM         3
#define COL_NUM         3

#define PX_LOW          ENLARGE_FACTOR_X
#define PX_HIGH         ENLARGE_FACTOR_X*ROW_NUM
#define PY_LOW          ENLARGE_FACTOR_Y
#define PY_HIGH         ENLARGE_FACTOR_Y*COL_NUM

#define SAVE_POINT_NUM 12
#define DEAL_POINT_NUM 50

enum touch_state{
    TOUCH_NONE,
    TOUCH_MOVE_LEFT = 0x01,
    TOUCH_MOVE_RIGHT = 0x02,
    TOUCH_MOVE_UP = 0x04,
    TOUCH_MOVE_DOWN = 0x08,
    TOUCH_CLICK_SINGLE = 0x10,
    TOUCH_CLICK_DOUBLE = 0x20,
    TOUCH_CLICK_RELEASE = 0x40,
};

enum click_state
{
    STA_CLICK_RELEASE,
    STA_SINGLE_START,
    STA_SINGLE_DONE,
    STA_DOUBLE_DONE,
};


void PeriodicDealData(void);
void touch_on_TMOS_init(void);
void tky_on_TMOS_dataProcess(void);
void tky_DealData_stop(void);
void tky_DealData_start(void);

#endif
