#ifndef TOUCH_KEY_H_
#define TOUCH_KEY_H_

#include "CH58x_common.h"
#include "TouchKey_CFG.h"
#include "wchtouch.h"
//是否开启触摸数据打印

#ifdef  DEBUG
#define PRINT_EN 0
#else
#define PRINT_EN 0
#endif

#if (PRINT_EN)
  #define dg_log               printf
#else
  #define dg_log(x,...)
#endif

/************************KEY_FIFO_DEFINE******************************/
#define KEY_COUNT              48               // 按键个数 

/*是否启用TMOS*/
#ifndef TMOS_EN
#define TMOS_EN     0
#endif
/*是否支持触摸休眠功能*/
#if TMOS_EN
#define TKY_SLEEP_EN HAL_SLEEP
#else
#define TKY_SLEEP_EN 0
#endif

#ifndef TKY_FILTER_MODE
#define TKY_FILTER_MODE FILTER_MODE_3
#endif

#define TKY_PollForFilter() TKY_PollForFilterMode_3()


#define TKY_MEMHEAP_SIZE   		(TKY_MAX_QUEUE_NUM*TKY_BUFLEN)     	 //外部定义数据缓冲区长度

#define TOUCH_OFF_VALUE    					(0xFFFF)

/* 按键ID, 主要用于tky_GetKeyState()函数的入口参数 */
typedef enum
{
    KID_K0 = 0,
    KID_K1,
    KID_K2,
    KID_K3,
    KID_K4,
    KID_K5,
    KID_K6,
    KID_K7,
    KID_K8,
    KID_K9,
    KID_K10,
    KID_K11
}KEY_ID_E;


#define NORMAL_KEY_MODE 0                //独立按键触发模式
#define TOUCH_KEY_MODE  1                //触摸按键触发模式

#define KEY_MODE    NORMAL_KEY_MODE      //按键模式设置
#define KEY_FILTER_TIME   2              //按键滤波次数 0 表示不进行按键滤波
#define KEY_LONG_TIME     0//100              //长按时间 0 表示不检测长按键事件，时间单位为按键扫描单位
#define KEY_REPEAT_TIME   0//100             //按键连发的速度，0表示不支持连发，时间单位为按键扫描单位

typedef uint8_t (*pIsKeyDownFunc)(void);

/*
    每个按键对应1个全局的结构体变量。
*/
typedef struct
{
    /* 下面是一个函数指针，指向判断按键手否按下的函数 */
    /* 按键按下的判断函数,1表示按下 */
    // pIsKeyDownFunc IsKeyDownFunc;
    uint8_t  Count;         //滤波器计数器
    uint16_t LongCount;     //长按计数器
    uint16_t LongTime;      //按键按下持续时间, 0表示不检测长按
    uint8_t  State;         //按键当前状态（按下还是弹起）
    uint8_t  RepeatSpeed;   //连续按键周期
    uint8_t  RepeatCount;   //连续按键计数器
}KEY_T;

/*
    定义键值代码, 必须按如下次序定时每个键的按下、弹起和长按事件

    推荐使用enum, 不用#define，原因：
    (1) 便于新增键值,方便调整顺序，使代码看起来舒服点
    (2) 编译器可帮我们避免键值重复。
*/
typedef enum
{
    KEY_NONE = 0,           //0 表示按键事件 */

    KEY_0_DOWN,             // 1键按下 
    KEY_0_UP,               // 1键弹起 
    KEY_0_LONG,             // 1键长按 

    KEY_1_DOWN,             // 2键按下 
    KEY_1_UP,               // 2键弹起 
    KEY_1_LONG,             // 2键长按 

    KEY_2_DOWN,             // 3键按下 
    KEY_2_UP,               // 3键弹起 
    KEY_2_LONG,             // 3键长按 

    KEY_3_DOWN,             // 4键按下 
    KEY_3_UP,               // 4键弹起 
    KEY_3_LONG,             // 4键长按 

    KEY_4_DOWN,             // 5键按下 
    KEY_4_UP,               // 5键弹起 
    KEY_4_LONG,             // 5键长按 

    KEY_5_DOWN,             // 6键按下 
    KEY_5_UP,               // 6键弹起 
    KEY_5_LONG,             // 6键长按 

    KEY_6_DOWN,             // 7键按下 
    KEY_6_UP,               // 7键弹起 
    KEY_6_LONG,             // 7键长按 

    KEY_7_DOWN,             // 8键按下 
    KEY_7_UP,               // 8键弹起 
    KEY_7_LONG,             // 8键长按 

    KEY_8_DOWN,             // 9键按下 
    KEY_8_UP,               // 9键弹起 
    KEY_8_LONG,             // 9键长按 

    KEY_9_DOWN,             // 0键按下 
    KEY_9_UP,               // 0键弹起 
    KEY_9_LONG,             // 0键长按 

    KEY_10_DOWN,            // #键按下 
    KEY_10_UP,              // #键弹起 
    KEY_10_LONG,            // #键长按 

    KEY_11_DOWN,            // *键按下 
    KEY_11_UP,              // *键弹起 
    KEY_11_LONG,            // *键长按 
}KEY_ENUM;

/* 按键FIFO用到变量 */
#define KEY_FIFO_SIZE   64          //可根据使用环境和硬件需要进行修改*/

typedef struct
{
    uint8_t Buf[KEY_FIFO_SIZE];     // 键值缓冲区
    uint8_t Read;                   // 缓冲区读指针
    uint8_t Write;                  // 缓冲区写指针
}KEY_FIFO_T;

/** Configuration of each button */
typedef struct st_touch_button_cfg
{
    const uint8_t* p_elem_index;      ///< Element number array used by this button.
    KEY_T       *p_stbtn;
    uint8_t     num_elements;      ///< Number of elements used by this button.
} touch_button_cfg_t;

/** Configuration of matrix button */
typedef struct st_touch_matrix_button_cfg
{
    const uint16_t* p_elem_index;      ///< Element number array used by this button.
    KEY_T       *p_stbtn;
    uint16_t     num_elements;      ///< Number of elements used by this button.
} touch_matrix_button_cfg_t;

/** Configuration of each slider */
typedef struct st_touch_slider_cfg
{
    const uint8_t* p_elem_index;      ///< Element number array used by this slider.
    uint8_t  num_elements;      ///< Number of elements used by this slider.
    uint16_t threshold;         ///< Position calculation start threshold value.
    uint16_t decimal_point_percision;
    uint16_t slider_resolution;
    uint16_t *pdata;
} touch_slider_cfg_t;

/** Configuration of each wheel */
typedef struct st_touch_wheel_cfg_t
{
    const uint8_t* p_elem_index;      ///< Element number array used by this wheel.
    uint8_t  num_elements;      ///< Number of elements used by this wheel.
    uint16_t threshold;         ///< Position calculation start threshold value.
    uint16_t decimal_point_percision;
    uint16_t wheel_resolution;
    uint16_t *pdata;
} touch_wheel_cfg_t;

/** Configuration of touch */
typedef struct st_touch_cfg_t
{
    touch_button_cfg_t *touch_button_cfg;
    touch_matrix_button_cfg_t *touch_matrix_button_cfg;
    touch_slider_cfg_t *touch_slider_cfg;
    touch_wheel_cfg_t *touch_wheel_cfg;
} touch_cfg_t;

extern uint8_t wakeupflag; // 0  sleep mode   1  wakeup sta
extern uint16_t tkyQueueAll;
extern uint8_t wakeUpCount, wakeupflag;

/* 供外部调用的函数声明 */
extern void touch_Init(touch_cfg_t *p);
extern void touch_ScanWakeUp(void);
extern void touch_ScanEnterSleep(void);
extern uint8_t touch_GetKey(void);
extern uint8_t touch_GetKeyState(KEY_ID_E _ucKeyID);
extern void touch_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed);
extern void touch_ClearKey(void);
extern void touch_Scan(void);
extern void touch_InfoDebug(void);
extern uint16_t touch_GetLineSliderData(void);
extern uint16_t touch_GetWheelSliderData(void);
extern void touch_GPIOModeCfg (GPIOModeTypeDef mode, uint32_t channel);
#endif
