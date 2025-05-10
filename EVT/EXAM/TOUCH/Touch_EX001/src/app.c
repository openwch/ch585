/********************************** (C) COPYRIGHT *******************************
 * File Name          : app_tmos.C
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2023/8/5
 * Description        : 触摸按键例程
 *******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "Touch.h"
#include "app.h"
#include "LED.h"
/*********************
 *      DEFINES
 *********************/

/**********************
 *      VARIABLES
 **********************/
uint8_t volatile timerFlag = 0;
uint8_t volatile TKY_ScanFlag = 0;
extern volatile uint8_t led_scanflag;

static const uint8_t touch_key_ch[ TOUCH_KEY_ELEMENTS ] = {TOUCH_KEY_CHS};
KEY_T s_tBtn[TOUCH_KEY_ELEMENTS] = {0};

static const uint8_t touch_wheel_ch[ TOUCH_WHEEL_ELEMENTS ] = {TOUCH_WHEEL_CHS};
uint16_t wheel_data[ TOUCH_WHEEL_ELEMENTS ] = {0};

static const uint8_t touch_slidel_ch[ TOUCH_SLIDER_ELEMENTS ] = {TOUCH_SLIDER_CHS};
uint16_t slider_data[ TOUCH_SLIDER_ELEMENTS ] = {0};

touch_button_cfg_t p_selfkey = 
{
    .num_elements = TOUCH_KEY_ELEMENTS,
    .p_elem_index = touch_key_ch,
    .p_stbtn = s_tBtn
};

touch_wheel_cfg_t p_wheel = {
    .num_elements = TOUCH_WHEEL_ELEMENTS,
    .p_elem_index = touch_wheel_ch,
    .threshold = 200,
    .decimal_point_percision = TOUCH_DECIMAL_POINT_PRECISION,
    .wheel_resolution = TOUCH_WHEEL_RESOLUTION,
    .pdata = wheel_data};

touch_slider_cfg_t p_slider = {
    .num_elements = TOUCH_SLIDER_ELEMENTS,
    .p_elem_index = touch_slidel_ch,
    .threshold = 100,
    .decimal_point_percision = TOUCH_DECIMAL_POINT_PRECISION,
    .slider_resolution = TOUCH_SLIDER_RESOLUTION,
    .pdata = slider_data
    };

touch_cfg_t touch_cfg = 
{
    .touch_button_cfg = &p_selfkey,
    .touch_slider_cfg = &p_slider,
    .touch_wheel_cfg = &p_wheel
};
/**********************
 *  STATIC PROTOTYPES
 **********************/
static void TKY_PeripheralInit(void);
void TKY_LineSliderLedProcess (uint16_t pros);
void TKY_WheelSliderLedProcess (uint16_t pros);
/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*********************************************************************
 * @fn      TKY_dataProcess
 *
 * @brief   触摸数据处理函数（裸跑），打印获取到的按键触发情况
 *
 * @return  none
 */
void TKY_dataProcess (void)
{
    static uint8_t key_val = 0xff;
    static uint16_t print_time = 0;
    uint16_t Wheel_pros = 0;
    uint16_t Slider_pros = 0;
    static uint8_t touchinsflag = 0;

    if (timerFlag)
    {
        timerFlag = 0;
        touch_Scan();
#if PRINT_EN
        print_time++;
        if (print_time == 500)
        {
            print_time = 0;
            touch_InfoDebug();
        }
#endif
    }
    key_val = touch_GetKey();
    switch (key_val)
    {
        case KEY_0_DOWN :
        case KEY_0_LONG :
            key_val = 0;
            touchinsflag |= 1 << 0;
            TKY_KeyBacklightOut (0, !getBacklightState(0));
            break;
        case KEY_1_DOWN :
        case KEY_1_LONG :
            key_val = 1;
            if (!(touchinsflag & 0x0C))//当滑轮实例或者线性滑条实例在触发时，不触发当前按键实例
            {
                touchinsflag |= 1 << 1;
                TKY_KeyBacklightOut (7, !getBacklightState(7));
            }
            break;
        case KEY_NONE :
            key_val = 0xfe;
            break;
        case KEY_0_UP :
            key_val = 0xff;
            touchinsflag &= ~(1 << 0);
            TKY_KeyBacklightOut (0, DISABLE);
            break;
        case KEY_1_UP :
            key_val = 0xff;
            touchinsflag &= ~(1 << 1);
            TKY_KeyBacklightOut (7, DISABLE);
            break;
        default :
            break;
    }

    if (!(touchinsflag & 0x0A))//当按键实例或者线性滑条实例在触发时，不触发当前滑轮实例
    {
        Wheel_pros = touch_GetWheelSliderData();
        if (Wheel_pros < TOUCH_OFF_VALUE)
        {
            touchinsflag |= 1 << 2;
        }
        else
        {
            touchinsflag &= ~(1 << 2);
        }
        TKY_WheelSliderLedProcess(Wheel_pros);
    }

    if (!(touchinsflag & 0x06))//当按键实例或者滑轮实例在触发时，不触发当前线性滑条实例
    {
        Slider_pros = touch_GetLineSliderData();
        if (Slider_pros < TOUCH_OFF_VALUE)
        {
            touchinsflag |= 1 << 3;
        }
        else
        {
            touchinsflag &= ~(1 << 3);
        }
        TKY_LineSliderLedProcess(Slider_pros);
    }
}

/*********************************************************************
 * @fn      TKY_Init
 *
 * @brief   触摸初始化函数（不使用tmos，需要设备开启定时器）
 *
 * @return  none
 */
void TKY_Init(void)
{
	TKY_PeripheralInit();       /* 初始化外设，例如背光和蜂鸣器等 */

	touch_Init(&touch_cfg);		/* 初始化触摸库 */

    TKY_SetSleepStatusValue( ~tkyQueueAll );

    TMR0_TimerInit(FREQ_SYS/1000);               //定时周期为1ms
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
    PFIC_EnableIRQ( TMR0_IRQn );

    dg_log("Touch Key init Finish!\n");
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/*********************************************************************
 * @fn      TKY_PeripheralInit
 *
 * @brief   触摸相关外设初始化函数
 *
 * @return  none
 */
static void TKY_PeripheralInit(void)
{
    /*You code here*/
    TKY_BacklightInit();
}

void TKY_LineSliderLedProcess (uint16_t pros)
{
    uint8_t idx = 0;
    if (pros != TOUCH_OFF_VALUE)
    {
        if (pros > 20)
            idx = (pros - 20) / 20 + 1;
        else
            idx = (pros + 100) / 20 + 1;
        for (uint8_t i = 1; i < 7; i++)
        {
            if (i == idx)
            {
                TKY_KeyBacklightOut (i, ENABLE);
            }
            else
            {
                TKY_KeyBacklightOut (i, DISABLE);
            }
        }
        printf("%d\n",pros);
    }
    else
    {
        for (uint8_t i = 1; i < 7; i++)
        {
            TKY_KeyBacklightOut (i, DISABLE);
        }
    }
}

void TKY_WheelSliderLedProcess (uint16_t pros)
{
    uint8_t idx = 0;
    if (pros != TOUCH_OFF_VALUE)
    {
        if (pros > 10)
            idx = (pros - 10) / 10 + 8;
        else
            idx = (pros + 110) / 10 + 8;

        for (uint8_t i = 8; i < 20; i++)
        {
            if (idx == i)
            {
                TKY_KeyBacklightOut (i, ENABLE);
            }
            else
            {
                TKY_KeyBacklightOut (i, DISABLE);
            }
        }
    }
    else
    {
        for (uint8_t i = 8; i < 20; i++)
        {
            TKY_KeyBacklightOut (i, DISABLE);
        }
    }
}
/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   定时器0中断服务函数
 *
 * @return  none
 */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler( void )
{
    if( TMR0_GetITFlag( TMR0_3_IT_CYC_END ) )
    {
        TMR0_ClearITFlag( TMR0_3_IT_CYC_END );
        timerFlag=1;
        if (led_scanflag)
        {
            /*led scan*/
            TKY_BacklightProcess();          // poll背光状态
        }
    }
}
